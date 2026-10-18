#!/usr/bin/env python3
"""
test_ankle_current_step.py -- tests for the physical-architecture ankle experiment.

WHAT THESE TESTS ARE FOR
    The ankle path has four places a silent error can hide, and none of them shows up
    as a crash or an obviously wrong plot:

      UNITS   the controller works in degrees and amps; the model works in radians and
              newton-metres.  A missing conversion changes a gain by 57x.
      SIGNS   three independent sign conventions meet here (the MuJoCo ankle axis, the
              encoder's positive direction, and the script's ENCODER_TO_MOTOR_SIGN).
              Get the product wrong and the loop is positive feedback -- which does NOT
              produce a NaN, because the authored ROM catches it.  It produces a
              finished run with plausible-looking numbers.
      RATIOS  the 9:1 planetary and the ~4.55:1 belt each have to be applied exactly
              once.  Applying one twice is a 9x or 4.5x torque error that still looks
              like a step response.
      DOUBLE  if the ankle position servo is left connected, the servo and the belt both
      DRIVE   drive the joint and every number is a double count.

    Each of those gets a test that fails loudly rather than a comment.

WHAT THEY ARE NOT FOR
    They do not validate anything against hardware.  No OSL V2 ankle measurement exists
    in this project.  A green run means "the conversions, signs and wiring are
    self-consistent", not "this is what the physical ankle does".

RUN IT
    python tests\\test_ankle_current_step.py
    python tests\\test_ankle_current_step.py -v
    python tests\\test_ankle_current_step.py -k sign

    Also collected by tests/run_tests.py.
"""

from __future__ import annotations

import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
for _p in (ROOT, HERE):
    if _p not in sys.path:
        sys.path.insert(0, _p)


class Skip(Exception):
    """Raised to skip a test that cannot run in this environment."""


try:
    import stub_mujoco
    stub_mujoco.install()           # returns REAL mujoco when importable
except Exception:                                               # noqa: BLE001
    pass

from oslbench.ankle_current_control import (ANKLE_PAPER_CAD_BELT,       # noqa: E402
                                            CAD_BELT_RATIO, CAD_BELT_TEETH,
                                            AnkleCurrentSim, ConversionAudit,
                                            CurrentLimits, CurrentPID,
                                            EncoderConvention, PIDGains,
                                            SCRIPT_KT_OUTPUT_NM_PER_A,
                                            TORQUE_CONSTANT_CHOICES,
                                            CurrentLoopModel,
                                            check_sign_consistency,
                                            derive_mjcf_q_to_encoder_sign,
                                            torque_constant)
from oslbench.drivetrain import PAPER                                   # noqa: E402
from oslbench.drivetrain_sim import DrivetrainLayer                     # noqa: E402

KP, KI, KD = 0.75, 0.0, 0.03
CLIP = 60.0
DT = 1.0e-3


def _pid(kp=KP, ki=KI, kd=KD, clip=CLIP, sign=-1, derivative_on="error"):
    return CurrentPID(PIDGains(kp, ki, kd, sign), CurrentLimits(sim_clip_A=clip),
                      derivative_on=derivative_on)


def _audit(choice="script", n_t=CAD_BELT_RATIO, params=ANKLE_PAPER_CAD_BELT, clip=CLIP):
    kt, prov = torque_constant(choice)
    return ConversionAudit(kt, choice, prov, n_t, "test", params,
                           CurrentLimits(sim_clip_A=clip))


def _src(path):
    """Read a source file, so a test can assert on documented reasoning as well as code."""
    if not os.path.exists(path):
        raise Skip(f"{os.path.relpath(path, ROOT)} not present")
    with open(path, "r", encoding="utf-8") as fh:
        return fh.read()


def _bench():
    from oslbench.model import load_bench_model
    b = load_bench_model(verbose=False)
    if b.failures:
        raise Skip(f"model preflight failed: {b.failures}")
    return b


# =====================================================================================
# 1-6: THE CONTROL LAW
# =====================================================================================
def test_zero_error_gives_zero_current():
    """Nothing in the law may add a bias: at e = 0 and at rest the command is exactly 0."""
    pid = _pid()
    for _ in range(5):
        out = pid.update(5.0, 5.0, DT)
        assert out == 0.0, f"zero error produced {out} A; the law has an offset term"
    assert pid.last["p_A"] == 0.0 and pid.last["i_A"] == 0.0 and pid.last["d_A"] == 0.0


def test_proportional_term_is_kp_times_error_in_degrees():
    """Kp is A/deg, so a 4 deg error is 4*Kp amps -- not 4*pi/180*Kp.

    This is the test that catches a stray radian conversion: the two answers differ by
    57.3x, which is far too big to notice as "slightly wrong gains".
    """
    pid = _pid(kd=0.0)
    pid.update(0.0, 0.0, DT)                    # consume the first update
    for err in (-8.0, -1.0, 0.25, 4.0, 11.0):
        pid.update(err, 0.0, DT)
        assert abs(pid.last["p_A"] - KP * err) < 1e-12, (
            f"P term is {pid.last['p_A']} A at {err} deg error, expected {KP * err} A")
        radian_answer = KP * math.radians(err)
        assert abs(pid.last["p_A"] - radian_answer) > 1e-9 or err == 0.0, (
            "the P term matches the RADIAN interpretation -- a conversion has crept in")


def test_derivative_uses_the_error_difference_over_dt():
    """D is on the ERROR and uses the actual dt handed in, not an assumed rate."""
    pid = _pid()
    pid.update(0.0, 0.0, DT)                    # first update: derivative suppressed
    pid.update(2.0, 0.0, DT)                    # error 0 -> 2 over one dt
    assert abs(pid.last["d_A"] - KD * (2.0 - 0.0) / DT) < 1e-12, pid.last["d_A"]
    pid.update(2.0, 1.0, DT)                    # error 2 -> 1
    assert abs(pid.last["d_A"] - KD * (1.0 - 2.0) / DT) < 1e-12, pid.last["d_A"]
    # and dt really is used, not hard-coded
    pid2 = _pid()
    pid2.update(0.0, 0.0, 2 * DT)
    pid2.update(2.0, 0.0, 2 * DT)
    assert abs(pid2.last["d_A"] - KD * 2.0 / (2 * DT)) < 1e-12


def test_first_update_suppresses_the_derivative_like_the_hardware_script():
    """THE one that matters most. The script's prev_error_deg is unset on update 1.

    With Kd = 0.03 A/(deg/s), a 5 deg step at 1 kHz would give 150 A of derivative on
    the first update -- 2.5x the 60 A clip. Suppressed, the first command is the P term
    alone, 3.75 A. That is the difference between "saturates instantly" and "never
    saturates", so it cannot be left to a comment.
    """
    pid = _pid()
    first = pid.update(5.0, 0.0, DT)
    assert pid.last["derivative_used"] is False
    assert pid.last["d_A"] == 0.0
    assert abs(first - (-1) * KP * 5.0) < 1e-12, (
        f"first command {first} A should be the sign times the P term alone")
    unsuppressed = KD * 5.0 / DT
    assert unsuppressed > CLIP, (
        f"sanity: an unsuppressed first derivative ({unsuppressed} A) should exceed the "
        f"{CLIP} A clip, which is why this behaviour matters")
    second = pid.update(5.0, 0.0, DT)
    assert pid.last["derivative_used"] is True
    assert second == first, "with a constant error the D term should be 0 on update 2"


def test_integral_is_inert_at_ki_zero_and_wound_with_antiwindup_otherwise():
    """Ki = 0 must leave the integrator EXACTLY zero, and a nonzero Ki must clamp."""
    pid = _pid()
    for _ in range(50):
        pid.update(5.0, 0.0, DT)
    assert pid.integral_deg_s == 0.0, (
        "with Ki = 0 the integral state must not accumulate at all, or switching Ki on "
        "later would start from a stale value")
    assert pid.last["i_A"] == 0.0

    ki = 2.0
    pid2 = _pid(ki=ki)
    for _ in range(20000):                      # 20 s of a standing 5 deg error
        pid2.update(5.0, 0.0, DT)
    assert abs(pid2.last["i_A"]) <= CLIP + 1e-9, (
        f"anti-windup failed: the I term reached {pid2.last['i_A']} A against a "
        f"{CLIP} A clip")
    assert abs(pid2.integral_deg_s) <= CLIP / ki + 1e-9


def test_saturation_clamps_both_signs_and_is_reported():
    """The clip is symmetric and the sign is applied AFTER it, so it stays symmetric."""
    pid = _pid(kp=1000.0, kd=0.0)               # guarantees saturation
    hi = pid.update(5.0, 0.0, DT)
    assert pid.last["saturated"] is True
    assert abs(hi) == CLIP, f"positive-error command {hi} should clip to +/-{CLIP}"
    pid.update(0.0, 0.0, DT)
    lo = pid.update(-5.0, 0.0, DT)
    assert pid.last["saturated"] is True
    assert abs(lo) == CLIP, f"negative-error command {lo} should clip to +/-{CLIP}"
    assert hi == -lo, "the clip must be symmetric about zero"
    pid.reset()
    small = pid.update(0.001, 0.0, DT)
    assert pid.last["saturated"] is False and abs(small) < CLIP


def test_request_and_applied_are_reported_in_the_same_frame():
    """`raw_signed_A` and `cmd_A` must differ ONLY by clipping.

    An earlier version logged the pre-sign raw value against the post-sign command, so
    the two differed by the motor sign on EVERY step and saturation was invisible in
    the comparison. A request/applied pair that always differs tells you nothing.
    """
    pid = _pid(kp=1000.0, kd=0.0)
    pid.update(5.0, 0.0, DT)
    assert pid.last["saturated"] is True
    assert pid.last["raw_signed_A"] != pid.last["cmd_A"], "saturation should separate them"
    pid.reset()
    pid.update(0.001, 0.0, DT)
    assert pid.last["raw_signed_A"] == pid.last["cmd_A"], (
        "unsaturated, request and applied must be identical -- same sign convention")


def test_derivative_on_measurement_is_available_but_not_the_default():
    """A derivative-on-measurement variant must be opt-in, and must actually differ."""
    assert _pid().derivative_on == "error"
    pid = _pid(derivative_on="measurement")
    pid.update(0.0, 0.0, DT)
    pid.update(5.0, 0.0, DT)                    # step in the REFERENCE, meas unchanged
    assert pid.last["d_A"] == 0.0, (
        "derivative-on-measurement must not react to a reference step; that is the "
        "whole difference between the two variants")
    pid_e = _pid()
    pid_e.update(0.0, 0.0, DT)
    pid_e.update(5.0, 0.0, DT)
    assert pid_e.last["d_A"] != 0.0, "derivative-on-error must react to a reference step"


# =====================================================================================
# 7-8: UNITS AND SIGNS
# =====================================================================================
def test_encoder_conversion_round_trips_and_carries_the_sign():
    """deg <-> rad with the sign, exactly invertible."""
    for sign in (-1, 1):
        enc = EncoderConvention(sign=sign)
        for q_deg in (-30.0, -5.0, 0.0, 5.0, 20.0):
            q_rad = math.radians(q_deg)
            e = enc.to_encoder_deg(q_rad)
            assert abs(e - sign * q_deg) < 1e-12
            assert abs(enc.to_model_rad(e) - q_rad) < 1e-15, "round trip is not exact"
        # velocity carries the sign but never the offset
        enc_off = EncoderConvention(sign=sign, offset_deg=7.0)
        assert abs(enc_off.to_encoder_deg_per_s(1.0) - sign * math.degrees(1.0)) < 1e-12


def test_mapping_1_is_derived_from_geometry_and_never_from_mapping_2():
    """The two sign mappings must be documented as separate, non-circular quantities.

    This is the review finding. `mjcf_q_to_encoder_sign` (MuJoCo coordinate -> encoder
    degrees) is a COORDINATE-FRAME fact. `controller_output_to_motor_current_sign` (the
    script's ENCODER_TO_MOTOR_SIGN) is a CONTROLLER-INTERNAL fact. I had justified the
    first partly via the loop-stability condition, which involves the second -- circular.
    """
    src = _src(os.path.join(ROOT, "oslbench", "ankle_current_control.py"))
    # the derivation must cite the geometry it rests on
    for needle in ("0.1665", "0.0756", "(0, 1, 0)", "dz/dtheta",
                   "DORSIFLEXION", "mjcf_q_to_encoder_sign"):
        assert needle in src, f"the mapping-1 derivation must cite {needle!r}"
    # and must say in writing that it does not use mapping 2
    assert "No reference to mapping 2" in src or "Independent of the controller" in src
    # the deriver exists under a name that says which mapping it is
    assert callable(derive_mjcf_q_to_encoder_sign)
    # mapping 2 is reachable under its own explicit name
    assert PIDGains().controller_output_to_motor_current_sign == -1
    # the consistency check must report the product, not assert a mapping
    c = check_sign_consistency(EncoderConvention(), PIDGains(), PAPER)
    assert set(c) >= {"mjcf_q_to_encoder_sign",
                      "controller_output_to_motor_current_sign",
                      "product", "negative_feedback"}
    assert c["product"] == 1 and c["negative_feedback"] is True


def test_the_geometric_derivation_of_mapping_1_reproduces_from_the_stl():
    """Re-derive mapping 1 from the actual mesh, so the docstring's numbers stay true.

    Parses the Vari-Flex foot STL, transforms it by its geom pos/quat into the foot-body
    frame (whose origin IS the ankle joint), and checks that positive rotation about +Y
    raises the toe and lowers the heel -- i.e. dorsiflexion, so the mapping is -1.
    Skips if the mesh is absent rather than silently passing.
    """
    import re
    import struct
    import numpy as np
    mesh = os.path.join(ROOT, "osl_v2_0_assembly", "meshes",
                        "z_lowProfileVariFlexFoot_ossur_VLPE5XX0.stl")
    if not os.path.exists(mesh):
        raise Skip("the Vari-Flex foot STL is not present")
    xml = _src(os.path.join(ROOT, "models", "osl_v2_bench.xml"))
    g = re.search(r'<geom class="visual" mesh="z_lowprofilevariflexfoot_ossur_vlpe5xx0" '
                  r'pos="([^"]+)" quat="([^"]+)"', xml)
    assert g, "the foot geom is no longer in the MJCF where the derivation expects it"
    with open(mesh, "rb") as fh:
        n = struct.unpack("<I", fh.read(84)[80:84])[0]
        raw = np.frombuffer(fh.read(n * 50), dtype=np.uint8)
    V = (raw.reshape(n, 50)[:, 12:48].copy().view("<f4")
         .reshape(n, 3, 3).astype(float).reshape(-1, 3))
    w, x, y, z = (float(v) for v in g.group(2).split())
    R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                  [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                  [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])
    W = V @ R.T + np.array([float(v) for v in g.group(1).split()])
    toe, heel = W[int(np.argmin(W[:, 0]))], W[int(np.argmax(W[:, 0]))]
    # anterior must be readable from the geometry: the foot is longer than it is tall
    assert (W[:, 0].max() - W[:, 0].min()) > (W[:, 2].max() - W[:, 2].min()), (
        "the foot is not longer along X than along Z, so anterior cannot be read off it")
    assert toe[0] < 0.0 < heel[0], (
        f"expected the toe at -X and the heel at +X, got {toe[0]:+.4f} / {heel[0]:+.4f}")
    # dz/dtheta = -x for a +Y hinge
    assert -toe[0] > 0.0, "positive q should RAISE the toe"
    assert -heel[0] < 0.0, "positive q should LOWER the heel"
    # finite angle, not just the derivative
    th = math.radians(20.0)
    Ry = np.array([[math.cos(th), 0, math.sin(th)], [0, 1, 0],
                   [-math.sin(th), 0, math.cos(th)]])
    assert (Ry @ toe)[2] > toe[2] and (Ry @ heel)[2] < heel[2], (
        "at +20 deg the toe must rise and the heel must drop")
    # therefore positive q = dorsiflexion, which opposes the encoder's positive
    assert EncoderConvention().positive_means == "plantarflexion"
    assert EncoderConvention().sign == -1, (
        "the geometry says positive q is dorsiflexion and the encoder calls "
        "plantarflexion positive, so mapping 1 must be -1")


def test_the_runtime_deriver_requires_toe_and_heel_to_disagree():
    """A single tracked point could be noise; a hinge must move its ends oppositely.

    On the stub there are no geoms, so this must return None WITH a reason rather than
    guessing -- the fallback is what keeps a stub run from inventing a mapping.
    """
    bench = _bench()
    sign, report = derive_mjcf_q_to_encoder_sign(bench)
    assert report, "the deriver must always explain itself"
    if sign is None:
        assert ("geom" in report or "foot" in report or "stub" in report), report
    else:
        assert sign in (-1, 1)
        assert "opposite" in report or "toe moved" in report
        if "compiled mesh" in report:
            assert "z_lowprofilevariflexfoot_ossur_vlpe5xx0" in report
            assert "556 actual vertices" in report
            assert "242 mm" in report and "56 mm" in report
            assert "offset already represented in compiled vertices" in report


def test_the_encoder_frame_rom_helper_swaps_the_interval_ends():
    """rom_in_encoder_frame must return a sorted interval in ENCODER degrees."""
    bench = _bench()
    for sign, want in ((-1, (-20.0, 30.0)), (+1, (-30.0, 20.0))):
        lo, hi = EncoderConvention(sign=sign).rom_in_encoder_frame(bench.ankle_range)
        assert abs(lo - want[0]) < 1e-9 and abs(hi - want[1]) < 1e-9, (
            f"sign {sign:+d}: got ({lo:.3f}, {hi:.3f}), expected {want}")
        assert lo < hi, "the helper must return (min, max)"


def test_the_loop_is_negative_feedback_only_for_one_sign_product():
    """The three sign conventions must multiply to negative feedback.

        meas    = s_enc * deg(q)
        err     = ref - meas
        cmd     = s_mot * Kp * err
        tau_j follows the sign of the command, so dq follows sign(cmd)
        d(meas) = s_enc * dq   ->   sign(d_meas) = s_enc * s_mot * sign(err)

    Negative feedback means the measurement moves TOWARD the reference, i.e. d_meas has
    the SAME sign as the error. That requires s_enc * s_mot = +1 -- the two signs must
    AGREE, not oppose.

    I got this backwards the first time I wrote this test (asserted < 0) and the test
    failed against correct code. The condition is checked below AND confirmed against
    the simulation in `test_the_sign_condition_matches_what_the_simulation_does`, which
    is the arbiter: an algebra test that encodes my own sign error is worth nothing.

    NOTE ON WHAT THIS IS NOT. This is a CONSEQUENCE of the two mappings, not evidence
    for either. Mapping 1 is derived from geometry in
    `test_the_geometric_derivation_of_mapping_1_reproduces_from_the_stl`; using the
    stability condition to infer it would be circular.
    """
    for s_enc in (-1, 1):
        for s_mot in (-1, 1):
            err = 5.0
            cmd = s_mot * KP * err
            d_meas = s_enc * cmd
            negative_feedback = (d_meas * err) > 0
            assert negative_feedback == (s_enc * s_mot > 0), (
                f"sign algebra is wrong for s_enc={s_enc}, s_mot={s_mot}")
    # and the shipped defaults must be a stable pair
    assert EncoderConvention().sign * PIDGains().encoder_to_motor_sign > 0, (
        f"the default encoder sign ({EncoderConvention().sign:+d}) and the script's "
        f"ENCODER_TO_MOTOR_SIGN ({PIDGains().encoder_to_motor_sign:+d}) multiply to "
        f"POSITIVE feedback; one of them is wrong")


def test_the_sign_condition_matches_what_the_simulation_does():
    """Run both sign products through the real loop; only the agreeing one tracks.

    This is the arbiter for the algebra above. It is deliberately a short run of the
    actual `AnkleCurrentSim`, not a restatement of the formula, so that a sign error in
    my reasoning cannot pass both tests.
    """
    bench_sign_results = {}
    for s_enc in (-1, +1):
        bench = _bench()
        sim = AnkleCurrentSim(bench, _audit())
        pid = _pid()                            # s_mot = -1
        enc = EncoderConvention(sign=s_enc)
        sim.reset(0.0)
        for _ in range(600):
            cmd = pid.update(5.0, enc.to_encoder_deg(sim.theta_j), 1e-3)
            for _ in range(2):
                sim.step(cmd)
        bench_sign_results[s_enc] = enc.to_encoder_deg(sim.theta_j)

    tracks = {s: abs(v - 5.0) < 1.0 for s, v in bench_sign_results.items()}
    assert tracks[-1] is True, (
        f"s_enc=-1 with s_mot=-1 (product +1) should TRACK the 5 deg reference, "
        f"ended at {bench_sign_results[-1]:.3f} deg")
    assert tracks[+1] is False, (
        f"s_enc=+1 with s_mot=-1 (product -1) should NOT track, "
        f"ended at {bench_sign_results[+1]:.3f} deg")
    # which is exactly the condition the algebra test asserts
    assert (EncoderConvention().sign * PIDGains().encoder_to_motor_sign > 0) is True


def test_current_sign_propagates_to_joint_torque_with_the_documented_chain():
    """I_q > 0 -> tau_m > 0 -> theta_a rises -> theta_s < 0 -> tau_j > 0."""
    layer = DrivetrainLayer(ANKLE_PAPER_CAD_BELT, enabled=True, substeps=1)
    layer.seed_rigid(0.0, 0.0)
    rec = layer.advance(0.0, +5.0, 5e-4)
    assert rec.tau_m > 0.0, "positive current must give positive motor torque"
    assert rec.theta_a_dot > 0.0, "positive motor torque must spin the actuator positive"
    rec2 = layer.advance(0.0, +5.0, 5e-4)
    assert rec2.theta_s < 0.0, "theta_s = theta_j - theta_a/n_t must go negative"
    assert rec2.tau_j > 0.0, "and paper (4) must turn that into a POSITIVE joint torque"
    # the mirror case
    layer2 = DrivetrainLayer(ANKLE_PAPER_CAD_BELT, enabled=True, substeps=1)
    layer2.seed_rigid(0.0, 0.0)
    layer2.advance(0.0, -5.0, 5e-4)
    rec3 = layer2.advance(0.0, -5.0, 5e-4)
    assert rec3.theta_s > 0.0 and rec3.tau_j < 0.0, "the chain must mirror exactly"


# =====================================================================================
# 9-11: RATIOS, APPLIED EXACTLY ONCE
# =====================================================================================
def test_the_cad_belt_ratio_comes_from_the_tooth_counts():
    """50/11, and it is NOT the paper's 4.61."""
    assert CAD_BELT_TEETH == (50, 11)
    assert abs(CAD_BELT_RATIO - 50.0 / 11.0) < 1e-15
    assert abs(CAD_BELT_RATIO - 4.545454545454545) < 1e-12
    assert CAD_BELT_RATIO != PAPER.n_t, (
        "the CAD and paper belt ratios must stay distinct; silently equating them is "
        "how a 1.4 % error becomes invisible")
    assert ANKLE_PAPER_CAD_BELT.n_t == CAD_BELT_RATIO
    assert PAPER.n_t == 4.61, "PAPER itself must never be mutated"


def test_the_planetary_ratio_is_applied_exactly_once():
    """The layer's own eq (1) owns n_a; the audit must not apply it again.

    The guard is `layer_current_for`, which rescales the controller's amps into the
    layer's rotor frame. If it were missing, or if it also multiplied by n_a, the motor
    torque would be wrong by 9x -- and the run would still look like a step response.
    """
    for choice in TORQUE_CONSTANT_CHOICES:
        a = _audit(choice)
        for i_ctrl in (-7.5, 0.0, 3.0, 12.0):
            want_output_torque = i_ctrl * a.k_t_output_Nm_per_A
            i_layer = a.layer_current_for(i_ctrl)
            got_output_torque = i_layer * PAPER.k_t * PAPER.n_a     # the layer's eq (1)
            assert abs(got_output_torque - want_output_torque) < 1e-12, (
                f"{choice}: the layer would produce {got_output_torque} N.m at the "
                f"output for a command meant to produce {want_output_torque} N.m")
        # and the implied rotor constant is the output one divided by n_a, once
        assert abs(a.rotor_kt_implied - a.k_t_output_Nm_per_A / PAPER.n_a) < 1e-15


def test_the_belt_ratio_is_applied_exactly_once_in_the_kinematics():
    """theta_s uses n_t once; the rigid slope uses it once; they are different things."""
    a = _audit()
    assert abs(a.rigid_joint_kt - a.k_t_output_Nm_per_A * CAD_BELT_RATIO) < 1e-12
    # the compliant path: n_t enters only through the deflection
    layer = DrivetrainLayer(ANKLE_PAPER_CAD_BELT, enabled=True, substeps=1)
    layer.seed_rigid(0.2, 0.0)
    th_s = layer.belt(0.2).theta_s
    assert abs(th_s) < 1e-12, "a rigid seed must give zero deflection"
    layer.theta_a = ANKLE_PAPER_CAD_BELT.n_t * 0.2 - 0.01 * ANKLE_PAPER_CAD_BELT.n_t
    st = layer.belt(0.2)
    assert abs(st.theta_s - 0.01) < 1e-12, (
        f"theta_s should be theta_j - theta_a/n_t = 0.01 rad, got {st.theta_s}")


def test_actuator_side_and_joint_side_torque_are_distinct_and_differ_by_n_t():
    """tau_a and tau_j are different channels; conflating them is a 4.5x error."""
    layer = DrivetrainLayer(ANKLE_PAPER_CAD_BELT, enabled=True, substeps=1)
    layer.seed_rigid(0.0, 0.0)
    for _ in range(40):
        rec = layer.advance(0.0, 6.0, 5e-4)
    assert rec.tau_a != rec.tau_j, "the two torque frames must not be the same number"
    assert abs(rec.tau_j - ANKLE_PAPER_CAD_BELT.n_t * rec.tau_a) < 1e-9, (
        f"tau_j ({rec.tau_j}) should be n_t x tau_a ({rec.tau_a}) exactly once")


def test_the_three_torque_constants_stay_distinct_and_are_all_labelled():
    """No candidate may silently become the default 'right' one."""
    vals = {k: v for k, (v, _) in TORQUE_CONSTANT_CHOICES.items()}
    assert len(set(round(v, 9) for v in vals.values())) == 3, (
        f"the three candidates must stay distinct: {vals}")
    assert abs(vals["script"] - SCRIPT_KT_OUTPUT_NM_PER_A) < 1e-15
    assert abs(vals["paper"] - PAPER.k_t * PAPER.n_a) < 1e-15
    for name, (_, prov) in TORQUE_CONSTANT_CHOICES.items():
        assert prov and len(prov) > 20, f"{name} has no provenance string"
    # the 6-vs-9 coincidence this project found, recorded so it cannot be lost
    assert abs(SCRIPT_KT_OUTPUT_NM_PER_A / 0.095 - 6.0) < 0.01, (
        "the script's KT is ~0.095 x 6, i.e. the AK80-6 ratio, not the AK80-9 one; "
        "that open question is documented in PHYSICAL_ANKLE_CONTROL_ALIGNMENT.md")


# =====================================================================================
# 12-15: WIRING, SCHEDULING, AND THE PROTECTED BASELINE
# =====================================================================================
def test_exactly_one_actuation_path_on_the_ankle():
    """The ankle servo must be dead, and the belt must be the only thing driving it."""
    bench = _bench()
    sim = AnkleCurrentSim(bench, _audit())
    assert sim.servo_connected is False
    sim.reset(0.0)
    for _ in range(50):
        sim.step(4.0)
        assert sim.ankle_servo_force() == 0.0, (
            f"the ankle position actuator produced {sim.ankle_servo_force()} N.m; the "
            f"servo and the belt would both be driving the joint")
    # and the belt really is doing something, or the test above is vacuous
    assert abs(sim.last_layer.tau_j) > 1e-6, "the belt produced no torque at all"
    assert abs(float(bench.data.qfrc_applied[bench.ankle_dof])) > 0.0


def test_the_knee_servo_is_deliberately_left_connected():
    """The knee holds the shank; disconnecting it would change the ankle's gravity load."""
    bench = _bench()
    sim = AnkleCurrentSim(bench, _audit())
    sim.reset(0.0)
    sim.step(0.0)
    assert bench.model.actuator_gainprm[bench.knee_act, 0] != 0.0, (
        "the knee actuator was disconnected; this experiment needs it holding the knee")
    assert bench.model.actuator_gainprm[bench.ankle_act, 0] == 0.0, (
        "the ANKLE actuator should be the disconnected one")


def test_the_ankle_sim_touches_only_the_ankle_dof():
    """qfrc_applied must be written at the ankle and nowhere else."""
    import numpy as np
    bench = _bench()
    sim = AnkleCurrentSim(bench, _audit())
    sim.reset(0.0)
    sim.step(5.0)
    q = np.asarray(bench.data.qfrc_applied, dtype=float)
    others = [i for i in range(len(q)) if i != bench.ankle_dof]
    assert all(q[i] == 0.0 for i in others), (
        f"qfrc_applied is nonzero away from the ankle dof: {q}")


def test_authored_limits_survive_the_run():
    """The disconnect touches gainprm/biasprm only; ROM and authority stay authored."""
    bench = _bench()
    before = (tuple(bench.model.actuator_forcerange[bench.ankle_act]),
              tuple(bench.model.actuator_ctrlrange[bench.ankle_act]))
    sim = AnkleCurrentSim(bench, _audit())
    sim.reset(0.0)
    for _ in range(100):
        sim.step(9.0)
    after = (tuple(bench.model.actuator_forcerange[bench.ankle_act]),
             tuple(bench.model.actuator_ctrlrange[bench.ankle_act]))
    assert before == after, f"authored ankle limits changed: {before} -> {after}"
    assert bench.limits_unchanged(), "the bench's own limit check failed"


def test_controller_scheduling_is_deterministic_and_the_command_is_held():
    """1 kHz controller against a 0.5 ms step = exactly 2 physics steps per update."""
    bench = _bench()
    dt = bench.timestep
    steps_per_update = int(round((1.0 / 1000.0) / dt))
    assert steps_per_update == 2, (
        f"expected 2 physics steps per control update at dt = {dt}, got "
        f"{steps_per_update}")
    assert abs(steps_per_update * dt - 1.0 / 1000.0) < 1e-15, (
        "1 kHz must divide exactly into the physics timestep, or the schedule drifts")
    # the same run twice must be bit-identical
    import numpy as np
    traces = []
    for _ in range(2):
        b = _bench()
        sim = AnkleCurrentSim(b, _audit())
        pid = _pid()
        enc = EncoderConvention(sign=-1)
        sim.reset(0.0)
        tr = []
        for u in range(40):
            cmd = pid.update(5.0, enc.to_encoder_deg(sim.theta_j), 1e-3)
            for _ in range(steps_per_update):
                sim.step(cmd)
                tr.append((sim.theta_j, sim.last_layer.tau_j, cmd))
        traces.append(np.asarray(tr, dtype=float))
    assert np.array_equal(traces[0], traces[1]), "the run is not deterministic"
    # the command really is held across the pair
    for i in range(0, len(traces[0]) - 1, 2):
        assert traces[0][i][2] == traces[0][i + 1][2], (
            "the current changed between the two physics steps of one control update")


def test_a_small_step_stays_finite_and_inside_the_configured_limits():
    """The headline sanity run: 0.4 s of the real loop, finite and bounded."""
    bench = _bench()
    audit = _audit()
    sim = AnkleCurrentSim(bench, audit)
    pid = _pid()
    enc = EncoderConvention(sign=-1)
    sim.reset(0.0)
    peak_i = peak_tau_j = peak_ths = 0.0
    for u in range(400):
        cmd = pid.update(5.0, enc.to_encoder_deg(sim.theta_j), 1e-3)
        assert abs(cmd) <= CLIP + 1e-12, f"command {cmd} A exceeded the {CLIP} A clip"
        for _ in range(2):
            sim.step(cmd)
            L = sim.last_layer
            for name, v in (("theta_j", sim.theta_j), ("tau_j", L.tau_j),
                            ("theta_s", L.theta_s), ("theta_a_dot", L.theta_a_dot)):
                assert math.isfinite(v), f"{name} went non-finite at update {u}"
            peak_i = max(peak_i, abs(cmd))
            peak_tau_j = max(peak_tau_j, abs(L.tau_j))
            peak_ths = max(peak_ths, abs(L.theta_s))
    lo, hi = bench.ankle_range
    assert lo - 1e-6 <= sim.theta_j <= hi + 1e-6, (
        f"the ankle left its authored ROM: {math.degrees(sim.theta_j):.3f} deg")
    assert peak_tau_j <= bench.ankle_forcerange[1], (
        f"peak joint torque {peak_tau_j:.3f} N.m exceeded the authored "
        f"{bench.ankle_forcerange[1]} N.m authority")
    # and it actually moved, or the bounds above are vacuous
    assert abs(enc.to_encoder_deg(sim.theta_j)) > 1.0, (
        "the ankle barely moved; the test would pass even with no actuation")


def test_a_reversed_encoder_sign_is_detectable_rather_than_silent():
    """Positive feedback does NOT produce a NaN here -- the ROM catches it.

    That is the dangerous case: the run completes, the metrics compute, and the numbers
    look like a step response. The detectable signature is a large standing error with
    heavy saturation, which is what the experiment's warning keys off.
    """
    bench = _bench()
    sim = AnkleCurrentSim(bench, _audit())
    pid = _pid()
    enc_wrong = EncoderConvention(sign=+1)      # product with s_mot = -1 is POSITIVE fb
    sim.reset(0.0)
    saturated = 0
    n = 0
    for u in range(600):
        cmd = pid.update(5.0, enc_wrong.to_encoder_deg(sim.theta_j), 1e-3)
        saturated += int(pid.last["saturated"])
        for _ in range(2):
            sim.step(cmd)
            n += 1
    final_err = 5.0 - enc_wrong.to_encoder_deg(sim.theta_j)
    assert math.isfinite(sim.theta_j), (
        "a reversed sign should NOT go non-finite -- the ROM bounds it. If this fails, "
        "the detection story in run_ankle_current_step.py is wrong.")
    assert abs(final_err) > 0.5 * 5.0, (
        f"a reversed sign should leave a large standing error, got {final_err:.3f} deg")
    assert saturated > 0 or abs(math.degrees(sim.theta_j)) > 0.5 * 5.0, (
        "a reversed sign should either saturate or visibly drive the joint away from "
        "the reference")


def test_the_protected_baseline_files_are_untouched_by_this_feature():
    """The ankle work must not have edited the knee controller or the model.

    A source-level check, so it fails in the same run as everything else rather than
    waiting for the oracle.
    """
    import hashlib
    expectations = {
        "oslbench/controller.py": ("KD = 17.253", "KP = 600.0"),
        "models/osl_v2_bench.xml": ('name="ankle_pos"', 'forcerange="-168.2 168.2"',
                                    'kp="60.0"'),
    }
    for rel, needles in expectations.items():
        path = os.path.join(ROOT, rel)
        with open(path, "r", encoding="utf-8", errors="replace") as fh:
            text = fh.read()
        for needle in needles:
            assert needle in text, f"{rel} no longer contains {needle!r}"
    # the ankle module must not import or monkey-patch the protected controller's gains
    with open(os.path.join(ROOT, "oslbench", "ankle_current_control.py"),
              encoding="utf-8") as fh:
        src = fh.read()
    for banned in ("controller.KP", "controller.KD", "KP =", "KD =",
                   "write_to_model(model, bench.knee_act)"):
        assert banned not in src, (
            f"the ankle module touches the protected controller: {banned!r}")
    assert "write_to_model" in src, "it should still REUSE write_to_model for the ankle"
    assert hashlib.sha256(b"").hexdigest()          # keeps the import honest


def test_the_experiment_requires_real_mujoco_by_default():
    """The default backend must REFUSE a stub, not silently fall back to one.

    This is the review finding that produced the gate. `StubModel.__init__(path)` stores
    the MJCF path and never parses it, so a stub run reports the right path, exposes
    `ankle_qpos`/`ankle_dof`, and draws correctly-labelled plots from a plant whose ankle
    has armature 0.0 (vs 0.008), damping 0.05 (vs 0.25) and frictionloss 0.05 (vs 0.3).
    A source-level check, because importing the module would resolve the backend.
    """
    path = os.path.join(ROOT, "experiments", "run_ankle_current_step.py")
    with open(path, encoding="utf-8") as fh:
        src = fh.read()
    assert 'choices=("mujoco", "stub"), default="mujoco"' in src, (
        "the --backend flag must exist and default to 'mujoco'")
    assert "REFUSING TO RUN: real MuJoCo is not importable" in src, (
        "the default path must fail with an explanation, not fall back to a stub")
    assert "_STUB" in src, "stub output must go to a separate directory"
    assert "STUB -- NOT MUJOCO" in src, "stub output must be marked"
    # the identity recorder must read the COMPILED model, not our own constants
    for needle in ("m.dof_armature[bench.ankle_dof]", "m.dof_damping[bench.ankle_dof]",
                   "ankle_properties_match_mjcf", "model_xml_sha256"):
        assert needle in src, f"model_identity must record {needle}"


def test_the_stub_really_does_disagree_with_the_mjcf():
    """Prove the three divergences the gate's message cites, so the message stays honest.

    If someone later aligns the stub's ankle with the MJCF, this test fails and the
    gate's wording has to be updated rather than quietly becoming wrong.
    """
    import re
    with open(os.path.join(ROOT, "models", "osl_v2_bench.xml"), encoding="utf-8") as fh:
        joint = re.search(r'<joint name="ankle"[^/]*/>', fh.read()).group(0)
    assert 'armature="0.008"' in joint and 'damping="0.25"' in joint
    assert 'frictionloss="0.3"' in joint
    import stub_mujoco as S
    assert S.B_ANKLE == 0.05 and S.FRIC_ANKLE == 0.05, (
        f"stub ankle damping/frictionloss are now {S.B_ANKLE}/{S.FRIC_ANKLE}; the "
        f"backend gate's message quotes 0.05/0.05 and must be updated")
    model = S.StubModel("models/osl_v2_bench.xml")
    assert model.dof_armature[1] == 0.0, (
        "the stub's ankle armature is no longer 0.0; update the gate's message")
    # and the stub genuinely ignores the file it is handed
    assert S.StubModel("/nonexistent/path.xml").jnt_range.shape == (2, 2), (
        "StubModel parses nothing -- it accepted a path that does not exist, which is "
        "exactly why a stub run can look like a real one")


def test_the_four_torque_frames_are_separately_named():
    """Rotor / actuator-output / belt-actuator-side / belt-joint-side, all distinct.

    In particular `I_q x k_t_output` must NOT be called motor-rotor torque: the chosen
    constant is claimed to be an actuator-OUTPUT constant, so the product is an
    estimated actuator-output torque.
    """
    path = os.path.join(ROOT, "experiments", "run_ankle_current_step.py")
    with open(path, encoding="utf-8") as fh:
        src = fh.read()
    for name in ("estimated_actuator_output_torque_Nm", "implied_rotor_torque_Nm",
                 "belt_torque_actuator_side_Nm", "belt_torque_joint_side_Nm"):
        assert name in src, f"the CSV must carry a distinctly named {name}"
    assert "tau_motor_actuator_Nm" not in src, (
        "the old ambiguous label is still present; I_q x k_t_output is an ESTIMATED "
        "ACTUATOR-OUTPUT torque, not motor torque")
    assert "ESTIMATED actuator-output torque" in src, (
        "the plot legend must say the quantity is an estimate through an unresolved "
        "torque constant")


def test_the_six_to_nine_lead_is_labelled_as_a_hypothesis_not_a_finding():
    """0.5701/0.095 ~ 6 is a lead. Nothing may call it confirmed."""
    src = ""
    for rel in ("oslbench/ankle_current_control.py",
                "docs/PHYSICAL_ANKLE_CONTROL_ALIGNMENT.md"):
        with open(os.path.join(ROOT, rel), encoding="utf-8") as fh:
            src += fh.read()
    assert "HYPOTHESIS" in src or "hypothesis" in src, (
        "the 6-vs-9 coincidence must be labelled a hypothesis")
    for banned in ("confirms that the script", "proves the script",
                   "established that the script applied"):
        assert banned not in src, f"over-claim about the 6-vs-9 lead: {banned!r}"
    # the arithmetic itself, so the lead is at least correctly stated
    assert abs(SCRIPT_KT_OUTPUT_NM_PER_A / 0.095 - 6.0) < 0.01


def test_the_encoder_frame_limit_mapping_is_computed_not_assumed():
    """CAD ROM in the encoder frame, with the interval ends SWAPPING for a negative sign.

    MJCF [-30, +20] deg with encoder sign -1 becomes encoder [-20, +30] deg. Getting
    this wrong makes a "within range" check silently test the wrong end.
    """
    bench = _bench()
    for sign, want in ((-1, (-20.0, 30.0)), (+1, (-30.0, 20.0))):
        enc = EncoderConvention(sign=sign)
        ends = sorted((enc.to_encoder_deg(bench.ankle_range[0]),
                       enc.to_encoder_deg(bench.ankle_range[1])))
        assert abs(ends[0] - want[0]) < 1e-9 and abs(ends[1] - want[1]) < 1e-9, (
            f"sign {sign:+d}: encoder-frame ROM {ends}, expected {list(want)}")
    # the reported physical range exceeds the CAD travel at one end -- documented, and
    # the 5 deg step is inside both
    from experiments.run_ankle_current_step import (PHYSICAL_ENCODER_RANGE_DEG,
                                                    STEP_DEG, limit_report)
    enc = EncoderConvention(sign=-1)
    lines = limit_report(bench, enc)
    assert any("MISMATCH" in ln for ln in lines), (
        "the 9 deg overhang of the reported physical range must be reported, not hidden")
    lo_e, hi_e = sorted((enc.to_encoder_deg(bench.ankle_range[0]),
                         enc.to_encoder_deg(bench.ankle_range[1])))
    assert lo_e <= STEP_DEG <= hi_e, "the test step must be inside the CAD ROM"
    assert (PHYSICAL_ENCODER_RANGE_DEG[0] <= STEP_DEG
            <= PHYSICAL_ENCODER_RANGE_DEG[1]), (
        "the test step must also be inside the reported physical range")


# =====================================================================================
# THE OPTIONAL CURRENT-LOOP MODEL
# =====================================================================================
def test_zero_time_constant_is_an_exact_passthrough():
    """tau_i = 0 must reproduce the ideal loop BIT-FOR-BIT, not approximately.

    The whole value of the default is that it leaves the established baseline alone. An
    'almost identical' passthrough would silently invalidate every earlier ankle result.
    """
    loop = CurrentLoopModel(0.0)
    assert loop.ideal is True
    for v in (0.0, -3.75, 60.0, 1.0 / 3.0, -1e-12, 12345.678):
        assert loop.update(v, 5e-4) == v, (
            f"tau_i = 0 returned {loop.update(v, 5e-4)!r} for a target of {v!r}; the "
            f"passthrough must be exact")
    # and no one-step delay: the very first update already equals the target
    fresh = CurrentLoopModel(0.0)
    assert fresh.update(-7.0, 5e-4) == -7.0


def test_positive_time_constant_matches_the_analytic_first_order_solution():
    """The exact exponential update must agree with the closed form, step for step.

    Checked against i(t) = i_target + (i0 - i_target)*exp(-t/tau) rather than against a
    forward-Euler reimplementation -- comparing two numerical schemes to each other
    would only prove they are the same scheme.
    """
    for tau, dt in ((1e-3, 5e-4), (2e-3, 5e-4), (5e-3, 5e-4), (20e-3, 2.5e-4)):
        loop = CurrentLoopModel(tau, i0=0.0)
        target = -3.75
        worst = 0.0
        for k in range(1, 81):
            got = loop.update(target, dt)
            want = target + (0.0 - target) * math.exp(-(k * dt) / tau)
            worst = max(worst, abs(got - want))
        assert worst < 1e-12, (
            f"tau={tau}, dt={dt}: worst deviation from the analytic solution {worst:.3e} A")
    # one time constant must reach 1 - 1/e of the way
    loop = CurrentLoopModel(5e-3, i0=0.0)
    for _ in range(10):                      # 10 x 0.5 ms = 5 ms = one tau
        loop.update(1.0, 5e-4)
    assert abs(loop.i_actual - (1.0 - math.exp(-1.0))) < 1e-12


def test_a_current_step_is_bounded_and_monotonic():
    """No overshoot, no ringing, never past the target: a first-order lag cannot."""
    for tau in (1e-3, 5e-3, 20e-3):
        for target in (-3.75, +12.0):
            loop = CurrentLoopModel(tau, i0=0.0)
            prev = 0.0
            for _ in range(400):
                v = loop.update(target, 5e-4)
                assert min(0.0, target) - 1e-12 <= v <= max(0.0, target) + 1e-12, (
                    f"tau={tau}: current {v} left the interval [0, {target}]")
                assert abs(v - target) <= abs(prev - target) + 1e-15, (
                    f"tau={tau}: moved AWAY from the target, {prev} -> {v}")
                prev = v
            assert abs(prev - target) < 1e-3 * abs(target), "did not converge"


def test_the_loop_applies_no_gearbox_or_belt_ratio():
    """It is an amps-in / amps-out filter. Any ratio here would double-count.

    n_a lives in ConversionAudit.layer_current_for and n_t in the layer's kinematics;
    a stray factor here would be a silent 9x or 4.5x torque error that still looks like
    a step response.
    """
    loop = CurrentLoopModel(0.0)
    for v in (1.0, -2.5, 7.25):
        out = loop.update(v, 5e-4)
        assert out == v
        for ratio, name in ((PAPER.n_a, "n_a"), (CAD_BELT_RATIO, "n_t"),
                            (PAPER.n_a * CAD_BELT_RATIO, "n_a*n_t")):
            assert abs(out - v * ratio) > 1e-9, f"the loop applied {name}"
    # settled value under lag is the target, with no scaling
    lagged = CurrentLoopModel(1e-3)
    for _ in range(200):
        lagged.update(3.0, 5e-4)
    assert abs(lagged.i_actual - 3.0) < 1e-9, "the settled current must equal the target"


def test_invalid_time_constants_fail_clearly():
    """Negative, NaN and infinite tau must raise with an explanation, not misbehave."""
    for bad in (-1e-6, -1.0, float("nan"), float("inf"), float("-inf")):
        try:
            CurrentLoopModel(bad)
        except ValueError as exc:
            assert "time constant" in str(exc), f"unhelpful message for {bad!r}: {exc}"
        else:
            raise AssertionError(f"CurrentLoopModel({bad!r}) was accepted")
    loop = CurrentLoopModel(1e-3)
    for bad_dt in (0.0, -5e-4, float("nan")):
        try:
            loop.update(1.0, bad_dt)
        except ValueError:
            pass
        else:
            raise AssertionError(f"dt={bad_dt!r} was accepted")
    try:
        loop.update(float("nan"), 5e-4)
    except ValueError:
        pass
    else:
        raise AssertionError("a non-finite target was accepted")


def test_the_loop_evolves_per_physics_step_and_is_deterministic():
    """It must keep responding between controller updates, and repeat exactly."""
    runs = []
    for _ in range(2):
        loop = CurrentLoopModel(2e-3, i0=0.0)
        trace = []
        for update in range(20):            # 1 kHz controller
            target = -3.75 if update >= 2 else 0.0
            for _ in range(2):              # two 0.5 ms physics steps, command HELD
                trace.append(loop.update(target, 5e-4))
        runs.append(trace)
    assert runs[0] == runs[1], "the loop is not deterministic"
    # within one held command the current must still be moving -- that is the point
    a, b = runs[0][6], runs[0][7]
    assert a != b, ("the current did not change between the two physics steps of one "
                    "control update; the loop is being advanced per control update, "
                    "not per physics step")


def test_the_filtered_current_drives_the_drivetrain_not_the_raw_command():
    """THE one that matters: plotting a filtered current while driving with the raw one.

    Mutation-checked below by comparing a lagged run against an ideal one: if the layer
    were fed the command, the two would produce identical plant motion.
    """
    bench = _bench()
    audit = _audit()
    out = {}
    for tau in (0.0, 5e-3):
        b = _bench()
        sim = AnkleCurrentSim(b, _audit(), current_loop=CurrentLoopModel(tau))
        sim.reset(0.0)
        for _ in range(200):
            sim.step(-3.75)
        out[tau] = (sim.theta_j, sim.last_layer.tau_j,
                    sim.last_current["actual_A"], sim.last_current["layer_rotor_A"])
    assert out[0.0][0] != out[5e-3][0], (
        "a 5 ms current-loop lag produced IDENTICAL joint motion to the ideal loop; "
        "the drivetrain is being driven by the raw command, not the filtered current")
    assert out[0.0][1] != out[5e-3][1], "joint torque was unaffected by the lag"
    # and the layer current is the rescale of ACTUAL, not of the command
    a = _audit()
    for tau in (0.0, 5e-3):
        b = _bench()
        sim = AnkleCurrentSim(b, a, current_loop=CurrentLoopModel(tau))
        sim.reset(0.0)
        for _ in range(50):
            sim.step(-3.75)
        want = a.layer_current_for(sim.last_current["actual_A"])
        assert abs(sim.last_current["layer_rotor_A"] - want) < 1e-12, (
            "layer_rotor_A is not the rescale of the MODELLED ACTUAL current")


def test_the_sign_is_applied_once_and_not_again_inside_the_loop():
    """The loop receives an already-signed, already-clipped command and must not re-sign."""
    loop = CurrentLoopModel(0.0)
    assert loop.update(-3.75, 5e-4) == -3.75, "the loop flipped or clipped the sign"
    assert loop.update(+3.75, 5e-4) == +3.75
    # a value beyond the clip passes through untouched: clipping is the PID's job and
    # doing it twice here would silently change the saturation metrics
    assert loop.update(999.0, 5e-4) == 999.0, (
        "the loop clipped its input; clipping belongs in CurrentPID, and applying it "
        "in both places would make the saturation duration wrong")


def test_ankle_sim_defaults_to_an_ideal_loop_for_backward_compatibility():
    """An existing caller that passes no loop must get exactly the old behaviour."""
    bench = _bench()
    sim = AnkleCurrentSim(bench, _audit())
    assert sim.current_loop.ideal is True
    assert sim.current_loop.tau_s == 0.0
    sim.reset(0.0)
    sim.step(-3.75)
    assert sim.last_current["actual_A"] == sim.last_current["target_A"], (
        "the default path is not an exact passthrough")


def test_reset_returns_the_loop_to_its_documented_initial_condition():
    """i0 = 0 A, and reset must restore it so repeated runs start identically."""
    bench = _bench()
    loop = CurrentLoopModel(5e-3, i0=0.0)
    sim = AnkleCurrentSim(bench, _audit(), current_loop=loop)
    sim.reset(0.0)
    assert loop.i_actual == 0.0 and loop.steps == 0
    for _ in range(100):
        sim.step(-3.75)
    assert loop.i_actual != 0.0, "the loop did not evolve"
    sim.reset(0.0)
    assert loop.i_actual == 0.0 and loop.steps == 0, "reset did not restore i0"


def test_zero_tau_reproduces_the_pre_change_step_bit_for_bit():
    """A/B the NEW step() against the PRE-CHANGE one on the same backend.

    The strongest available regression proof: it reconstructs the exact step() body
    this feature replaced and runs both through 600 control updates, comparing joint
    angle, joint torque, belt deflection, actuator angle and motor torque sample by
    sample. If tau_i = 0 is a true passthrough the two must agree to the bit.

    Comparing against a SAVED baseline file would not do: the saved ankle run is a real
    MuJoCo result and this test runs on whatever backend is present, so a stub-vs-real
    comparison would differ for reasons that have nothing to do with the current loop.
    (I made exactly that mistake once and briefly thought I had a regression.)
    """
    import mujoco
    def build(old):
        bench = _bench()
        sim = AnkleCurrentSim(bench, _audit(), current_loop=CurrentLoopModel(0.0))
        if old:
            def old_step(self, i_q_output_frame):
                b = self.bench
                dt = float(b.model.opt.timestep)
                theta_j = self.theta_j
                i_q_layer = self.audit.layer_current_for(i_q_output_frame)
                rec = self.layer.advance(theta_j, i_q_layer, dt)
                b.data.qfrc_applied[b.ankle_dof] = rec.tau_j_mean
                if self.hold_knee:
                    b.data.ctrl[b.knee_act] = float(b.data.ctrl[b.knee_act])
                mujoco.mj_step(b.model, b.data)
                self.last_layer = rec
                return rec
            sim.step = old_step.__get__(sim, AnkleCurrentSim)
        pid = _pid()
        enc = EncoderConvention(sign=-1)
        sim.reset(0.0)
        trace = []
        for _ in range(600):
            cmd = pid.update(5.0, enc.to_encoder_deg(sim.theta_j), 1e-3)
            for _ in range(2):
                sim.step(cmd)
                L = sim.last_layer
                trace.append((sim.theta_j, L.tau_j, L.theta_s, L.theta_a, L.tau_m))
        return trace

    old, new = build(True), build(False)
    assert len(old) == len(new) == 1200
    bad = [i for i, (a, b) in enumerate(zip(old, new)) if a != b]
    assert not bad, (
        f"tau_i = 0 is NOT a faithful passthrough: {len(bad)} of {len(old)} samples "
        f"differ, first at index {bad[0]}: {old[bad[0]]} vs {new[bad[0]]}")


def test_the_experiment_exposes_the_tau_flag_and_defaults_to_zero():
    """--current-loop-tau-ms must exist, default 0, and reject invalid values."""
    src = _src(os.path.join(ROOT, "experiments", "run_ankle_current_step.py"))
    assert '"--current-loop-tau-ms"' in src
    assert "default=0.0" in src, "the default must keep the ideal baseline"
    assert "must be finite and >= 0" in src, "invalid values must fail with a reason"
    for name in ("pid_output_pre_sign_A", "current_request_A", "current_applied_A",
                 "current_actual_modelled_A", "layer_rotor_current_A"):
        assert name in src, f"the five distinct current channels must include {name}"
    assert "current_loop_tau_ms" in src and "current_loop_model" in src, (
        "the run metadata must record the time constant and the model")


def test_the_sweep_driver_requires_real_mujoco_and_keeps_outputs_separate():
    """The sensitivity driver must not be able to fall back to the stub."""
    path = os.path.join(ROOT, "experiments", "run_ankle_current_loop_sensitivity.py")
    src = _src(path)
    assert '"--backend", "mujoco"' in src, (
        "every sweep run must demand the real engine")
    assert '"stub"' not in src.split("USAGE")[-1] or "refuses to fall back" in src
    assert "ankle_current_loop_sensitivity" in src, "outputs must be in their own tree"
    assert "manual_rotor" in src and "cad" in src, "the working hypothesis is fixed"
    assert "WORKING HYPOTHESIS" in src, "and must be labelled as one"
    for t in ("0.0", "1.0", "2.0", "5.0", "10.0", "20.0"):
        assert t in src, f"the sweep must include tau = {t} ms"
    # It must not duplicate the control law or the physics. Checked against CALLS and
    # CONSTRUCTIONS, not bare substrings: an earlier version of this test banned the
    # string "theta_s", which matched the metrics COLUMN NAME `max_abs_theta_s_rad` and
    # failed on a driver that was doing nothing wrong. A test that cannot tell a column
    # label from a computation is worse than none.
    for banned in ("AnkleCurrentSim(", "CurrentPID(", "DrivetrainLayer(",
                   "CurrentLoopModel(", "torque_unclamped(", "mj_step(",
                   "qfrc_applied[", "math.exp(-"):
        assert banned not in src, (
            f"the sweep driver re-implements part of the simulation: {banned!r}. It "
            f"must shell out to run_ankle_current_step.py instead.")
    assert "subprocess.run(" in src, (
        "the driver must invoke the existing experiment rather than import and rebuild "
        "it, so the two can never drift apart")


# =====================================================================================
def main(argv=None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    verbose = "-v" in argv or "--verbose" in argv
    pattern = argv[argv.index("-k") + 1] if "-k" in argv else None

    g = globals()
    names = [n for n in g if n.startswith("test_")]
    names.sort(key=lambda n: g[n].__code__.co_firstlineno)
    if pattern:
        names = [n for n in names if pattern in n]

    print("ankle current-step tests -- oslbench/ankle_current_control.py")
    print(f"python {sys.version.split()[0]}   SIMULATION ONLY, no hardware claim")
    print("-" * 78)
    failed, skipped = [], []
    for name in names:
        try:
            g[name]()
        except Skip as exc:
            skipped.append(name)
            print(f"SKIP  {name}: {exc}")
        except AssertionError as exc:
            failed.append(name)
            print(f"FAIL  {name}: {exc}")
        except Exception as exc:                                  # noqa: BLE001
            failed.append(name)
            print(f"ERROR {name}: {type(exc).__name__}: {exc}")
        else:
            if verbose:
                print(f"ok    {name}")
    print("-" * 78)
    print(f"{len(names) - len(failed) - len(skipped)} passed, {len(failed)} failed, "
          f"{len(skipped)} skipped, {len(names)} total")
    if failed:
        print("failed: " + ", ".join(failed))
    return len(failed)


if __name__ == "__main__":
    raise SystemExit(main())
