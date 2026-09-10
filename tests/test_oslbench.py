#!/usr/bin/env python3
"""
test_oslbench.py -- lightweight tests for the bench experiment.  Plain asserts, no framework.

WHAT THESE TESTS ARE FOR
    They check the WIRING and the ALGEBRA: that the AB19 CSV loads with the columns and
    the phase range it is supposed to have, that the control law really is
    tau = Kp*(q_ref - q) - Kd*qdot, that the ROM clip and the forcerange clamp are
    applied, that the model loads with the authored limits, that one simulation step
    does what the docstring says, that the metrics compute what their names claim, and
    that the four protected artefacts are byte-for-byte unchanged.

WHAT THESE TESTS ARE NOT FOR
    They do NOT verify the validated result.  That is a different question, it needs
    real MuJoCo, and it has its own script:

        .venv\\Scripts\\python.exe experiments\\verify_against_oracle.py

    If MuJoCo is not importable these tests fall back to tests/stub_mujoco.py, a small
    deterministic plant with the same interface.  Under the stub the physics is NOT the
    experiment's physics, so every test here is written to assert only things that are
    true of any correct plant -- an identity, a bound, a clamp, a count -- never a
    validated number.  Tests that need real MuJoCo say so and skip.

RUN IT
    python tests\\run_tests.py                       (numpy only; uses the stub)
    .venv\\Scripts\\python.exe tests\\run_tests.py   (real MuJoCo)
"""

from __future__ import annotations

import ast
import hashlib
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)
if HERE not in sys.path:
    sys.path.insert(0, HERE)

import stub_mujoco                                                        # noqa: E402

MJ = stub_mujoco.install()          # real MuJoCo if importable, else the stub
USING_STUB = stub_mujoco.is_stub(MJ)

from oslbench import metrics as mt                                        # noqa: E402
from oslbench.controller import (AUTHORED_ANKLE_KD, AUTHORED_ANKLE_KP,    # noqa: E402
                                 KD, KP, PDController, ZETA_DEFAULT,
                                 kd_for_damping_ratio)
from oslbench.model import BENCH_XML, load_bench_model                    # noqa: E402
from oslbench.reference import (DEFAULT_REFERENCE_CSV, RAW_COLUMNS,       # noqa: E402
                                audit_reference, load_reference,
                                resample_reference, synthetic_references)
from oslbench.simulation import BenchSimulation                           # noqa: E402

# ---------------------------------------------------------------------- constants
# sha256 of the two files the experiment is NOT allowed to change.  Recorded when the
# validated result in tests/oracle/ was produced.
SHA_BENCH_XML = "c417e691ff90a6f4a47ef581e6caaa341b0e1c829038f6d3e6740927ae69053d"
SHA_AB19_CSV = "83fed583b80c61c5bcd7978f2f2dd0870fe3c1efa25db68dbe9606b20da8a00d"

KNEE_ROM_RAD = (-0.0872664625997, 2.09439510239)      # [-5, 120] deg, authored
KNEE_FORCE_LIMIT = 142.2                              # N.m, authored
DT = 0.0005                                           # s, authored
N_RAW = 101                                           # AB19 samples in the CSV
N_STEPS = 2410                                        # resampled steps for one cycle
PERIOD_S = 1.2050                                     # s, AB19 stride period

_MODEL = None          # loaded once; loading is the slowest thing here


def bench():
    global _MODEL
    if _MODEL is None:
        _MODEL = load_bench_model(verbose=False)
    return _MODEL


def sha256(path: str) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 16), b""):
            h.update(chunk)
    return h.hexdigest()


class Skip(Exception):
    """Raised by a test that cannot run in this environment."""


# ============================================================ 1. the model loads
def test_model_loads_with_no_failures():
    b = bench()
    assert not b.failures, f"model preconditions failed: {b.failures}"
    assert b.model.nq == 2 and b.model.nv == 2 and b.model.nu == 2, \
        f"expected a 2-DOF bench, got nq={b.model.nq} nv={b.model.nv} nu={b.model.nu}"


def test_model_is_a_fixed_base_bench():
    """knee_prox is welded to the world.  This is what makes it a BENCH and not a leg:
    the actuator torque opposes only inertia, gravity, damping and friction."""
    b = bench()
    assert b.model.body_jntnum[1] == 0, \
        "the proximal body has a joint -- this is no longer a fixed-base bench"


def test_authored_limits_are_what_the_experiment_assumes():
    b = bench()
    lo, hi = b.knee_ctrlrange
    assert abs(lo - KNEE_ROM_RAD[0]) < 1e-12 and abs(hi - KNEE_ROM_RAD[1]) < 1e-12, \
        f"knee ctrlrange {b.knee_ctrlrange} != authored {KNEE_ROM_RAD}"
    assert abs(b.force_limit - KNEE_FORCE_LIMIT) < 1e-9, \
        f"knee force limit {b.force_limit} != authored {KNEE_FORCE_LIMIT}"
    assert abs(b.timestep - DT) < 1e-15, f"timestep {b.timestep} != {DT}"


def test_command_range_equals_joint_range():
    """The command IS an angle, so ctrlrange and jnt_range must agree -- otherwise the
    'reference clipped into the ROM' story is not what the actuator actually does."""
    b = bench()
    j = b.model.jnt_range[0]
    assert abs(j[0] - b.knee_ctrlrange[0]) < 1e-12, "jnt_range lo != ctrlrange lo"
    assert abs(j[1] - b.knee_ctrlrange[1]) < 1e-12, "jnt_range hi != ctrlrange hi"


# ====================================================== 2. the reference loads
def test_reference_loads_all_columns():
    ref = load_reference()
    assert ref.n == N_RAW, f"expected {N_RAW} samples, got {ref.n}"
    for col in RAW_COLUMNS:
        assert col in ref.raw, f"missing column {col}"
        assert len(ref.raw[col]) == N_RAW, f"column {col} has {len(ref.raw[col])} rows"


def test_reference_phase_covers_one_cycle_monotonically():
    ref = load_reference()
    p = ref.phase
    assert abs(p[0] - 0.0) < 1e-9, f"phase starts at {p[0]}, not 0"
    assert abs(p[-1] - 100.0) < 1e-9, f"phase ends at {p[-1]}, not 100"
    assert np.all(np.diff(p) > 0), "gait phase is not strictly increasing"
    assert p.min() >= 0.0 and p.max() <= 100.0, "gait phase leaves [0, 100]"


def test_reference_period_and_time_are_sane():
    ref = load_reference()
    assert abs(ref.period_s - PERIOD_S) < 1e-6, f"period {ref.period_s} != {PERIOD_S}"
    assert np.all(np.diff(ref.time_s) > 0), "time_s is not strictly increasing"


def test_reference_angle_is_inside_the_bench_rom():
    """If this fails the reference cannot be tracked without clipping, and the
    experiment's '0 clamped steps' claim would be false."""
    ref = load_reference()
    lo_deg, hi_deg = (math.degrees(x) for x in KNEE_ROM_RAD)
    a = ref.angle_deg
    assert a.min() >= lo_deg and a.max() <= hi_deg, \
        f"reference spans [{a.min():.3f}, {a.max():.3f}] deg, ROM is " \
        f"[{lo_deg:.3f}, {hi_deg:.3f}]"


def test_resample_lands_on_the_bench_timestep():
    ref = load_reference()
    res = resample_reference(ref, DT, 1, "cubic", verbose=False)
    assert res.n == N_STEPS, f"expected {N_STEPS} steps, got {res.n}"
    assert abs(res.dt - DT) < 1e-15
    t = res.time_s
    assert abs(t[0]) < 1e-15, f"resampled time starts at {t[0]}, not 0"
    assert np.allclose(np.diff(t), DT, atol=1e-12), "resampled time is not uniform"


def test_resampled_phase_stays_in_range_and_wraps_analytically():
    """phase = 100*(t mod T)/T, so it must stay in [0, 100) -- never reach 100, because
    100 % and 0 % are the same instant of the next cycle."""
    ref = load_reference()
    res = resample_reference(ref, DT, 2, "cubic", verbose=False)
    p = res.phase
    assert p.min() >= 0.0, f"phase min {p.min()} < 0"
    assert p.max() < 100.0, f"phase max {p.max()} reached 100"
    assert res.n == 2 * N_STEPS, f"two cycles should be {2 * N_STEPS} steps, got {res.n}"
    # The second cycle repeats the first: the reference is periodic by construction.  Not
    # BIT-identical, because T = 1.2049999999999983 s is not an exact binary multiple of
    # dt, so the two cycles are sampled a fraction of a ULP apart.
    a, b_ = res.ref_rad[:N_STEPS], res.ref_rad[N_STEPS:]
    d = float(np.max(np.abs(a - b_)))
    assert d < 1e-12, f"cycle 2 differs from cycle 1 by {d:.3e} rad"


def test_resample_preserves_the_sampled_values():
    """The spline interpolates: at the ORIGINAL sample times it must return the original
    values, otherwise the resampler is filtering the human data, not resampling it."""
    ref = load_reference()
    res = resample_reference(ref, DT, 1, "cubic", verbose=False)
    # sample 0 and the peak-flexion sample must survive to the nearest resampled step
    for i in (0, int(np.argmax(ref.angle_deg))):
        t_i = ref.time_s[i] - ref.time_s[0]
        k = int(round(t_i / DT))
        if k >= res.n:
            continue
        got = math.degrees(res.ref_rad[k])
        assert abs(got - ref.angle_deg[i]) < 5e-3, \
            f"sample {i}: reference {ref.angle_deg[i]:.4f} deg became {got:.4f} deg"


def test_audit_reports_and_does_not_repair():
    """The audit is a REPORT.  It must find the known defects and change nothing."""
    ref = load_reference()
    before = {k: v.copy() for k, v in ref.raw.items()}
    a = audit_reference(ref, math.degrees(KNEE_ROM_RAD[0]),
                        math.degrees(KNEE_ROM_RAD[1]), verbose=False)
    for k, v in before.items():
        assert np.array_equal(ref.raw[k], v), f"the audit MODIFIED column {k}"
    assert a.exceeded_rom is False, "reference should fit the ROM"
    assert a.n_below == 0 and a.n_above == 0
    assert bool(a.sign_ok), "flexion-positive landmark check failed"
    assert len(a.flags) == 5, \
        f"expected the 5 known moment/power defects, got {len(a.flags)}: {a.flags}"


# ================================================== 3. THE CONTROLLER (the point)
def test_pd_equation_is_exactly_kp_error_minus_kd_qdot():
    """tau = Kp*(q_ref - q) - Kd*qdot.  Checked against arithmetic done here, by hand."""
    c = PDController(600.0, 17.253, KNEE_ROM_RAD, 142.2, "test")
    for q_ref, q, qd in ((0.5, 0.4, 0.0), (1.0, 1.2, 3.0), (0.2, 0.2, -1.5),
                         (0.0, 0.0, 0.0), (1.5, 0.9, -2.25)):
        expect = 600.0 * (q_ref - q) - 17.253 * qd
        got = c.torque_unclamped(q_ref, q, qd)
        assert abs(got - expect) < 1e-12, \
            f"Kp*(ref-q)-Kd*qd: expected {expect:.9f}, got {got:.9f}"


def test_zero_error_and_zero_velocity_gives_zero_torque():
    c = PDController(600.0, 17.253, KNEE_ROM_RAD, 142.2, "test")
    assert c.torque_unclamped(0.7, 0.7, 0.0) == 0.0


def test_kd_is_derived_from_the_damping_ratio_not_tuned():
    """Kd = 2*zeta*sqrt(Kp*I_eff) - b_joint, and the shipped KD is that value at Kp=600."""
    i_eff, b_joint = 0.261998, 0.3
    got = kd_for_damping_ratio(600.0, i_eff, b_joint, ZETA_DEFAULT)
    expect = 2 * ZETA_DEFAULT * math.sqrt(600.0 * i_eff) - b_joint
    assert abs(got - expect) < 1e-12, f"formula mismatch: {got} vs {expect}"
    assert abs(got - KD) < 5e-4, \
        f"the shipped KD={KD} is not kd_for_damping_ratio(600) = {got:.6f}"
    # and the resulting closed loop really is at zeta = 0.7
    c = PDController(600.0, got, KNEE_ROM_RAD, 142.2, "test")
    assert abs(c.damping_ratio(i_eff, b_joint) - ZETA_DEFAULT) < 1e-12


def test_gains_are_the_validated_pair():
    assert KP == 600.0, f"KP has been changed to {KP}"
    assert abs(KD - 17.253) < 1e-12, f"KD has been changed to {KD}"


def test_command_is_clipped_into_the_range_of_motion():
    lo, hi = KNEE_ROM_RAD
    c = PDController(600.0, 17.253, KNEE_ROM_RAD, 142.2, "test")
    assert c.command(lo - 1.0) == lo, "below-ROM reference was not clipped"
    assert c.command(hi + 1.0) == hi, "above-ROM reference was not clipped"
    assert c.command(0.5) == 0.5, "in-ROM reference was altered"
    assert c.command_clamped(hi + 1.0) is True
    assert c.command_clamped(0.5) is False


def test_torque_saturates_at_the_authored_forcerange():
    """Two DIFFERENT limits act here.  This is the TORQUE clamp: whatever the law asks
    for, the actuator delivers at most +/-142.2 N.m."""
    c = PDController(600.0, 17.253, KNEE_ROM_RAD, 142.2, "test")
    big = c.torque(2.0, 0.0, 0.0)                  # asks for 1200 N.m
    assert abs(big - 142.2) < 1e-12, f"positive clamp gave {big}"
    small = c.torque(0.0, 2.0, 0.0)                # asks for -1200 N.m
    assert abs(small + 142.2) < 1e-12, f"negative clamp gave {small}"
    assert c.torque_saturated(2.0, 0.0, 0.0) is True
    assert c.torque_saturated(0.01, 0.0, 0.0) is False
    # the clamp must not touch a request that fits
    inside = c.torque_unclamped(0.01, 0.0, 0.0)
    assert c.torque(0.01, 0.0, 0.0) == inside


def test_saturation_flag_and_clamped_value_agree():
    c = PDController(600.0, 17.253, KNEE_ROM_RAD, 142.2, "test")
    for q_ref, q, qd in ((2.0, 0.0, 0.0), (0.0, 2.0, 0.0), (0.05, 0.0, 0.0),
                         (0.0, 0.0, 50.0)):
        req, out = c.torque_unclamped(q_ref, q, qd), c.torque(q_ref, q, qd)
        if c.torque_saturated(q_ref, q, qd):
            assert abs(abs(out) - 142.2) < 1e-9, "flagged saturated but not clamped"
        else:
            assert out == req, "not flagged, but the value was changed"


def test_gains_reach_the_compiled_model_in_mujoco_form():
    """MuJoCo computes gainprm[0]*ctrl + biasprm[0] + biasprm[1]*q + biasprm[2]*qdot.
    For that to equal Kp*(ctrl - q) - Kd*qdot the four numbers must be Kp, 0, -Kp, -Kd."""
    b = bench()
    m = b.model
    a = b.knee_act
    before = (m.actuator_forcerange[a].copy(), m.actuator_ctrlrange[a].copy())
    PDController(123.0, 4.5, b.knee_ctrlrange, b.force_limit).write_to_model(m, a)
    assert m.actuator_gainprm[a, 0] == 123.0, "Kp did not reach gainprm[0]"
    assert m.actuator_biasprm[a, 0] == 0.0, "a constant bias was introduced"
    assert m.actuator_biasprm[a, 1] == -123.0, "-Kp did not reach biasprm[1]"
    assert m.actuator_biasprm[a, 2] == -4.5, "-Kd did not reach biasprm[2]"
    assert np.array_equal(m.actuator_forcerange[a], before[0]), "forcerange was written"
    assert np.array_equal(m.actuator_ctrlrange[a], before[1]), "ctrlrange was written"
    PDController.knee(b).write_to_model(m, a)       # restore the validated pair


def test_the_ankle_keeps_its_authored_servo():
    """The ankle is a fixed boundary condition, not a second variable under study."""
    b = bench()
    a = PDController.ankle_hold(b)
    assert a.kp == AUTHORED_ANKLE_KP and a.kd == AUTHORED_ANKLE_KD, \
        f"the ankle servo was changed to kp={a.kp} kv={a.kd}"


# ============================================ 4. one simulation step, and the loop
def test_reset_seeds_the_knee_on_the_reference():
    """The run must not begin with an artificial position error."""
    b = bench()
    sim = BenchSimulation(b, PDController.knee(b))
    q0 = 0.2217
    ankle_hold = sim.reset(q0)
    assert abs(float(b.data.qpos[b.knee_qpos]) - q0) < 1e-12, "knee was not seeded"
    assert abs(float(b.data.qvel[b.knee_qpos])) < 1e-12, "qvel was not zeroed"
    assert abs(float(b.data.qpos[b.ankle_qpos]) - ankle_hold) < 1e-12
    assert abs(float(b.data.time)) < 1e-15, "the clock was not reset"


def test_one_step_advances_exactly_one_timestep():
    b = bench()
    sim = BenchSimulation(b, PDController.knee(b))
    sim.reset(0.2)
    t0 = float(b.data.time)
    st = sim.step(0.2, 0)
    assert abs(st.time_s - (t0 + DT)) < 1e-12, \
        f"one step moved the clock by {st.time_s - t0}, not {DT}"
    assert st.k == 0


def test_one_step_reports_the_command_it_actually_used():
    b = bench()
    sim = BenchSimulation(b, PDController.knee(b))
    sim.reset(0.2)
    hi = b.knee_ctrlrange[1]
    st = sim.step(hi + 1.0, 3)                 # deliberately out of ROM
    assert abs(st.command - hi) < 1e-12, "the step did not clip the command"
    assert st.command_clamped is True, "the clip was not reported"
    assert st.q_ref == hi + 1.0, "q_ref must stay the UNCLIPPED reference"


def test_the_step_torque_is_the_controllers_torque():
    """The identity that makes controller.py the single source of truth: evaluated on the
    PRE-step state, MuJoCo's actuator force equals the PD law's clamped output."""
    b = bench()
    knee = PDController.knee(b)
    sim = BenchSimulation(b, knee)
    sim.reset(0.30)
    q_pre = float(b.data.qpos[b.knee_qpos])
    qd_pre = float(b.data.qvel[b.knee_qpos])
    q_ref = 0.45
    expect = knee.torque(q_ref, q_pre, qd_pre)
    st = sim.step(q_ref, 0)
    assert abs(st.tau_actuator - expect) < 1e-7, \
        f"actuator force {st.tau_actuator:.9f} != PD law {expect:.9f} on the pre-step state"


def test_error_and_power_are_defined_as_documented():
    b = bench()
    sim = BenchSimulation(b, PDController.knee(b))
    sim.reset(0.30)
    st = sim.step(0.45, 0)
    assert abs(st.error_rad - (st.q - st.q_ref)) < 1e-15, "error_rad is not q - q_ref"
    assert abs(st.error_deg - math.degrees(st.q - st.q_ref)) < 1e-12
    assert abs(st.power_W - st.tau * st.qdot) < 1e-12, "power is not tau * qdot"


def test_a_short_run_is_deterministic():
    b = bench()
    sim = BenchSimulation(b, PDController.knee(b))
    ref = np.full(200, 0.4)
    a = sim.run(ref).q.copy()
    c = sim.run(ref).q.copy()
    assert np.array_equal(a, c), "two identical runs differ -- the run is not deterministic"


def test_a_constant_reference_reaches_static_equilibrium():
    """Hold a constant angle and check the STATICS, which any correct plant must satisfy.

    At rest the actuator torque has to balance gravity, and whatever is left over can
    only be carried by joint friction:

        Kp*(target - q) - m*g*d*sin(q)  =  the friction reaction,  |.| <= frictionloss

    That is a two-sided bound, so it is a real check: it fails if the controller does not
    hold against gravity, and it also fails if the joint is stiffer or looser than the
    authored friction allows.  (A naive 'settles at the target' test would be wrong --
    gravity guarantees a standing offset of about 0.45 deg at this angle.)
    """
    b = bench()
    knee = PDController.knee(b)
    sim = BenchSimulation(b, knee)
    target = 0.6                                     # rad, ~34 deg: gravity is real here
    res = sim.run(np.full(6000, target))             # 3 s, long enough to stop moving
    q_end, qd_end = float(res.q[-1]), float(res.qdot[-1])

    assert abs(qd_end) < 0.02, f"still moving at {qd_end:.4f} rad/s after 3 s"
    residual = knee.kp * (target - q_end) - b.mgd * math.sin(q_end)
    assert abs(residual) <= b.frictionloss + 0.05, (
        f"static balance broken: Kp*err - m*g*d*sin(q) = {residual:.4f} N.m, "
        f"which friction ({b.frictionloss} N.m) cannot supply")
    assert 0.0 < target - q_end < 0.02, \
        f"expected a small gravity-induced offset below the target, got {target - q_end:.5f} rad"
    assert not res.saturated.any(), "a 0.6 rad hold should not saturate 142.2 N.m"


def test_the_run_records_one_row_per_step():
    b = bench()
    sim = BenchSimulation(b, PDController.knee(b))
    res = sim.run(np.full(50, 0.3))
    assert res.n == 50
    for name in ("time_s", "q", "qdot", "tau", "tau_unclamped", "ctrl", "saturated"):
        assert len(getattr(res, name)) == 50, f"{name} has {len(getattr(res, name))} rows"
    assert res.clamped_steps == 0, "an in-ROM reference should never be clipped"


def test_out_of_rom_reference_is_counted_as_clamped():
    b = bench()
    sim = BenchSimulation(b, PDController.knee(b))
    res = sim.run(np.full(20, b.knee_ctrlrange[1] + 0.5))
    assert res.clamped_steps == 20, \
        f"20 out-of-ROM commands should all be clipped, got {res.clamped_steps}"


def test_the_model_limits_survive_a_run():
    b = bench()
    BenchSimulation(b, PDController.knee(b)).run(np.full(100, 0.3))
    assert b.limits_unchanged(), "forcerange or ctrlrange changed during the run"


# ================================================================= 5. the metrics
def test_rms_mae_and_peak_on_a_known_vector():
    x = np.array([3.0, -4.0, 0.0])
    assert abs(mt.rms(x) - 5.0 / math.sqrt(3.0)) < 1e-12
    assert abs(mt.mean_abs(x) - 7.0 / 3.0) < 1e-12
    assert abs(mt.peak_abs(x) - 4.0) < 1e-12
    assert mt.rms(np.zeros(10)) == 0.0
    assert mt.peak_abs(np.zeros(10)) == 0.0


def test_percent_authority_and_saturation_percent():
    assert abs(mt.percent_authority(71.1, 142.2) - 50.0) < 1e-12
    assert abs(mt.percent_authority(142.2, 142.2) - 100.0) < 1e-12
    assert abs(mt.percent_authority(16.0041205874, 142.2) - 11.2546558280) < 1e-9
    assert abs(mt.saturation_percent(np.array([True, False, False, False])) - 25.0) < 1e-12
    assert mt.saturation_percent(np.zeros(8, bool)) == 0.0


def test_tracking_metrics_names_mean_what_they_say():
    err = np.array([1.0, -2.0, 3.0])
    tau = np.array([10.0, -20.0, 5.0])
    qdot = np.array([0.5, -1.5, 1.0])
    m = mt.tracking_metrics(err, tau, qdot, 142.2, np.zeros(3, bool))
    assert abs(m["rms_err_deg"] - mt.rms(err)) < 1e-12
    assert abs(m["peak_err_deg"] - 3.0) < 1e-12
    assert abs(m["mae_err_deg"] - 2.0) < 1e-12
    assert abs(m["peak_tau_Nm"] - 20.0) < 1e-12
    assert abs(m["peak_vel_rad_s"] - 1.5) < 1e-12
    assert abs(m["pct_authority"] - 20.0 / 142.2 * 100.0) < 1e-12
    assert m["sat_pct"] == 0.0


def test_perfect_tracking_has_zero_error_metrics():
    m = mt.tracking_metrics(np.zeros(5), np.zeros(5), np.zeros(5), 142.2,
                            np.zeros(5, bool))
    for k in ("rms_err_deg", "peak_err_deg", "mae_err_deg", "peak_tau_Nm", "sat_pct"):
        assert m[k] == 0.0, f"{k} should be 0 for perfect tracking, got {m[k]}"


def test_lag_diagnostic_finds_a_shift_it_was_given():
    """Feed it a reference and the SAME reference delayed by a known number of steps.
    It must recover that delay and attribute nearly all the error to it."""
    n, shift = 2000, 40                              # 40 steps = 20 ms
    t = np.arange(n) * DT
    ref = 0.5 + 0.4 * np.sin(2 * math.pi * t / 1.0)
    q = np.empty(n)
    q[:shift] = ref[0]
    q[shift:] = ref[:n - shift]
    rms_err = mt.rms(np.degrees(q - ref))
    d = mt.lag_diagnostic(q, ref, DT, rms_err)       # max_lag_s defaults to 0.15 s
    assert abs(d["lag_ms"] - shift * DT * 1e3) < 0.6, \
        f"a {shift * DT * 1e3:.1f} ms shift was measured as {d['lag_ms']:.1f} ms"
    assert d["pct_err_from_lag"] > 90.0, \
        f"a pure time shift should be ~100 % lag, got {d['pct_err_from_lag']:.1f} %"


def test_bench_metrics_agrees_with_its_own_inputs():
    """Whatever plant produced the run, the reported metrics must be recomputable from
    the logged arrays.  This is the check that the metrics describe THIS run."""
    b = bench()
    ref = load_reference()
    res = resample_reference(ref, DT, 1, "cubic", verbose=False)
    sim = BenchSimulation(b, PDController.knee(b))
    out = sim.run(res.ref_rad, res.n, ref_vel0=float(res.ref_vel_rad_s[0]))
    M = mt.bench_metrics(out, res, DT)
    err = np.degrees(out.q - out.ref_rad)
    assert abs(M["rms_err_deg"] - mt.rms(err)) < 1e-12
    assert abs(M["peak_err_deg"] - float(np.max(np.abs(err)))) < 1e-12
    assert abs(M["peak_tau_Nm"] - float(np.max(np.abs(out.tau)))) < 1e-12
    assert abs(M["peak_vel_rad_s"] - float(np.max(np.abs(out.qdot)))) < 1e-12
    assert M["n_steps"] == N_STEPS
    assert abs(M["final_time_s"] - N_STEPS * DT) < 1e-9
    assert M["ref_clamped_steps"] == 0, "the AB19 reference must need no ROM clipping"
    assert M["sat_pct"] == 0.0, "the AB19 run must not saturate 142.2 N.m"


def test_sweep_metrics_and_step_shape_run_on_a_synthetic_reference():
    b = bench()
    refs = synthetic_references(DT, b.knee_ctrlrange[0], b.knee_ctrlrange[1])
    assert "step_45deg" in refs and "sine_gait" in refs, f"missing refs: {sorted(refs)}"
    t, r, _desc, astart = refs["step_45deg"]
    sim = BenchSimulation(b, PDController.knee(b))
    out = sim.run(r)
    m = mt.sweep_metrics(out, DT, astart)
    for k in ("rms_err", "peak_err", "ss_err", "peak_tau", "pct_auth", "sat_pct"):
        assert k in m, f"sweep metric {k} missing"
        assert np.isfinite(m[k]), f"sweep metric {k} is not finite"
    over, settle = mt.step_shape(t, r, out.q, DT)
    assert np.isfinite(over), "overshoot should be finite for a step"
    assert over >= 0.0, f"overshoot cannot be negative, got {over}"


# ============================================= 6. the protected artefacts, unchanged
def test_the_bench_xml_is_byte_for_byte_unchanged():
    got = sha256(BENCH_XML)
    assert got == SHA_BENCH_XML, (
        f"models/osl_v2_bench.xml CHANGED\n  expected sha256 {SHA_BENCH_XML}\n"
        f"  found          {got}\n  the refactor is not allowed to touch the model")


def test_the_ab19_reference_csv_is_byte_for_byte_unchanged():
    got = sha256(DEFAULT_REFERENCE_CSV)
    assert got == SHA_AB19_CSV, (
        f"the AB19 reference CSV CHANGED\n  expected sha256 {SHA_AB19_CSV}\n"
        f"  found          {got}\n  the human data is an input, not something we tune")


def test_the_frozen_oracle_is_present_and_the_right_size():
    """The old pathway's validated output.  verify_against_oracle.py compares against it,
    so if it goes missing the equivalence claim silently loses its evidence."""
    p = os.path.join(HERE, "oracle", "bench_track_ab19.csv")
    m = os.path.join(HERE, "oracle", "bench_track_ab19_metrics.csv")
    assert os.path.isfile(p), f"missing frozen trace {p}"
    assert os.path.isfile(m), f"missing frozen metrics {m}"
    with open(p, "r", encoding="utf-8") as f:
        rows = sum(1 for _ in f)
    assert rows == N_STEPS + 4, \
        f"the frozen trace has {rows} lines, expected {N_STEPS + 4} " \
        f"(3 provenance + header + {N_STEPS} steps)"


def test_no_bench_code_reaches_into_myoassist_or_ka_l1():
    """The bench experiment is self-contained.  Those projects live in a different
    repository and are explicitly out of scope for this experiment.

    The check is on CODE, not on prose: it walks the AST and looks at imported module
    names and at string literals that are not docstrings.  A comment or a docstring that
    mentions the other project is fine -- saying 'this is not that' is useful.  An
    `import`, or a path string pointing at it, is not.
    """
    banned = ("myoassist", "ka_l1", "myosuite", "myo_sim", "myo_folder")
    checked, hits = 0, []
    for d in ("oslbench", "experiments", "tests"):
        for dirpath, dirnames, filenames in os.walk(os.path.join(ROOT, d)):
            dirnames[:] = [x for x in dirnames if x != "__pycache__"]
            for fn in sorted(filenames):
                if not fn.endswith(".py"):
                    continue
                path = os.path.join(dirpath, fn)
                if os.path.abspath(path) == os.path.abspath(__file__):
                    continue        # this file OWNS the word list; scanning it self-fires
                checked += 1
                rel = os.path.relpath(path, ROOT)
                src = open(path, "r", encoding="utf-8", errors="replace").read()
                try:
                    tree = ast.parse(src, filename=path)
                except SyntaxError as e:
                    hits.append(f"{rel} does not parse: {e}")
                    continue
                docstrings = {id(n.body[0].value) for n in ast.walk(tree)
                              if isinstance(n, (ast.Module, ast.ClassDef,
                                                ast.FunctionDef, ast.AsyncFunctionDef))
                              and n.body and isinstance(n.body[0], ast.Expr)
                              and isinstance(n.body[0].value, ast.Constant)
                              and isinstance(n.body[0].value.value, str)}
                for node in ast.walk(tree):
                    mods = []
                    if isinstance(node, ast.Import):
                        mods = [a.name for a in node.names]
                    elif isinstance(node, ast.ImportFrom):
                        mods = [node.module or ""]
                    for m in mods:
                        for w in banned:
                            if w in m.lower():
                                hits.append(f"{rel}:{node.lineno} imports '{m}'")
                    if (isinstance(node, ast.Constant) and isinstance(node.value, str)
                            and id(node) not in docstrings):
                        low = node.value.lower()
                        for w in banned:
                            if w in low:
                                hits.append(f"{rel}:{node.lineno} string literal "
                                            f"contains '{w}'")
    assert checked > 10, f"only {checked} python files scanned -- the walk found nothing"
    assert not hits, "bench code reaches into out-of-scope projects:\n  " + \
                     "\n  ".join(hits)


def test_plotting_and_dashboard_carry_no_physics():
    """Structural separation, asserted rather than promised: the offline plotter and the
    live second window must not import MuJoCo or the simulation.  This is what makes
    'the dashboard is a consumer, not a second controller' checkable."""
    for mod, why in (("plotting.py", "offline figures must run without MuJoCo"),
                     ("dashboard.py", "the second window must not hold a simulation")):
        src = open(os.path.join(ROOT, "oslbench", mod), "r", encoding="utf-8").read()
        for bad in ("import mujoco", "from .simulation", "import simulation"):
            for line in src.splitlines():
                s = line.strip()
                if s.startswith(bad):
                    raise AssertionError(f"oslbench/{mod}: '{s}' -- {why}")


def test_the_stub_is_not_pretending_to_be_the_experiment():
    """If these tests ran on the stub, say so loudly.  A green test run under the stub is
    NOT evidence for the validated numbers."""
    if USING_STUB:
        raise Skip("MuJoCo not importable: ran on tests/stub_mujoco.py, so the physics "
                   "here is a stand-in. Run experiments/verify_against_oracle.py in "
                   ".venv for the validated result.")
