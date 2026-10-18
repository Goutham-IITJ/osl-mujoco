#!/usr/bin/env python3
"""
run_ankle_current_step.py -- the PHYSICAL ankle's current-controlled step, simulated.

    SIMULATION ONLY -- NOT A VALIDATED MODEL OF THE PHYSICAL ANKLE
    Every actuator and belt parameter is paper-derived or reported from a script that
    is not in this workspace.  No quantity here has been measured on OSL V2 hardware.

WHAT THIS IS, AND WHAT IT IS NOT A SUBSTITUTE FOR
    `experiments/run_step_response.py` steps the KNEE through MuJoCo's position actuator
    with a joint-torque-domain PD law in radians (Kp = 600 N.m/rad).  The physical ankle
    rig runs a different architecture: an outer loop reads an external encoder in
    DEGREES, outputs a CURRENT in amps (Kp = 0.75 A/deg), and the driver's current loop
    closes inside the motor.  Those gains are not comparable and neither experiment
    replaces the other.  This file is the second one, on the ankle, and it changes
    nothing about the first.

THE FOUR THINGS THIS EXPERIMENT IS FOR
    1. Make the current -> torque -> joint chain explicit, with the reference frame named
       at every stage, and show each ratio applied exactly once (`--explain`).
    2. Reproduce the hardware controller's UNITS and TIMING: degrees in, amps out,
       1 kHz outer loop against a 0.5 ms physics step, command held between updates.
    3. Drive the REAL ankle joint of the CAD bench, with the ankle position servo
       disconnected so the belt is the only actuation on that joint.
    4. Record enough telemetry to explain a disagreement with hardware, with commanded
       and applied current kept distinct and with actuator-side and joint-side torque
       never conflated.

WHAT IS ASSUMED, PROMINENTLY
    * IDEAL CURRENT LOOP.  The applied motor current equals the clipped command within
      each control interval.  The driver's actual current response is NOT modelled: no
      winding electrical model and no first-order lag, because no measured time constant
      is available and inventing one would make the simulation look more credible
      without being more correct.
    * The torque constant's reference frame is UNRESOLVED.  Three candidates disagree by
      up to 1.75x; `--kt` selects one and the default merely reproduces the script.
    * The belt is the paper's COMPLIANT model with the CAD's 50/11 ratio -- a labelled
      hybrid.  The MJCF itself has no transmission at all: its pulleys and belt are
      visual geoms, so without this layer the joint is directly actuated.

USAGE (Windows PowerShell)
    .venv\\Scripts\\python.exe experiments\\run_ankle_current_step.py --explain
    .venv\\Scripts\\python.exe experiments\\run_ankle_current_step.py
    .venv\\Scripts\\python.exe experiments\\run_ankle_current_step.py --kt paper
    .venv\\Scripts\\python.exe experiments\\run_ankle_current_step.py --belt paper
    .venv\\Scripts\\python.exe experiments\\run_ankle_current_step.py --step-deg 20 --outdir build\\ankle_current_step_20deg
    .venv\\Scripts\\python.exe experiments\\run_ankle_current_step.py --ki 0.5

OUTPUT
    build/ankle_current_step/ -- a directory of its own, so nothing in
    build/step_response/ or build/bench_track_ab19/ is touched.
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

# ---- the backend has to be settled BEFORE the oslbench imports below -----------------
# `oslbench/model.py` does `try: import mujoco / except ImportError: mujoco = None` at
# ITS import time and keeps that binding. So installing a stub after importing it is too
# late -- the module has already decided mujoco is absent. argparse has not run yet at
# this point, hence the deliberate argv peek. It reads one flag and nothing else.
_WANT_STUB = "stub" in sys.argv[1:] and "--backend" in sys.argv[1:]
if _WANT_STUB:
    sys.path.insert(0, os.path.join(ROOT, "tests"))
    import stub_mujoco as _stub
    _stub.install()

from oslbench.control.current import ANKLE_PAPER_CAD_BELT, CAD_BELT_RATIO, CAD_BELT_TEETH, ConversionAudit, CurrentLimits, CurrentPID, EncoderConvention, PIDGains, TORQUE_CONSTANT_CHOICES, CurrentLoopModel, check_sign_consistency, torque_constant
from oslbench.sim.ankle import AnkleCurrentSim, derive_mjcf_q_to_encoder_sign
from oslbench.control.drivetrain import PAPER
from oslbench.sim.model import load_bench_model


# =====================================================================================
# THE BACKEND GATE.  This experiment is worthless on a test double; see the docstring.
# =====================================================================================
def require_backend(choice: str):
    """Return (mujoco_module, label, is_stub) or raise SystemExit with an explanation.

    WHY THIS GATE EXISTS, in one paragraph, because it cost a whole review cycle.
    `tests/stub_mujoco.StubModel.__init__(path)` STORES the MJCF path and never parses
    it -- it hardcodes its own ranges and inertias from module constants. So a stub run
    reports `bench.path = models/osl_v2_bench.xml`, exposes `bench.ankle_qpos` and
    `bench.ankle_dof`, and produces plots whose every axis label is correct, while no
    line of the CAD model was ever compiled. Worse, the stub's ankle DISAGREES with the
    MJCF on three properties that dominate a step response:

        property        stub     real MJCF    factor
        armature        0.0      0.008        infinite
        damping         0.05     0.25         5x
        frictionloss    0.05     0.3          6x

    A stub step response is therefore not a less accurate version of the real one; it is
    a different plant. The default REFUSES to run without real MuJoCo rather than
    producing a number that looks like a result.
    """
    if choice == "stub":
        mod = sys.modules.get("mujoco")
        if mod is None:                         # --backend stub given after the peek
            sys.path.insert(0, os.path.join(ROOT, "tests"))
            import stub_mujoco
            mod = stub_mujoco.install()
        is_stub = bool(getattr(mod, "__stub__", False))
        return mod, ("REAL MuJoCo (stub requested, but the real engine was importable "
                     "and it wins)" if not is_stub
                     else "tests/stub_mujoco.py"), is_stub
    try:
        import mujoco
    except ImportError as exc:
        raise SystemExit(
            "\nREFUSING TO RUN: real MuJoCo is not importable in this interpreter "
            f"({exc}).\n"
            "\n  This experiment requires the compiled CAD model. The test stub is NOT a\n"
            "  substitute: it never parses models/osl_v2_bench.xml, and its ankle has\n"
            "  armature 0.0 (vs 0.008), damping 0.05 (vs 0.25) and frictionloss 0.05\n"
            "  (vs 0.3) -- a different plant, not a rougher one.\n"
            "\n  Use the project's virtual environment:\n"
            "    .venv\\Scripts\\python.exe experiments\\run_ankle_current_step.py\n"
            "\n  To exercise the wiring without the engine (unit-test use only, results\n"
            "  are written to a separate directory and marked STUB -- NOT MUJOCO):\n"
            "    python experiments\\run_ankle_current_step.py --backend stub\n")
    return mujoco, "REAL MuJoCo", False


def model_identity(bench, mj, is_stub: bool) -> dict:
    """Facts about the plant that ACTUALLY got loaded, read back from the compiled model.

    Deliberately read from `bench.model`, not from the XML text or from our own
    constants: the point is to record what the engine built, so a stub cannot
    impersonate it.
    """
    m = bench.model
    ident = dict(
        backend="STUB -- NOT MUJOCO" if is_stub else "real MuJoCo",
        mujoco_version=("n/a (stub)" if is_stub
                        else getattr(mj, "__version__", "unknown")),
        model_path=os.path.relpath(bench.path, ROOT).replace("\\", "/"),
        model_xml_sha256="unavailable",
        nq=int(getattr(m, "nq", -1)), nv=int(getattr(m, "nv", -1)),
        nu=int(getattr(m, "nu", -1)),
        nbody=int(getattr(m, "nbody", -1)),
        ngeom=int(getattr(m, "ngeom", -1)),
        nmesh=int(getattr(m, "nmesh", -1)),
        timestep_s=float(m.opt.timestep),
        integrator=int(getattr(m.opt, "integrator", -1)),
        ankle_qpos_adr=int(bench.ankle_qpos), ankle_dof_adr=int(bench.ankle_dof),
        ankle_armature=float(m.dof_armature[bench.ankle_dof]),
        ankle_damping=float(m.dof_damping[bench.ankle_dof]),
        ankle_frictionloss=float(m.dof_frictionloss[bench.ankle_dof]),
    )
    try:
        import hashlib
        with open(bench.path, "rb") as fh:
            ident["model_xml_sha256"] = hashlib.sha256(fh.read()).hexdigest()[:16]
    except Exception:                                           # noqa: BLE001
        pass
    # The MJCF-authored ankle properties. A mismatch means the plant is not the CAD
    # model, whatever the path says.
    ident["ankle_properties_match_mjcf"] = (
        abs(ident["ankle_armature"] - 0.008) < 1e-12
        and abs(ident["ankle_damping"] - 0.25) < 1e-12
        and abs(ident["ankle_frictionloss"] - 0.3) < 1e-12)
    return ident

OUTDIR = os.path.join(ROOT, "build", "ankle_current_step")
BANNER = "PHYSICAL-ARCHITECTURE ANKLE STEP -- CURRENT CONTROL THROUGH A COMPLIANT BELT"
DISCLAIMER = "SIMULATION ONLY -- NOT A VALIDATED MODEL OF THE PHYSICAL ANKLE"

# Reported from the hardware script.  Defaults reproduce the original 5 degree
# experiment; --step-deg can run another reference without changing that baseline.
STEP_DEG = 5.0
BASELINE_DEG = 0.0
RECORD_S = 4.0
CONTROL_HZ = 1000.0
SETTLING_FRACTION = 0.02        # same band the knee step experiment uses
FIT_EDGE_RAD = 0.055            # ASSUMED: Best et al. Fig. 3 abscissa
# REPORTED range of the physical controller, in the ENCODER frame. Second-hand: the
# script is not in this workspace. Carried so the mismatch in `limit_report` is explicit
# rather than discovered during a wider sweep.
PHYSICAL_ENCODER_RANGE_DEG = (-29.0, 29.0)


def limit_report(bench, enc) -> list[str]:
    """The CAD ROM expressed in the ENCODER frame, against the physical controller's.

    Two frames, one joint, and they are not the same numbers. The MJCF authors the ankle
    in MuJoCo generalized coordinates; the physical controller thinks in encoder degrees.
    With encoder sign s the mapping is enc = s*deg(q), so a NEGATIVE s SWAPS the ends of
    the interval -- which is exactly the kind of thing that silently turns a
    "within range" check into a wrong one.
    """
    lo_q, hi_q = (math.degrees(x) for x in bench.ankle_range)
    lo_e, hi_e = enc.rom_in_encoder_frame(bench.ankle_range)
    plo, phi = PHYSICAL_ENCODER_RANGE_DEG
    out = [
        f"CAD ROM, MuJoCo frame    [{lo_q:+.1f}, {hi_q:+.1f}] deg   CONFIRMED from the MJCF",
        f"CAD ROM, ENCODER frame   [{lo_e:+.1f}, {hi_e:+.1f}] deg   "
        f"(mapping 1 = {enc.sign:+d} applied; the interval ENDS SWAP for a negative "
        f"sign, so {'+30 = plantarflexion, -20 = dorsiflexion' if enc.sign < 0 else 'the ends keep their order'})",
        f"physical ctrl, ENCODER   [{plo:+.1f}, {phi:+.1f}] deg   REPORTED, second-hand",
    ]
    short_lo, short_hi = lo_e - plo, phi - hi_e
    if short_lo > 0.05 or short_hi > 0.05:
        out.append(f"MISMATCH: the physical controller's encoder range extends "
                   f"{max(short_lo, 0.0):+.1f} deg beyond the CAD ROM at the low end and "
                   f"{max(short_hi, 0.0):+.1f} deg at the high end.")
        out.append("  So a command the physical rig would accept can be OUTSIDE this "
                   "model's authored travel. The MJCF is protected and is NOT widened "
                   "to hide that; the overshoot would simply hit a joint stop here.")
        out.append(f"  The {STEP_DEG:g} deg step used below is inside BOTH ranges, which "
                   f"is why it is the right test while the mapping is still provisional. "
                   f"Do not sweep wider until this is resolved.")
    else:
        out.append("the two ranges agree to within 0.05 deg")
    return out

CSV_HEADER = (
    "time_s", "control_update", "reference_deg", "ankle_angle_deg",
    "encoder_meas_deg", "error_deg", "ankle_velocity_rad_s", "encoder_rate_deg_s",
    # FIVE DISTINCT CURRENTS, in chain order. The first three already existed and keep
    # their names so the six-configuration sensitivity driver keeps parsing.
    "pid_output_pre_sign_A",      # 1 controller output, BEFORE the motor-sign mapping
    "current_request_A",          # 2 signed target, BEFORE clipping
    "current_applied_A",          # 3 signed target AFTER clipping = the command
    "current_actual_modelled_A",  # 4 after the current-loop model (== 3 when tau_i = 0)
    "layer_rotor_current_A",      # 5 rescaled into the layer's rotor-amp convention
    "current_saturated",
    "pid_p_A", "pid_i_A", "pid_d_A", "pid_derivative_used",
    "estimated_actuator_output_torque_Nm", "implied_rotor_torque_Nm",
    "friction_torque_actuator_Nm", "belt_torque_actuator_side_Nm",
    "belt_torque_joint_side_Nm", "theta_a_rad", "theta_a_dot_rad_s", "theta_s_rad",
    "K_s_Nm_per_rad", "belt_energy_J", "ankle_servo_force_Nm",
)


# =====================================================================================
def build_audit(args):
    """Choose the torque constant and the belt ratio, explicitly and once."""
    kt, prov = torque_constant(args.kt)
    if args.belt == "cad":
        n_t, src = CAD_BELT_RATIO, (f"CAD mesh tooth counts {CAD_BELT_TEETH[0]}/"
                                    f"{CAD_BELT_TEETH[1]} (CONFIRMED from the MJCF)")
        params = ANKLE_PAPER_CAD_BELT
    else:
        n_t, src = PAPER.n_t, "Best et al. 2025 (PAPER), a different build"
        params = PAPER
    return ConversionAudit(
        k_t_output_Nm_per_A=kt, k_t_output_choice=args.kt, k_t_output_provenance=prov,
        belt_ratio=n_t, belt_ratio_source=src, params=params,
        limits=CurrentLimits(sim_clip_A=args.current_clip))


def resolve_encoder(bench, args):
    """Derive the encoder sign from geometry if possible; otherwise say so."""
    if args.encoder_sign != 0:
        return (EncoderConvention(sign=int(args.encoder_sign),
                                  provenance="set explicitly by --encoder-sign"),
                f"forced to {args.encoder_sign:+d} on the command line")
    sign, report = derive_mjcf_q_to_encoder_sign(bench)
    if sign is None:
        enc = EncoderConvention()        # the documented default, sign = -1
        return enc, (f"NOT derived ({report}); falling back to the documented default "
                     f"sign = {enc.sign:+d}")
    return (EncoderConvention(sign=sign,
                              provenance=f"MEASURED from the compiled model: {report}"),
            report)


def run(bench, sim, pid, enc, args, dt):
    """1 kHz controller, 2 kHz physics, command held between controller updates.

    The ratio is computed from the model's own timestep rather than hard-coded, so a
    model with a different dt produces a documented ratio instead of a silent mismatch.
    """
    steps_per_update = int(round((1.0 / CONTROL_HZ) / dt))
    if steps_per_update < 1:
        raise RuntimeError(f"physics dt {dt} s is coarser than the {CONTROL_HZ:g} Hz "
                           f"control period; refusing to fake a faster controller")
    if abs(steps_per_update * dt - 1.0 / CONTROL_HZ) > 1e-12:
        raise RuntimeError(f"{CONTROL_HZ:g} Hz does not divide into dt = {dt} s "
                           f"(got {steps_per_update} steps/update); the schedule would "
                           f"drift. Refusing rather than rounding.")
    n_updates = int(round(RECORD_S * CONTROL_HZ))
    n_steps = n_updates * steps_per_update

    rows = []
    t = 0.0
    i_cmd = 0.0
    diverged_at = None
    for u in range(n_updates):
        # ---- controller update, in the ENCODER's units ------------------------------
        meas_deg = enc.to_encoder_deg(sim.theta_j)
        rate_deg_s = enc.to_encoder_deg_per_s(sim.theta_j_dot)
        i_cmd = pid.update(STEP_DEG, meas_deg, 1.0 / CONTROL_HZ)
        snap = dict(pid.last)

        # ---- physics, with the command HELD ----------------------------------------
        for s in range(steps_per_update):
            rec = sim.step(i_cmd)
            t += dt
            L = sim.last_layer
            q = sim.theta_j
            if diverged_at is None and not all(
                    math.isfinite(x) for x in (q, L.tau_j, L.theta_s, L.theta_a_dot)):
                diverged_at = t
            rows.append((
                t, u, STEP_DEG, math.degrees(q), enc.to_encoder_deg(q),
                snap["error_deg"], sim.theta_j_dot,
                enc.to_encoder_deg_per_s(sim.theta_j_dot),
                snap["raw_A"], snap["raw_signed_A"], i_cmd,
                sim.last_current["actual_A"], sim.last_current["layer_rotor_A"],
                int(snap["saturated"]),
                snap["p_A"], snap["i_A"], snap["d_A"], int(snap["derivative_used"]),
                L.tau_m, L.tau_m / PAPER.n_a, L.tau_f, L.tau_a, L.tau_j,
                L.theta_a, L.theta_a_dot,
                L.theta_s, L.K_s, L.U, sim.ankle_servo_force(),
            ))
        if diverged_at is not None:
            break
    return rows, dict(steps_per_update=steps_per_update, n_updates=n_updates,
                      n_steps=n_steps, diverged_at=diverged_at)


# =====================================================================================
def metrics(rows, sched, audit, enc, pid, args, dt, loop):
    """Angle metrics via the KNEE experiment's own definitions; the rest named here.

    `calculate_metrics` from `experiments/run_step_response.py` is imported rather than
    reimplemented so that rise time, overshoot and settling mean the SAME thing in both
    experiments -- two private definitions would make the two step responses
    incomparable, which is the whole point of having both.

    That function's four generic request/delivered/saturation keys are named for TORQUE
    because that is what the knee experiment feeds it. Here they are fed CURRENT, so
    they are renamed on the way out. Nothing is silently mislabelled.
    """
    from experiments.run_step_response import calculate_metrics

    a = np.asarray(rows, dtype=object)
    col = {name: i for i, name in enumerate(CSV_HEADER)}

    def f(name):
        return np.asarray([float(r[col[name]]) for r in rows], dtype=float)

    t = f("time_s")
    enc_meas = f("encoder_meas_deg")
    req = f("current_request_A")
    app = f("current_applied_A")
    sat = f("current_saturated").astype(bool)
    ref = np.full_like(t, STEP_DEG)
    ref[0] = BASELINE_DEG           # calculate_metrics reads ref[0] as the start level

    out = {}
    if sched["diverged_at"] is None:
        m = calculate_metrics(t, ref, enc_meas, req, app, sat,
                              step_time_s=float(t[0]), target_deg=STEP_DEG,
                              torque_limit=audit.limits.sim_clip_A, dt=dt)
        # rename the generic keys, which are CURRENT here and not torque
        m.pop("kp_Nm_per_rad", None)
        m.pop("kd_Nms_per_rad", None)
        out["peak_current_request_A"] = m.pop("peak_torque_request_Nm")
        out["peak_current_applied_A"] = m.pop("peak_torque_delivered_Nm")
        out["current_clip_A"] = m.pop("torque_limit_Nm")
        out.update(m)
        # A maximum at the final sample is not an overshoot peak.  Treat the same
        # 2%-of-step band used by calculate_metrics as the meaningful-overshoot
        # threshold, and report the peak time as not applicable when the trace never
        # exceeds target by that amount.
        if (float(m["overshoot_pct"]) <= 100.0 * SETTLING_FRACTION
                and float(m["final_angle_deg"]) <= STEP_DEG
                + SETTLING_FRACTION * abs(STEP_DEG - BASELINE_DEG)):
            out["peak_time_s"] = "not applicable (no meaningful overshoot)"
            out["peak_time_applicable"] = False
            out["overshoot_definition"] = (
                f"overshoot above target by more than {SETTLING_FRACTION:g} "
                f"of the {abs(STEP_DEG - BASELINE_DEG):g} deg step")
        else:
            out["peak_time_applicable"] = True
    else:
        out["DIVERGED_at_s"] = sched["diverged_at"]

    # --- torque, with the frame IN THE KEY NAME -------------------------------------
    out["peak_current_actual_modelled_A"] = float(
        np.max(np.abs(f("current_actual_modelled_A"))))
    out["peak_estimated_actuator_output_torque_Nm"] = float(
        np.max(np.abs(f("estimated_actuator_output_torque_Nm"))))
    out["peak_implied_rotor_torque_Nm"] = float(
        np.max(np.abs(f("implied_rotor_torque_Nm"))))
    out["peak_belt_torque_ACTUATOR_side_Nm"] = float(
        np.max(np.abs(f("belt_torque_actuator_side_Nm"))))
    out["peak_belt_torque_JOINT_side_Nm"] = float(
        np.max(np.abs(f("belt_torque_joint_side_Nm"))))
    out["final_belt_torque_JOINT_side_Nm"] = float(f("belt_torque_joint_side_Nm")[-1])
    ths = f("theta_s_rad")
    out["max_abs_theta_s_rad"] = float(np.max(np.abs(ths)))
    out["theta_s_as_multiple_of_paper_fit_edge"] = out["max_abs_theta_s_rad"] / FIT_EDGE_RAD
    out["max_abs_theta_a_dot_rad_s"] = float(np.max(np.abs(f("theta_a_dot_rad_s"))))
    out["peak_K_s_Nm_per_rad"] = float(np.max(f("K_s_Nm_per_rad")))

    # --- the current envelope, as LABELS not permissions ----------------------------
    pk = out.get("peak_current_applied_A", float(np.max(np.abs(app))))
    out["peak_applied_current_envelope"] = audit.limits.classify(pk)
    out["saturation_occurred"] = bool(sat.any())

    # --- provenance, carried in the metrics file itself ------------------------------
    out["k_t_output_Nm_per_A"] = audit.k_t_output_Nm_per_A
    out["k_t_output_choice"] = audit.k_t_output_choice
    out["belt_ratio_n_t"] = audit.belt_ratio
    out["belt_ratio_source"] = audit.belt_ratio_source
    out["rigid_equivalent_joint_kt_Nm_per_A"] = audit.rigid_joint_kt
    out["mjcf_q_to_encoder_sign"] = enc.sign
    out["mjcf_q_to_encoder_sign_provenance"] = enc.provenance
    out["encoder_positive_means"] = enc.positive_means
    out["kp_A_per_deg"] = pid.gains.kp_A_per_deg
    out["ki_A_per_deg_s"] = pid.gains.ki_A_per_deg_s
    out["kd_A_per_deg_per_s"] = pid.gains.kd_A_per_deg_per_s
    out["controller_output_to_motor_current_sign"] = (
        pid.gains.controller_output_to_motor_current_sign)
    out["sign_product_negative_feedback"] = (
        enc.sign * pid.gains.controller_output_to_motor_current_sign > 0)
    out["derivative_on"] = pid.derivative_on
    out["control_hz"] = CONTROL_HZ
    out["physics_dt_s"] = dt
    out["physics_steps_per_control_update"] = sched["steps_per_update"]
    out["current_loop_model"] = ("ideal passthrough" if loop.ideal else
                                 "first-order, exact exponential update")
    out["current_loop_tau_ms"] = loop.tau_s * 1e3
    out["current_loop_i0_A"] = loop.i0
    out["current_loop_description"] = loop.describe()
    out["ideal_current_loop"] = (
        "YES -- tau_i = 0, applied current equals the clipped command exactly"
        if loop.ideal else
        f"NO -- first-order lag with tau_i = {loop.tau_s * 1e3:g} ms. ILLUSTRATIVE "
        f"ASSUMPTION, not a measured driver response.")
    out["hardware_validated"] = "NO"
    return out


def write_csv(path, rows, stub=False):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(("# " + BANNER,))
        w.writerow(("# " + DISCLAIMER,))
        if stub:
            w.writerow(("# *** STUB -- NOT MUJOCO: these rows come from "
                        "tests/stub_mujoco.py, whose ankle has armature 0.0 (vs 0.008), "
                        "damping 0.05 (vs 0.25) and frictionloss 0.05 (vs 0.3). A "
                        "DIFFERENT PLANT, not a rougher one. Wiring check only. ***",))
        w.writerow(("# current_request_A is the CONTROLLER'S COMMAND before clipping; "
                    "current_applied_A is the clipped command. NEITHER is measured "
                    "current -- the driver's current loop is assumed ideal.",))
        w.writerow(("# FOUR DISTINCT TORQUE FRAMES, never to be compared directly: "
                    "implied_rotor_torque_Nm [ROTOR, = output/n_a, DERIVED and only as "
                    "trustworthy as n_a]; estimated_actuator_output_torque_Nm "
                    "[ACTUATOR OUTPUT, = I_q x k_t_output -- an ESTIMATE through an "
                    "UNRESOLVED torque constant, NOT motor-rotor torque]; "
                    "belt_torque_actuator_side_Nm [ACTUATOR OUTPUT]; "
                    "belt_torque_joint_side_Nm [JOINT, = actuator side x n_t]. "
                    "None is a measured torque.",))
        w.writerow(CSV_HEADER)
        for r in rows:
            w.writerow([f"{v:.9g}" if isinstance(v, float) else v for v in r])
    return path


def write_metrics(path, m):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(("# " + DISCLAIMER,))
        if str(m.get("WARNING", "")).startswith("STUB"):
            w.writerow(("# *** STUB -- NOT MUJOCO ***",))
        w.writerow(("metric", "value"))
        for k, v in m.items():
            w.writerow((k, f"{v:.9g}" if isinstance(v, float) else v))
    return path


def plot(path, rows, m, audit, enc, stub=False):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as exc:                                    # noqa: BLE001
        print(f"  [plot] skipped: matplotlib unavailable ({type(exc).__name__}: {exc})")
        return None
    col = {n: i for i, n in enumerate(CSV_HEADER)}

    def f(n):
        return np.asarray([float(r[col[n]]) for r in rows], dtype=float)

    t = f("time_s")
    fig, ax = plt.subplots(6, 1, figsize=(11, 16), sharex=True)
    title = f"{BANNER}\n{DISCLAIMER}"
    if stub:
        title = "*** STUB -- NOT MUJOCO: a DIFFERENT plant, wiring check only ***\n" + title
    fig.suptitle(title, fontsize=10.5, fontweight="bold",
                 color=("#9a2b11" if stub else "black"))

    ax[0].axhline(STEP_DEG, color="#9a9aa4", lw=1.2, ls="--",
                  label=f"reference {STEP_DEG:g} deg (encoder frame)")
    ax[0].plot(t, f("encoder_meas_deg"), color="#1f4e8c", lw=1.4,
               label="encoder measurement [deg]")
    ax[0].plot(t, f("ankle_angle_deg"), color="#2c6e49", lw=1.0, ls=":",
               label=f"MuJoCo ankle qpos [deg] (encoder sign {enc.sign:+d})")
    ax[0].set_ylabel("ankle angle [deg]")
    ax[0].legend(fontsize=8, loc="best")

    ax[1].axhline(0.0, color="#b4b4ac", lw=0.8)
    ax[1].plot(t, f("error_deg"), color="#9a2b11", lw=1.3)
    ax[1].set_ylabel("tracking error [deg]\n(encoder frame)")

    ax[2].plot(t, f("ankle_velocity_rad_s"), color="#6a3d9a", lw=1.3,
               label="MuJoCo ankle angular velocity [rad/s]")
    ax[2].plot(t, f("encoder_rate_deg_s"), color="#1f78b4", lw=1.0, ls="--",
               label="encoder-frame angular velocity [deg/s]")
    ax[2].axhline(0.0, color="#b4b4ac", lw=0.8)
    ax[2].set_ylabel("angular velocity")
    ax[2].legend(fontsize=7.5, loc="best")

    ax[3].plot(t, f("current_request_A"), color="#8c4a1f", lw=1.0,
               label="PID request, pre-clip [A]")
    ax[3].plot(t, f("current_applied_A"), color="#1f4e8c", lw=1.4,
               label="applied (clipped) command [A]")
    for v, lab, c in ((audit.limits.sim_clip_A, "sim clip (script value)", "#9a2b11"),
                      (audit.limits.reported_rated_A, "reported rated", "#2c6e49"),
                      (audit.limits.reported_peak_A, "reported peak", "#c08a2e")):
        if v:
            for s in (+1, -1):
                ax[3].axhline(s * v, color=c, lw=0.9, ls="--",
                              label=lab if s > 0 else None)
    ax[3].set_ylabel("current [A]\nCOMMANDED, not measured")
    ax[3].legend(fontsize=7.5, loc="best", ncol=2)

    ax[4].plot(t, f("belt_torque_actuator_side_Nm"), color="#8c4a1f", lw=1.2,
               label="belt torque, ACTUATOR side [N.m]")
    ax[4].plot(t, f("estimated_actuator_output_torque_Nm"), color="#c08a2e",
               lw=0.9, ls=":",
               label="I_q x k_t_output = ESTIMATED actuator-output torque [N.m]")
    ax[4].set_ylabel("actuator-side torque\n[N.m]")
    ax[4].legend(fontsize=7.5, loc="best")
    axb = ax[4].twinx()
    axb.plot(t, f("belt_torque_joint_side_Nm"), color="#1f4e8c", lw=1.4,
             label="belt torque, JOINT side [N.m]")
    axb.set_ylabel("joint-side torque [N.m]", color="#1f4e8c")
    axb.legend(fontsize=7.5, loc="lower right")

    ths = f("theta_s_rad")
    pk_ths = float(np.max(np.abs(ths))) or 1e-9
    # Two regimes. If the deflection is anywhere near the fitted edge, show the edge as
    # a shaded band so "outside the paper's data" is unmissable. If it is far inside --
    # which is the normal case for a gentle 5 deg step -- pinning the axis to the edge
    # would flatten the trace into a straight line at zero and hide the only thing the
    # panel is for. Then scale to the data and state the edge in the legend instead.
    near_edge = pk_ths > 0.2 * FIT_EDGE_RAD
    if near_edge:
        lim = max(FIT_EDGE_RAD * 1.4, pk_ths * 1.2)
        ax[5].axhspan(FIT_EDGE_RAD, lim, color="#efe3c8",
                      label=f"|theta_s| > {FIT_EDGE_RAD:g} rad = outside the paper's "
                            f"fitted range")
        ax[5].axhspan(-lim, -FIT_EDGE_RAD, color="#efe3c8")
        ax[5].set_ylim(-lim, lim)
        edge_note = ""
    else:
        lim = pk_ths * 1.3
        ax[5].set_ylim(-lim, lim)
        edge_note = (f"  [axis scaled to the data: peak is {pk_ths / FIT_EDGE_RAD:.3f}x "
                     f"the {FIT_EDGE_RAD:g} rad fitted edge, which is off-scale here -- "
                     f"the belt is operating well inside the paper's data]")
    ax[5].plot(t, ths, color="#2c6e49", lw=1.3,
               label=f"belt deflection theta_s [rad], peak {pk_ths:.3e}")
    ax[5].set_ylabel("belt deflection\n[rad]")
    ax[5].set_xlabel("time since PID engagement [s]" + edge_note)
    ax[5].legend(fontsize=7.5, loc="best")

    for a in ax:
        a.grid(alpha=0.25, lw=0.6)
    fig.tight_layout(rect=(0, 0, 1, 0.965))
    os.makedirs(os.path.dirname(path), exist_ok=True)
    fig.savefig(path, dpi=130)
    plt.close(fig)
    return path


# =====================================================================================
def main() -> int:
    global STEP_DEG
    ap = argparse.ArgumentParser(
        description=f"{BANNER}. {DISCLAIMER}.",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--kt", choices=sorted(TORQUE_CONSTANT_CHOICES), default="script",
                    help="which ACTUATOR-OUTPUT torque constant to use. The default "
                         "reproduces the hardware script; none is known to be correct.")
    ap.add_argument("--belt", choices=("cad", "paper"), default="cad",
                    help="belt ratio: 'cad' = 50/11 from the MJCF mesh names "
                         "(default), 'paper' = Best et al.'s 4.61")
    ap.add_argument("--step-deg", type=float, default=STEP_DEG,
                    help="encoder-frame ankle step reference in degrees (default: 5)")
    ap.add_argument("--kp", type=float, default=PIDGains.kp_A_per_deg, help="A/deg")
    ap.add_argument("--ki", type=float, default=PIDGains.ki_A_per_deg_s, help="A/(deg.s)")
    ap.add_argument("--kd", type=float, default=PIDGains.kd_A_per_deg_per_s,
                    help="A/(deg/s)")
    ap.add_argument("--current-clip", type=float, default=CurrentLimits.sim_clip_A,
                    help="SIMULATION-ONLY current clip in A. Defaults to the script's "
                         "reported code-level value. NOT a safety limit.")
    ap.add_argument("--encoder-sign", type=int, choices=(-1, 0, 1), default=0,
                    help="0 (default) = derive it from the model geometry; -1/+1 forces it")
    ap.add_argument("--derivative-on", choices=("error", "measurement"), default="error",
                    help="'error' reproduces the hardware script (default)")
    ap.add_argument("--current-loop-tau-ms", type=float, default=0.0,
                    help="first-order current-loop time constant in MILLISECONDS. "
                         "0 (default) = the historical ideal loop, reproduced exactly. "
                         "A nonzero value is an ILLUSTRATIVE MODEL ASSUMPTION: the real "
                         "driver's time constant has not been measured.")
    ap.add_argument("--backend", choices=("mujoco", "stub"), default="mujoco",
                    help="'mujoco' (default) REQUIRES the real engine and fails loudly "
                         "without it. 'stub' uses tests/stub_mujoco.py for wiring "
                         "checks only: a DIFFERENT plant (ankle armature 0.0 vs 0.008, "
                         "damping 0.05 vs 0.25, frictionloss 0.05 vs 0.3), written to a "
                         "separate directory and marked STUB -- NOT MUJOCO.")
    ap.add_argument("--outdir", default=None,
                    help="defaults to build/ankle_current_step, or "
                         "build/ankle_current_step_STUB under --backend stub")
    ap.add_argument("--explain", action="store_true",
                    help="print the conversion audit and exit without simulating")
    ap.add_argument("--no-plot", action="store_true")
    args = ap.parse_args()
    if not math.isfinite(args.step_deg) or args.step_deg <= 0.0:
        ap.error("--step-deg must be finite and greater than zero")
    STEP_DEG = args.step_deg

    mj, backend_label, is_stub = require_backend(args.backend)
    if args.outdir is None:
        args.outdir = OUTDIR + ("_STUB" if is_stub else "")
    stub_mark = "    *** STUB -- NOT MUJOCO ***" if is_stub else ""

    print("=" * 78)
    print(f"  {BANNER}")
    print(f"  {DISCLAIMER}")
    if is_stub:
        print("  *** STUB -- NOT MUJOCO. These numbers describe tests/stub_mujoco.py,")
        print("      not the CAD model. Wiring check only. ***")
    print("=" * 78)
    print(f"  backend   {backend_label}{stub_mark}")

    audit = build_audit(args)
    if args.explain:
        for ln in audit.explain():
            print("  " + ln)
        print()
        print("  CANDIDATE OUTPUT-SHAFT TORQUE CONSTANTS, all provisional:")
        for name, (v, prov) in sorted(TORQUE_CONSTANT_CHOICES.items()):
            mark = " <- selected" if name == args.kt else ""
            print(f"    {name:13s} {v:8.4f} N.m/A{mark}")
            print(f"                  {prov}")
        print()
        print(f"  BELT RATIOS: cad {CAD_BELT_RATIO:.6f} "
              f"({CAD_BELT_TEETH[0]}/{CAD_BELT_TEETH[1]}, CONFIRMED from the MJCF) vs "
              f"paper {PAPER.n_t:g}")
        print(f"  overall nominal reduction: cad {9 * CAD_BELT_RATIO:.4f}:1, "
              f"paper {9 * PAPER.n_t:.4f}:1")
        return 0

    bench = load_bench_model(verbose=False)
    if bench.failures:
        print(f"\nREFUSING TO RUN: {len(bench.failures)} model precondition(s) failed")
        for f_ in bench.failures:
            print(f"  - {f_}")
        return len(bench.failures)
    dt = bench.timestep
    ident = model_identity(bench, mj, is_stub)
    print(f"  loaded    {ident['model_path']}  sha256[:16] {ident['model_xml_sha256']}")
    print(f"            nq={ident['nq']} nv={ident['nv']} nu={ident['nu']} "
          f"nbody={ident['nbody']} ngeom={ident['ngeom']} nmesh={ident['nmesh']}  "
          f"dt={ident['timestep_s'] * 1e3:.3f} ms")
    print(f"            ankle armature={ident['ankle_armature']:g} "
          f"damping={ident['ankle_damping']:g} "
          f"frictionloss={ident['ankle_frictionloss']:g}  "
          f"matches the MJCF: {ident['ankle_properties_match_mjcf']}")
    if not ident["ankle_properties_match_mjcf"]:
        print("            ^^^ the loaded plant's ankle does NOT match the authored "
              "MJCF, so this is not the CAD model whatever the path says")

    enc, enc_report = resolve_encoder(bench, args)
    gains = PIDGains(kp_A_per_deg=args.kp, ki_A_per_deg_s=args.ki,
                     kd_A_per_deg_per_s=args.kd)

    pid = CurrentPID(gains, audit.limits, derivative_on=args.derivative_on)
    tau_ms = float(args.current_loop_tau_ms)
    if not math.isfinite(tau_ms) or tau_ms < 0.0:
        print(f"\nREFUSING TO RUN: --current-loop-tau-ms must be finite and >= 0, got "
              f"{args.current_loop_tau_ms!r}.")
        return 2
    loop = CurrentLoopModel(tau_ms * 1e-3)
    sim = AnkleCurrentSim(bench, audit, current_loop=loop)

    print(f"  model     models/osl_v2_bench.xml   dt {dt * 1e3:.3f} ms")
    print(f"  joint     ANKLE (qpos {bench.ankle_qpos}, dof {bench.ankle_dof}), "
          f"authored ROM [{math.degrees(bench.ankle_range[0]):.1f}, "
          f"{math.degrees(bench.ankle_range[1]):.1f}] deg, authority "
          f"+/-{bench.ankle_forcerange[1]:.1f} N.m")
    cons = check_sign_consistency(enc, gains, audit.params)
    rom_enc = enc.rom_in_encoder_frame(bench.ankle_range)
    print(f"  mapping 1 mjcf_q_to_encoder_sign = {cons['mjcf_q_to_encoder_sign']:+d}  "
          f"(encoder positive = {enc.positive_means}, REPORTED)")
    print(f"            {enc_report}")
    print(f"  mapping 2 controller_output_to_motor_current_sign = "
          f"{cons['controller_output_to_motor_current_sign']:+d}  "
          f"(the script's ENCODER_TO_MOTOR_SIGN, REPORTED)")
    print(f"            a controller-internal sign, NOT a coordinate-frame statement; "
          f"mapping 1 is derived WITHOUT it")
    print(f"  loop      product {cons['product']:+d} -> "
          f"{'NEGATIVE feedback' if cons['negative_feedback'] else 'POSITIVE FEEDBACK'}"
          f"  (a CONSEQUENCE of the two mappings, not evidence for either)")
    if not cons["negative_feedback"]:
        print(f"            {cons['note']}")
    for ln in limit_report(bench, enc):
        print(f"  limits    {ln}" if ln.startswith(("CAD", "physical", "the two"))
              else f"            {ln}")
    print(f"  law       Kp {gains.kp_A_per_deg:g} A/deg, Ki {gains.ki_A_per_deg_s:g} "
          f"A/(deg.s), Kd {gains.kd_A_per_deg_per_s:g} A/(deg/s), "
          f"sign {gains.encoder_to_motor_sign:+d}, derivative on "
          f"{args.derivative_on}")
    print(f"            REPORTED from the hardware script -- NOT tuned, NOT validated")
    print(f"  k_t       {audit.k_t_output_Nm_per_A:.6f} N.m/A at the ACTUATOR OUTPUT "
          f"({args.kt})")
    print(f"  belt      n_t {audit.belt_ratio:.6f} ({args.belt}); the MJCF has NO "
          f"transmission, its pulleys are visual geoms")
    print(f"  current   clip {audit.limits.sim_clip_A:g} A (simulation only)")
    print(f"  loop      {loop.describe()}")
    if not loop.ideal:
        print(f"            evolves at the {1e3 * dt:.3f} ms PHYSICS step while the "
              f"{CONTROL_HZ:g} Hz command is held, so it keeps responding between "
              f"controller updates")

    # Baseline as an INITIAL CONDITION, not as an extra recorded second. The script's
    # 1 s of baseline preparation before PID engages leaves the ankle at the baseline
    # angle with zero velocity, which is exactly what reset() establishes; t = 0 here is
    # PID engagement, matching the physical log's own time zero.
    sim.reset(enc.to_model_rad(BASELINE_DEG))
    rows, sched = run(bench, sim, pid, enc, args, dt)
    print(f"  schedule  {CONTROL_HZ:g} Hz controller, {1.0 / dt:g} Hz physics -> "
          f"{sched['steps_per_update']} physics step(s) per update, command held "
          f"between updates")

    servo = np.asarray([float(r[CSV_HEADER.index("ankle_servo_force_Nm")])
                        for r in rows], float)
    viol = int(np.count_nonzero(servo != 0.0))
    print(f"  servo     ankle position actuator contributed {viol} nonzero force(s) "
          f"over {len(rows)} steps, peak |force| {np.max(np.abs(servo)):.3e} N.m")
    if viol:
        print("\nREFUSING TO REPORT METRICS: the ankle servo acted, so the belt and the "
              "servo were both driving the joint and every number would be a double "
              "count.")
        return 1
    print(f"            that zero is the proof the belt is the only actuation on the "
          f"ankle; the KNEE servo stays connected on purpose, holding the knee so the "
          f"ankle's gravity load does not drift")

    if sched["diverged_at"] is not None:
        print(f"\n  *** the run went non-finite at t = {sched['diverged_at']:.4f} s. ***")

    m = metrics(rows, sched, audit, enc, pid, args, dt, loop)

    # ---- BACKEND AND PLANT IDENTITY, carried in the metrics file itself --------------
    # So a CSV found on disk in six months can still answer "which engine made this?"
    # without anyone having to trust a filename.
    m.update({f"model_{k}": v for k, v in ident.items()})
    m["n_physics_steps"] = len(rows)
    m["n_control_updates"] = sched["n_updates"]
    m["control_update_rate_Hz"] = CONTROL_HZ
    m["ankle_servo_nonzero_steps"] = viol
    m["ankle_servo_peak_abs_force_Nm"] = float(np.max(np.abs(servo)))
    m["joint_force_source"] = ("Option-C DrivetrainLayer via qfrc_applied[ankle_dof]"
                               if viol == 0 else "AMBIGUOUS -- the servo also acted")
    ends = sorted((enc.to_encoder_deg(bench.ankle_range[0]),
                   enc.to_encoder_deg(bench.ankle_range[1])))
    m["cad_rom_encoder_frame_deg"] = f"[{ends[0]:+.1f}, {ends[1]:+.1f}]"
    m["cad_rom_mujoco_frame_deg"] = (
        f"[{math.degrees(bench.ankle_range[0]):+.1f}, "
        f"{math.degrees(bench.ankle_range[1]):+.1f}]")
    m["physical_ctrl_rom_encoder_frame_deg"] = (
        f"[{PHYSICAL_ENCODER_RANGE_DEG[0]:+.1f}, {PHYSICAL_ENCODER_RANGE_DEG[1]:+.1f}] "
        f"(REPORTED, second-hand)")
    if is_stub:
        m["WARNING"] = ("STUB -- NOT MUJOCO. tests/stub_mujoco.py never parses the "
                        "MJCF and its ankle is a different plant. Wiring check only.")

    # ---- the WRONG-SIGN detector ----------------------------------------------------
    # A reversed encoder sign does NOT produce a NaN here: the authored ankle ROM
    # catches the runaway, so the trace stays finite and the metrics still compute. The
    # signature is instead a loop driven away from the reference -- a large standing
    # error together with either saturation or substantial displacement. Saturation is
    # not guaranteed when the current clip is above the wrong-sign steady command.
    err_final = abs(float(m.get("steady_state_error_deg", 0.0)))
    sat_pct = float(m.get("saturation_pct", 0.0))
    final_angle = abs(float(m.get("final_angle_deg", 0.0)))
    if (err_final > 0.5 * STEP_DEG
            and (sat_pct > 1.0 or final_angle > 0.5 * STEP_DEG)):
        print(f"\n  *** POSITIVE-FEEDBACK SIGNATURE: final error {err_final:.2f} deg "
              f"against a {STEP_DEG:g} deg step, final angle "
              f"{final_angle:.2f} deg, command saturated {sat_pct:.1f} % of the run.")
        print(f"      The loop is driving AWAY from the reference and parking against "
              f"a joint limit, not tracking it.")
        print(f"      Negative feedback needs encoder_sign * ENCODER_TO_MOTOR_SIGN = "
              f"+1 (the two must AGREE). This run had "
              f"{enc.sign:+d} * {gains.encoder_to_motor_sign:+d} = "
              f"{enc.sign * gains.encoder_to_motor_sign:+d}.")
        print(f"      Treat these metrics as a diagnosis of the sign, NOT as a step "
              f"response.")
        m["POSITIVE_FEEDBACK_SUSPECTED"] = "yes -- see the encoder sign"
    p_csv = write_csv(os.path.join(args.outdir, "ankle_current_step.csv"),
                      rows, stub=is_stub)
    p_met = write_metrics(os.path.join(args.outdir, "ankle_current_step_metrics.csv"), m)
    p_png = None if args.no_plot else plot(
        os.path.join(args.outdir, "ankle_current_step.png"), rows, m, audit, enc,
        stub=is_stub)

    print("\n" + "-" * 78)
    print("  STEP METRICS  (encoder frame unless the key says otherwise)")
    for k in ("final_angle_deg", "steady_state_error_deg", "rise_time_10_90_s",
              "overshoot_pct", "peak_time_s", "settling_time_s",
              "peak_current_request_A", "peak_current_applied_A",
              "saturation_duration_s", "saturation_pct",
              "peak_applied_current_envelope",
              "peak_tau_motor_ACTUATOR_Nm", "peak_tau_belt_ACTUATOR_Nm",
              "peak_tau_belt_JOINT_Nm", "max_abs_theta_s_rad",
              "theta_s_as_multiple_of_paper_fit_edge"):
        if k in m:
            v = m[k]
            print(f"    {k:42s} {v:.6g}" if isinstance(v, float)
                  else f"    {k:42s} {v}")
    unavailable = [k for k, v in m.items()
                   if isinstance(v, float) and not math.isfinite(v)]
    if unavailable:
        print(f"    UNAVAILABLE (reported as nan rather than guessed): "
              f"{', '.join(unavailable)}")

    print("\n  outputs")
    for p in (p_csv, p_met, p_png):
        if p:
            print(f"    {os.path.relpath(p, ROOT)}")
    print("\n  WHAT THIS DOES NOT SHOW")
    print("  No parameter here was measured on OSL V2 hardware. The torque constant's")
    print("  reference frame is unresolved, the current loop is assumed ideal, and the")
    print("  belt is the paper's model with the CAD's ratio. Treat the comparison with")
    print("  the physical rig as PROVISIONAL. See docs/PHYSICAL_ANKLE_CONTROL_ALIGNMENT.md")
    print(f"  {DISCLAIMER}")
    if is_stub:
        print("  *** STUB -- NOT MUJOCO. Re-run in .venv for a simulation result. ***")
    return 0


if __name__ == "__main__":
    sys.exit(main())
