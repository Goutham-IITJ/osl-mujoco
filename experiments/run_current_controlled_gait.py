#!/usr/bin/env python3
"""
run_current_controlled_gait.py -- AB19 gait tracking through the CURRENT-domain controller.

    SIMULATION ONLY.  PROVISIONAL.
    The gains are the ANKLE step-test script's, provisionally transferred to the knee
    and NOT hardware-confirmed for the knee. The torque constant is a working
    hypothesis. Nothing here is validated against hardware.

WHAT THIS IS, AND HOW IT DIFFERS FROM THE THREE EXISTING BASELINES
    Four controller architectures now exist in this repository and they are NOT
    gain-comparable with one another:

      1. `run_bench_ab19.py`              torque-domain PD, MuJoCo position actuator,
                                          radians, Kp = 600 N.m/rad.  THE ORACLE.
      2. `run_bench_ab19_drivetrain.py`   torque-domain PD -> current -> Option C belt.
                                          Same gains, different path. (Case A/B.)
      3. `run_ankle_current_step.py`      current-domain PD on the ANKLE, degrees in,
                                          amps out, 5 deg step. Subsystem validation.
      4. THIS FILE                        current-domain PD on the KNEE, degrees in,
                                          amps out, following the full AB19 cycle.

    600 N.m/rad and 0.75 A/deg are different kinds of number. Comparing (1) and (4) is a
    comparison of CONTROL PATHWAYS, not of tuning, and the output says so.

    The step experiments are subsystem validation. This is the first time the
    current-controlled drivetrain follows the actual target trajectory.

THE JOINT, AND WHY IT IS THE KNEE
    `build/AB19_knee_gait_reference.csv` is a human KNEE-angle curve, 101 samples over
    1.2050 s, starting at 12.7055 deg. The target is therefore the KNEE following the
    knee reference. No ankle reference is involved and nothing is reinterpreted.

INITIALISATION
    The original protocol (`BenchSimulation.reset`) already seeds the knee ON the first
    reference sample with zero velocity, and its own report notes that the reference
    starts at +0.44 rad/s so a brief start transient is expected. So "aligned" is not a
    deviation -- it IS the validated protocol, and it is the default here.
    `--init zero` starts at 0 deg instead, which makes a deliberate 12.7 deg startup
    step for a transient experiment. Both are recorded in the metadata.

USAGE (Windows PowerShell)
    .venv\\Scripts\\python.exe experiments\\run_current_controlled_gait.py
    .venv\\Scripts\\python.exe experiments\\run_current_controlled_gait.py --kp 1.5
    .venv\\Scripts\\python.exe experiments\\run_current_controlled_gait.py --current-loop first_order --current-loop-tau-ms 2
    .venv\\Scripts\\python.exe experiments\\run_current_controlled_gait.py --view
    .venv\\Scripts\\python.exe experiments\\run_current_controlled_gait.py --init zero

OUTPUT
    build/current_controlled_gait/<tag>/  -- a directory per configuration, named with
    the gains and the current-loop mode, so no run can overwrite another.
"""

from __future__ import annotations

import argparse
import contextlib
import csv
import io
import json
import math
import os
import sys
import time

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

# The backend must be settled before the oslbench imports; see the ankle experiment's
# note -- oslbench/model.py binds `mujoco` at ITS import time.
_WANT_STUB = "--backend" in sys.argv[1:] and "stub" in sys.argv[1:]
if _WANT_STUB:
    sys.path.insert(0, os.path.join(ROOT, "tests"))
    import stub_mujoco as _stub
    _stub.install()

from oslbench.ankle_current_control import (CAD_BELT_RATIO, ConversionAudit,   # noqa: E402
                                            CurrentLimits, CurrentLoopModel,
                                            PIDGains, TORQUE_CONSTANT_CHOICES,
                                            torque_constant)
from oslbench.controller import PDController                              # noqa: E402
from oslbench.drivetrain import PAPER                                     # noqa: E402
from oslbench.drivetrain_sim import (DrivetrainBenchSimulation,            # noqa: E402
                                     DrivetrainLayer)
from oslbench.joint_current_control import (GAIN_SOURCE_NOTE,              # noqa: E402
                                            make_gait_source,
                                            GaitCurrentSource,
                                            JointCurrentConfig)
from oslbench.model import load_bench_model                               # noqa: E402
from oslbench.reference import (DEFAULT_REFERENCE_CSV, STANCE_END,         # noqa: E402
                                SUBJECT, TRIAL, audit_reference,
                                load_reference, resample_reference)

OUTROOT = os.path.join(ROOT, "build", "current_controlled_gait")
CONTROL_HZ = 1000.0
FIT_EDGE_RAD = 0.055
BANNER = "AB19 GAIT TRACKING THROUGH THE CURRENT-CONTROLLED OPTION-C DRIVETRAIN"
DISCLAIMER = ("SIMULATION ONLY. Gains are the ANKLE script's, provisionally "
              "transferred to the knee and NOT hardware-confirmed for the knee.")

CSV_HEADER = (
    "time_s", "gait_phase_pct", "control_update",
    "reference_deg", "knee_angle_deg", "tracking_error_deg",
    "knee_velocity_rad_s", "reference_velocity_rad_s",
    # five distinct currents, in chain order; none is a measurement
    "pid_output_pre_sign_A", "current_request_A", "current_applied_A",
    "current_actual_modelled_A", "layer_rotor_current_A", "current_saturated",
    "pid_p_A", "pid_i_A", "pid_d_A", "pid_derivative_used",
    # torque, with the frame in the name
    "estimated_actuator_output_torque_Nm", "implied_rotor_torque_Nm",
    "friction_torque_actuator_Nm", "belt_torque_actuator_side_Nm",
    "belt_torque_joint_side_Nm",
    "theta_a_rad", "theta_a_dot_rad_s", "theta_s_rad", "K_s_Nm_per_rad",
    "belt_energy_J", "knee_servo_force_Nm",
)


def require_backend(choice: str):
    """Real MuJoCo by default; the stub is explicit, separate and marked."""
    if choice == "stub":
        mod = sys.modules.get("mujoco")
        if mod is None:
            sys.path.insert(0, os.path.join(ROOT, "tests"))
            import stub_mujoco
            mod = stub_mujoco.install()
        is_stub = bool(getattr(mod, "__stub__", False))
        return mod, ("tests/stub_mujoco.py" if is_stub else "REAL MuJoCo"), is_stub
    try:
        import mujoco
    except ImportError as exc:
        raise SystemExit(
            f"\nREFUSING TO RUN: real MuJoCo is not importable ({exc}).\n"
            "\n  This experiment requires the compiled CAD model. The stub never parses\n"
            "  models/osl_v2_bench.xml and its joints are a different plant.\n"
            "\n  Use the project's virtual environment:\n"
            "    .venv\\Scripts\\python.exe experiments\\run_current_controlled_gait.py\n"
            "\n  Wiring-check only (separate directory, marked STUB -- NOT MUJOCO):\n"
            "    python experiments\\run_current_controlled_gait.py --backend stub\n")
    return mujoco, "REAL MuJoCo", False


def model_identity(bench, mj, is_stub):
    m = bench.model
    ident = dict(
        backend="STUB -- NOT MUJOCO" if is_stub else "real MuJoCo",
        mujoco_version="n/a (stub)" if is_stub else getattr(mj, "__version__", "unknown"),
        model_path=os.path.relpath(bench.path, ROOT).replace("\\", "/"),
        model_xml_sha256="unavailable",
        nq=int(getattr(m, "nq", -1)), nv=int(getattr(m, "nv", -1)),
        nu=int(getattr(m, "nu", -1)), nbody=int(getattr(m, "nbody", -1)),
        ngeom=int(getattr(m, "ngeom", -1)), nmesh=int(getattr(m, "nmesh", -1)),
        timestep_s=float(m.opt.timestep),
        knee_qpos_adr=int(bench.knee_qpos), knee_dof_adr=int(bench.knee_dof),
        knee_armature=float(m.dof_armature[bench.knee_dof]),
        knee_damping=float(m.dof_damping[bench.knee_dof]),
        knee_frictionloss=float(m.dof_frictionloss[bench.knee_dof]),
    )
    try:
        import hashlib
        with open(bench.path, "rb") as fh:
            ident["model_xml_sha256"] = hashlib.sha256(fh.read()).hexdigest()[:16]
    except Exception:                                           # noqa: BLE001
        pass
    ident["knee_properties_match_mjcf"] = (
        abs(ident["knee_armature"] - 0.01) < 1e-12
        and abs(ident["knee_damping"] - 0.3) < 1e-12
        and abs(ident["knee_frictionloss"] - 0.4) < 1e-12)
    return ident


# =====================================================================================
def build(args, mj, is_stub):
    bench = load_bench_model(verbose=False)
    if bench.failures:
        print(f"\nREFUSING TO RUN: {len(bench.failures)} model precondition(s) failed")
        for f in bench.failures:
            print(f"  - {f}")
        return None
    if not bench.path.replace("\\", "/").endswith("models/osl_v2_bench.xml"):
        print(f"\nREFUSING TO RUN: loaded {bench.relpath()}")
        return None
    dt = bench.timestep

    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        ref = load_reference(args.csv)
        audit_r = audit_reference(ref, math.degrees(bench.knee_ctrlrange[0]),
                                  math.degrees(bench.knee_ctrlrange[1]))
        res = resample_reference(ref, dt, args.cycles, "cubic", audit=audit_r)
    if args.verbose:
        print("-" * 78 + "\n" + buf.getvalue().rstrip() + "\n" + "-" * 78)

    steps_per_update = int(round((1.0 / CONTROL_HZ) / dt))
    if steps_per_update < 1 or abs(steps_per_update * dt - 1.0 / CONTROL_HZ) > 1e-12:
        print(f"\nREFUSING TO RUN: {CONTROL_HZ:g} Hz does not divide into dt = {dt} s; "
              f"the schedule would drift.")
        return None

    kt, kt_prov = torque_constant(args.kt)
    params = PAPER.with_belt_ratio(CAD_BELT_RATIO) if args.belt == "cad" else PAPER
    n_t = CAD_BELT_RATIO if args.belt == "cad" else PAPER.n_t
    audit = ConversionAudit(
        k_t_output_Nm_per_A=kt, k_t_output_choice=args.kt,
        k_t_output_provenance=kt_prov, belt_ratio=n_t,
        belt_ratio_source=("CAD mesh tooth counts 50/11 (CONFIRMED from the MJCF)"
                           if args.belt == "cad" else "Best et al. 2025 (PAPER)"),
        params=params, limits=CurrentLimits(sim_clip_A=args.current_clip))

    tau_s = 0.0 if args.current_loop == "ideal" else args.current_loop_tau_ms * 1e-3
    cfg = JointCurrentConfig(
        joint="knee",
        gains=PIDGains(kp_A_per_deg=args.kp, ki_A_per_deg_s=args.ki,
                       kd_A_per_deg_per_s=args.kd, encoder_to_motor_sign=+1),
        limits=audit.limits,
        measurement_sign=+1,
        measurement_frame=("MuJoCo knee generalized coordinate in degrees; the AB19 "
                           "reference is in the same frame, so no encoder convention "
                           "enters and the sign is +1 by construction"),
        gain_source=GAIN_SOURCE_NOTE,
        sign_source=("measurement_sign +1 by construction; motor_sign +1 for negative "
                     "feedback, verified against the model's torque chain. SIMULATION "
                     "WIRING, not a knee-hardware claim."),
        derivative_on=args.derivative_on,
        current_loop_tau_s=tau_s)
    if not cfg.negative_feedback:
        print("\nREFUSING TO RUN: the configured signs give POSITIVE feedback.")
        return None

    from oslbench.contracts import GaitContractAdapter

    layer = DrivetrainLayer(params, enabled=True, substeps=1)
    src = make_gait_source(cfg, audit, res["ref_rad"], steps_per_update, layer,
                           getattr(args, 'feedback_mode', 'legacy'))
    src.dt = dt
    # `knee=` is the torque-domain PDController the base class needs for its own
    # bookkeeping; it is DISCONNECTED below, so its gains never act on the joint. The
    # current-domain law in `src` is the only thing driving the knee.
    pd = PDController.knee(bench, 600.0, 17.253)
    sim = DrivetrainBenchSimulation(bench, pd, layer=layer, current_source=GaitContractAdapter(src, layer),
                                    disconnect_servo=True)
    if sim.servo_connected:
        print("\nREFUSING TO RUN: the knee position servo did not disconnect.")
        return None
    if src.audit.params is not layer.p and (src.audit.params.n_t != layer.p.n_t):
        print("\nREFUSING TO RUN: the audit and the layer disagree about the belt ratio.")
        return None
    return dict(bench=bench, sim=sim, src=src, cfg=cfg, audit=audit, res=res,
                dt=dt, steps_per_update=steps_per_update, n=res.n,
                ident=model_identity(bench, mj, is_stub), ref=ref)


def iter_run(ctx, args, on_frame=None):
    """One pass through the reference, logging every physics step.

    Uses `sim.run(..., on_step=...)` so the log and any viewer see the SAME simulation
    state -- there is no second integration anywhere.
    """
    bench, sim, src = ctx["bench"], ctx["sim"], ctx["src"]
    res, n, dt = ctx["res"], ctx["n"], ctx["dt"]
    ref_rad = np.asarray(res["ref_rad"], float)
    phase = np.asarray(res["gait_phase_percent"], float)
    refvel = np.asarray(res["ref_vel_rad_s"], float)
    rows = []
    k_seen = {"k": 0}

    def on_step(st):
        k = k_seen["k"]
        L = sim.last_layer
        s = src.last
        q = float(bench.data.qpos[bench.knee_qpos])
        qd = float(bench.data.qvel[bench.knee_dof])
        # Older callers may supply a raw source; production build always wraps it.
        from oslbench.contracts import GaitContractAdapter
        if isinstance(sim.current_source, GaitContractAdapter):
            ctx["typed_state"] = sim.current_source.post_step(k, q, qd, L)
            ctx["typed_currents"] = sim.current_source.currents
        servo = sim.knee_servo_force()
        ref_deg_k = math.degrees(float(ref_rad[k]))
        meas_deg_k = math.degrees(q)
        tracking_err_k = ref_deg_k - meas_deg_k
        rows.append((
            k * dt, float(phase[k]), s["control_update"],
            ref_deg_k, meas_deg_k,
            tracking_err_k,
            qd, float(refvel[k]),
            s["pid_output_pre_sign_A"], s["current_request_A"], s["current_applied_A"],
            s["current_actual_modelled_A"], s["layer_rotor_current_A"], s["saturated"],
            s["p_A"], s["i_A"], s["d_A"], s["derivative_used"],
            L.tau_m, L.tau_m / PAPER.n_a, L.tau_f, L.tau_a, L.tau_j,
            L.theta_a, L.theta_a_dot, L.theta_s, L.K_s, L.U, servo))
        k_seen["k"] = k + 1
        if on_frame is not None:
            on_frame(k, q, s, L)

    q0 = float(ref_rad[0]) if args.init == "aligned" else 0.0
    v0 = float(refvel[0]) if args.init == "aligned" else 0.0
    src.reset()
    sim.reset(q0, ref_vel0=v0)
    for k in range(n):
        st = sim.step(float(ref_rad[k]), k)
        on_step(st)
        yield rows.pop()


def run(ctx, args, on_frame=None):
    return list(iter_run(ctx, args, on_frame))


# =====================================================================================
def metrics(rows, ctx, args):
    col = {c: i for i, c in enumerate(CSV_HEADER)}

    def f(name):
        return np.asarray([float(r[col[name]]) for r in rows], float)

    t, ref, act = f("time_s"), f("reference_deg"), f("knee_angle_deg")
    err = f("tracking_error_deg")
    sat = f("current_saturated").astype(bool)
    tau_j = f("belt_torque_joint_side_Nm")
    ths = f("theta_s_rad")
    dt = ctx["dt"]
    finite = bool(np.isfinite(act).all() and np.isfinite(tau_j).all()
                  and np.isfinite(ths).all())

    out = {}
    out["numerically_stable"] = finite
    if not finite:
        bad = int(np.argmax(~np.isfinite(act)))
        out["first_nonfinite_step"] = bad
        out["first_nonfinite_time_s"] = bad * dt

    out["rms_error_deg"] = float(np.sqrt(np.mean(err ** 2)))
    out["mae_error_deg"] = float(np.mean(np.abs(err)))
    out["peak_abs_error_deg"] = float(np.max(np.abs(err)))
    out["final_error_deg"] = float(err[-1])
    # drop the documented start transient, so the cyclic number is not dominated by it
    i50 = int(round(0.050 / dt))
    out["rms_error_deg_after_50ms"] = float(np.sqrt(np.mean(err[i50:] ** 2)))

    # phase lag by cross-correlation, the same idea the knee benchmark uses
    best, best_rms = 0, float("inf")
    for lag in range(0, int(round(0.30 / dt))):
        if lag >= len(act):
            break
        e = ref[:len(act) - lag] - act[lag:]
        r = float(np.sqrt(np.mean(e ** 2)))
        if r < best_rms:
            best_rms, best = r, lag
    out["best_fit_lag_ms"] = best * dt * 1e3
    out["rms_error_deg_lag_removed"] = best_rms
    out["pct_error_from_lag"] = (
        100.0 * (1.0 - best_rms / out["rms_error_deg"])
        if out["rms_error_deg"] > 0 else 0.0)

    out["peak_current_request_A"] = float(np.max(np.abs(f("current_request_A"))))
    out["peak_current_applied_A"] = float(np.max(np.abs(f("current_applied_A"))))
    out["peak_current_actual_modelled_A"] = float(
        np.max(np.abs(f("current_actual_modelled_A"))))
    out["saturation_duration_s"] = float(np.count_nonzero(sat) * dt)
    out["saturation_pct"] = float(100.0 * np.mean(sat))
    out["peak_applied_current_envelope"] = ctx["audit"].limits.classify(
        out["peak_current_applied_A"])

    out["peak_estimated_actuator_output_torque_Nm"] = float(
        np.max(np.abs(f("estimated_actuator_output_torque_Nm"))))
    out["peak_implied_rotor_torque_Nm"] = float(np.max(np.abs(f("implied_rotor_torque_Nm"))))
    out["peak_belt_torque_ACTUATOR_side_Nm"] = float(
        np.max(np.abs(f("belt_torque_actuator_side_Nm"))))
    out["peak_belt_torque_JOINT_side_Nm"] = float(np.max(np.abs(tau_j)))
    out["pct_of_knee_authority"] = (
        100.0 * out["peak_belt_torque_JOINT_side_Nm"] / ctx["bench"].force_limit)
    out["max_abs_theta_s_rad"] = float(np.max(np.abs(ths)))
    out["theta_s_as_multiple_of_paper_fit_edge"] = (
        out["max_abs_theta_s_rad"] / FIT_EDGE_RAD)
    out["max_abs_theta_a_dot_rad_s"] = float(np.max(np.abs(f("theta_a_dot_rad_s"))))

    lo, hi = ctx["bench"].knee_range
    out["knee_min_deg"] = float(np.min(act))
    out["knee_max_deg"] = float(np.max(act))
    out["rom_violation_steps"] = int(np.count_nonzero(
        (act < math.degrees(lo) - 1e-6) | (act > math.degrees(hi) + 1e-6)))
    out["stayed_inside_authored_rom"] = out["rom_violation_steps"] == 0
    servo = f("knee_servo_force_Nm")
    out["knee_servo_nonzero_steps"] = int(np.count_nonzero(servo != 0.0))
    out["knee_servo_peak_abs_force_Nm"] = float(np.max(np.abs(servo)))
    out["joint_force_source"] = ("Option-C DrivetrainLayer via qfrc_applied[knee_dof]"
                                 if out["knee_servo_nonzero_steps"] == 0
                                 else "AMBIGUOUS -- the servo also acted")
    return out


def metadata(ctx, args, loop_mode):
    cfg, audit, ident = ctx["cfg"], ctx["audit"], ctx["ident"]
    md = dict(
        experiment="run_current_controlled_gait",
        feedback_mode=getattr(args, 'feedback_mode', 'legacy'),
        banner=BANNER, disclaimer=DISCLAIMER,
        joint_identity="knee", reference="AB19 human KNEE angle",
        reference_subject=SUBJECT, reference_trial=TRIAL,
        reference_rows=int(ctx["ref"].n),
        reference_cycle_duration_s=float(ctx["ref"].period_s),
        reference_first_sample_deg=float(ctx["ref"].angle_deg[0]),
        cycles=int(args.cycles), n_physics_steps=int(ctx["n"]),
        n_control_updates=int(ctx["n"] // ctx["steps_per_update"]),
        control_hz=CONTROL_HZ, physics_dt_s=ctx["dt"],
        physics_steps_per_control_update=ctx["steps_per_update"],
        init_mode=args.init,
        init_note=("aligned: knee seeded ON the first reference sample with zero "
                   "velocity -- this IS the original validated protocol; the reference "
                   "starts at +0.44 rad/s so a brief start transient is expected"
                   if args.init == "aligned" else
                   "zero: knee starts at 0 deg against a 12.7055 deg reference, a "
                   "deliberate startup-transient experiment"),
        kp_A_per_deg=cfg.gains.kp_A_per_deg,
        ki_A_per_deg_s=cfg.gains.ki_A_per_deg_s,
        kd_A_per_deg_per_s=cfg.gains.kd_A_per_deg_per_s,
        gain_source=cfg.gain_source,
        measurement_sign=cfg.measurement_sign,
        motor_sign=cfg.gains.encoder_to_motor_sign,
        sign_source=cfg.sign_source,
        derivative_on=cfg.derivative_on,
        current_loop_mode=loop_mode,
        current_loop_tau_ms=cfg.current_loop_tau_s * 1e3,
        current_loop_note=CurrentLoopModel(cfg.current_loop_tau_s).describe(),
        current_clip_A=audit.limits.sim_clip_A,
        current_clip_note="SIMULATION ONLY. Not a safe hardware limit.",
        k_t_output_Nm_per_A=audit.k_t_output_Nm_per_A,
        k_t_output_choice=audit.k_t_output_choice,
        k_t_output_provenance=audit.k_t_output_provenance,
        belt_ratio_n_t=audit.belt_ratio, belt_ratio_source=audit.belt_ratio_source,
        rigid_equivalent_joint_kt_Nm_per_A=audit.rigid_joint_kt,
        planetary_n_a=PAPER.n_a,
        architecture=("current-domain PD (deg in, A out) -> optional current loop -> "
                      "ConversionAudit rescale -> Option-C DrivetrainLayer -> "
                      "qfrc_applied[knee_dof]; legacy knee position servo DISCONNECTED"),
        not_comparable_with=("run_bench_ab19.py uses a torque-domain PD at 600 N.m/rad; "
                             "these are different control pathways, not different tuning"),
        hardware_validated="NO",
    )
    md.update({f"model_{k}": v for k, v in ident.items()})
    return md


def write_outputs(outdir, rows, m, md, is_stub):
    os.makedirs(outdir, exist_ok=True)
    stub_hdr = ("# *** STUB -- NOT MUJOCO: tests/stub_mujoco.py never parses the MJCF "
                "and its joints are a different plant. Wiring check only. ***",)
    p_csv = os.path.join(outdir, "gait_trace.csv")
    with open(p_csv, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(("# " + BANNER,))
        w.writerow(("# " + DISCLAIMER,))
        if is_stub:
            w.writerow(stub_hdr)
        w.writerow(("# FIVE DISTINCT CURRENTS, chain order: pid_output_pre_sign_A -> "
                    "current_request_A (signed, pre-clip) -> current_applied_A (signed, "
                    "clipped = the command) -> current_actual_modelled_A (after the "
                    "current loop) -> layer_rotor_current_A (rescaled for the layer). "
                    "NONE is a measured current.",))
        w.writerow(("# FOUR TORQUE FRAMES, never to be compared directly: "
                    "implied_rotor_torque_Nm [ROTOR, derived]; "
                    "estimated_actuator_output_torque_Nm [ACTUATOR OUTPUT, an estimate "
                    "through an unresolved torque constant]; "
                    "belt_torque_actuator_side_Nm [ACTUATOR OUTPUT]; "
                    "belt_torque_joint_side_Nm [JOINT]. None is measured.",))
        w.writerow(CSV_HEADER)
        for r in rows:
            w.writerow([f"{v:.9g}" if isinstance(v, float) else v for v in r])

    p_met = os.path.join(outdir, "gait_metrics.csv")
    with open(p_met, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(("# " + DISCLAIMER,))
        if is_stub:
            w.writerow(stub_hdr)
        w.writerow(("metric", "value"))
        for k, v in m.items():
            w.writerow((k, f"{v:.9g}" if isinstance(v, float) else v))

    p_md = os.path.join(outdir, "run_metadata.json")
    with open(p_md, "w", encoding="utf-8") as fh:
        json.dump(md, fh, indent=2, default=str)
    return p_csv, p_met, p_md


def plots(outdir, rows, m, md, ctx, is_stub):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as exc:                                    # noqa: BLE001
        print(f"  [plot] skipped: matplotlib unavailable ({type(exc).__name__}: {exc})")
        return []
    col = {c: i for i, c in enumerate(CSV_HEADER)}

    def f(n):
        return np.asarray([float(r[col[n]]) for r in rows], float)

    ph, t = f("gait_phase_pct"), f("time_s")
    sub = (f"Kp {md['kp_A_per_deg']:g} A/deg, Kd {md['kd_A_per_deg_per_s']:g} "
           f"A/(deg/s), current loop {md['current_loop_mode']}"
           + (f" tau_i {md['current_loop_tau_ms']:g} ms"
              if md["current_loop_mode"] != "ideal" else "")
           + f", k_t {md['k_t_output_choice']}, belt {md['belt_ratio_n_t']:.4f}")
    head = ("*** STUB -- NOT MUJOCO *** " if is_stub else "") + BANNER
    made = []

    def finish(fig, ax, name, ylab, title):
        import textwrap
        for a in (ax if isinstance(ax, (list, np.ndarray)) else [ax]):
            a.grid(alpha=0.25, lw=0.6)
            a.axvspan(0, STANCE_END, color="#ececeb", zorder=0)
        subtitle = '\n'.join(textwrap.wrap(sub, width=110))
        fig.suptitle(f"{head}\n{title}\n{subtitle}\n{DISCLAIMER}", fontsize=9,
                     fontweight="bold", color="#9a2b11" if is_stub else "black")
        fig.tight_layout(rect=(0, 0, 1, 0.84))
        p = os.path.join(outdir, name)
        fig.savefig(p, dpi=130)
        plt.close(fig)
        made.append(p)

    fig, ax = plt.subplots(figsize=(11, 4.6))
    ax.plot(ph, f("reference_deg"), color="#9a9aa4", lw=1.6, label="AB19 knee reference")
    ax.plot(ph, f("knee_angle_deg"), color="#1f4e8c", lw=1.4, label="simulated knee")
    ax.set_xlabel("gait phase [%]  (shaded 0-60 % = human stance, orientation only)")
    ax.set_ylabel("knee flexion [deg]")
    ax.legend(fontsize=8)
    finish(fig, ax, "gait_reference_vs_actual.png", "", "1  Reference vs actual knee angle")

    fig, ax = plt.subplots(figsize=(11, 4.0))
    ax.axhline(0, color="#b4b4ac", lw=0.8)
    ax.plot(ph, f("tracking_error_deg"), color="#9a2b11", lw=1.3)
    ax.set_xlabel("gait phase [%]")
    ax.set_ylabel("Tracking error [deg]\nreference - measured")
    ax.set_title(f"RMS {m['rms_error_deg']:.4f} deg | peak {m['peak_abs_error_deg']:.4f} "
                 f"deg | lag {m['best_fit_lag_ms']:.1f} ms | lag-removed RMS "
                 f"{m['rms_error_deg_lag_removed']:.4f} deg", fontsize=9)
    finish(fig, ax, "gait_tracking_error.png", "", "2  Tracking error (reference - measured)")

    fig, ax = plt.subplots(figsize=(11, 4.4))
    ax.plot(ph, f("current_request_A"), color="#8c4a1f", lw=0.9,
            label="PID request, pre-clip [A]")
    ax.plot(ph, f("current_applied_A"), color="#1f4e8c", lw=1.3,
            label="applied (clipped) command [A]")
    ax.plot(ph, f("current_actual_modelled_A"), color="#2c6e49", lw=1.1, ls="--",
            label="modelled actual, after the current loop [A]")
    for v, lab, c in ((md["current_clip_A"], "sim clip", "#9a2b11"),
                      (12.0, "reported rated", "#2c6e49"),
                      (28.0, "reported peak", "#c08a2e")):
        for s in (+1, -1):
            ax.axhline(s * v, color=c, lw=0.8, ls=":", label=lab if s > 0 else None)
    ax.set_xlabel("gait phase [%]")
    ax.set_ylabel("Current [A]\ncommanded / modelled, not measured")
    ax.legend(fontsize=7.5, ncol=3)
    finish(fig, ax, "gait_current.png", "", "3  Current: request, applied, modelled actual")

    fig, ax = plt.subplots(figsize=(11, 4.4))
    ax.plot(ph, f("belt_torque_actuator_side_Nm"), color="#8c4a1f", lw=1.1,
            label="belt torque, ACTUATOR side [N.m]")
    ax.plot(ph, f("estimated_actuator_output_torque_Nm"), color="#c08a2e", lw=0.9, ls=":",
            label="I_q x k_t_output = ESTIMATED actuator-output torque [N.m]")
    ax.set_xlabel("gait phase [%]")
    ax.set_ylabel("actuator-side torque [N.m]")
    ax.legend(fontsize=7.5, loc="upper left")
    axb = ax.twinx()
    axb.plot(ph, f("belt_torque_joint_side_Nm"), color="#1f4e8c", lw=1.4,
             label="belt torque, JOINT side [N.m]")
    axb.set_ylabel("joint-side torque [N.m]", color="#1f4e8c")
    axb.legend(fontsize=7.5, loc="lower right")
    finish(fig, ax, "gait_torque.png", "",
           "4  Torque, actuator side and joint side (separate scales, they differ by n_t)")

    ths = f("theta_s_rad")
    pk = max(float(np.max(np.abs(ths))), 1e-9)
    fig, ax = plt.subplots(figsize=(11, 4.0))
    near = pk > 0.2 * FIT_EDGE_RAD
    lim = max(FIT_EDGE_RAD * 1.4, pk * 1.2) if near else pk * 1.3
    if near:
        ax.axhspan(FIT_EDGE_RAD, lim, color="#efe3c8",
                   label=f"|theta_s| > {FIT_EDGE_RAD:g} rad = outside the paper's fit")
        ax.axhspan(-lim, -FIT_EDGE_RAD, color="#efe3c8")
    ax.plot(ph, ths, color="#2c6e49", lw=1.3,
            label=f"belt deflection theta_s [rad], peak {pk:.3e}"
                  + ("" if near else f"  ({pk / FIT_EDGE_RAD:.3f}x the fit edge, "
                                     f"off-scale here)"))
    ax.set_ylim(-lim, lim)
    ax.set_xlabel("gait phase [%]")
    ax.set_ylabel("belt deflection [rad]")
    ax.legend(fontsize=7.5)
    finish(fig, ax, "gait_belt_deflection.png", "", "5  Belt deflection")
    return made


# =====================================================================================
def view(ctx, args):
    """Compatibility entry point for the unified current-controlled viewer."""
    from oslbench.current_gait_live import run_live
    return run_live(ctx, args)


# =====================================================================================
def main() -> int:
    ap = argparse.ArgumentParser(description=f"{BANNER}. {DISCLAIMER}")
    ap.add_argument("--csv", default=DEFAULT_REFERENCE_CSV)
    ap.add_argument("--kp", type=float, default=0.75, help="A/deg (ankle-script value)")
    ap.add_argument("--ki", type=float, default=0.0, help="A/(deg.s)")
    ap.add_argument("--kd", type=float, default=0.03, help="A/(deg/s)")
    ap.add_argument("--current-loop", choices=("ideal", "first_order"), default="ideal",
                    help="'ideal' (default) isolates the outer loop and drivetrain")
    ap.add_argument("--current-loop-tau-ms", type=float, default=0.0)
    ap.add_argument("--current-clip", type=float, default=60.0,
                    help="SIMULATION-ONLY current clip in A. Not a safety limit.")
    ap.add_argument("--kt", choices=sorted(TORQUE_CONSTANT_CHOICES),
                    default="manual_rotor")
    ap.add_argument("--belt", choices=("cad", "paper"), default="cad")
    ap.add_argument("--derivative-on", choices=("error", "measurement"), default="error")
    ap.add_argument("--feedback-mode", choices=("legacy", "actuator-difference"), default="legacy",
                    help="actuator-difference selects verified Variant C (requires Ki=0 and derivative-on=error)")
    ap.add_argument("--dashboard", choices=("web", "none"), default="web")
    ap.add_argument("--port", type=int, default=8789)
    ap.add_argument("--no-browser", action="store_true")
    ap.add_argument("--start-paused", action="store_true")
    ap.add_argument("--exit-on-complete", action="store_true")
    ap.add_argument("--init", choices=("aligned", "zero"), default="aligned")
    ap.add_argument("--cycles", type=int, default=1)
    ap.add_argument("--backend", choices=("mujoco", "stub"), default="mujoco")
    ap.add_argument("--outdir", default=None)
    ap.add_argument("--tag", default=None, help="override the auto output tag")
    ap.add_argument("--view", action="store_true", help="live MuJoCo viewer (no files)")
    ap.add_argument("--speed", type=float, default=0.5, choices=(0.25, 0.5, 1.0))
    ap.add_argument("--max-cycles", type=int, default=0)
    ap.add_argument("--zoom", type=float, default=0.45)
    ap.add_argument("--elevation", type=float, default=-6.0)
    ap.add_argument("--azimuth", type=float, default=None)
    ap.add_argument("--distance", type=float, default=None)
    ap.add_argument("--no-plot", action="store_true")
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args()

    tau = args.current_loop_tau_ms
    if not math.isfinite(tau) or tau < 0.0:
        print(f"REFUSING TO RUN: --current-loop-tau-ms must be finite and >= 0, got {tau!r}")
        return 2
    if args.current_loop == "first_order" and tau == 0.0:
        print("NOTE: --current-loop first_order with tau = 0 ms is identical to 'ideal'.")

    mj, backend, is_stub = require_backend(args.backend)
    print("=" * 78)
    print(f"  {BANNER}")
    print(f"  {DISCLAIMER}")
    if is_stub:
        print("  *** STUB -- NOT MUJOCO. Wiring check only, not a simulation result. ***")
    print("=" * 78)
    print(f"  backend   {backend}")

    ctx = build(args, mj, is_stub)
    if ctx is None:
        return 1
    ident = ctx["ident"]
    print(f"  loaded    {ident['model_path']}  sha256[:16] {ident['model_xml_sha256']}")
    print(f"            nq={ident['nq']} nv={ident['nv']} nu={ident['nu']} "
          f"ngeom={ident['ngeom']} nmesh={ident['nmesh']}  "
          f"dt={ident['timestep_s'] * 1e3:.3f} ms")
    print(f"            knee armature={ident['knee_armature']:g} "
          f"damping={ident['knee_damping']:g} "
          f"frictionloss={ident['knee_frictionloss']:g}  matches the MJCF: "
          f"{ident['knee_properties_match_mjcf']}")
    print(f"  joint     KNEE (qpos {ctx['bench'].knee_qpos}, dof {ctx['bench'].knee_dof}), "
          f"authority +/-{ctx['bench'].force_limit:.1f} N.m")
    print(f"  reference {SUBJECT} knee angle, {ctx['ref'].n} rows, cycle "
          f"{ctx['ref'].period_s:.4f} s, first sample "
          f"{ctx['ref'].angle_deg[0]:.4f} deg -> {ctx['n']} steps")
    for line in ctx["cfg"].describe():
        print(f"  cfg       {line}")
    print(f"  schedule  {CONTROL_HZ:g} Hz controller / {1 / ctx['dt']:g} Hz physics "
          f"-> {ctx['steps_per_update']} step(s) per update, command held")
    print(f"  init      {args.init}")
    print(f"  k_t       {ctx['audit'].k_t_output_Nm_per_A:.6f} N.m/A at the ACTUATOR "
          f"OUTPUT ({args.kt})  -- WORKING HYPOTHESIS")
    print(f"  belt      n_t {ctx['audit'].belt_ratio:.6f} ({args.belt})")

    if args.view:
        if is_stub:
            print("\nREFUSING TO VIEW: the stub has no renderer. Use real MuJoCo.")
            return 1
        view(ctx, args)
        return 0

    rows = run(ctx, args)
    m = metrics(rows, ctx, args)
    loop_mode = "ideal" if ctx["cfg"].current_loop_tau_s == 0.0 else "first_order"
    md = metadata(ctx, args, loop_mode)
    m.update({f"meta_{k}": v for k, v in md.items()})

    tag = args.tag or (
        f"kp{args.kp:g}_kd{args.kd:g}_{loop_mode}"
        + (f"_tau{tau:g}ms" if loop_mode != "ideal" else "")
        + f"_{args.kt}_{args.belt}_{args.init}"
        + ("_C" if args.feedback_mode == "actuator-difference" else "") + ("_STUB" if is_stub else ""))
    outdir = args.outdir or os.path.join(OUTROOT, tag)
    if os.path.exists(outdir) and not args.outdir:
        i = 2
        while os.path.exists(f"{outdir}_{i}"):
            i += 1
        outdir = f"{outdir}_{i}"
        print(f"  note      output directory existed; writing to {os.path.basename(outdir)} "
              f"instead of overwriting")

    print("-" * 78)
    print(f"  servo     knee position actuator contributed "
          f"{m['knee_servo_nonzero_steps']} nonzero force(s) over {len(rows)} steps, "
          f"peak |force| {m['knee_servo_peak_abs_force_Nm']:.3e} N.m")
    if m["knee_servo_nonzero_steps"]:
        print("\nREFUSING TO REPORT METRICS: the knee servo acted, so the servo and the "
              "belt both drove the joint and every number would be a double count.")
        return 1
    print(f"            that zero is the proof the belt is the only actuation")
    print(f"  stable    numerically stable: {m['numerically_stable']}   "
          f"inside authored ROM: {m['stayed_inside_authored_rom']} "
          f"({m['knee_min_deg']:.2f}..{m['knee_max_deg']:.2f} deg)")
    print("\n  TRACKING")
    for k in ("rms_error_deg", "rms_error_deg_after_50ms", "mae_error_deg",
              "peak_abs_error_deg", "final_error_deg", "best_fit_lag_ms",
              "rms_error_deg_lag_removed", "pct_error_from_lag"):
        print(f"    {k:38s} {m[k]:.6g}")
    print("  CURRENT   (commanded / modelled; never measured)")
    for k in ("peak_current_request_A", "peak_current_applied_A",
              "peak_current_actual_modelled_A", "saturation_duration_s",
              "saturation_pct", "peak_applied_current_envelope"):
        v = m[k]
        print(f"    {k:38s} {v:.6g}" if isinstance(v, float) else f"    {k:38s} {v}")
    print("  TORQUE    (frame in the name; never compare across frames)")
    for k in ("peak_implied_rotor_torque_Nm",
              "peak_estimated_actuator_output_torque_Nm",
              "peak_belt_torque_ACTUATOR_side_Nm", "peak_belt_torque_JOINT_side_Nm",
              "pct_of_knee_authority", "max_abs_theta_s_rad",
              "theta_s_as_multiple_of_paper_fit_edge"):
        print(f"    {k:38s} {m[k]:.6g}")

    p_csv, p_met, p_md = write_outputs(outdir, rows, m, md, is_stub)
    pl = [] if args.no_plot else plots(outdir, rows, m, md, ctx, is_stub)
    print("\n  outputs")
    for p in [p_csv, p_met, p_md] + pl:
        print(f"    {os.path.relpath(p, ROOT)}")

    ok = ctx["bench"].limits_unchanged()
    print("-" * 78)
    print(f"  [F] authored knee forcerange/ctrlrange unchanged: {ok}; the MJCF was "
          f"never opened for writing")
    print("\n  HOW TO READ THIS")
    print("  This is a CURRENT-domain controller. run_bench_ab19.py is a TORQUE-domain")
    print("  position servo at 600 N.m/rad. Comparing their RMS errors compares two")
    print("  control PATHWAYS, not two tunings. The gains here came from the ANKLE")
    print("  step-test script and are NOT hardware-confirmed for the knee.")
    print(f"  {DISCLAIMER}")
    if is_stub:
        print("  *** STUB -- NOT MUJOCO. Re-run in .venv for a simulation result. ***")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
