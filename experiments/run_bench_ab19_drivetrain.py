#!/usr/bin/env python3
"""
run_bench_ab19_drivetrain.py -- THE ACTUATOR-MODEL COMPARISON.  Entry point only.

THE ONE QUESTION THIS ANSWERS
    The existing benchmark drives the CAD knee with an IDEAL joint-torque source: a
    MuJoCo position actuator that produces whatever torque the PD law asks for, with no
    rotor inertia, no gearbox friction and no compliance between drive and joint.  This
    script runs the SAME reference, the SAME gains and the SAME model with the Best et
    al. 2025 actuator and belt model in the path instead, and reports what changed.

        Case A   drivetrain OFF -- the existing servo benchmark, unmodified
        Case B   drivetrain ON  -- PD request -> current -> actuator shaft -> belt -> knee

    Kp = 600 N.m/rad and Kd = 17.253 N.m.s/rad are IDENTICAL in both cases and are not
    touched.  This is an actuator-model comparison, NOT controller tuning.  If Case B
    tracks worse, that is the measurement, not a problem to tune away.

WHAT IS NEW IN CASE B, AND WHY IT IS THE POINT
    Eight signals exist in Case B that have no counterpart in Case A at all: the
    commanded current I_q, the actuator-output angle and velocity theta_a / theta_a_dot,
    the belt deflection theta_s, and the torques tau_m / tau_f / tau_a / tau_j.  The
    comparison table can only show the shared metrics; the new channels are in the CSV
    and in the figure.

WHY EACH CASE LOADS ITS OWN COPY OF THE MODEL
    Case B disconnects the knee position actuator by writing zeros into the COMPILED
    mjModel.  Sharing one model object between the cases would make the result depend
    on the order they run in, which is exactly the kind of hidden coupling that
    produces a confident wrong number.  Two loads cost a second and remove the question.

NOTHING HERE IS A HARDWARE RESULT
    The actuator parameters are PAPER-DERIVED from Best et al. 2025 (IEEE/ASME T-MECH
    30(6):4732-4743) and are NOT measurements of this hardware.  The reported torques
    are BENCH ACTUATOR and BELT torques on a fixed-base bench carrying no body weight,
    and none of them is a human knee moment.  NOT YET VALIDATED ON HARDWARE.

USAGE (Windows PowerShell)
    .venv\\Scripts\\python.exe experiments\\run_bench_ab19_drivetrain.py
    .venv\\Scripts\\python.exe experiments\\run_bench_ab19_drivetrain.py --no-plot
"""

from __future__ import annotations

import argparse
import math
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from oslbench import logging as blog                                      # noqa: E402
from oslbench.controller import KD, KP, PDController                      # noqa: E402
from oslbench.drivetrain import PAPER                                     # noqa: E402
from oslbench.drivetrain_sim import (DrivetrainBenchSimulation,           # noqa: E402
                                     DrivetrainLayer, PDCurrentSource)
from oslbench.metrics import (bench_metrics, lag_diagnostic,              # noqa: E402
                              tracking_metrics)
from oslbench.model import load_bench_model                               # noqa: E402
from oslbench.reference import (SUBJECT, TRIAL, audit_reference,          # noqa: E402
                                load_reference, resample_reference)
from oslbench.simulation import BenchSimulation                           # noqa: E402

CSV_A = "bench_track_ab19_servo.csv"
CSV_B = "bench_track_ab19_drivetrain.csv"
MET_B = "bench_track_ab19_drivetrain_metrics.csv"


# --------------------------------------------------------------------- the two runs
def run_case_a(args, res, n, audit):
    """The existing servo benchmark, run exactly as experiments/run_bench_ab19.py does.

    Deliberately NOT a call into that script: it writes its own CSV at its own path and
    would overwrite the frozen benchmark output.  The simulation code is the same
    objects either way, so there is no second implementation of anything here.
    """
    bench = load_bench_model(verbose=False)
    if bench.failures:
        return None, None, None, bench.failures
    sim = BenchSimulation(bench, PDController.knee(bench, args.kp, args.kv))
    log = blog.BenchLog(bench.force_limit)
    result = sim.run(res["ref_rad"], n, ref_vel0=float(res["ref_vel_rad_s"][0]),
                     on_step=lambda st: log.append(st, res, st.k))
    if not bench.limits_unchanged():
        return None, None, None, ["Case A: authored limits changed during the run"]
    return bench, result, log, []


def run_case_b(args, res, n, audit):
    """The same reference and gains, with the actuator layer in the path.

    The PD law is reused VERBATIM through PDCurrentSource: same gains, same reference,
    same position error signal, same `torque_unclamped` call.  Two things differ from
    Case A, and only two:

      1. the PATH -- the law's output becomes a current, which drives the actuator shaft
         through J_a / B_a / friction, and the joint feels only what the belt transmits;
      2. the DERIVATIVE FEEDBACK POINT -- Kd now multiplies theta_a_dot/n_t (the
         actuator's own velocity, referred to the joint) instead of theta_j_dot.

    (2) is not a tuning choice.  With theta_j_dot the loop is non-collocated across the
    belt and diverges at t = 0.3485 s on real MuJoCo at these exact gains; see
    `docs/DRIVETRAIN_INSTABILITY_DIAGNOSIS.md`.  The gains themselves are UNCHANGED:
    Kp = 600, Kd = 17.253.

    The knee position actuator is disconnected so the belt is the only actuation --
    otherwise both would act on the joint in the same mj_step and the result would be a
    double-count rather than a measurement.
    """
    bench = load_bench_model(verbose=False)
    if bench.failures:
        return None, None, None, None, bench.failures
    layer = DrivetrainLayer(PAPER, enabled=True, substeps=1)
    pd = PDController.knee(bench, args.kp, args.kv)
    # The SAME layer instance goes to the current source and to the simulation.  If
    # these were two different DrivetrainLayer objects the derivative term would damp a
    # shaft that nothing was driving, which would look stable and mean nothing.
    src = PDCurrentSource(pd, res["ref_rad"], layer, PAPER)
    sim = DrivetrainBenchSimulation(bench, pd, layer=layer, current_source=src,
                                    disconnect_servo=True)
    if sim.servo_connected:
        return None, None, None, None, ["Case B: the servo did not disconnect"]
    if src.layer is not sim.layer:
        return None, None, None, None, ["Case B: current source and simulation hold "
                                        "different DrivetrainLayer instances"]
    log = blog.DrivetrainBenchLog(bench.force_limit)
    log.servo_violations = 0

    def on_step(st):
        servo = sim.knee_servo_force()
        if servo != 0.0:
            log.servo_violations += 1
        log.append(st, res, st.k, layer=sim.last_layer,
                   request_Nm=src.last_request_Nm, servo_Nm=servo)

    result = sim.run(res["ref_rad"], n, ref_vel0=float(res["ref_vel_rad_s"][0]),
                     on_step=on_step)
    if not bench.limits_unchanged():
        return None, None, None, None, ["Case B: authored limits changed during the run"]
    return bench, result, log, sim, []


# ------------------------------------------------------------------- the comparison
# The belt law was fitted by Best et al. out to roughly this deflection (their Fig. 3
# abscissa).  Past it, rho(theta_s) is EXTRAPOLATION of their regression, so a torque
# quoted from beyond here is not a belt measurement.  ASSUMED: read off a figure axis.
FIT_EDGE_RAD = 0.055
# A run that is not ringing still has a nonzero detrended residual.  Below this
# oscillatory fraction no frequency is reported, because one read off a tracking
# residual is noise.  JUDGEMENT CALL, same value as the diagnostic script uses.
OSC_GATE = 0.35
# Joint-referred actuator inertia and the knee's own inertia, for the 2-mass belt-mode
# prediction.  Both DERIVED: J_ar = n_t^2*J_a from the paper; I_J measured once from the
# compiled model (body 0.251998 + armature 0.01) and transcribed.
J_AR = PAPER.n_t ** 2 * PAPER.J_a
I_J = 0.261998


def _col(log, name):
    """One logged column as floats.  Blank cells (drivetrain OFF rows) are skipped."""
    i = blog.DRIVETRAIN_CSV_HEADER.index(name)
    out = []
    for r in log.rows():
        v = r[i]
        if v != "":
            out.append(float(v))
    return out


def stability_verdict(result, theta_s, tau_j, n_planned, force_limit):
    """Three outcomes, not two.  Finishing the loop is not the same as being stable.

    A gait cycle is 1.205 s.  A mode with a growth rate of +0.8 /s grows by less than
    3x in that time, so a binary completed/diverged test scores it "stable" for want of
    time rather than for want of a growing mode.  MARGINAL is for runs that finished but
    went somewhere the model cannot be quoted from: past the authored knee authority, or
    outside the deflection range the paper actually fitted.

    WHY THIS DOES NOT TEST THE STEP COUNT
        `BenchSimulation.run` always iterates the full `range(n)` and MuJoCo does not
        raise on divergence -- it fills `qacc` with NaN and carries on.  So the logged
        arrays are ALWAYS n long and a "stopped after X of N steps" test could never
        fire.  Divergence is detected by finiteness instead, and the index of the first
        non-finite sample is reported as the time it happened, which is the number the
        diagnostic's abort guards would have stopped at.
    """
    def first_bad(seq):
        for i, x in enumerate(seq):
            if not math.isfinite(x):
                return i
            if abs(x) > 1e12:           # pre-NaN runaway: finite but meaningless
                return i
        return None

    bad = [b for b in (first_bad(theta_s), first_bad(tau_j), first_bad(result.q))
           if b is not None]
    diverged_at = min(bad) if bad else None
    finite = diverged_at is None
    good = len(theta_s) if finite else diverged_at
    max_ts = max((abs(x) for x in theta_s[:good]), default=0.0)
    max_tj = max((abs(x) for x in tau_j[:good]), default=0.0)
    inside_fit = max_ts <= FIT_EDGE_RAD
    reasons = []
    if len(theta_s) < n_planned:
        reasons.append(f"only {len(theta_s)} of {n_planned} steps were logged")
    if not finite:
        reasons.append(f"a non-finite or runaway value appeared at step {diverged_at}; "
                       f"every number after it is meaningless")
    if not inside_fit:
        reasons.append(f"|theta_s| reached {max_ts:.6f} rad, past the "
                       f"{FIT_EDGE_RAD} rad fitted range -- torques beyond it are "
                       f"extrapolation of the paper's regression")
    if max_tj > force_limit:
        reasons.append(f"|tau_j| reached {max_tj:.2f} N.m, above the authored "
                       f"+/-{force_limit:.1f} N.m knee authority")
    if not finite:
        verdict = "UNSTABLE"
    elif reasons:
        verdict = "MARGINAL"
    else:
        verdict = "STABLE (completed, inside authority and inside the fitted range)"
    return dict(verdict=verdict, reasons=reasons, finite=finite,
                completed=(len(theta_s) >= n_planned), diverged_at=diverged_at,
                inside_fit=inside_fit, max_theta_s=max_ts, max_tau_j=max_tj,
                n_steps=len(theta_s), n_good=good)


def belt_mode(theta_s, k_s_series, dt, frac=0.35):
    """Measured vs predicted belt-mode frequency over the last `frac` of the run.

    Measured by detrending theta_s (subtract a straight line, so the slow tracking
    signal does not masquerade as a half cycle) and counting hysteretic zero crossings.
    Predicted from the same 2-mass model the diagnosis used:

        f = sqrt(K_s * (1/J_ar + 1/I_j)) / (2*pi)

    K_s = p1 + 2*p2*|theta_s| hardens with amplitude, so the prediction is a BAND --
    mean K_s and peak K_s over the window -- and the measured value is expected to sit
    inside it rather than on a point.  The percentage difference is quoted against the
    mean-K_s prediction and is NOT forced toward it.
    """
    m = len(theta_s)
    if m < 40:
        return dict(is_mode=False, osc_frac=0.0, f_meas=0.0, f_pred_mean=0.0,
                    f_pred_peak=0.0, k_s_mean=0.0, pct=0.0, sigma=0.0, half_cycles=0)
    i0 = max(0, int(m * (1.0 - frac)))
    w = theta_s[i0:]
    k = len(w)
    # detrend: least-squares straight line, computed without numpy so this stays cheap
    xm = (k - 1) / 2.0
    ym = sum(w) / k
    sxx = sum((i - xm) ** 2 for i in range(k))
    sxy = sum((i - xm) * (w[i] - ym) for i in range(k))
    slope = sxy / sxx if sxx > 0 else 0.0
    d = [w[i] - (ym + slope * (i - xm)) for i in range(k)]
    rms_d = math.sqrt(sum(x * x for x in d) / k)
    rms_w = math.sqrt(sum((x - ym) ** 2 for x in w) / k)
    osc_frac = (rms_d / rms_w) if rms_w > 0 else 0.0
    # hysteretic zero crossings of the detrended signal -> half cycles
    thr = 0.1 * rms_d
    crossings, state, peaks = [], 0, []
    run_max = 0.0
    for i, x in enumerate(d):
        run_max = max(run_max, abs(x))
        if state <= 0 and x > thr:
            if state < 0:
                crossings.append(i)
                peaks.append(run_max)
                run_max = 0.0
            state = 1
        elif state >= 0 and x < -thr:
            if state > 0:
                crossings.append(i)
                peaks.append(run_max)
                run_max = 0.0
            state = -1
    half = len(crossings) - 1
    f_meas = 0.0
    if half >= 1:
        f_meas = 0.5 / (((crossings[-1] - crossings[0]) / half) * dt)
    # envelope growth rate from successive half-cycle peaks
    sigma = 0.0
    if len(peaks) >= 3 and peaks[0] > 0 and peaks[-1] > 0:
        span = (crossings[-1] - crossings[0]) * dt
        if span > 0:
            sigma = math.log(peaks[-1] / peaks[0]) / span
    if len(k_s_series) != m:
        # Refuse rather than guess.  Falling back to the whole K_s series would compute
        # the prediction over a different window than the measurement and report the
        # percentage difference with full confidence -- a wrong answer, not an error.
        # K_s hardens with amplitude, so the two windows are not interchangeable.
        return dict(is_mode=False, osc_frac=osc_frac, f_meas=f_meas, f_pred_mean=0.0,
                    f_pred_peak=0.0, k_s_mean=0.0, pct=0.0, sigma=sigma,
                    half_cycles=half,
                    note=f"K_s series is {len(k_s_series)} long but theta_s is {m}; "
                         f"no prediction made")
    ks_w = k_s_series[i0:]
    k_s_mean = sum(ks_w) / len(ks_w) if ks_w else PAPER.p1
    k_s_peak = max(ks_w) if ks_w else PAPER.p1

    def f_of(ks):
        return math.sqrt(ks * (1.0 / J_AR + 1.0 / I_J)) / (2.0 * math.pi)

    f_pred_mean, f_pred_peak = f_of(k_s_mean), f_of(k_s_peak)
    pct = 100.0 * (f_meas - f_pred_mean) / f_pred_mean if f_pred_mean > 0 else 0.0
    return dict(is_mode=(osc_frac > OSC_GATE and half >= 2), osc_frac=osc_frac,
                f_meas=f_meas, f_pred_mean=f_pred_mean, f_pred_peak=f_pred_peak,
                k_s_mean=k_s_mean, pct=pct, sigma=sigma, half_cycles=half)


def col(name, a, b, unit="", fmt="{:9.4f}", note=""):
    """One comparison row: Case A, Case B, and the change, as text."""
    sa = "--" if a is None else fmt.format(a)
    sb = "--" if b is None else fmt.format(b)
    if a is None or b is None:
        sd = "--"
    elif abs(a) < 1e-12:
        sd = "n/a" if abs(b) < 1e-12 else "from 0"
    else:
        sd = f"{100.0 * (b - a) / abs(a):+8.2f} %"
    return f"  {name:<30s} {sa:>12s} {sb:>12s} {sd:>10s}  {unit:<9s} {note}"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--csv", default=None, help="AB19 reference CSV")
    ap.add_argument("--outdir", default=blog.DEFAULT_OUTDIR)
    ap.add_argument("--kp", type=float, default=KP,
                    help="N.m/rad -- IDENTICAL in both cases; do not tune here")
    ap.add_argument("--kv", type=float, default=KD, help="N.m.s/rad (= Kd)")
    ap.add_argument("--cycles", type=int, default=1)
    ap.add_argument("--interp", choices=("cubic", "linear"), default="cubic")
    ap.add_argument("--no-plot", action="store_true")
    args = ap.parse_args()
    os.makedirs(args.outdir, exist_ok=True)

    print("=" * 78)
    print(f"ACTUATOR-MODEL COMPARISON on the CAD bench -- {SUBJECT}")
    print(f"  reference  : {args.csv or 'build/AB19_knee_gait_reference.csv'}")
    print(f"  trial      : {TRIAL}")
    print(f"  controller : kp={args.kp:.1f} N.m/rad  kv={args.kv:.3f} N.m.s/rad  "
          f"-- IDENTICAL in both cases")
    print(f"  Case A     : drivetrain OFF, MuJoCo position actuator (ideal torque "
          f"source)")
    print(f"  Case B     : drivetrain ON, Best et al. 2025 actuator + belt, servo "
          f"disconnected")
    print("=" * 78)

    bench0 = load_bench_model()
    if bench0.failures:
        print(f"\nREFUSING TO RUN: {len(bench0.failures)} preflight failure(s)")
        for f in bench0.failures:
            print(f"  - {f}")
        return len(bench0.failures)
    dt = bench0.timestep
    lo_deg, hi_deg = (math.degrees(x) for x in bench0.knee_ctrlrange)

    ref = load_reference(args.csv)
    audit = audit_reference(ref, lo_deg, hi_deg)
    res = resample_reference(ref, dt, args.cycles, args.interp, audit=audit)
    n = res.n
    print(f"\n    reference resampled to {n} samples at dt = {dt} s")

    print("\n[A] CASE A -- drivetrain OFF (the existing servo benchmark)")
    bench_a, result_a, log_a, fail = run_case_a(args, res, n, audit)
    if fail:
        for f in fail:
            print(f"  - {f}")
        return 1
    MA = bench_metrics(result_a, res, dt)
    print(f"    done: RMS {MA['rms_err_deg']:.4f} deg, peak |tau| "
          f"{MA['peak_tau_Nm']:.4f} N.m")

    print("\n[B] CASE B -- drivetrain ON (paper actuator + belt, servo disconnected)")
    bench_b, result_b, log_b, sim_b, fail = run_case_b(args, res, n, audit)
    if fail:
        for f in fail:
            print(f"  - {f}")
        return 1
    MB = bench_metrics(result_b, res, dt)
    print(f"    done: RMS {MB['rms_err_deg']:.4f} deg, peak |I_q| "
          f"{log_b.peak_i_q:.4f} A")

    # ---- the servo-off evidence, before any metric is quoted ----------------------
    print("\n[C] THE SERVO IS OFF IN CASE B, as data and not as a claim")
    print(f"    steps with nonzero position-actuator force : {log_b.servo_violations} "
          f"of {n}")
    print(f"    peak |position-actuator force|             : {log_b.peak_servo:.6e} N.m")
    print(f"    bench_metrics peak_tau_Nm for Case B       : {MB['peak_tau_Nm']:.6e} N.m")
    print("    bench_metrics reads the actuatorfrc sensor on knee_pos, so for Case B it")
    print("    MUST read 0. That zero is the PROOF the servo is disconnected, not a")
    print("    missing measurement -- the Case B joint torque is tau_j, below.")
    if log_b.servo_violations or log_b.peak_servo != 0.0:
        print("    *** THE SERVO IS STILL ACTING. The comparison below would be a "
              "DOUBLE-COUNT. ***")
        return 1

    # ---- Case B's joint-side numbers, from tau_j rather than from the dead sensor --
    C = blog.read_bench_csv  # noqa: F841  (named for the reader; CSV read happens below)
    tau_j = [float(r[blog.DRIVETRAIN_CSV_HEADER.index("tau_j_Nm")])
             for r in log_b.rows()]
    err_deg_b = [float(r[blog.DRIVETRAIN_CSV_HEADER.index("err_deg")])
                 for r in log_b.rows()]
    qdot_b = [float(r[blog.DRIVETRAIN_CSV_HEADER.index("sim_vel_rad_s")])
              for r in log_b.rows()]
    TB = tracking_metrics(err_deg_b, tau_j, qdot_b, bench_b.force_limit,
                          [0] * len(tau_j))
    LB = lag_diagnostic(result_b.q, res["ref_rad"][:n], dt, TB["rms_err_deg"])

    # ---- the actuator-side state, which exists only in Case B ----------------------
    theta_s = _col(log_b, "theta_s_rad")
    theta_a_dot = _col(log_b, "theta_a_dot_rad_s")
    k_s_series = _col(log_b, "K_s_Nm_rad")
    i_q_series = _col(log_b, "i_q_A")
    stab = stability_verdict(result_b, theta_s, tau_j, n, bench_b.force_limit)
    mode = belt_mode(theta_s, k_s_series, dt)

    print("\n[D] CASE B STABILITY, as a verdict with its reasons attached")
    print(f"    steps completed                : {stab['n_steps']} of {n}"
          f"  ({'FULL CYCLE' if stab['completed'] else 'SHORT'})")
    print(f"    all states finite              : {'yes' if stab['finite'] else 'NO'}")
    print(f"    max |belt deflection|          : {stab['max_theta_s']:.6f} rad"
          f"  = {stab['max_theta_s'] / FIT_EDGE_RAD:.2f}x the {FIT_EDGE_RAD} rad fit edge")
    print(f"    inside the paper's fitted range: "
          f"{'yes' if stab['inside_fit'] else 'NO -- torques beyond it are extrapolation'}")
    print(f"    max |joint torque| vs authority: {stab['max_tau_j']:.4f} N.m"
          f"  = {100.0 * stab['max_tau_j'] / bench_b.force_limit:.1f} % of "
          f"+/-{bench_b.force_limit:.1f} N.m")
    print(f"    VERDICT                        : {stab['verdict']}")
    if stab["reasons"]:
        for r in stab["reasons"]:
            print(f"      - {r}")
    print("    A completed run is NOT automatically a stable one: a cycle can end before")
    print("    a slow mode has had time to grow. The reasons above are what the verdict")
    print("    is made of, so a reader can disagree with it on the evidence.")

    print("\n[E] BELT-MODE BEHAVIOUR in Case B")
    if mode["is_mode"]:
        print(f"    coherent oscillation in theta_s : yes (oscillatory RMS is "
              f"{mode['osc_frac']:.2f} of total, gate {OSC_GATE})")
        print(f"    measured dominant frequency     : {mode['f_meas']:.3f} Hz "
              f"over {mode['half_cycles']} half cycles")
        print(f"    predicted from the 2-mass model : {mode['f_pred_mean']:.3f} Hz at "
              f"mean K_s = {mode['k_s_mean']:.1f} N.m/rad  "
              f"({mode['pct']:+.2f} % vs measured)")
        print(f"    predicted at peak K_s           : {mode['f_pred_peak']:.3f} Hz "
              f"(the belt HARDENS with amplitude, so a band is the honest prediction)")
        print(f"    growth rate of the envelope     : {mode['sigma']:+.3f} /s "
              f"({'GROWING' if mode['sigma'] > 0 else 'decaying'})")
    else:
        print(f"    coherent oscillation in theta_s : NO MODE (oscillatory RMS is "
              f"{mode['osc_frac']:.2f} of total, below the {OSC_GATE} gate)")
        print("    theta_s is dominated by the slow tracking signal, not by ringing.")
        print("    No frequency is quoted: one read off a tracking residual would be")
        print("    noise dressed as a measurement. THIS IS THE EXPECTED RESULT for a")
        print("    collocated derivative term -- the belt mode is damped, not excited.")

    # ---- the comparison table ------------------------------------------------------
    print("\n" + "=" * 78)
    print("COMPARISON -- Case A (drivetrain OFF) vs Case B (drivetrain ON)")
    print("=" * 78)
    print(f"  {'quantity':<30s} {'Case A':>12s} {'Case B':>12s} {'change':>10s}"
          f"  {'unit':<9s}")
    print("  " + "-" * 74)
    print("  TRACKING")
    print(col("RMS angle error", MA["rms_err_deg"], MB["rms_err_deg"], "deg"))
    print(col("peak angle error", MA["peak_err_deg"], MB["peak_err_deg"], "deg"))
    print(col("mean absolute error", MA["mae_err_deg"], MB["mae_err_deg"], "deg"))
    print(col("RMS error after 50 ms", MA["rms_err_deg_after_50ms"],
              MB["rms_err_deg_after_50ms"], "deg"))
    print(col("peak |angular velocity|", MA["peak_vel_rad_s"], MB["peak_vel_rad_s"],
              "rad/s", note=f"reference {MA['peak_ref_vel_rad_s']:.4f}"))
    print("  PHASE LAG  (diagnostic; changes nothing)")
    print(col("best-fit servo lag", MA["lag_ms"], LB["lag_ms"], "ms"))
    print(col("RMS with lag removed", MA["rms_err_deg_lag_removed"],
              LB["rms_err_deg_lag_removed"], "deg"))
    print(col("share of error that is lag", MA["pct_err_from_lag"],
              LB["pct_err_from_lag"], "%"))
    print("  JOINT-SIDE TORQUE   -- Case A: position actuator;  Case B: belt tau_j")
    print(col("peak |joint torque|", MA["peak_tau_Nm"], TB["peak_tau_Nm"], "N.m"))
    print(col("RMS joint torque", None, None, "N.m"))
    print(col("% of authored authority", MA["pct_authority"], TB["pct_authority"], "%",
              note=f"of +/-{bench_b.force_limit:.1f} N.m"))
    print(col("torque saturation", MA["sat_pct"], None, "%",
              note="Case B: forcerange cannot clamp qfrc_applied"))
    print("  WHAT ONLY EXISTS IN CASE B")
    print(col("stability verdict", None, None, "",
              note=f"A: STABLE (validated);  B: {stab['verdict'].split(' (')[0]}"))
    print(col("steps completed", float(n), float(stab["n_steps"]), "steps",
              fmt="{:9.0f}", note=f"of {n} planned"))
    print(col("max |belt deflection|", None, stab["max_theta_s"], "rad",
              fmt="{:9.6f}",
              note=f"{stab['max_theta_s'] / FIT_EDGE_RAD:.2f}x the {FIT_EDGE_RAD} fit edge"))
    print(col("max |actuator velocity|", None,
              max((abs(x) for x in theta_a_dot), default=0.0), "rad/s",
              note=f"joint-referred {max((abs(x) for x in theta_a_dot), default=0.0) / PAPER.n_t:.4f}"))
    print(col("peak |commanded current|", None, log_b.peak_i_q, "A",
              note="ASSUMED: no current limit"))
    print(col("belt-mode frequency", None,
              mode["f_meas"] if mode["is_mode"] else None, "Hz",
              note=("predicted band "
                    f"{mode['f_pred_mean']:.1f}-{mode['f_pred_peak']:.1f} Hz"
                    if mode["is_mode"] else "no coherent mode -- belt not ringing")))
    print(col("peak |position-actuator force|", MA["peak_tau_Nm"], log_b.peak_servo,
              "N.m", note="Case B must be exactly 0"))
    print("\n  Case B's saturation percentage is deliberately BLANK rather than 0: the")
    print("  +-142.2 N.m forcerange belongs to the actuator that has been disconnected,")
    print("  it cannot clamp qfrc_applied, and oslbench/drivetrain.py documents NO drive")
    print("  current limit from the paper. There is nothing to saturate against, so")
    print("  reporting 0 % would assert a limit the model does not have. Peak |I_q| is")
    print(f"  reported instead. For scale only, {bench_b.force_limit:.1f} N.m at "
          f"k_t_joint = {PAPER.n_t * PAPER.k_t * PAPER.n_a:.6f} N.m/A")
    print(f"  would be {bench_b.force_limit / (PAPER.n_t * PAPER.k_t * PAPER.n_a):.4f} A "
          f"-- a REFERENCE LINE, not a hardware current limit.")

    # ---- outputs -------------------------------------------------------------------
    prov_common = [
        f"subject {SUBJECT}; {TRIAL}",
        f"model {bench_b.relpath()} (UNMODIFIED on disk); kp={args.kp} kv={args.kv} "
        f"set at runtime; dt={dt}",
        "tau_sensor_Nm is BENCH ACTUATOR torque; human_knee_moment is a BIOMECHANICAL "
        "HUMAN moment. Different quantities, never combined.",
    ]
    p_a = blog.write_bench_csv(os.path.join(args.outdir, CSV_A), log_a,
                               prov_common + ["CASE A: drivetrain OFF, MuJoCo position "
                                              "actuator as an ideal torque source"])
    p_b = blog.write_bench_csv(
        os.path.join(args.outdir, CSV_B), log_b,
        prov_common + [
            "CASE B: drivetrain ON (Option C). Best et al. 2025 T-MECH "
            "30(6):4732-4743, eqs (1)-(12). Parameters PAPER-DERIVED, not measured here.",
            f"n_a={PAPER.n_a} n_t={PAPER.n_t} k_t={PAPER.k_t} J_a={PAPER.J_a} "
            f"B_a={PAPER.B_a} f_c={PAPER.f_c} f_g={PAPER.f_g} p1={PAPER.p1} "
            f"p2={PAPER.p2}",
            "knee position actuator DISCONNECTED at runtime (kp=kd=0); tau_j is "
            "injected through qfrc_applied; servo_force_Nm must be 0 on every row.",
            "tau_a_Nm is ACTUATOR-SIDE and tau_j_Nm is JOINT-SIDE; they differ by n_t "
            "and must not be compared directly. Neither is a human knee moment.",
        ])
    MBout = dict(MB)
    # `MB = bench_metrics(result_b, ...)` reads the actuatorfrc sensor on knee_pos, which
    # is identically 0 with the servo disconnected.  That zero is the servo-off PROOF,
    # but written to a metrics CSV as `peak_tau_Nm = 0.0` next to
    # `caseA_peak_tau_Nm = 16.0041` it reads as a measurement of no torque.  The console
    # explains this; `write_metrics_csv` emits no comments, so the explanation would not
    # travel with the file.  So the dead-sensor keys are RENAMED to say what they are and
    # self-describing rows are added.  Nothing is fabricated and nothing is hidden -- the
    # zeros are still there, under names that cannot be misread.
    # These are every torque/power key `bench_metrics` returns, all of which read 0 for
    # Case B.  Listed explicitly rather than pattern-matched so that a new metric added
    # upstream shows up as an un-renamed zero and gets noticed, instead of being
    # silently swept up by a regex.
    DEAD_SENSOR = ("peak_tau_Nm", "pct_authority", "sat_pct",
                   "peak_power_W", "mean_abs_power_W")
    for key in DEAD_SENSOR:
        if key in MBout:
            MBout[f"DEADSENSOR_{key}_servo_is_off"] = MBout.pop(key)
    MBout.update({f"tau_j_{k}": v for k, v in TB.items()})
    # `tracking_metrics` is handed an all-zero saturation flag array because
    # `forcerange` cannot clamp `qfrc_applied` and the paper documents no drive-current
    # limit.  There is nothing to saturate against, so a 0 % would assert a limit the
    # model does not have.  (`tau_j_saturated_steps` does not currently exist; popping
    # it is a no-op that costs nothing and survives an upstream rename.)
    for key in ("tau_j_sat_pct", "tau_j_saturated_steps"):
        MBout.pop(key, None)
    MBout.update({f"lag_{k}": v for k, v in LB.items()})
    MBout["peak_i_q_A"] = log_b.peak_i_q
    MBout["peak_servo_force_Nm"] = log_b.peak_servo
    MBout["servo_violation_steps"] = log_b.servo_violations
    MBout["NOTE_no_torque_saturation_metric"] = (
        "forcerange cannot clamp qfrc_applied and the paper documents no drive current "
        "limit; peak_i_q_A is reported instead of a saturation percentage")
    MBout["NOTE_derivative_feedback_side"] = PDCurrentSource.FEEDBACK_SIDE
    MBout["stability_verdict"] = stab["verdict"].split(" (")[0]
    MBout["max_abs_theta_s_rad"] = stab["max_theta_s"]
    MBout["theta_s_as_multiple_of_paper_fit_edge"] = stab["max_theta_s"] / FIT_EDGE_RAD
    MBout["max_abs_theta_a_dot_rad_s"] = max((abs(x) for x in theta_a_dot), default=0.0)
    MBout["belt_mode_is_coherent"] = int(mode["is_mode"])
    if mode["is_mode"]:
        MBout["belt_mode_measured_Hz"] = mode["f_meas"]
        MBout["belt_mode_predicted_Hz_at_mean_Ks"] = mode["f_pred_mean"]
        MBout["belt_mode_growth_sigma_per_s"] = mode["sigma"]
    for k, v in MA.items():
        MBout[f"caseA_{k}"] = v
    p_met = blog.write_metrics_csv(os.path.join(args.outdir, MET_B), MBout,
                                   audit.fields(), audit.flags)
    print(f"\noutputs\n  {p_a}\n  {p_b}\n  {p_met}")

    if not args.no_plot:
        from oslbench.plotting import plot_bench_results, plot_drivetrain_results
        plot_drivetrain_results(p_b, args.outdir, args.kp, args.kv, path_csv_servo=p_a)
        # plot_bench_results writes the FIXED names bench_track_ab19.png and
        # bench_track_ab19_human_reference.png, which are the frozen professor-facing
        # figures from the servo benchmark.  Rendering Case B into them would replace
        # them with a plot whose torque panel is identically zero (the actuatorfrc
        # sensor reads 0 with the servo disconnected -- that zero is the servo-off
        # PROOF, not a measurement) under a title that does not mention the drivetrain.
        # So Case B's copy goes to its own subdirectory.  DELIBERATE: this script must
        # not be able to overwrite the validated benchmark's output.
        panel_dir = os.path.join(args.outdir, "case_b_panels")
        os.makedirs(panel_dir, exist_ok=True)
        plot_bench_results(p_b, panel_dir, args.kp, args.kv, audit.flags)
        print(f"  Case B's standard panels -> {panel_dir}{os.sep}  (kept out of "
              f"{args.outdir}{os.sep} so the frozen servo figures survive)")

    print("\nWHAT THIS DOES NOT SHOW")
    print("  The actuator parameters are PAPER-DERIVED from Best et al. 2025 and are")
    print("  NOT measurements of this hardware. The belt of the real OSL V2 has never")
    print("  been measured here. Gains were held fixed on purpose, so Case B is NOT a")
    print("  tuned result and its tracking error is not the best this architecture can")
    print("  do. NOT YET VALIDATED ON HARDWARE.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
