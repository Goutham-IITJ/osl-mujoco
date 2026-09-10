#!/usr/bin/env python3
"""
run_bench_ab19.py -- THE QUANTITATIVE BENCH EXPERIMENT.  Entry point, not implementation.

WHAT IT DOES, in the order it does it:
      1  load models/osl_v2_bench.xml and verify it is the validated bench
      2  load the AB19 human knee-angle reference, audit it, resample it to the 0.5 ms grid
      3  create the PD controller (Kp = 600 N.m/rad, Kd = 17.253 N.m.s/rad)
      4  run the simulation, logging every step
      5  write the CSV, compute the metrics, write the metrics CSV
      6  draw the offline figures (skipped automatically if matplotlib is absent)

    Every one of those lines is one call into oslbench/.  The control law lives in
    oslbench/controller.py; the loop lives in oslbench/simulation.py.  Nothing is
    implemented in this file.

WHAT THE EXPERIMENT IS
    The CAD-derived OSL V2 knee, on a FIXED-BASE bench, tracking a human knee-angle
    trajectory.  It is joint-level trajectory tracking.

WHAT IT IS NOT
    Not whole-body walking.  The bench has no pelvis, no ground contact and no body
    weight, so the reported torque is BENCH ACTUATOR TORQUE, not a human knee moment.
    The hardware has not been validated against this simulation.

USAGE (Windows PowerShell)
    .venv\\Scripts\\python.exe experiments\\run_bench_ab19.py
    .venv\\Scripts\\python.exe experiments\\run_bench_ab19.py --no-plot
    .venv-analysis\\Scripts\\python.exe experiments\\plot_bench_results.py   (figures only)
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
from oslbench.metrics import bench_metrics                                # noqa: E402
from oslbench.model import load_bench_model                               # noqa: E402
from oslbench.reference import (SUBJECT, TRIAL, audit_reference,          # noqa: E402
                                load_reference, resample_reference)
from oslbench.simulation import BenchSimulation                           # noqa: E402


def report(M, audit, n, cycles, lo_deg, hi_deg, p_csv, p_met):
    """Print the professor-facing summary.  Reading only -- computes nothing."""
    print("\n" + "=" * 78)
    print("RESULTS")
    print("=" * 78)
    print(f"  RMS angle error            {M['rms_err_deg']:9.4f} deg")
    print(f"  peak angle error           {M['peak_err_deg']:9.4f} deg")
    print(f"  mean absolute error        {M['mae_err_deg']:9.4f} deg")
    print(f"  RMS  err after 50 ms       {M['rms_err_deg_after_50ms']:9.4f} deg  "
          f"(start transient excluded)")
    print(f"  peak err after 50 ms       {M['peak_err_deg_after_50ms']:9.4f} deg")
    print(f"  peak |actuator torque|     {M['peak_tau_Nm']:9.4f} N.m")
    print(f"  % of OSL torque authority  {M['pct_authority']:9.4f} %  "
          f"(limit +/-{M['force_limit_Nm']:.1f} N.m, UNCHANGED)")
    print(f"  torque saturation          {M['sat_pct']:9.4f} %  of steps")
    print(f"  peak |angular velocity|    {M['peak_vel_rad_s']:9.4f} rad/s  "
          f"(reference {M['peak_ref_vel_rad_s']:.4f})")
    print(f"  peak |actuator power|      {M['peak_power_W']:9.4f} W")
    print(f"  ankle drift from keyframe  {M['ankle_dev_deg']:9.4f} deg")
    print(f"  steps simulated            {M['n_steps']:9d}  "
          f"({M['final_time_s']:.4f} s = {cycles} gait cycle(s))")

    print("\nIS THE RESIDUAL ERROR LAG OR FAILURE?  (diagnostic, changes nothing)")
    print(f"  best-fit servo lag         {M['lag_ms']:9.4f} ms "
          f"({M['lag_pct_gc']:.2f} % of the gait cycle)")
    print(f"  RMS error with lag removed {M['rms_err_deg_lag_removed']:9.4f} deg  "
          f"<- vs {M['rms_err_deg']:.4f} deg raw")
    print(f"  share of RMS error that is pure time shift  {M['pct_err_from_lag']:.1f} %")
    if M["pct_err_from_lag"] > 60.0:
        print("  => the residual is mostly a TIME SHIFT, removable by advancing the "
              "reference\n     or adding feedforward. Do NOT raise kp to fix it.")

    print("\nROM / CLAMPING")
    print(f"  human reference exceeded the bench ROM [{lo_deg:.2f}, {hi_deg:.2f}] deg : "
          f"{'YES' if audit.exceeded_rom else 'NO'}"
          + (f"  ({audit.n_below} below, {audit.n_above} above)"
             if audit.exceeded_rom else ""))
    print(f"  reference clamping actually applied to ctrl        : "
          f"{'YES' if M['ref_clamped_steps'] else 'NO'}"
          f"  ({M['ref_clamped_steps']} of {n} steps)")

    print("\nQUANTITY SEPARATION")
    print("  tau_sensor_Nm      = OSL bench ACTUATOR torque (fixed base, no body weight)")
    print("  human_knee_moment  = BIOMECHANICAL human joint moment, retained only")
    print("  These are different physical quantities and are never combined here.")
    if audit.flags:
        print(f"\nAUDIT FLAGS ({len(audit.flags)}) -- reference quality, not simulation "
              f"failures")
        for f in audit.flags:
            print(f"  ! {f}")
    print(f"\noutputs\n  {p_csv}\n  {p_met}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--csv", default=None, help="AB19 reference CSV")
    ap.add_argument("--outdir", default=blog.DEFAULT_OUTDIR)
    ap.add_argument("--kp", type=float, default=KP, help="N.m/rad")
    ap.add_argument("--kv", type=float, default=KD, help="N.m.s/rad (= Kd)")
    ap.add_argument("--cycles", type=int, default=1, help="gait cycles to run")
    ap.add_argument("--interp", choices=("cubic", "linear"), default="cubic")
    ap.add_argument("--no-plot", action="store_true")
    ap.add_argument("--plot-from", default=None,
                    help="skip the simulation and rebuild the figures from an existing CSV")
    args = ap.parse_args()
    os.makedirs(args.outdir, exist_ok=True)

    if args.plot_from:                     # runs in .venv-analysis, which has no mujoco
        from oslbench.plotting import plot_bench_results
        plot_bench_results(args.plot_from, args.outdir, args.kp, args.kv, [])
        return 0

    print("=" * 78)
    print(f"BENCH TRACKING EXPERIMENT -- {SUBJECT}")
    print(f"  reference : {args.csv or 'build/AB19_knee_gait_reference.csv'}")
    print(f"  trial     : {TRIAL}")
    print(f"  controller: kp={args.kp:.1f} N.m/rad  kv={args.kv:.3f} N.m.s/rad "
          f"(runtime only)")
    print("=" * 78)

    # ---- 1. the model -------------------------------------------------------------
    bench = load_bench_model()
    if bench.failures:
        print(f"\nREFUSING TO RUN: {len(bench.failures)} preflight failure(s)")
        for f in bench.failures:
            print(f"  - {f}")
        return len(bench.failures)
    dt = bench.timestep
    lo_deg, hi_deg = (math.degrees(x) for x in bench.knee_ctrlrange)

    # ---- 2. the human reference ---------------------------------------------------
    ref = load_reference(args.csv)
    audit = audit_reference(ref, lo_deg, hi_deg)
    res = resample_reference(ref, dt, args.cycles, args.interp, audit=audit)
    n = res.n

    # ---- 3. the controller --------------------------------------------------------
    sim = BenchSimulation(bench, PDController.knee(bench, args.kp, args.kv))
    print(f"\n    controller: {sim.knee!r}")

    # ---- 4. run, logging every step ----------------------------------------------
    log = blog.BenchLog(bench.force_limit)
    result = sim.run(res["ref_rad"], n,
                     ref_vel0=float(res["ref_vel_rad_s"][0]),
                     report=audit.report,
                     on_step=lambda st: log.append(st, res, st.k))

    # ---- integrity: the authored limits must still be exactly as compiled ---------
    print("\n[S2] INTEGRITY  (the model must be exactly as authored)")
    print(f"    knee forcerange {bench.knee_forcerange}  ankle "
          f"{bench.ankle_forcerange}  knee ctrlrange {bench.knee_ctrlrange}")
    if not bench.limits_unchanged():
        print("    CHANGED -- INVALID RUN")
        return 1
    print("    unchanged from the authored values")

    # ---- 5. outputs --------------------------------------------------------------
    M = bench_metrics(result, res, dt)
    p_csv = blog.write_bench_csv(
        blog.bench_csv_path(args.outdir), log,
        [f"subject {SUBJECT}; {TRIAL}",
         f"model {bench.relpath()} (UNMODIFIED); kp={args.kp} kv={args.kv} "
         f"set at runtime; dt={dt}",
         "tau_sensor_Nm is BENCH ACTUATOR torque; human_knee_moment is a BIOMECHANICAL "
         "HUMAN moment. Different quantities, never combined."])
    p_met = blog.write_metrics_csv(blog.metrics_csv_path(args.outdir), M,
                                   audit.fields(), audit.flags)
    report(M, audit, n, args.cycles, lo_deg, hi_deg, p_csv, p_met)

    # ---- 6. figures, from the CSV that was just written --------------------------
    if not args.no_plot:
        from oslbench.plotting import plot_bench_results
        plot_bench_results(p_csv, args.outdir, args.kp, args.kv, audit.flags)
    return 0


if __name__ == "__main__":
    sys.exit(main())
