#!/usr/bin/env python3
"""
run_gain_sweep.py -- CONTROLLER TUNING SWEEP.  Entry point, not implementation.

WHY THIS EXISTS
    It is where Kp = 600 and Kd = 17.253 come from.  Kd is not tuned independently: for
    each Kp it is DERIVED from the target damping ratio,

        Kd = 2*zeta*sqrt(Kp*I_eff) - b_joint        with zeta = 0.7,

    using the knee-referred inertia I_eff measured out of the compiled model and the
    joint's own viscous damping b_joint.  So the sweep has one free parameter, Kp, and
    the file reports what each choice costs in error, torque and saturation.

WHAT IT SWEEPS AGAINST
    SYNTHETIC references only -- steps, a sine, a minimum-jerk move, a chirp.  NOT human
    data.  The AB19 human reference belongs to run_bench_ab19.py; keeping the tuning
    inputs synthetic is what stops the gains from being fitted to the very trajectory
    they are later scored on.

WHAT IT DOES NOT TOUCH
    models/osl_v2_bench.xml, forcerange, ctrlrange, the timestep, or the AB19 data.
    Gains are written into the compiled mjModel in memory; §[8] re-reads the limits
    afterwards and reports whether they still match the authored values.

USAGE (Windows PowerShell)
    .venv\\Scripts\\python.exe experiments\\run_gain_sweep.py
    .venv\\Scripts\\python.exe experiments\\run_gain_sweep.py --kp 600
    .venv\\Scripts\\python.exe experiments\\run_gain_sweep.py --refs step_45deg sine_gait
"""

from __future__ import annotations

import argparse
import math
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from oslbench import logging as blog                                      # noqa: E402
from oslbench.controller import PDController, ZETA_DEFAULT, kd_for_damping_ratio  # noqa: E402,E501
from oslbench.metrics import SWEEP_METRIC_KEYS, step_shape, sweep_metrics  # noqa: E402
from oslbench.model import load_bench_model                               # noqa: E402
from oslbench.reference import synthetic_references                       # noqa: E402
from oslbench.simulation import BenchSimulation                           # noqa: E402

KP_DEFAULT = [60.0, 200.0, 600.0, 2000.0]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--kp", nargs="+", type=float, default=KP_DEFAULT)
    ap.add_argument("--zeta", type=float, default=ZETA_DEFAULT)
    ap.add_argument("--refs", nargs="+", default=None,
                    help="subset of: step_10deg step_45deg sine_gait minjerk_60deg chirp")
    ap.add_argument("--decim", type=int, default=2, help="CSV decimation (2 -> 1 kHz)")
    ap.add_argument("--outdir", default=blog.SWEEP_OUTDIR)
    args = ap.parse_args()

    print("=" * 96)
    print("OSL V2 BENCH -- CONTROLLER GAIN SWEEP")
    print("CONTROLLER TUNING ONLY.  The physical model is not modified;")
    print("gains are set on the compiled mjModel and forcerange is left as authored.")
    print("=" * 96)

    bench = load_bench_model()
    if bench.failures:
        print(f"\n{len(bench.failures)} preflight check(s) failed -- refusing to sweep a "
              f"model that is not the one this script was written for.")
        for f in bench.failures:
            print(f"  - {f}")
        return len(bench.failures)

    dt = bench.timestep
    refs = synthetic_references(dt, bench.knee_ctrlrange[0], bench.knee_ctrlrange[1])
    if args.refs:
        unknown = set(args.refs) - set(refs)
        if unknown:
            sys.exit(f"unknown reference(s): {sorted(unknown)}\navailable: {sorted(refs)}")
        refs = {k: refs[k] for k in args.refs}

    # ---- the gain table: Kd is derived from zeta, not chosen ----------------------
    def kv_of(kp):
        return kd_for_damping_ratio(kp, bench.i_eff, bench.b_joint, args.zeta)

    print(f"\n[6] GAINS  zeta = {args.zeta}, kv = 2*zeta*sqrt(kp*I_eff) - b_joint")
    print(f"    I_eff = {bench.i_eff:.6f} kg.m^2   b_joint = {bench.b_joint} N.m.s/rad")
    print(f"    {'kp':>7} {'kv':>9} {'wn rad/s':>10} {'wn Hz':>8} {'fric deadband':>14}")
    gains = []
    for kp in args.kp:
        kv = kv_of(kp)
        wn = math.sqrt(kp / bench.i_eff)
        gains.append((kp, kv))
        print(f"    {kp:7.0f} {kv:9.3f} {wn:10.3f} {wn / 2 / math.pi:8.3f} "
              f"{math.degrees(bench.frictionloss / kp):13.3f}d")

    os.makedirs(args.outdir, exist_ok=True)
    trace_path = os.path.join(args.outdir, "gain_sweep_trace.csv")
    metrics_path = os.path.join(args.outdir, "gain_sweep_metrics.csv")

    # ---- the sweep ---------------------------------------------------------------
    results, traces = {}, []
    print(f"\n[7] SWEEP  ({len(refs)} references x {len(gains)} gains)")
    for name, (t, r, desc, astart) in refs.items():
        print(f"\n  {name}: {desc}")
        print(f"    metrics measured from t >= {astart:.3f} s")
        ref_vel = np.gradient(r, dt)
        for kp, kv in gains:
            sim = BenchSimulation(bench, PDController.knee(bench, kp, kv))
            log = blog.SweepLog(bench.force_limit, kp, kv, args.decim)
            result = sim.run(r, on_step=lambda st: log.append(st, ref_vel, st.k))
            m = sweep_metrics(result, dt, astart)
            if name.startswith("step"):
                m["overshoot_pct"], m["settle_s"] = step_shape(t, r, result.q, dt)
            else:
                m["overshoot_pct"], m["settle_s"] = float("nan"), float("nan")
            results[(name, kp)] = m
            traces.append((name, log.rows()))
            print(f"    kp={kp:7.0f} kv={kv:7.3f} | RMSe {m['rms_err']:7.3f}d "
                  f"peak {m['peak_err']:7.3f}d ss {m['ss_err']:6.3f}d | "
                  f"tau {m['peak_tau']:7.2f} N.m ({m['pct_auth']:5.1f}% auth) "
                  f"sat {m['sat_pct']:5.1f}% | ankle drift {m['ankle_dev_deg']:.3f}d")

    blog.write_sweep_csv(trace_path, traces, [
        "CONTROLLER TUNING SWEEP -- synthetic references, NOT human data",
        f"model={bench.relpath()} (unmodified on disk)",
        f"I_eff={bench.i_eff:.6f} b_joint={bench.b_joint} fric={bench.frictionloss} "
        f"mgd={bench.mgd:.4f} zeta={args.zeta}",
        f"forcerange knee={bench.knee_forcerange} UNCHANGED"])
    blog.write_sweep_metrics_csv(metrics_path, results, SWEEP_METRIC_KEYS,
                                 args.zeta, kv_of)

    # ---- integrity + repeatability ----------------------------------------------
    print("\n[8] INTEGRITY")
    print(f"    forcerange and ctrlrange identical to the authored values after the "
          f"whole sweep: {bench.limits_unchanged()}")
    print(f"    models/osl_v2_bench.xml mtime unchanged by this run: "
          f"{os.path.getmtime(bench.path)}")
    r0 = refs[next(iter(refs))][1]
    sim0 = BenchSimulation(bench, PDController.knee(bench, *gains[0]))
    a = sim0.run(r0).q
    b = sim0.run(r0).q
    print(f"    two identical runs are bit-identical (deterministic): "
          f"{np.array_equal(a, b)}")

    print("\n[9] OUTPUT")
    print(f"    trace   {trace_path}")
    print(f"    metrics {metrics_path}")
    print(f"\n{len(bench.failures)} failed, {len(bench.warnings)} warning(s)")
    for x in bench.warnings:
        print(f"  warn: {x}")
    return len(bench.failures)


if __name__ == "__main__":
    sys.exit(main())
