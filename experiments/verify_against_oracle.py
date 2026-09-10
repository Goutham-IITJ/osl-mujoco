#!/usr/bin/env python3
"""
verify_against_oracle.py -- DOES THE REFACTORED CODE STILL PRODUCE THE VALIDATED RESULT?

WHY THIS FILE EXISTS
    The experiment was reorganised into oslbench/ after it had already been run and
    reported.  A reorganisation that changes the numbers is not a reorganisation, it is a
    new experiment.  So the result of the ORIGINAL validated run was frozen under
    tests/oracle/, and this script re-runs the pipeline as it stands today and compares
    every logged column and every metric against that frozen file.

    tests/oracle/bench_track_ab19.csv          2410 steps x 21 columns, the frozen trace
    tests/oracle/bench_track_ab19_metrics.csv  the frozen metrics, audit fields and flags

    They are kept OUTSIDE build/ deliberately: build/ is gitignored and is overwritten by
    every run, so an oracle stored there would be destroyed by the first re-run.

THE HEADLINE NUMBERS BEING DEFENDED
    RMS error   4.3616 deg      peak error    9.0907 deg
    peak torque 16.0041 N.m     saturation    0 %
    peak velocity 5.0559 rad/s

WHAT "PASS" MEANS
    Every one of the 21 columns agrees to the resolution the CSV is written at (the
    formatting resolution, so a difference smaller than the last printed digit is not
    detectable in the file at all), and every metric agrees to 1e-9 relative.  In
    practice the run is bit-identical: the refactor moved code between files, it did not
    change the order of a single MuJoCo call.

    A PASS is a REPRODUCIBILITY statement about the software.  It says nothing about the
    hardware: the physical OSL V2 has not been validated against this simulation.

USAGE (Windows PowerShell)
    .venv\\Scripts\\python.exe experiments\\verify_against_oracle.py
    .venv\\Scripts\\python.exe experiments\\verify_against_oracle.py --update-oracle
"""

from __future__ import annotations

import argparse
import math
import os
import shutil
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from oslbench import logging as blog                                       # noqa: E402
from oslbench.controller import KD, KP, PDController                       # noqa: E402
from oslbench.metrics import bench_metrics                                 # noqa: E402
from oslbench.model import load_bench_model                                # noqa: E402
from oslbench.reference import (SUBJECT, audit_reference, load_reference,   # noqa: E402
                                resample_reference)
from oslbench.simulation import BenchSimulation                            # noqa: E402

ORACLE_DIR = os.path.join(ROOT, "tests", "oracle")
WORK_DIR = os.path.join(ROOT, "build", "verify_oracle")

# The resolution each column is WRITTEN at.  A disagreement below this is invisible in
# the CSV, so it is the only defensible tolerance for a file comparison.
FMT_TOL = {"time_s": 5e-7, "gait_phase_percent": 5e-7,
           "ref_rad": 5e-9, "ref_deg": 5e-7, "sim_rad": 5e-9, "sim_deg": 5e-7,
           "err_deg": 5e-7, "ref_vel_rad_s": 5e-7, "sim_vel_rad_s": 5e-7,
           "ctrl_rad": 5e-9, "tau_sensor_Nm": 5e-7, "tau_actforce_Nm": 5e-7,
           "tau_unclamped_Nm": 5e-7, "pct_authority": 5e-5, "saturated": 0.0,
           "knee_power_W": 5e-7, "ankle_q_rad": 5e-9, "ankle_tau_Nm": 5e-7,
           "human_knee_angle_deg": 5e-7, "human_knee_moment": 5e-7,
           "human_knee_power": 5e-7}
METRIC_RTOL = 1e-9

HEADLINE = ("rms_err_deg", "peak_err_deg", "peak_tau_Nm", "sat_pct", "peak_vel_rad_s")


def rerun(csv=None, kp=KP, kv=KD, cycles=1, interp="cubic"):
    """Run the CURRENT pipeline exactly as run_bench_ab19.py runs it, quietly.

    Same six calls, same order, same objects.  If this function and run_bench_ab19.py
    ever disagree, that is itself the bug this script is looking for.
    """
    bench = load_bench_model(verbose=False)
    if bench.failures:
        for f in bench.failures:
            print(f"  model FAIL  {f}")
        sys.exit(len(bench.failures))
    dt = bench.timestep
    ref = load_reference(csv)
    audit = audit_reference(ref, math.degrees(bench.knee_ctrlrange[0]),
                            math.degrees(bench.knee_ctrlrange[1]), verbose=False)
    res = resample_reference(ref, dt, cycles, interp, audit=audit, verbose=False)
    sim = BenchSimulation(bench, PDController.knee(bench, kp, kv))
    log = blog.BenchLog(bench.force_limit)
    result = sim.run(res["ref_rad"], res.n,
                     ref_vel0=float(res["ref_vel_rad_s"][0]),
                     on_step=lambda st: log.append(st, res, st.k))
    M = bench_metrics(result, res, dt)
    return bench, res, audit, log, M, dt


def compare_trace(new_path: str, old_path: str) -> tuple[int, list[str]]:
    """Column-by-column comparison of the fresh trace against the frozen one."""
    new = blog.read_bench_csv(new_path)
    old = blog.read_bench_csv(old_path)
    bad, lines = 0, []

    if new["_header"] != old["_header"]:
        lines.append(f"  FAIL  header differs\n        new {new['_header']}\n"
                     f"        old {old['_header']}")
        return 1, lines
    n_new, n_old = len(new["time_s"]), len(old["time_s"])
    if n_new != n_old:
        lines.append(f"  FAIL  step count {n_new} != frozen {n_old}")
        return 1, lines
    lines.append(f"  {n_new} steps x {len(old['_header'])} columns compared")
    lines.append(f"  {'column':<22} {'max |new - frozen|':>20} {'tolerance':>12}   result")
    for name in old["_header"]:
        a = np.asarray(new[name], float)
        b = np.asarray(old[name], float)
        d = float(np.max(np.abs(a - b))) if len(a) else 0.0
        tol = FMT_TOL.get(name, 5e-7)
        ok = d <= tol
        bad += (not ok)
        lines.append(f"  {name:<22} {d:20.3e} {tol:12.1e}   "
                     f"{'ok' if ok else 'FAIL'}{'  (exact)' if d == 0.0 else ''}")
    return bad, lines


def compare_metrics(M: dict, audit, old: dict) -> tuple[int, list[str]]:
    """Every metric, then the audit fields, then the audit flag sentences."""
    bad, lines = 0, []
    lines.append(f"  {'metric':<26} {'new':>22} {'frozen':>22}   result")
    for k, v in M.items():
        if k not in old:
            lines.append(f"  {k:<26} {v!s:>22} {'ABSENT':>22}   FAIL")
            bad += 1
            continue
        ov = float(old[k])
        nv = float(v)
        ok = (nv == ov) or abs(nv - ov) <= METRIC_RTOL * max(1.0, abs(ov))
        bad += (not ok)
        lines.append(f"  {k:<26} {nv:22.12g} {ov:22.12g}   "
                     f"{'ok' if ok else 'FAIL'}{'  (exact)' if nv == ov else ''}")

    for k, v in audit.fields().items():
        key = f"ref_{k}"
        ov = old.get(key)
        ok = ov is not None and (str(v) == ov or _num_eq(v, ov))
        bad += (not ok)
        lines.append(f"  {key:<26} {str(v):>22} {str(ov):>22}   "
                     f"{'ok' if ok else 'FAIL'}")

    old_flags = [v for k, v in old.items() if k.startswith("audit_flag_")]
    if len(audit.flags) != len(old_flags):
        lines.append(f"  audit flag COUNT {len(audit.flags)} != frozen {len(old_flags)}"
                     f"   FAIL")
        bad += 1
    else:
        same = all(a.strip() == b.strip() for a, b in zip(audit.flags, old_flags))
        lines.append(f"  {len(old_flags)} audit flags, same text and same order: "
                     f"{'ok' if same else 'FAIL'}")
        bad += (not same)
    return bad, lines


def _num_eq(a, b) -> bool:
    try:
        return abs(float(a) - float(b)) <= 1e-12 * max(1.0, abs(float(b)))
    except (TypeError, ValueError):
        return False


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--oracle", default=ORACLE_DIR)
    ap.add_argument("--workdir", default=WORK_DIR,
                    help="where the fresh trace is written (never the oracle)")
    ap.add_argument("--csv", default=None, help="AB19 reference CSV")
    ap.add_argument("--kp", type=float, default=KP)
    ap.add_argument("--kv", type=float, default=KD)
    ap.add_argument("--update-oracle", action="store_true",
                    help="OVERWRITE the frozen result with this run. Only legitimate if "
                         "the physics was intentionally changed and re-validated.")
    args = ap.parse_args()

    o_csv = os.path.join(args.oracle, blog.BENCH_CSV_NAME)
    o_met = os.path.join(args.oracle, blog.METRICS_CSV_NAME)

    print("=" * 78)
    print("VERIFY THE REFACTORED PIPELINE AGAINST THE FROZEN VALIDATED RESULT")
    print(f"  frozen : {os.path.relpath(o_csv, ROOT)}")
    print(f"  gains  : kp={args.kp:g} kv={args.kv:g}   subject {SUBJECT}")
    print("=" * 78)
    for p in (o_csv, o_met):
        if not os.path.isfile(p):
            print(f"\nmissing oracle file: {p}")
            return 2

    bench, res, audit, log, M, dt = rerun(args.csv, args.kp, args.kv)
    os.makedirs(args.workdir, exist_ok=True)
    n_csv = blog.write_bench_csv(
        blog.bench_csv_path(args.workdir), log,
        [f"subject {SUBJECT}; re-run by experiments/verify_against_oracle.py",
         f"model {bench.relpath()} (UNMODIFIED); kp={args.kp} kv={args.kv} "
         f"set at runtime; dt={dt}",
         "this file is a VERIFICATION artefact; the reported result lives in "
         "build/bench_track_ab19/"])

    print("\n[1] PER-STEP TRACE")
    bad_trace, lines = compare_trace(n_csv, o_csv)
    print("\n".join(lines))

    print("\n[2] METRICS")
    bad_met, lines = compare_metrics(M, audit, blog.read_metrics_csv(o_met))
    print("\n".join(lines))

    print("\n[3] THE MODEL WAS NOT TOUCHED")
    print(f"  forcerange / ctrlrange unchanged after the run : "
          f"{bench.limits_unchanged()}")
    print(f"  knee authority still +/-{bench.force_limit:.1f} N.m, knee ROM still "
          f"[{math.degrees(bench.knee_ctrlrange[0]):.2f}, "
          f"{math.degrees(bench.knee_ctrlrange[1]):.2f}] deg")

    print("\n" + "=" * 78)
    ok = (bad_trace == 0 and bad_met == 0 and bench.limits_unchanged())
    if ok:
        print("PASS -- the refactored code reproduces the validated result.")
        print("  " + "   ".join(f"{k} {M[k]:.4f}" for k in HEADLINE))
    else:
        print(f"FAIL -- {bad_trace} column(s) and {bad_met} metric(s) disagree with the "
              f"frozen result.")
        print("  Do NOT report these numbers. Find the difference first.")
    print("=" * 78)

    if args.update_oracle:
        if not ok:
            print("\nrefusing --update-oracle while the comparison FAILS")
            return 1
        shutil.copyfile(n_csv, o_csv)
        blog.write_metrics_csv(o_met, M, audit.fields(), audit.flags)
        print(f"\noracle updated: {os.path.relpath(o_csv, ROOT)}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
