#!/usr/bin/env python3
"""
run_live_demo.py -- THE LIVE DEMO.  Entry point, not implementation.

WHAT IT IS
    The same experiment as run_bench_ab19.py, shown while it happens: a MuJoCo window
    with the CAD device moving, and a second window plotting the signals as they are
    measured.  Nothing here is a re-implementation.  It calls the same
    oslbench.simulation.BenchSimulation.step() the quantitative run calls, with the same
    oslbench.controller.PDController and the same resampled AB19 reference.

WHAT THE SECOND WINDOW IS AND IS NOT
    It is a CONSUMER of state.  oslbench/dashboard.py holds no model, takes no timestep
    and computes no torque; it is handed the values the step just returned and draws
    them.  It cannot write a command.  If you closed it, the physics would be identical.
    (The tests assert this: dashboard.py imports neither mujoco nor oslbench.simulation.)

WHAT IS ADDED, RELATIVE TO THE QUANTITATIVE RUN
    A camera, a frame rate, wall-clock pacing, keyboard control, and drawing.  That is
    the whole list.  --speed changes how many 0.5 ms physics steps are packed into one
    rendered frame; it NEVER changes dt.

PROOF, PRINTED BEFORE THE WINDOW OPENS
    §preflight re-runs the model preconditions, the reference audit, a determinism check,
    and a comparison of this run's metrics against the recorded
    build/bench_track_ab19/bench_track_ab19_metrics.csv.  It then prints the numbers you
    should expect to see, so the demo can be checked against the CSV live.

USAGE (Windows PowerShell)
    .venv\\Scripts\\python.exe experiments\\run_live_demo.py
    .venv\\Scripts\\python.exe experiments\\run_live_demo.py --speed 1.0 --max-cycles 3
    .venv\\Scripts\\python.exe experiments\\run_live_demo.py --dashboard none --print-every 5
    .venv\\Scripts\\python.exe experiments\\run_live_demo.py --record build\\demo.mp4

    Keys, in either window:  Space pause/resume   R restart the cycle   Esc exit
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

from oslbench import dashboard as dash                                     # noqa: E402
from oslbench import viewer as vw                                          # noqa: E402
from oslbench.controller import KD, KP                                     # noqa: E402
from oslbench.reference import (DEFAULT_REFERENCE_CSV, STANCE_END,         # noqa: E402
                                SUBJECT, TRIAL)


def build_dashboard(res, n, metrics, args):
    """Create the second window, with axes SIZED FROM the run preflight just measured.

    The limits are derived, not hard-coded, so a different reference or a different gain
    cannot silently push the traces off the visible area.
    """
    phase = np.asarray(res["gait_phase_percent"], float)
    ref_deg = np.degrees(np.asarray(res["ref_rad"], float))
    stride = max(1, n // vw.MAX_PLOT_PTS)          # display decimation only
    pad = 6.0
    limits = dict(
        ang=(math.floor((ref_deg.min() - pad) / 5) * 5,
             math.ceil((ref_deg.max() + pad) / 5) * 5),
        err=(-math.ceil(1.3 * metrics["peak_err_deg"] / 5) * 5,
             math.ceil(1.3 * metrics["peak_err_deg"] / 5) * 5),
        tau=(-math.ceil(1.3 * metrics["peak_tau_Nm"] / 5) * 5,
             math.ceil(1.3 * metrics["peak_tau_Nm"] / 5) * 5),
        stance_end=STANCE_END)
    return dash.make_dashboard(
        vw.TITLE, vw.SUBTITLE, dict(subject=SUBJECT, trial=TRIAL),
        (phase[::stride], ref_deg[::stride]), limits,
        geometry=args.dash_geometry, force=args.dashboard,
        port=args.port, open_browser=not args.no_browser)


def main() -> int:
    ap = argparse.ArgumentParser(
        description="Live view of the AB19 bench knee-tracking experiment "
                    "(fixed-base bench -- NOT human walking).")
    ap.add_argument("--csv", default=DEFAULT_REFERENCE_CSV)
    ap.add_argument("--kp", type=float, default=KP, help="N.m/rad")
    ap.add_argument("--kv", type=float, default=KD, help="N.m.s/rad (= Kd)")
    ap.add_argument("--speed", type=float, default=0.5,
                    help="0.5 = half real time (default, easier to watch); 1.0 = real "
                         "time. Scales steps per frame only -- dt never changes.")
    ap.add_argument("--max-cycles", type=int, default=0, help="0 = loop forever")
    ap.add_argument("--print-every", type=float, default=0.0,
                    help="also print a terminal row every N %% of gait phase (0 = off)")
    ap.add_argument("--zoom", type=float, default=0.45,
                    help="multiplies the derived camera distance; <1 = model larger")
    ap.add_argument("--elevation", type=float, default=-6.0)
    ap.add_argument("--azimuth", type=float, default=None)
    ap.add_argument("--distance", type=float, default=None)
    ap.add_argument("--show-joints", action="store_true")
    ap.add_argument("--linear", action="store_true",
                    help="linear instead of cubic reference interpolation (diagnostic)")
    ap.add_argument("--dashboard", choices=dash.BACKENDS, default="auto",
                    help="second window backend (default auto: browser, else tkinter, "
                         "else an in-terminal readout). 'none' disables it.")
    ap.add_argument("--port", type=int, default=8787,
                    help="port for the browser dashboard on 127.0.0.1 (0 = any free)")
    ap.add_argument("--no-browser", action="store_true",
                    help="serve the dashboard but do not open a browser automatically")
    ap.add_argument("--dash-geometry", default="+40+40",
                    help="window position for the tkinter dashboard only, e.g. +1000+40")
    ap.add_argument("--verbose", action="store_true",
                    help="also print the full verification transcript")
    ap.add_argument("--record", default=None,
                    help="headless frame-by-frame recording to this .mp4 (no viewer)")
    ap.add_argument("--record-fps", type=int, default=30)
    ap.add_argument("--record-width", type=int, default=1280)
    ap.add_argument("--record-height", type=int, default=720)
    ap.add_argument("--record-cycles", type=int, default=2)
    args = ap.parse_args()

    print("=" * 78)
    print(f"  {vw.TITLE_TTY}")
    print(f"  {vw.SUBTITLE}")
    print("=" * 78)

    # ---- verify everything, quietly, before a window exists -----------------------
    pf = vw.preflight(csv=args.csv, kp=args.kp, kv=args.kv,
                      interp="linear" if args.linear else "cubic",
                      azimuth=args.azimuth, elevation=args.elevation,
                      distance=args.distance)
    if not isinstance(pf, vw.Preflight):
        return int(pf)
    print("\n".join(pf.lines))
    if args.verbose:
        print("-" * 78 + "\n" + pf.transcript + "-" * 78)

    # ---- show it ------------------------------------------------------------------
    if args.record:
        rc = vw.record_video(pf.bench, pf.sim, pf.res, pf.n, args.record, pf.cam,
                             fps=args.record_fps, width=args.record_width,
                             height=args.record_height, cycles=args.record_cycles,
                             zoom=args.zoom, elevation=args.elevation)
        if rc:
            return int(rc)
    else:
        d = build_dashboard(pf.res, pf.n, pf.metrics, args)
        vw.run_live_view(pf.bench, pf.sim, pf.res, pf.n, pf.cam, pf.metrics,
                         speed=args.speed, max_cycles=args.max_cycles, zoom=args.zoom,
                         elevation=args.elevation, show_joints=args.show_joints,
                         dash=d, print_every=args.print_every)

    # ---- the model must still be exactly as authored ------------------------------
    ok = pf.bench.limits_unchanged()
    print(f"  [F] knee forcerange {pf.bench.knee_forcerange} and ctrlrange "
          f"{pf.bench.knee_ctrlrange} unchanged: {ok}")
    print("      the model XML was never opened for writing.")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
