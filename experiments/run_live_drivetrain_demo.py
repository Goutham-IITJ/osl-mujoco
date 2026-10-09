#!/usr/bin/env python3
"""
run_live_drivetrain_demo.py -- THE OPTION-C LIVE DEMO.  Entry point, not implementation.

    OPTION C -- PAPER-DERIVED ACTUATOR + COMPLIANT BELT
    SIMULATION ONLY -- NOT HARDWARE VALIDATION

WHAT IT IS
    `experiments/run_bench_ab19_drivetrain.py`'s Case B, shown while it happens: a MuJoCo
    window with the CAD OSL V2 bench moving, and a second window plotting the actuator
    and belt signals as they are produced.

WHAT IS NEW HERE, EXHAUSTIVELY
    A camera, a frame rate, wall-clock pacing, keyboard control, and drawing.  That is
    the whole list.  There is NO second physics or control implementation in this file:

      * the control law is `oslbench.drivetrain_sim.PDCurrentSource` -- the same object
        the quantitative run uses.  The PD equation is NOT restated here; grep this file
        for `kp` and you will find it passed, never multiplied.
      * the plant is `oslbench.drivetrain_sim.DrivetrainBenchSimulation.step()` -- the
        same call, with the same `DrivetrainLayer` carrying Best et al.'s equations.
      * the servo-disconnect is the same `disconnect_servo=True` mechanism, and its
        result is verified before the window opens rather than assumed.
      * the gains are imported from `oslbench.controller` (KP, KD).  This file cannot
        change them and `--kp`/`--kv` are deliberately NOT offered.
      * the reference is the same resampled AB19 cycle.

    `--speed` changes how many 0.5 ms physics steps are packed into one rendered frame
    and how long the loop sleeps.  It NEVER changes dt, which stays at the model's
    authored 0.5 ms at every playback rate.

WHAT THE SECOND WINDOW IS AND IS NOT
    A CONSUMER of state.  `oslbench/drivetrain_dashboard.py` holds no model, takes no
    timestep and computes no torque or current; it is handed the numbers the step just
    produced and draws them, in the browser's process.  It cannot write a command.  Close
    it and the physics is identical.

WHY THE TORQUE PANEL HAS TWO SCALES, AND THE ANGLE PANEL ONE
    `tau_j` (joint side) and `tau_a` (actuator side) differ by n_t = 4.61 and are never
    drawn on one axis -- plot 4 gives the current its own right-hand scale for the same
    reason.  Plot 2 is the deliberate exception: `theta_a/n_t` has already been referred
    THROUGH the transmission, so it and `theta_j` are the same physical quantity measured
    at the two ends of the belt, and the gap between them IS the deflection.  A rigid
    transmission would draw a single line there.

USAGE (Windows PowerShell)
    .venv\\Scripts\\python.exe experiments\\run_live_drivetrain_demo.py
    .venv\\Scripts\\python.exe experiments\\run_live_drivetrain_demo.py --speed 0.25
    .venv\\Scripts\\python.exe experiments\\run_live_drivetrain_demo.py --speed 1.0
    .venv\\Scripts\\python.exe experiments\\run_live_drivetrain_demo.py --dashboard terminal
    .venv\\Scripts\\python.exe experiments\\run_live_drivetrain_demo.py --headless --max-cycles 1

    Keys, in either window:  Space pause/resume   R restart the cycle   Esc exit

WHAT IT WRITES
    Nothing, by default.  No CSV, no PNG, no model.  `--dump-html` writes one standalone
    copy of the dashboard page for inspection and is the only output path in the file.
"""

from __future__ import annotations

import argparse
import contextlib
import io
import math
import os
import sys
import time

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from oslbench import drivetrain_dashboard as ddash                          # noqa: E402
from oslbench import viewer as vw                                           # noqa: E402
from oslbench.controller import KD, KP, PDController                        # noqa: E402
from oslbench.drivetrain import PAPER                                       # noqa: E402
from oslbench.drivetrain_sim import (DrivetrainBenchSimulation,             # noqa: E402
                                     DrivetrainLayer, PDCurrentSource)
from oslbench.model import load_bench_model                                 # noqa: E402
from oslbench.reference import (DEFAULT_REFERENCE_CSV, STANCE_END,          # noqa: E402
                                SUBJECT, TRIAL, audit_reference,
                                load_reference, resample_reference)

BANNER = ddash.BANNER
DISCLAIMER = ddash.DISCLAIMER
SPEEDS = (0.25, 0.5, 1.0)
FIT_EDGE_RAD = 0.055        # ASSUMED: Best et al. Fig. 3 abscissa; past it the belt law
#                             is extrapolation of their regression, not their data
K_T_JOINT = PAPER.n_t * PAPER.k_t * PAPER.n_a        # 4.597092 N.m/A, PAPER-DERIVED


# =====================================================================================
# PHYSICS -- every object here is existing, tested code.  Nothing is reimplemented.
# =====================================================================================
def build(args):
    """Load the model and reference, wire the Option-C stack, verify the servo is off.

    Returns (bench, sim, layer, src, res, n) or an int exit code.
    """
    bench = load_bench_model(verbose=args.verbose)
    if bench.failures:
        print(f"\nREFUSING TO RUN: {len(bench.failures)} model precondition(s) failed")
        for f in bench.failures:
            print(f"  - {f}")
        return len(bench.failures)
    # The demo must be showing the authored bench, not something else that loaded.
    if not bench.path.replace("\\", "/").endswith("models/osl_v2_bench.xml"):
        print(f"\nREFUSING TO RUN: loaded {bench.relpath()}, expected "
              f"models/osl_v2_bench.xml")
        return 1

    dt = bench.timestep
    # The reference loader and auditor are deliberately chatty -- the quantitative runs
    # want all of it. A demo that opens with 40 lines of column-audit FLAGs about the
    # CSV's human moment/power columns (a known, documented defect in columns this demo
    # never reads) is unwatchable, so the transcript is captured and only printed under
    # --verbose. Nothing is skipped: the same audit runs and the same object is returned.
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        ref = load_reference(args.csv)
        audit = audit_reference(ref, math.degrees(bench.knee_ctrlrange[0]),
                                math.degrees(bench.knee_ctrlrange[1]))
        res = resample_reference(ref, dt, 1, "cubic", audit=audit)
    if args.verbose:
        print("-" * 78)
        print(buf.getvalue().rstrip())
        print("-" * 78)
    n = res.n

    # ---- the Option-C stack, assembled from the tested pieces ----------------------
    layer = DrivetrainLayer(PAPER, enabled=True, substeps=1)
    pd = PDController.knee(bench, KP, KD)
    # THE control law.  Imported, constructed, and then only CALLED -- see the module
    # docstring.  The same layer instance goes to the source and the simulation; two
    # different ones would damp a shaft nothing is driving, which would look stable and
    # mean nothing.
    src = PDCurrentSource(pd, res["ref_rad"], layer, PAPER)
    sim = DrivetrainBenchSimulation(bench, pd, layer=layer, current_source=src,
                                    disconnect_servo=True)

    if sim.servo_connected:
        print("\nREFUSING TO RUN: the knee position servo did not disconnect. The belt "
              "and the servo would both act in the same mj_step and everything on "
              "screen would be a double count.")
        return 1
    if src.layer is not sim.layer:
        print("\nREFUSING TO RUN: the current source and the simulation hold different "
              "DrivetrainLayer instances.")
        return 1
    if not bench.limits_unchanged():
        print("\nREFUSING TO RUN: the authored force/ctrl ranges changed during setup.")
        return 1
    return bench, sim, layer, src, res, n


def read_step(bench, sim, res, k):
    """Everything the dashboard shows, read off the step that just ran.

    Pure extraction: every value is either a MuJoCo array entry or a field of the
    `LayerStep` the drivetrain layer returned.  No value is recomputed here -- in
    particular the current is READ from the layer, not re-derived from the gains, which
    is what keeps this file from becoming a second controller.
    """
    L = sim.last_layer
    q = float(bench.data.qpos[bench.knee_qpos])
    ref_deg = math.degrees(float(res["ref_rad"][k]))
    act_deg = math.degrees(q)
    return dict(
        phase=float(res["gait_phase_percent"][k]),
        ref=ref_deg,
        act=act_deg,
        err=act_deg - ref_deg,
        tau_j=float(L.tau_j),                       # JOINT side, N.m
        i_q=float(L.i_q),                           # drive current, A
        th_a=float(L.theta_a),                      # ACTUATOR side, rad
        th_ad=float(L.theta_a_dot),                 # ACTUATOR side, rad/s
        th_s=float(L.theta_s),                      # belt deflection, rad
        tau_a=float(L.tau_a),                       # ACTUATOR side, N.m
        k_s=float(L.K_s),                           # N.m/rad, tangent stiffness
        # theta_a referred THROUGH the transmission, so plot 2 can put it on theta_j's
        # axis.  This is the one place that division is correct.
        #
        # ONE HONEST WRINKLE, since the panel invites the comparison: the gap plot 2
        # draws is theta_a/n_t - theta_j with theta_j read from qpos AFTER the step,
        # while `th_s` below is the layer's own theta_s, evaluated at the configuration
        # the belt force was computed at -- i.e. BEFORE the step.  So the two differ by
        # one 0.5 ms step of knee motion, up to ~0.01 deg here.  Both are correct; they
        # are sampled half a step apart.  Panel 3 plots the layer's value because that
        # is the one the physics used, and the demo does NOT recompute eq (3) to force
        # them to agree -- restating a drivetrain equation in the presentation layer is
        # exactly what this file must not do.
        thar_deg=math.degrees(float(L.theta_a) / PAPER.n_t),
    )


# =====================================================================================
# PRESENTATION -- axes, pacing, drawing.  Reads state; never writes a command.
# =====================================================================================
def build_axes(res, n, peak):
    """Axis limits DERIVED from the reference and from a dry run, never hard-coded.

    A hard-coded axis is how a demo silently clips the signal it exists to show.  The
    peaks come from `probe()` below, which runs the real loop once, headless.
    """
    phase = np.asarray(res["gait_phase_percent"], float)
    ref_deg = np.degrees(np.asarray(res["ref_rad"], float))
    stride = max(1, n // ddash.MAX_PLOT_PTS)

    def sym(v, step):
        return math.ceil(max(v, step) * 1.25 / step) * step

    ang_lo = math.floor((min(ref_deg.min(), peak["ang_lo"]) - 6.0) / 5) * 5
    ang_hi = math.ceil((max(ref_deg.max(), peak["ang_hi"]) + 6.0) / 5) * 5
    # the deflection axis always shows the fitted edge, so "inside the paper's range"
    # is legible even on a run that never approaches it
    ths = max(sym(peak["ths"], 0.005), FIT_EDGE_RAD * 1.2)
    # Panel 2's right-hand axis, in DEGREES, sized from the measured deflection rather
    # than from `ths` above: `ths` is padded out to the fit edge so the extrapolation
    # band is always legible, which would leave the gap trace using a fifth of the
    # height. theta_s = theta_j - theta_a/n_t, so the gap drawn is -theta_s.
    gap_deg = max(math.degrees(peak["ths"]) * 1.25, 0.05)
    limits = dict(ang=(ang_lo, ang_hi),
                  ths=(-ths, ths),
                  gap=(-gap_deg, gap_deg),
                  tau=(-sym(peak["tau"], 5.0), sym(peak["tau"], 5.0)),
                  iq=(-sym(peak["iq"], 1.0), sym(peak["iq"], 1.0)),
                  fit_edge=FIT_EDGE_RAD,
                  stance_end=STANCE_END)
    return (phase[::stride], ref_deg[::stride]), limits


def probe(bench, sim, res, n):
    """Run one full cycle headless to size the axes and audit the servo, then reset.

    Cheap (2410 steps of a 2-dof model) and it buys three things: axes that cannot clip;
    a per-step check that the position servo contributed nothing, rather than a single
    sample of it; and a chance to refuse to open a window on a run that diverges --
    which this exact configuration did, at t = 0.3485 s, before the collocation fix.

    The servo audit has to happen HERE and not at construction time.
    `knee_servo_force()` reads `data.actuator_force`, which is only meaningful after a
    `mj_forward` or `mj_step`. Reading it straight after building the simulation returns
    whatever the model-loading preflight left in the buffer -- a large nonzero number
    from BEFORE the disconnect -- which would print a flat contradiction next to
    "servo disconnected". Asked and answered the wrong way once; now it is sampled only
    after steps that have actually run.
    """
    ref = np.asarray(res["ref_rad"], float)
    sim.reset(float(ref[0]), ref_vel0=float(res["ref_vel_rad_s"][0]))
    pk = dict(ang_lo=0.0, ang_hi=0.0, ths=0.0, tau=0.0, iq=0.0, thad=0.0,
              servo_peak=0.0, servo_violations=0)
    first_bad = None
    for k in range(n):
        sim.step(float(ref[k]), k)
        servo = sim.knee_servo_force()
        pk["servo_peak"] = max(pk["servo_peak"], abs(servo))
        if servo != 0.0:
            pk["servo_violations"] += 1
        s = read_step(bench, sim, res, k)
        if first_bad is None and not all(
                math.isfinite(s[f]) for f in ("act", "tau_j", "th_s", "i_q")):
            first_bad = k
            break
        pk["ang_lo"] = min(pk["ang_lo"], s["act"], s["thar_deg"])
        pk["ang_hi"] = max(pk["ang_hi"], s["act"], s["thar_deg"])
        pk["ths"] = max(pk["ths"], abs(s["th_s"]))
        pk["tau"] = max(pk["tau"], abs(s["tau_j"]))
        pk["iq"] = max(pk["iq"], abs(s["i_q"]))
        pk["thad"] = max(pk["thad"], abs(s["th_ad"]))
    sim.reset(float(ref[0]), ref_vel0=float(res["ref_vel_rad_s"][0]))
    pk["first_bad"] = first_bad
    pk["steps"] = n if first_bad is None else first_bad + 1
    return pk


def footer(bench, n, dt):
    """The provenance block, on screen for the whole demo rather than in a README."""
    return (f"{BANNER}\n{DISCLAIMER}\n"
            f"model   models/osl_v2_bench.xml (unmodified; never opened for writing)\n"
            f"plant   Best et al. 2025, IEEE/ASME T-MECH 30(6):4732-4743, eqs (1)-(12). "
            f"Parameters are PAPER-DERIVED for a different (ankle) build and are NOT "
            f"measurements of this hardware.\n"
            f"gains   Kp {KP:g} N.m/rad, Kd {KD:g} N.m.s/rad, set at runtime, NOT "
            f"retuned. Kd derives from a single-mass calculation with no belt in it.\n"
            f"wiring  knee position servo DISCONNECTED (kp=kd=0 written into the "
            f"compiled model); the belt torque is the only actuation, injected through "
            f"qfrc_applied\n"
            f"law     derivative feedback on {PDCurrentSource.FEEDBACK_SIDE}-side "
            f"velocity (theta_a_dot/n_t), collocated with the current it commands\n"
            f"limit   {bench.force_limit:.1f} N.m authored knee authority -- shown as a "
            f"REFERENCE LINE only. forcerange cannot clamp qfrc_applied and the paper "
            f"documents no drive-current limit, so nothing here saturates.\n"
            f"scale   {n} steps x {dt * 1e3:.3f} ms = one AB19 gait cycle; theta_s "
            f"beyond +/-{FIT_EDGE_RAD:g} rad is extrapolation of the paper's regression\n"
            f"shaded  0-{STANCE_END:g} % = human stance phase, for orientation only\n"
            f"torque is BENCH torque on a FIXED BASE. It is not a human knee moment and "
            f"this is not walking.")


def run(bench, sim, res, n, cam, dash, args):
    """The loop: pace, step, read, draw.  The only physics call is sim.step()."""
    import mujoco
    import mujoco.viewer

    dt = bench.timestep
    ref = np.asarray(res["ref_rad"], float)
    phase = np.asarray(res["gait_phase_percent"], float)
    stride = max(1, n // ddash.MAX_PLOT_PTS)
    # full-rate logs; the dashboard plots a strict subsample of THESE
    log = {f: np.full(n, np.nan) for f in
           ("act", "thar", "ths", "tau", "iq")}

    st = {"paused": False, "restart": False, "quit": False}

    def on_code(code):                      # MuJoCo window (GLFW key codes)
        if code == vw.K_SPACE:
            st["paused"] = not st["paused"]
            print(f"  [{'PAUSED' if st['paused'] else 'RESUMED'}]")
        elif code in (vw.K_R_UPPER, vw.K_R_LOWER):
            st["restart"] = True

    def on_sym(sym):                        # dashboard window (browser keys)
        s = (sym or "").lower()
        if s == "space":
            st["paused"] = not st["paused"]
        elif s == "r":
            st["restart"] = True
        elif s == "escape":
            st["quit"] = True

    dash.bind_keys(on_sym)
    ready = dash.wait_ready(6.0)

    spf = max(1, int(round(args.speed * (1.0 / vw.FRAME_HZ) / dt)))
    frame_budget = spf * dt / args.speed            # wall seconds one frame may take
    print(f"  windows  MuJoCo viewer + {dash.kind} dashboard"
          + (f" ({'page loaded' if ready else 'page not loaded yet'})"
             if dash.kind == "web" else ""))
    print(f"  pacing   {args.speed:g}x real time = {spf} step(s)/frame at "
          f"{vw.FRAME_HZ:g} Hz; frame budget {frame_budget * 1e3:.1f} ms; "
          f"dt stays {dt * 1e3:.3f} ms")
    print("  keys     Space pause/resume   R restart cycle   Esc exit   (either window)")
    print("-" * 78)

    sim.reset(float(ref[0]), ref_vel0=float(res["ref_vel_rad_s"][0]))
    k, cyc = 0, 1
    wall0, sim0 = time.perf_counter(), float(bench.data.time)
    next_frame = time.perf_counter()
    late = 0

    with mujoco.viewer.launch_passive(bench.model, bench.data, key_callback=on_code,
                                      show_left_ui=False,
                                      show_right_ui=False) as viewer:
        with viewer.lock():
            vw.apply_camera(viewer.cam, cam, args.zoom, args.elevation)
            viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_JOINT] = bool(args.show_joints)

        while viewer.is_running() and not st["quit"]:
            if not dash.pump():
                break

            if st["restart"]:
                sim.reset(float(ref[0]), ref_vel0=float(res["ref_vel_rad_s"][0]))
                for v in log.values():
                    v[:] = np.nan
                k, st["restart"] = 0, False
                dash.clear_traces()
                print("  [R] restarted at 0 % gait phase")
                wall0 = next_frame = time.perf_counter()
                sim0 = float(bench.data.time)
                continue

            if st["paused"]:
                viewer.sync()
                time.sleep(1.0 / vw.FRAME_HZ)
                wall0, sim0 = time.perf_counter(), float(bench.data.time)
                next_frame = time.perf_counter()
                continue

            done = False
            for _ in range(spf):
                sim.step(float(ref[k]), k)          # <-- THE shared step
                s = read_step(bench, sim, res, k)
                log["act"][k] = s["act"]
                log["thar"][k] = s["thar_deg"]
                log["ths"][k] = s["th_s"]
                log["tau"][k] = s["tau_j"]
                log["iq"][k] = s["i_q"]
                k += 1
                if k >= n:
                    done = True
                    break

            viewer.sync()
            rt = ((float(bench.data.time) - sim0)
                  / max(1e-9, time.perf_counter() - wall0))
            dash.update(dict(
                **{f: s[f] for f in ddash.SCALARS},
                x=phase[:k:stride],
                y_act=log["act"][:k:stride], y_thj=log["act"][:k:stride],
                y_thar=log["thar"][:k:stride], y_ths=log["ths"][:k:stride],
                y_tau=log["tau"][:k:stride], y_iq=log["iq"][:k:stride],
                foot=footer(bench, n, dt),
                status=(f"cycle {cyc}   {args.speed:g}x requested, {rt:.2f}x achieved"
                        f"{'  [CANNOT KEEP UP]' if rt < 0.75 * args.speed else ''}   "
                        f"step {k}/{n}   dt {dt * 1e3:.3f} ms   "
                        f"Space pause | R restart | Esc exit")))

            # ---- WALL-CLOCK PACING ---------------------------------------------------
            # Without this, --speed only set steps-per-frame and the achieved rate was
            # whatever the renderer could manage -- so the number on screen was not the
            # number requested.  Sleeping to a frame deadline makes --speed mean what it
            # says.  The deadline is RE-BASELINED rather than accumulated when the loop
            # falls behind, so a slow frame cannot cause a catch-up burst that looks like
            # the simulation jumping.
            next_frame += frame_budget
            slack = next_frame - time.perf_counter()
            if slack > 0:
                time.sleep(slack)
            elif slack < -5.0 * frame_budget:
                late += 1
                next_frame = time.perf_counter()

            if done:
                el = time.perf_counter() - wall0
                fin = np.isfinite(log["tau"])
                print(f"  cycle {cyc}   peak |tau_j| {np.nanmax(np.abs(log['tau'])):8.3f} "
                      f"N.m | peak |I_q| {np.nanmax(np.abs(log['iq'])):7.3f} A | "
                      f"peak |theta_s| {np.nanmax(np.abs(log['ths'])):9.6f} rad "
                      f"({np.nanmax(np.abs(log['ths'])) / FIT_EDGE_RAD:.2f}x fit edge) | "
                      f"{n * dt / max(el, 1e-9):.2f}x real time")
                if not fin.all():
                    print(f"  cycle {cyc}   *** {int((~fin).sum())} NON-FINITE step(s): "
                          f"the run diverged. Nothing after the first one is physics. ***")
                cyc += 1
                k = 0
                dash.clear_traces()
                wall0, sim0 = time.perf_counter(), float(bench.data.time)
                next_frame = time.perf_counter()
                if args.max_cycles and cyc > args.max_cycles:
                    break

    if late:
        print(f"  note     the loop fell more than 5 frame budgets behind {late} time(s) "
              f"and re-baselined its clock; lower --speed for a smoother picture.")
    dash.close()
    return 0


def run_headless(bench, sim, res, n, dash, args):
    """The same loop with no viewer and no pacing -- for CI and for a machine with no GL.

    It exercises every signal and the dashboard's publish path, which is what makes this
    file checkable without a display.
    """
    dt = bench.timestep
    ref = np.asarray(res["ref_rad"], float)
    phase = np.asarray(res["gait_phase_percent"], float)
    stride = max(1, n // ddash.MAX_PLOT_PTS)
    log = {f: np.full(n, np.nan) for f in ("act", "thar", "ths", "tau", "iq")}
    # Phase cadence, not wall-clock: headless runs ~19x real time, so a time-throttled
    # readout would print one row for the whole cycle and prove nothing.
    every = max(1, n // 20)
    print(f"  headless: no viewer, no pacing; {args.max_cycles or 1} cycle(s); "
          f"readout every {every} steps (~5 % of gait phase)")
    print("-" * 78)
    for cyc in range(1, (args.max_cycles or 1) + 1):
        sim.reset(float(ref[0]), ref_vel0=float(res["ref_vel_rad_s"][0]))
        t0 = time.perf_counter()
        for k in range(n):
            sim.step(float(ref[k]), k)
            s = read_step(bench, sim, res, k)
            log["act"][k] = s["act"]
            log["thar"][k] = s["thar_deg"]
            log["ths"][k] = s["th_s"]
            log["tau"][k] = s["tau_j"]
            log["iq"][k] = s["i_q"]
            if k % every == 0 or k == n - 1:
                dash.update(dict(
                    **{f: s[f] for f in ddash.SCALARS},
                    x=phase[:k + 1:stride],
                    y_act=log["act"][:k + 1:stride], y_thj=log["act"][:k + 1:stride],
                    y_thar=log["thar"][:k + 1:stride], y_ths=log["ths"][:k + 1:stride],
                    y_tau=log["tau"][:k + 1:stride], y_iq=log["iq"][:k + 1:stride],
                    foot=footer(bench, n, dt),
                    status=f"headless   cycle {cyc}   step {k + 1}/{n}"))
        el = time.perf_counter() - t0
        fin = np.isfinite(log["tau"])
        print(f"  cycle {cyc}   peak |tau_j| {np.nanmax(np.abs(log['tau'])):8.3f} N.m | "
              f"peak |I_q| {np.nanmax(np.abs(log['iq'])):7.3f} A | peak |theta_s| "
              f"{np.nanmax(np.abs(log['ths'])):9.6f} rad | all finite: {bool(fin.all())} "
              f"| {n * dt / max(el, 1e-9):.1f}x real time")
    dash.close()
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(
        description=f"{BANNER}. {DISCLAIMER}. Live view of the AB19 reference driving "
                    f"the CAD OSL V2 bench through the Best et al. actuator and belt "
                    f"(fixed-base bench -- NOT human walking).")
    ap.add_argument("--csv", default=DEFAULT_REFERENCE_CSV)
    ap.add_argument("--speed", type=float, default=0.5, choices=SPEEDS,
                    help="playback rate: 0.25, 0.5 (default) or 1.0 x real time. "
                         "Scales steps per frame and the frame budget only -- dt never "
                         "changes.")
    ap.add_argument("--max-cycles", type=int, default=0, help="0 = loop forever")
    ap.add_argument("--zoom", type=float, default=0.45,
                    help="multiplies the derived camera distance; <1 = model larger")
    ap.add_argument("--elevation", type=float, default=-6.0)
    ap.add_argument("--azimuth", type=float, default=None)
    ap.add_argument("--distance", type=float, default=None)
    ap.add_argument("--show-joints", action="store_true")
    ap.add_argument("--dashboard", choices=ddash.BACKENDS, default="auto",
                    help="second window backend (default auto: browser, else an "
                         "in-terminal readout). 'none' disables it.")
    ap.add_argument("--port", type=int, default=8788,
                    help="port for the browser dashboard on 127.0.0.1 (0 = any free). "
                         "Defaults to 8788 so it cannot collide with the servo demo's "
                         "8787 if both are open.")
    ap.add_argument("--no-browser", action="store_true",
                    help="serve the dashboard but do not open a browser automatically")
    ap.add_argument("--headless", action="store_true",
                    help="no MuJoCo window: run the loop and feed the dashboard only")
    ap.add_argument("--dump-html", default=None,
                    help="write the dashboard page to this file and exit (inspection)")
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args()

    print("=" * 78)
    print(f"  {BANNER}")
    print(f"  {DISCLAIMER}")
    print("=" * 78)

    built = build(args)
    if isinstance(built, int):
        return built
    bench, sim, layer, src, res, n = built
    dt = bench.timestep

    print(f"  model    models/osl_v2_bench.xml   dt {dt * 1e3:.3f} ms   "
          f"knee authority +/-{bench.force_limit:.1f} N.m")
    print(f"  input    {SUBJECT} {TRIAL}")
    print(f"           {n} steps = one gait cycle")
    print(f"  gains    Kp {KP:g} N.m/rad   Kd {KD:g} N.m.s/rad   (imported, not "
          f"settable here)")
    print(f"  plant    Best et al. 2025 actuator + belt; k_t_joint {K_T_JOINT:.6f} "
          f"N.m/A; n_t {PAPER.n_t:g}")
    print(f"  law      {type(src).__name__}, derivative on "
          f"{PDCurrentSource.FEEDBACK_SIDE}-side velocity")

    # ---- size the axes from a real cycle, audit the servo, refuse on divergence ----
    pk = probe(bench, sim, res, n)
    if pk["first_bad"] is not None:
        print(f"\nREFUSING TO OPEN A WINDOW: the run goes non-finite at step "
              f"{pk['first_bad']} (t = {pk['first_bad'] * dt:.4f} s). A demo of a "
              f"diverging simulation is not a demo. See "
              f"docs/DRIVETRAIN_INSTABILITY_DIAGNOSIS.md.")
        return 1
    print(f"  servo    position actuator DISCONNECTED: servo_connected="
          f"{sim.servo_connected}; across {pk['steps']} stepped samples the knee "
          f"position actuator contributed {pk['servo_violations']} nonzero force(s), "
          f"peak |force| {pk['servo_peak']:.3e} N.m")
    if pk["servo_violations"]:
        print("\nREFUSING TO OPEN A WINDOW: the servo acted on at least one step, so "
              "the belt and the servo would both be driving the joint and every number "
              "on screen would be a double count.")
        return 1
    print(f"           so the belt is the only actuation -- that zero is the proof, not "
          f"a missing measurement")
    print(f"  dry run  peak |tau_j| {pk['tau']:.3f} N.m | peak |I_q| {pk['iq']:.3f} A | "
          f"peak |theta_s| {pk['ths']:.6f} rad ({pk['ths'] / FIT_EDGE_RAD:.2f}x fit "
          f"edge) | peak |theta_a_dot| {pk['thad']:.3f} rad/s")
    print(f"           axes sized from these, so nothing on screen can clip")

    static, limits = build_axes(res, n, pk)

    if args.dump_html:
        d = ddash.DrivetrainWebDashboard(dict(subject=SUBJECT, trial=TRIAL), static,
                                         limits, port=0, open_browser=False)
        with open(args.dump_html, "wb") as fh:
            fh.write(d.html)
        d.close()
        print(f"\n  wrote {args.dump_html} ({len(d.html)} bytes) -- page only, no run")
        return 0

    dash = ddash.make_drivetrain_dashboard(
        dict(subject=SUBJECT, trial=TRIAL), static, limits,
        force=args.dashboard, port=args.port, open_browser=not args.no_browser,
        every=0.0 if args.headless else 0.10)

    if args.headless:
        rc = run_headless(bench, sim, res, n, dash, args)
    else:
        cam = vw.derive_camera(bench, res, azimuth=args.azimuth,
                               elevation=args.elevation, distance=args.distance)
        rc = run(bench, sim, res, n, cam, dash, args)

    ok = bench.limits_unchanged()
    print("-" * 78)
    print(f"  [F] knee forcerange {bench.knee_forcerange} and ctrlrange "
          f"{bench.knee_ctrlrange} unchanged: {ok}")
    print(f"      the model XML was never opened for writing; no parameter, gain or "
          f"timestep was changed by this demo.")
    print(f"  {DISCLAIMER}")
    return rc if ok else 1


if __name__ == "__main__":
    sys.exit(main())
