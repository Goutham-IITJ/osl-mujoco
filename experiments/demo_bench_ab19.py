#!/usr/bin/env python3
"""
demo_bench_ab19.py -- PRESENTATION-QUALITY live demo of the validated AB19 bench
                      knee-tracking experiment.  Nothing about the physics is new.

WHAT THIS SHOWS
    The Open-Source Leg V2 knee, as built from the Onshape CAD export, tracking the
    knee-angle trajectory of a real human subject (AB19) through one complete gait
    cycle, looping continuously.  Angles, torque and error on the dashboard are read
    out of the SAME MuJoCo step that produced the frame you are looking at.

WHAT THIS IS NOT
    Not whole-body walking.  models/osl_v2_bench.xml is a FIXED-BASE bench: the knee's
    parent body is welded to the world, there is no pelvis, no ground contact and no
    body weight.  The actuator therefore fights only inertia, gravity on the shank and
    foot, joint damping and friction.  The torque shown is BENCH ACTUATOR TORQUE and is
    NOT a human knee moment; only the swing phase (~60-100 %) is loosely comparable.

ONE CONTROLLER, NOT TWO
    This file contains no control law.  It imports view_bench_ab19, which imports
    bench_track_ab19, which imports gain_sweep_bench, and drives the joint through the
    single shared step_once().  Before the window opens, §[V] replays
    bench_track_ab19.simulate() through that same function and refuses to start unless
    the two agree to 1e-12.  So "the demo and the numbers come from the same code" is a
    checked property, not a claim.

NOT MODIFIED BY THIS SCRIPT
    models/osl_v2_bench.xml, bench_track_ab19.py, view_bench_ab19.py,
    gain_sweep_bench.py, gain_sweep_analytic.py, build/AB19_knee_gait_reference.csv,
    KA_L1, MyoAssist.  Gains are written only into the in-memory compiled mjModel, and
    §[F] re-asserts forcerange/ctrlrange afterwards.  The demo writes nothing except an
    optional recording.

NOT SHOWN, ON PURPOSE
    The human_knee_moment / human_knee_power columns of the AB19 CSV.  Those two are
    known-defective (bench_track_ab19.audit_reference §R4: they are one signal scaled,
    correlated at 0.9996, with a ~1000x scale error and a start-up transient).  They are
    unverified, so they stay out of the presentation.

TWO WINDOWS, ONE SIMULATION
    Window 1 is the MuJoCo passive viewer.  Window 2 is a dashboard served on
    127.0.0.1 and opened in your browser automatically (stdlib http.server; no install,
    no internet, no CDN).  Both are fed by the SAME loop below: each frame calls
    vb.step_once() once per physics step, and the numbers the dashboard shows are the
    values that call returned, on that step.  The dashboard runs no simulation of its
    own -- it cannot, it has no model.  Because it draws in the browser's process it
    also cannot slow the physics, and it is opened once and never raised again, so it
    cannot keep stealing focus.

CONTROLS   Space = pause / resume     R = restart the cycle     Esc = exit
           Either window accepts them; the demo ends when the MuJoCo window is closed.

USAGE (Windows, the venv that runs the quantitative experiment)
    .venv\\Scripts\\python.exe experiments\\demo_bench_ab19.py
    .venv\\Scripts\\python.exe experiments\\demo_bench_ab19.py --speed 1.0
    .venv\\Scripts\\python.exe experiments\\demo_bench_ab19.py --record build\\demo.mp4
    .venv\\Scripts\\python.exe experiments\\demo_dashboard.py     (dashboard only, no sim)
"""

from __future__ import annotations

import argparse
import contextlib
import io
import math
import os
import re
import shutil
import subprocess
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
if HERE not in sys.path:
    sys.path.insert(0, HERE)

import bench_track_ab19 as bt          # noqa: E402  the quantitative experiment
import demo_dashboard as dash          # noqa: E402  drawing only, no physics

# view_bench_ab19 -> bench_track_ab19 -> gain_sweep_bench.  gain_sweep_bench sys.exit()s
# at import when mujoco is missing; view_bench_ab19 already absorbs that, so importing
# it can only fail for a real reason.
import view_bench_ab19 as vb           # noqa: E402  the shared step_once / checks

try:
    import mujoco
    import mujoco.viewer
except ImportError:                                              # pragma: no cover
    mujoco = None

# The dashboard header the professor sees (real em dash).  TITLE_TTY is the same string
# for the terminal: a legacy Windows console code page (cp437/cp850) cannot encode U+2014
# and print() would raise UnicodeEncodeError mid-demo.
TITLE = "OSL V2 CAD BENCH — AB19 HUMAN GAIT TRACKING"
TITLE_TTY = "OSL V2 CAD BENCH - AB19 HUMAN GAIT TRACKING"
SUBTITLE = ("Fixed-base bench experiment - not whole-body walking. Torque shown is BENCH "
            "ACTUATOR TORQUE, not a human knee moment.")
TRIAL = "levelground / ccw / normal / 01_01"
CYCLE_S = 1.2050                       # s, from the AB19 CSV time column
MAX_PLOT_PTS = 320                     # display decimation only; see note in run_demo()


def fail(msg):
    print(f"\n  FATAL: {msg}\n")
    return 2


# ================================================================ [V] quiet verification
def verify(args):
    """Run every check the quantitative experiment runs, but print a short summary
    instead of the full transcript (requirement: concise startup, not hundreds of lines).

    Nothing is skipped or weakened -- the output is captured and re-shown by --verbose.
    Returns (info, res, n, M, cam, ankle_hold, lines) or None.
    """
    buf = io.StringIO()
    out = []

    def ok(label, detail):
        out.append(f"  {label:<8} {detail}")

    with contextlib.redirect_stdout(buf):
        info = vb.gsb.load_and_verify()
    if vb.gsb.FAILS:
        sys.stdout.write(buf.getvalue())
        return fail(f"{len(vb.gsb.FAILS)} model precondition(s) failed:\n    "
                    + "\n    ".join(vb.gsb.FAILS))

    model = info["model"]
    dt = float(model.opt.timestep)
    rel = os.path.relpath(info["path"], ROOT).replace("/", os.sep)

    # (19a) the model really is models/osl_v2_bench.xml, with the authored timestep
    want = os.path.join("models", "osl_v2_bench.xml")
    if not info["path"].replace("\\", "/").endswith("models/osl_v2_bench.xml"):
        return fail(f"loaded model is {rel}, expected {want}")
    if abs(dt - bt.DT_EXPECT) > 1e-12:
        return fail(f"timestep {dt} s != {bt.DT_EXPECT} s; the step grid would differ "
                    f"from the quantitative run")
    lim = float(info["forcerange0"][0][1])
    ok("model", f"{want}   dt {dt * 1e3:.3f} ms   knee authority +/-{lim:.1f} N.m   "
                f"[{len(vb.gsb.WARNS)} warn, 0 fail]")

    # (19b) the CSV really is the AB19 reference
    if not os.path.isfile(args.csv):
        return fail(f"reference CSV not found: {args.csv}")
    with contextlib.redirect_stdout(buf):
        ref = bt.load_reference(args.csv)
        cr = info["ctrlrange"]
        bt.audit_reference(ref, math.degrees(cr[0]), math.degrees(cr[1]))
        res, n = bt.resample(ref, dt, 1, "linear" if args.linear else "cubic")
    nrow = len(ref["gait_phase_percent"])
    span = float(ref["time_s"][-1] - ref["time_s"][0])
    if abs(span - CYCLE_S) > 5e-3:
        return fail(f"CSV spans {span:.4f} s, expected the {CYCLE_S:.4f} s AB19 cycle -- "
                    f"this is not the validated reference")
    a_lo = float(np.min(ref["human_knee_angle_deg"]))
    a_hi = float(np.max(ref["human_knee_angle_deg"]))
    ok("input", f"{os.path.relpath(args.csv, ROOT).replace('/', os.sep)}   "
                f"{bt.SUBJECT} {nrow} rows -> {n} steps   T {span:.4f} s   "
                f"knee {a_lo:.2f}..{a_hi:.2f} deg")

    # (19c) kp / kv are the validated pair
    tag = "validated pair, set at runtime"
    if abs(args.kp - bt.KP) > 1e-9 or abs(args.kv - bt.KV) > 1e-9:
        tag = (f"OVERRIDDEN from the validated {bt.KP:g} / {bt.KV:g} -- the printed "
               f"metrics will NOT match the quantitative run")
    ok("gains", f"kp {args.kp:g} N.m/rad   kv {args.kv:g} N.m.s/rad   ({tag})")

    # (19d) the live control path IS bench_track_ab19's -- proved, not asserted
    with contextlib.redirect_stdout(buf):
        M = vb.consistency_check(info, res, n, args.kp, args.kv)
    if M is None:
        sys.stdout.write(buf.getvalue())
        return fail("the live control path does NOT reproduce bench_track_ab19.py. "
                    "Refusing to open the demo: a visual that shows different physics "
                    "from the CSV would be misleading.")
    txt = buf.getvalue()
    # consistency_check() has ALREADY enforced max|delta| <= 1e-12 and returns None
    # otherwise, so M is not None means the paths match.  Parsing its line is only so the
    # actual number appears in the summary; if the format ever changes, say so honestly
    # rather than print a made-up figure.
    dmax = None
    for ln in txt.splitlines():
        if "max |delta|" in ln:
            v = [float(x) for x in re.findall(r"[-+]?\d+\.\d+e[-+]\d+", ln)]
            if v:
                dmax = max(v)
    shown = f"max|d| {dmax:.3e} (limit 1e-12)" if dmax is not None \
        else "verified <= 1e-12 inside consistency_check()"
    ok("checks", f"live control path == bench_track_ab19.simulate(): {shown};   "
                 f"reference inside ROM ({M['ref_clamped_steps']} clamped steps)")

    with contextlib.redirect_stdout(buf):
        cam = vb.derive_camera(info, res, args)
        ankle_hold = vb.init_state(info, res, args.kp, args.kv)
    ok("expect", f"cycle 1: RMS {M['rms_err_deg']:.4f} deg   peak "
                 f"{M['peak_err_deg']:.4f} deg   peak |tau| {M['peak_tau_Nm']:.4f} N.m   "
                 f"sat {M['sat_pct']:.2f} %")

    info["_lim"] = lim
    info["_cam_note"] = (f"sagittal view derived by probing the flexion direction: "
                         f"azimuth {cam[2]:.0f}, elevation {args.elevation:.0f}, "
                         f"flexion swings toward screen-right")
    return info, res, n, M, cam, ankle_hold, out, txt


# ========================================================================= [R] recording
def open_sink(path, w, h, fps):
    """Frame-by-frame video sink.  ONE frame is ever in memory (w*h*3 bytes) -- there is
    deliberately no in-memory frame array anywhere in this file."""
    ff = shutil.which("ffmpeg")
    if ff:
        cmd = [ff, "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24",
               "-s", f"{w}x{h}", "-r", str(fps), "-i", "-",
               "-c:v", "libx264", "-preset", "medium", "-crf", "20",
               "-pix_fmt", "yuv420p", path]
        p = subprocess.Popen(cmd, stdin=subprocess.PIPE)
        print(f"  recording -> {path}   (piping rgb24 to ffmpeg, one frame at a time)")
        return p.stdin, (lambda: (p.stdin.close(), p.wait())), path
    raw = os.path.splitext(path)[0] + ".rgb"
    fh = open(raw, "wb")
    print(f"  ffmpeg not on PATH -- writing a raw stream instead: {raw}")
    print(f"  convert it with:\n    ffmpeg -f rawvideo -pix_fmt rgb24 -s {w}x{h} "
          f"-r {fps} -i \"{raw}\" -c:v libx264 -pix_fmt yuv420p \"{path}\"")
    return fh, fh.close, raw


def record(info, res, n, args, ankle_hold, cam):
    """Headless offscreen render of the same controller.  No viewer window: an offscreen
    GL context plus an interactive one is the classic source of driver trouble, and a
    recording without window decorations is the better artefact anyway."""
    model, data = info["model"], info["data"]
    dt = float(model.opt.timestep)
    w, h, fps = args.record_width, args.record_height, args.record_fps
    spf = max(1, int(round(1.0 / (fps * dt))))
    frames = int(args.record_cycles * n / spf)

    print(f"\n  [R] RECORD  {w}x{h} @ {fps} fps, {args.record_cycles} cycle(s), "
          f"{spf} steps/frame, {frames} frames")
    try:
        renderer = mujoco.Renderer(model, h, w)
    except Exception as exc:
        return fail(f"offscreen rendering is unavailable ({type(exc).__name__}: {exc}).\n"
                    f"       Run without --record to use the interactive viewer.")
    c = mujoco.MjvCamera()
    mujoco.mjv_defaultFreeCamera(model, c)
    c.lookat[:] = cam[0]
    c.distance = cam[1] * args.zoom
    c.azimuth, c.elevation = cam[2], args.elevation

    sink, closer, where = open_sink(args.record, w, h, fps)
    k, t0 = 0, time.perf_counter()
    try:
        for f in range(frames):
            for _ in range(spf):
                vb.step_once(info, res["ref_rad"], k, ankle_hold)
                k = (k + 1) % n
            renderer.update_scene(data, c)
            sink.write(renderer.render().tobytes())
            if f % max(1, frames // 10) == 0:
                print(f"      {100.0 * f / frames:5.1f} %", flush=True)
    finally:
        closer()
        renderer.close()
    print(f"  wrote {where}  ({frames} frames in {time.perf_counter() - t0:.1f} s)")
    return 0


# ============================================================================ [D] demo
def cycle_metrics(act, tau, qd, ref_deg, lim, k):
    """Requirement: the same headline numbers as the quantitative experiment, measured
    from the frames actually shown."""
    e = act[:k] - ref_deg[:k]
    return dict(rms=float(np.sqrt(np.mean(e ** 2))), peak=float(np.max(np.abs(e))),
                tau=float(np.max(np.abs(tau[:k]))),
                sat=100.0 * float(np.mean(np.abs(tau[:k]) >= 0.999 * lim)),
                vel=float(np.max(np.abs(qd[:k]))),
                auth=100.0 * float(np.max(np.abs(tau[:k]))) / lim)


def run_demo(info, res, n, args, ankle_hold, cam, M):
    model, data = info["model"], info["data"]
    dt = float(model.opt.timestep)
    lim = info["_lim"]
    phase = np.asarray(res["gait_phase_percent"], float)
    ref_deg = np.degrees(np.asarray(res["ref_rad"], float))
    ref_rad = res["ref_rad"]

    # Per-cycle logs at FULL simulation rate.  The dashboard plots a strict subsample of
    # THESE arrays (stride below) purely to keep drawing cheap -- every plotted point is
    # a real simulation sample, and no value is ever recomputed or approximated for the
    # display.
    act = np.zeros(n)
    err = np.zeros(n)
    tau = np.zeros(n)
    qdl = np.zeros(n)
    stride = max(1, n // MAX_PLOT_PTS)

    # axes sized from the measured cycle that §[V] just ran, so they are derived
    pad = 6.0
    limits = dict(
        ang=(math.floor((ref_deg.min() - pad) / 5) * 5,
             math.ceil((ref_deg.max() + pad) / 5) * 5),
        err=(-math.ceil(1.3 * M["peak_err_deg"] / 5) * 5,
             math.ceil(1.3 * M["peak_err_deg"] / 5) * 5),
        tau=(-math.ceil(1.3 * M["peak_tau_Nm"] / 5) * 5,
             math.ceil(1.3 * M["peak_tau_Nm"] / 5) * 5),
        stance_end=bt.STANCE_END)
    d = dash.make_dashboard(
        TITLE, SUBTITLE,
        dict(subject=bt.SUBJECT, trial=TRIAL),
        (phase[::stride], ref_deg[::stride]), limits,
        geometry=args.dash_geometry, force=args.dashboard,
        port=args.port, open_browser=not args.no_browser)

    st = dict(paused=False, restart=False, quit=False)

    def on_code(code):                          # MuJoCo window (GLFW key codes)
        if code == vb.K_SPACE:
            st["paused"] = not st["paused"]
        elif code in (vb.K_R_UPPER, vb.K_R_LOWER):
            st["restart"] = True

    def on_sym(sym):                            # dashboard window (browser / Tk keys)
        s = (sym or "").lower()
        if s == "space":
            st["paused"] = not st["paused"]
        elif s == "r":
            st["restart"] = True
        elif s == "escape":
            st["quit"] = True

    d.bind_keys(on_sym)
    # Give the browser a moment to load before the viewer opens, so the presentation
    # starts with a populated page.  Skipped when we did not launch a browser.
    ready = d.wait_ready(0.0 if args.no_browser else 6.0)

    spf = max(1, int(round(args.speed * (1.0 / vb.FRAME_HZ) / dt)))
    print(f"  windows  MuJoCo viewer + {d.kind} dashboard"
          + (f" ({'page loaded' if ready else 'page not loaded yet'})"
             if d.kind == "web" else "")
          + f";  {args.speed:g}x real time = {spf} step(s)/frame at {vb.FRAME_HZ:g} Hz, "
            f"dt stays {dt * 1e3:.3f} ms")
    print(f"  keys     Space pause/resume   R restart cycle   Esc exit   "
          f"(either window)")
    print("-" * 78)

    k, cyc = 0, 1
    wall0, sim0 = time.perf_counter(), float(data.time)
    cyc_wall = wall0

    with mujoco.viewer.launch_passive(model, data, key_callback=on_code,
                                      show_left_ui=False,
                                      show_right_ui=False) as viewer:
        with viewer.lock():
            viewer.cam.lookat[:] = cam[0]
            viewer.cam.distance = cam[1] * args.zoom      # bigger, centred on the knee
            viewer.cam.azimuth, viewer.cam.elevation = cam[2], args.elevation
            viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_JOINT] = bool(args.show_joints)

        while viewer.is_running() and not st["quit"]:
            if not d.pump():
                break

            if st["restart"]:
                ankle_hold = vb.init_state(info, res, args.kp, args.kv)
                k, st["restart"] = 0, False
                d.clear_traces()
                print(f"  [R] restarted at 0 % gait phase")
                wall0, sim0, cyc_wall = time.perf_counter(), float(data.time), \
                    time.perf_counter()

            if st["paused"]:
                viewer.sync()
                time.sleep(1.0 / vb.FRAME_HZ)
                wall0, sim0 = time.perf_counter(), float(data.time)
                continue

            done = False
            for _ in range(spf):
                cmd, q, qd, t = vb.step_once(info, ref_rad, k, ankle_hold)
                act[k] = math.degrees(q)
                err[k] = act[k] - ref_deg[k]
                tau[k] = t
                qdl[k] = qd
                k += 1
                if k >= n:
                    done = True
                    break

            i = k - 1
            viewer.sync()
            rt = (float(data.time) - sim0) / max(1e-9, time.perf_counter() - wall0)
            d.update(dict(
                phase=float(phase[i]), ref_deg=float(ref_deg[i]),
                act_deg=float(act[i]), err_deg=float(err[i]), tau=float(tau[i]),
                auth=100.0 * abs(float(tau[i])) / lim,
                x=phase[:k:stride], y_ang=act[:k:stride], y_err=err[:k:stride],
                y_tau=tau[:k:stride],
                foot=(f"model  models/osl_v2_bench.xml (unmodified)\n"
                      f"gains  kp {args.kp:g} N.m/rad, kv {args.kv:g} N.m.s/rad "
                      f"(runtime)\n"
                      f"limit  {lim:.1f} N.m knee actuator authority\n"
                      f"shaded 0-{bt.STANCE_END:g} % = human stance, for reference only\n"
                      f"torque is BENCH torque, not a human knee moment"),
                status=(f"cycle {cyc}   {args.speed:g}x requested, {rt:.2f}x achieved   "
                        f"step {k}/{n}   dt {dt * 1e3:.3f} ms   "
                        f"Space pause | R restart | Esc exit")))

            if done:
                m = cycle_metrics(act, tau, qdl, ref_deg, lim, n)
                el = time.perf_counter() - cyc_wall
                # Requirement: the same five headline numbers at the end of EVERY cycle,
                # measured from the frames that were actually displayed.
                print(f"  cycle {cyc}   RMS {m['rms']:7.4f} deg | peak "
                      f"{m['peak']:7.4f} deg | peak |tau| {m['tau']:8.4f} N.m "
                      f"({m['auth']:.1f} % of {lim:.1f}) | sat {m['sat']:.2f} % | "
                      f"peak |omega| {m['vel']:.4f} rad/s | {n * dt / el:.2f}x real time")
                if cyc == 1:
                    print(f"            quantitative run for comparison: RMS "
                          f"{M['rms_err_deg']:.4f} deg | peak {M['peak_err_deg']:.4f} deg "
                          f"| peak |tau| {M['peak_tau_Nm']:.4f} N.m | sat "
                          f"{M['sat_pct']:.2f} %")
                cyc += 1
                k = 0
                d.clear_traces()
                cyc_wall = time.perf_counter()
                if args.max_cycles and cyc > args.max_cycles:
                    break
                # cycles 2+ start from the previous cycle's end state, so they are not
                # bit-identical to cycle 1; the [0,T) grid also steps the command by
                # +0.690 deg at the wrap (~9 % of authority for a few ms).  Benign, but
                # it is why the per-cycle numbers move in the 4th decimal.
                wall0, sim0 = time.perf_counter(), float(data.time)

            slack = wall0 + (float(data.time) - sim0) / max(1e-9, args.speed) \
                - time.perf_counter()
            if slack > 0:
                time.sleep(slack)
            elif slack < -0.25:
                wall0, sim0 = time.perf_counter(), float(data.time)

    d.close()
    print(f"\n  demo ended after {cyc - 1} complete cycle(s).")


# ============================================================================== main
def main() -> int:
    ap = argparse.ArgumentParser(
        description="Presentation-quality live demo of the AB19 bench knee-tracking "
                    "experiment (fixed-base bench -- NOT human walking).")
    ap.add_argument("--csv", default=bt.DEFAULT_CSV)
    ap.add_argument("--kp", type=float, default=bt.KP)
    ap.add_argument("--kv", type=float, default=bt.KV)
    ap.add_argument("--speed", type=float, default=0.5,
                    help="0.5 = half real time (default, easier to watch); 1.0 = real "
                         "time. Scales steps per frame only -- dt never changes.")
    ap.add_argument("--max-cycles", type=int, default=0, help="0 = loop forever")
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
    print(f"  {TITLE_TTY}")
    print(f"  {SUBTITLE}")
    print("=" * 78)

    if mujoco is None or vb.gsb is None:
        return fail("mujoco is not importable in this interpreter"
                    + (f" ({vb._GSB_EXIT})" if vb._GSB_EXIT else "") + ".\n"
                    "       Use the venv that runs the quantitative experiment:\n"
                    "         .venv\\Scripts\\python.exe experiments\\demo_bench_ab19.py")

    got = verify(args)
    if not isinstance(got, tuple):
        return got
    info, res, n, M, cam, ankle_hold, lines, txt = got
    print("\n".join(lines))
    if args.verbose:
        print("-" * 78)
        print(f"  camera   {info['_cam_note']}")
        print("-" * 78 + "\n" + txt + "-" * 78)

    if args.record:
        rc = record(info, res, n, args, ankle_hold, cam)
        if rc:
            return rc
    else:
        run_demo(info, res, n, args, ankle_hold, cam, M)

    # ------------------------------------------------- [F] the model is still authored
    model, cr = info["model"], info["ctrlrange"]
    fr_k = tuple(float(x) for x in model.actuator_forcerange[info["ka"]])
    cr_k = tuple(float(x) for x in model.actuator_ctrlrange[info["ka"]])
    ok = (abs(fr_k[0] - info["forcerange0"][0][0]) < 1e-9
          and abs(fr_k[1] - info["forcerange0"][0][1]) < 1e-9
          and abs(cr_k[0] - cr[0]) < 1e-12 and abs(cr_k[1] - cr[1]) < 1e-12)
    print(f"  [F] knee forcerange {fr_k} and ctrlrange {cr_k} unchanged: {ok}")
    print(f"      the model XML was never opened for writing.")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
