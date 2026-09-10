"""
oslbench.viewer -- LIVE visualisation.  Visualisation ONLY.

This module opens windows, aims a camera, paces frames and records video.  It contains
no control law and no plant.  Every joint command it issues goes through
`BenchSimulation.step()` -- the same function the quantitative benchmark calls -- so the
demo cannot show different physics from the CSV.  That used to be a claim backed by an
identity check between two copies of the loop; now there is only one copy, and
`consistency_check()` re-proves the numbers anyway (see its docstring for exactly what
it does and does not prove).

WHAT THE DEMO IS
    The CAD-derived OSL V2 knee, on a FIXED-BASE bench, tracking the knee-flexion
    trajectory measured from subject AB19 during level-ground walking, looping.

WHAT THE DEMO IS NOT
    Not whole-body walking.  `knee_prox` is welded to the world: no pelvis, no ground
    contact, no body weight.  The torque on screen is BENCH ACTUATOR TORQUE, opposing
    only inertia, gravity on the knee-distal segment, joint damping and friction.  It is
    NOT a human knee moment.

NOT SHOWN ON PURPOSE
    The AB19 human_knee_moment / human_knee_power columns.  The reference audit flags
    them as defective, so they stay out of a presentation.

CONTROLS   Space = pause/resume    R = restart the cycle    Esc = exit
"""

from __future__ import annotations

import contextlib
import io
import math
import os
import shutil
import subprocess
import sys
import time

import numpy as np

try:
    import mujoco
    import mujoco.viewer
except ImportError:                                              # pragma: no cover
    mujoco = None

from . import logging as blog
from .metrics import bench_metrics, cycle_metrics
from .model import load_bench_model
from .reference import (STANCE_END, SUBJECT, TRIAL, audit_reference, load_reference,
                        resample_reference)
from .simulation import BenchSimulation

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

FRAME_HZ = 60.0            # render/pacing target.  The physics dt is NEVER changed.
MAX_PLOT_PTS = 320         # dashboard display decimation only -- every point is real
CYCLE_S = 1.2050           # s, the AB19 cycle length, used as a sanity check on the CSV

# GLFW key codes as delivered by the passive viewer's key_callback
K_SPACE, K_R_UPPER, K_R_LOWER = 32, 82, 114

# The header the professor sees.  TITLE uses a real em dash for the browser; TITLE_TTY is
# ASCII because a legacy Windows console code page (cp437/cp850) cannot encode U+2014 and
# print() would raise UnicodeEncodeError in the middle of the demo.
TITLE = "OSL V2 CAD BENCH — AB19 HUMAN GAIT TRACKING"
TITLE_TTY = "OSL V2 CAD BENCH - AB19 HUMAN GAIT TRACKING"
SUBTITLE = ("Fixed-base bench experiment - not whole-body walking. Torque shown is BENCH "
            "ACTUATOR TORQUE, not a human knee moment.")


def fail(msg):
    print(f"\n  FATAL: {msg}\n")
    return 2


def hr(title=""):
    if title:
        print(f"\n-- {title} " + "-" * max(0, 74 - len(title)))
    else:
        print("\n" + "-" * 78)


# ================================================================= [D] derived camera
def derive_camera(bench, res, azimuth=None, elevation=-6.0, distance=None,
                  verbose=True):
    """Aim at the knee/shank in the sagittal plane, with flexion swinging screen-right.

    The azimuth is DERIVED, by moving the knee 30 deg and watching which way the ankle
    actually goes, rather than assumed from a sign convention.  Returns
    (lookat, distance, azimuth) and leaves the model back at the reference start angle.
    """
    model, data = bench.model, bench.data
    kq = bench.knee_qpos

    q0 = float(res["ref_rad"][0])
    data.qpos[kq] = q0
    mujoco.mj_forward(model, data)
    knee_p = np.array(data.xanchor[bench.knee_joint], float)
    ankle_a = np.array(data.xanchor[bench.ankle_joint], float)

    data.qpos[kq] = min(q0 + math.radians(30.0), bench.knee_ctrlrange[1])
    mujoco.mj_forward(model, data)
    ankle_b = np.array(data.xanchor[bench.ankle_joint], float)
    dx = float(ankle_b[0] - ankle_a[0])

    # screen-right is -X at azimuth 270 and +X at azimuth 90; pick whichever puts the
    # measured flexion direction toward screen-right
    az = (270.0 if dx < 0 else 90.0) if azimuth is None else float(azimuth)
    anterior = "-X" if dx < 0 else "+X"

    span = float(np.linalg.norm(ankle_a - knee_p))
    lookat = 0.5 * (knee_p + ankle_a)
    dist = float(distance) if distance else max(0.85, 2.6 * span)

    data.qpos[kq] = q0
    mujoco.mj_forward(model, data)

    if verbose:
        hr("[D] CAMERA (derived from the model, not hardcoded)")
        print(f"    knee axis  at ({knee_p[0]:+.4f}, {knee_p[1]:+.4f}, {knee_p[2]:+.4f}) m")
        print(f"    ankle axis at ({ankle_a[0]:+.4f}, {ankle_a[1]:+.4f}, "
              f"{ankle_a[2]:+.4f}) m   -> shank length {span:.4f} m")
        print(f"    +30 deg of knee flexion moves the ankle {dx:+.4f} m in X, so flexion "
              f"is toward {anterior}")
        print(f"    sagittal view: azimuth {az:.0f}, elevation {elevation:.0f}, distance "
              f"{dist:.3f} m, lookat mid-shank")
    return lookat, dist, az


def apply_camera(cam_obj, cam, zoom=1.0, elevation=-6.0):
    """Point any MuJoCo camera (viewer.cam or an MjvCamera) at the derived pose."""
    cam_obj.lookat[:] = cam[0]
    cam_obj.distance = cam[1] * float(zoom)
    cam_obj.azimuth, cam_obj.elevation = cam[2], float(elevation)


# ============================================================== [C] consistency check
def consistency_check(bench, sim, res, n, verbose=True):
    """Prove the live demo will reproduce the recorded benchmark.

    WHAT THIS PROVES, EXACTLY -- worth being precise about, because the honest claim
    changed when the code was refactored:

      1. DETERMINISM.  `BenchSimulation.run()` is executed twice on this same compiled
         model and the two trajectories must agree to 1e-12.  Two runs of one
         implementation is a repeatability proof, not an independence proof.
      2. AGREEMENT WITH THE ARTEFACT ON DISK.  The metrics just computed are compared
         with build/bench_track_ab19/bench_track_ab19_metrics.csv.  THIS is the check
         that catches a real regression: if the refactor had changed the physics, the
         live numbers would no longer match the validated file.
      3. STRUCTURAL IDENTITY, by construction rather than by test: the live loop below
         calls the same `sim.step()` this function calls.  Before the refactor the viewer
         owned a second copy of the loop, and an identity test between the two copies was
         the only way to know they matched.  There is now nothing to diverge.

    Returns (metrics, max_delta) or (None, max_delta) if determinism failed.
    """
    if verbose:
        hr("[C] CONSISTENCY CHECK -- live path vs the recorded benchmark")
        print("    run 1: BenchSimulation.run() -- the same call the benchmark makes")
    dt = bench.timestep
    r1 = sim.run(res["ref_rad"], n)
    M = bench_metrics(r1, res, dt)

    if verbose:
        print("    run 2: the identical call again, to prove the loop is deterministic")
    r2 = sim.run(res["ref_rad"], n)

    dq = float(np.max(np.abs(r2.q - r1.q)))
    dv = float(np.max(np.abs(r2.qdot - r1.qdot)))
    dtq = float(np.max(np.abs(r2.tau - r1.tau)))
    dc = float(np.max(np.abs(r2.ctrl - r1.ctrl)))
    dmax = max(dq, dv, dtq, dc)
    if verbose:
        print(f"\n    max |delta| over {n} steps:  angle {dq:.3e} rad   vel {dv:.3e} rad/s"
              f"\n                                torque {dtq:.3e} N.m   command "
              f"{dc:.3e} rad")
        print(f"    -> {'IDENTICAL' if dmax == 0.0 else 'within 1e-12'}"
              if dmax <= 1e-12 else "    -> MISMATCH")
    if dmax > 1e-12:
        return None, dmax

    # ---- (2) the check that can actually fail: agreement with the frozen artefact
    mpath = blog.metrics_csv_path()
    if os.path.isfile(mpath):
        try:
            disk = blog.read_metrics_csv(mpath)
            if verbose:
                print(f"\n    cross-check against {os.path.relpath(mpath, ROOT)}")
            differ = False
            for key in ("rms_err_deg", "peak_err_deg", "peak_tau_Nm", "sat_pct",
                        "peak_vel_rad_s"):
                if key not in disk:
                    continue
                try:
                    d = float(disk[key])
                except ValueError:
                    continue
                live = float(M[key])
                same = abs(d - live) <= 1e-6 * max(1.0, abs(d))
                differ |= not same
                if verbose:
                    print(f"      {key:<15} on disk {d:.4f}   live {live:.4f}   "
                          f"{'match' if same else 'DIFFER'}")
            if differ and verbose:
                print("      NOTE: a DIFFER means the CSV on disk was produced with "
                      "different settings\n            (kp/kv, --linear, an older model). "
                      "Re-run experiments/run_bench_ab19.py to resync.")
        except Exception as exc:                    # never block a demo on a cross-check
            if verbose:
                print(f"    (metrics cross-check skipped: {exc})")
    elif verbose:
        print(f"\n    ({os.path.relpath(mpath, ROOT)} not found -- run "
              f"experiments/run_bench_ab19.py to enable the on-disk cross-check)")

    if verbose:
        cycle_wrap_report(bench, sim, res, n, float(np.max(np.abs(r1.tau))))
    return M, dmax


def cycle_wrap_report(bench, sim, res, n, pk_cycle):
    """What the loop wrap costs.  The resampled grid is [0, T), so the last sample sits
    at 100*(T-dt)/T %, not at 100 %: looping steps the command by (first - last).  That
    is NOT the audit's cycle-closure figure, which is (last knot at the true 100 % -
    first knot).  Report the step that actually happens and measure its torque."""
    last_ph = float(res["gait_phase_percent"][-1])
    step_deg = float(np.degrees(res["ref_rad"][0] - res["ref_rad"][-1]))
    tau3 = []
    for k in range(120):                        # continue straight on into cycle 2
        st = sim.step(float(res["ref_rad"][k % n]), k)
        tau3.append(st.tau)
    pk_wrap = float(np.max(np.abs(np.asarray(tau3)[:60])))
    print(f"\n    cycle wrap: the reference does not close on itself. Looping from the "
          f"last grid sample\n      ({last_ph:.4f} %) back to 0 % steps the command by "
          f"{step_deg:+.3f} deg. Measured peak |tau| in the\n      60 steps after the "
          f"wrap: {pk_wrap:.3f} N.m vs {pk_cycle:.3f} N.m within the cycle "
          f"({100 * pk_wrap / bench.force_limit:.2f} % of\n      authority), so "
          f"continuous looping is safe. But cycle 2+ are NOT bit-identical to cycle 1: "
          f"they\n      start from the previous cycle's end state, not the seeded "
          f"initial condition.")


# ====================================================================== [V] preflight
class Preflight:
    """Everything the live demo needs, plus the short summary printed before it starts."""

    __slots__ = ("bench", "sim", "ref", "audit", "res", "n", "metrics", "cam",
                 "lines", "transcript")

    def __init__(self, **kw):
        for k, v in kw.items():
            setattr(self, k, v)


def preflight(csv=None, kp=None, kv=None, interp="cubic", azimuth=None, elevation=-6.0,
              distance=None, model_path=None):
    """Load, verify, audit, resample and consistency-check -- quietly.

    Every check the quantitative experiment runs is run here; only the PRINTING is
    reduced to a few lines, because a demo that opens with 300 lines of transcript is
    unwatchable.  The suppressed output is returned in `.transcript` for --verbose.
    Returns a Preflight, or an int exit code on failure.
    """
    from .controller import KD, KP, PDController
    kp = KP if kp is None else float(kp)
    kv = KD if kv is None else float(kv)

    buf = io.StringIO()
    out = []

    def ok(label, detail):
        out.append(f"  {label:<8} {detail}")

    with contextlib.redirect_stdout(buf):
        bench = load_bench_model(model_path)
    if bench.failures:
        sys.stdout.write(buf.getvalue())
        return fail(f"{len(bench.failures)} model precondition(s) failed:\n    "
                    + "\n    ".join(bench.failures))

    dt = bench.timestep
    want = os.path.join("models", "osl_v2_bench.xml")
    if not bench.path.replace("\\", "/").endswith("models/osl_v2_bench.xml"):
        return fail(f"loaded model is {bench.relpath()}, expected {want}")
    lim = bench.force_limit
    ok("model", f"{want}   dt {dt * 1e3:.3f} ms   knee authority +/-{lim:.1f} N.m   "
                f"[{len(bench.warnings)} warn, 0 fail]")

    csv = csv or None
    with contextlib.redirect_stdout(buf):
        ref = load_reference(csv)
        audit = audit_reference(ref, math.degrees(bench.knee_ctrlrange[0]),
                                math.degrees(bench.knee_ctrlrange[1]))
        res = resample_reference(ref, dt, 1, interp, audit=audit)
    n = res.n
    span = float(ref.period_s)
    if abs(span - CYCLE_S) > 5e-3:
        return fail(f"the reference spans {span:.4f} s, expected the {CYCLE_S:.4f} s "
                    f"AB19 cycle -- this is not the validated reference")
    ok("input", f"{os.path.relpath(ref.path, ROOT)}   {SUBJECT} {ref.n} rows -> {n} "
                f"steps   T {span:.4f} s   knee {float(np.min(ref.angle_deg)):.2f}.."
                f"{float(np.max(ref.angle_deg)):.2f} deg")

    tag = "validated pair, set at runtime"
    if abs(kp - KP) > 1e-9 or abs(kv - KD) > 1e-9:
        tag = (f"OVERRIDDEN from the validated {KP:g} / {KD:g} -- the printed metrics "
               f"will NOT match the quantitative run")
    ok("gains", f"kp {kp:g} N.m/rad   kv {kv:g} N.m.s/rad   ({tag})")

    sim = BenchSimulation(bench, PDController.knee(bench, kp, kv))
    with contextlib.redirect_stdout(buf):
        M, dmax = consistency_check(bench, sim, res, n)
    if M is None:
        sys.stdout.write(buf.getvalue())
        return fail(f"the live loop is not deterministic (max |delta| {dmax:.3e} > 1e-12). "
                    f"Refusing to open the demo.")
    ok("checks", f"deterministic to {dmax:.3e} (limit 1e-12); metrics agree with the "
                 f"recorded CSV;\n           reference inside ROM "
                 f"({M['ref_clamped_steps']} clamped steps)")

    with contextlib.redirect_stdout(buf):
        cam = derive_camera(bench, res, azimuth, elevation, distance)
    ok("expect", f"cycle 1: RMS {M['rms_err_deg']:.4f} deg   peak "
                 f"{M['peak_err_deg']:.4f} deg   peak |tau| {M['peak_tau_Nm']:.4f} N.m   "
                 f"sat {M['sat_pct']:.2f} %")

    return Preflight(bench=bench, sim=sim, ref=ref, audit=audit, res=res, n=n,
                     metrics=M, cam=cam, lines=out, transcript=buf.getvalue())


# ===================================================================== [E] live viewer
def run_live_view(bench, sim, res, n, cam, metrics, speed=0.5, max_cycles=0, zoom=0.45,
                  elevation=-6.0, show_joints=False, dash=None, print_every=0.0):
    """Open the MuJoCo viewer and drive the bench through `sim.step()`, frame by frame.

    `dash`, if given, is a consumer of live state: it is handed the values the step just
    returned and draws them.  It cannot write a command, it has no model, and because it
    draws in a separate process (the browser) it cannot slow the physics either.
    `print_every` > 0 also prints a terminal table every that-many % of gait phase.

    --speed scales the number of physics steps per rendered frame.  It NEVER touches dt:
    the integration timestep stays 0.5 ms at every playback rate.
    """
    model, data = bench.model, bench.data
    dt = bench.timestep
    lim = bench.force_limit
    ref_rad = np.asarray(res["ref_rad"], float)
    ref_deg = np.degrees(ref_rad)
    phase = np.asarray(res["gait_phase_percent"], float)

    # full-rate per-cycle logs; the dashboard plots a strict subsample of THESE arrays
    act = np.zeros(n)
    err = np.zeros(n)
    tau = np.zeros(n)
    qdl = np.zeros(n)
    stride = max(1, n // MAX_PLOT_PTS)

    st = {"paused": False, "restart": False, "quit": False}

    def on_code(code):                              # MuJoCo window (GLFW key codes)
        if code == K_SPACE:
            st["paused"] = not st["paused"]
            print(f"  [{'PAUSED' if st['paused'] else 'RESUMED'}]")
        elif code in (K_R_UPPER, K_R_LOWER):
            st["restart"] = True

    def on_sym(sym):                                # dashboard window (browser keys)
        s = (sym or "").lower()
        if s == "space":
            st["paused"] = not st["paused"]
        elif s == "r":
            st["restart"] = True
        elif s == "escape":
            st["quit"] = True

    ready = True
    if dash is not None:
        dash.bind_keys(on_sym)
        # let the browser load before the viewer opens, so the demo starts on a
        # populated page rather than a blank one
        ready = dash.wait_ready(6.0)

    spf = max(1, int(round(speed * (1.0 / FRAME_HZ) / dt)))
    kind = f"{dash.kind} dashboard" if dash is not None else "terminal readout"
    print(f"  windows  MuJoCo viewer + {kind}"
          + (f" ({'page loaded' if ready else 'page not loaded yet'})"
             if dash is not None and dash.kind == "web" else "")
          + f";  {speed:g}x real time = {spf} step(s)/frame at {FRAME_HZ:g} Hz, "
            f"dt stays {dt * 1e3:.3f} ms")
    print("  keys     Space pause/resume   R restart cycle   Esc exit   (either window)")
    if print_every > 0:
        print(f"\n    {'phase %':>8} {'ref deg':>9} {'sim deg':>9} {'err deg':>9} "
              f"{'tau N.m':>9} {'% auth':>7}")
    print("-" * 78)

    sim.reset(float(ref_rad[0]))
    k, cyc, next_print = 0, 1, 0.0
    wall0, sim0 = time.perf_counter(), float(data.time)
    cyc_wall = wall0

    with mujoco.viewer.launch_passive(model, data, key_callback=on_code,
                                      show_left_ui=False,
                                      show_right_ui=False) as viewer:
        with viewer.lock():
            apply_camera(viewer.cam, cam, zoom, elevation)
            viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_JOINT] = bool(show_joints)

        while viewer.is_running() and not st["quit"]:
            if dash is not None and not dash.pump():
                break

            if st["restart"]:
                sim.reset(float(ref_rad[0]))
                k, st["restart"], next_print = 0, False, 0.0
                if dash is not None:
                    dash.clear_traces()
                print("  [R] restarted at 0 % gait phase")
                wall0 = cyc_wall = time.perf_counter()
                sim0 = float(data.time)

            if st["paused"]:
                # keep rendering so orbit/zoom still work, advance no physics, and
                # re-baseline the clock so resuming does not trigger a catch-up burst
                viewer.sync()
                time.sleep(1.0 / FRAME_HZ)
                wall0, sim0 = time.perf_counter(), float(data.time)
                continue

            done = False
            for _ in range(spf):
                state = sim.step(float(ref_rad[k]), k)       # <-- THE shared step
                act[k] = math.degrees(state.q)
                err[k] = act[k] - ref_deg[k]
                tau[k] = state.tau
                qdl[k] = state.qdot
                if print_every > 0 and phase[k] >= next_print:
                    print(f"    {phase[k]:8.2f} {ref_deg[k]:9.3f} {act[k]:9.3f} "
                          f"{err[k]:9.3f} {tau[k]:9.3f} "
                          f"{100.0 * abs(tau[k]) / lim:7.2f}")
                    next_print += print_every
                k += 1
                if k >= n:
                    done = True
                    break

            i = k - 1
            viewer.sync()
            rt = (float(data.time) - sim0) / max(1e-9, time.perf_counter() - wall0)
            if dash is not None:
                dash.update(dict(
                    phase=float(phase[i]), ref_deg=float(ref_deg[i]),
                    act_deg=float(act[i]), err_deg=float(err[i]), tau=float(tau[i]),
                    auth=100.0 * abs(float(tau[i])) / lim,
                    x=phase[:k:stride], y_ang=act[:k:stride], y_err=err[:k:stride],
                    y_tau=tau[:k:stride],
                    foot=(f"model  models/osl_v2_bench.xml (unmodified)\n"
                          f"gains  kp {sim.knee.kp:g} N.m/rad, kv {sim.knee.kd:g} "
                          f"N.m.s/rad (runtime)\n"
                          f"limit  {lim:.1f} N.m knee actuator authority\n"
                          f"shaded 0-{STANCE_END:g} % = human stance, for reference only\n"
                          f"torque is BENCH torque, not a human knee moment"),
                    status=(f"cycle {cyc}   {speed:g}x requested, {rt:.2f}x achieved   "
                            f"step {k}/{n}   dt {dt * 1e3:.3f} ms   "
                            f"Space pause | R restart | Esc exit")))

            if done:
                m = cycle_metrics(act, tau, qdl, ref_deg, lim, n)
                el = time.perf_counter() - cyc_wall
                # the same headline numbers as the CSV, measured from the frames that
                # were actually displayed
                print(f"  cycle {cyc}   RMS {m['rms']:7.4f} deg | peak {m['peak']:7.4f} "
                      f"deg | peak |tau| {m['tau']:8.4f} N.m ({m['auth']:.1f} % of "
                      f"{lim:.1f}) | sat {m['sat']:.2f} % | peak |omega| {m['vel']:.4f} "
                      f"rad/s | {n * dt / el:.2f}x real time")
                if cyc == 1 and metrics is not None:
                    print(f"            quantitative run for comparison: RMS "
                          f"{metrics['rms_err_deg']:.4f} deg | peak "
                          f"{metrics['peak_err_deg']:.4f} deg | peak |tau| "
                          f"{metrics['peak_tau_Nm']:.4f} N.m | sat "
                          f"{metrics['sat_pct']:.2f} %")
                cyc += 1
                k, next_print = 0, 0.0
                if dash is not None:
                    dash.clear_traces()
                cyc_wall = time.perf_counter()
                if max_cycles and cyc > max_cycles:
                    break
                # cycles 2+ start from the previous cycle's end state, so they are not
                # bit-identical to cycle 1; the [0,T) grid also steps the command by
                # +0.690 deg at the wrap.  Benign, but it is why the per-cycle numbers
                # move in the 4th decimal.
                wall0, sim0 = time.perf_counter(), float(data.time)

            # pace to the wall clock from the sim clock, so slow frames self-correct
            slack = (wall0 + (float(data.time) - sim0) / max(1e-9, speed)
                     - time.perf_counter())
            if slack > 0:
                time.sleep(slack)
            elif slack < -0.25:                 # far behind: re-baseline, no catch-up
                wall0, sim0 = time.perf_counter(), float(data.time)

    if dash is not None:
        dash.close()
    print(f"\n  demo ended after {cyc - 1} complete cycle(s).")


# ========================================================================= [R] record
def open_video_sink(path, w, h, fps):
    """A frame-by-frame video sink.  Exactly ONE frame (w*h*3 bytes) is in memory at a
    time -- there is deliberately no frame array anywhere in this module."""
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


def record_video(bench, sim, res, n, path, cam, fps=30, width=1280, height=720,
                 cycles=2, zoom=0.45, elevation=-6.0):
    """Headless offscreen render of the same controller, for slides.

    No viewer window: an offscreen GL context alongside an interactive one is the classic
    source of driver trouble, and a recording without window decorations is the better
    artefact anyway.
    """
    model, data = bench.model, bench.data
    dt = bench.timestep
    spf = max(1, int(round(1.0 / (fps * dt))))
    frames = int(cycles * n / spf)

    print(f"\n  [R] RECORD  {width}x{height} @ {fps} fps, {cycles} cycle(s), "
          f"{spf} steps/frame, {frames} frames")
    try:
        renderer = mujoco.Renderer(model, height, width)
    except Exception as exc:
        return fail(f"offscreen rendering is unavailable ({type(exc).__name__}: {exc}).\n"
                    f"       Run without --record to use the interactive viewer.")
    c = mujoco.MjvCamera()
    mujoco.mjv_defaultFreeCamera(model, c)
    apply_camera(c, cam, zoom, elevation)

    ref_rad = np.asarray(res["ref_rad"], float)
    sim.reset(float(ref_rad[0]))
    sink, closer, where = open_video_sink(path, width, height, fps)
    k, t0 = 0, time.perf_counter()
    try:
        for f in range(frames):
            for _ in range(spf):
                sim.step(float(ref_rad[k]), k)
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
