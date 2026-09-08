#!/usr/bin/env python3
"""
view_bench_ab19.py -- LIVE MuJoCo visualisation of the AB19 bench tracking experiment.

WHAT THIS IS
    The OSL V2 bench knee, driven by subject AB19's measured knee-flexion trajectory,
    shown in the standard MuJoCo passive viewer and looping in real time.  It is the
    same experiment as bench_track_ab19.py -- same model, same reference, same servo,
    same gains -- with a window instead of a CSV.

WHAT THIS IS **NOT**  (read this before showing it to anyone)
    models/osl_v2_bench.xml is a FIXED-BASE BENCH.  Its root body `knee_prox` has no
    freejoint, so the device is bolted to the world.  There is no pelvis, no torso, no
    contralateral leg, no ground contact and no body weight.

    Therefore:
      * This demonstrates ONE THING: the OSL knee joint tracking a human-derived
        reference angle.  It is joint-level trajectory tracking on a test bench.
      * It is NOT whole-body human walking, NOT a gait simulation, and NOT a
        human-prosthesis interaction study.
      * The actuator torque you see fights only inertia, gravity on the shank+foot,
        joint damping and friction.  It is NOT a human knee moment: a real knee in
        stance carries body weight, and this bench carries none.  Stance-phase torque
        here is not comparable to biomechanical stance-phase knee moment.
      * The shading convention used in the quantitative figures (0-60 % stance,
        60-100 % swing) labels the HUMAN gait cycle the reference came from.  The bench
        itself has no stance or swing.

    The honest one-line description is:
        "the CAD-derived OSL V2 knee, on a fixed bench, tracking a human knee-angle
         trajectory measured from subject AB19 during level-ground walking."

SHARED CONTROLLER -- NO SECOND IMPLEMENTATION
    This script does not contain a controller.  It imports bench_track_ab19 (which in
    turn imports gain_sweep_bench) and drives the joint through the identical
    clip -> ctrl -> mj_step sequence, factored into step_once() below and used by BOTH
    the live loop and the pre-flight consistency check.  Section [C] runs the
    quantitative experiment in-process and replays it through step_once(), and refuses
    to open the viewer unless the two trajectories agree to machine precision.

    The model XML is never written.  Gains go into the compiled mjModel at runtime.
    forcerange (+/-142.2 N.m knee) and ctrlrange are never touched, so the authored
    torque authority and ROM still bound everything seen on screen.

SCOPE
    models/osl_v2_bench.xml only.  OpenSourceLeg_KA_L1 is not used, MyoAssist is not
    used, no optimiser runs, and the 4-DOF compliant socket is not involved.

CONTROLS
    Space   pause / resume
    R       restart the gait cycle at 0 %
    Esc     close the viewer
    (the viewer's own mouse controls still work: drag = orbit, scroll = zoom)

USAGE (Windows, the venv that runs the quantitative experiment)
    .venv\\Scripts\\python.exe experiments\\view_bench_ab19.py
    .venv\\Scripts\\python.exe experiments\\view_bench_ab19.py --speed 0.35
    .venv\\Scripts\\python.exe experiments\\view_bench_ab19.py --show-joints
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "scripts"))

import bench_track_ab19 as bt              # noqa: E402  the quantitative experiment

# gain_sweep_bench sys.exit()s at import time if mujoco is missing.  Catch that so
# --help still works in an interpreter without mujoco, and report it properly instead.
_GSB_EXIT = ""
try:
    import gain_sweep_bench as gsb         # noqa: E402  the one servo implementation
except SystemExit as _exc:                                   # pragma: no cover
    gsb, _GSB_EXIT = None, str(_exc)

try:
    import mujoco
    import mujoco.viewer
except ImportError:                                          # pragma: no cover
    mujoco = None

FRAME_HZ = 60.0                 # render/pace target; physics dt is NEVER changed
METRICS_CSV = os.path.join(bt.OUTDIR, "bench_track_ab19_metrics.csv")

# GLFW key codes delivered by the passive viewer's key_callback
K_SPACE, K_R_UPPER, K_R_LOWER = 32, 82, 114


def hr(title=""):
    if title:
        print(f"\n-- {title} " + "-" * max(0, 74 - len(title)))
    else:
        print("\n" + "-" * 78)


def fatal(msg):
    print(f"\nFATAL: {msg}")
    return 2


# ==================================================================== [A] the one law
def init_state(info, res, kp, kv):
    """Seed exactly as bench_track_ab19.simulate() does, and return the ankle hold."""
    model, data = info["model"], info["data"]
    gsb.set_gains(model, info["ka"], kp, kv)
    # ankle stays on its OWN authored servo (kp=60, kv=0), holding its keyframe angle:
    # a fixed boundary condition, not a second variable
    gsb.set_gains(model, info["aa"], gsb.EXPECT["ankle_kp_authored"],
                  gsb.EXPECT["ankle_kv_authored"])
    kid = info["keyframe"]
    mujoco.mj_resetDataKeyframe(model, data, kid if kid >= 0 else 0)
    ankle_hold = float(data.qpos[info["ankle_qpos"]])
    data.qpos[info["knee_qpos"]] = float(res["ref_rad"][0])   # start ON the reference
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)
    return ankle_hold


def step_once(info, ref_rad, k, ankle_hold):
    """THE control law.  Byte-for-byte the inner loop of bench_track_ab19.simulate():
    clamp the reference to ctrlrange, write it as the servo command, step, read sensors.

    Used by the live viewer AND by the consistency check, so the two cannot drift apart.
    Returns (cmd, q, qd, tau) in rad, rad, rad/s, N.m.
    """
    model, data = info["model"], info["data"]
    cr, S = info["ctrlrange"], info["sens"]
    cmd = float(np.clip(ref_rad[k], cr[0], cr[1]))
    data.ctrl[info["ka"]] = cmd
    data.ctrl[info["aa"]] = ankle_hold
    mujoco.mj_step(model, data)
    return (cmd,
            float(data.sensordata[S["knee_q"]]),
            float(data.sensordata[S["knee_qd"]]),
            float(data.sensordata[S["knee_tau"]]))


# ======================================================= [C] prove it matches (req 17)
def consistency_check(info, res, n, kp, kv):
    """Requirement 17: prove the LIVE setup is the quantitative experiment.

    Not a similarity argument -- an identity argument.  Run bench_track_ab19.simulate()
    in this same process on this same mjModel, then replay the identical span through
    step_once(), and require the trajectories to match to machine precision.  Anything
    above 1e-12 means the viewer is showing different physics from the CSV and the
    demonstration would be misleading, so it aborts.
    """
    hr("[C] CONSISTENCY CHECK -- live setup vs the quantitative experiment")
    print("    running bench_track_ab19.simulate() in-process (the authoritative path)")
    _rows, M, tr = bt.simulate(info, res, n, kp, kv)

    print("    replaying the same span through this script's step_once()")
    ankle_hold = init_state(info, res, kp, kv)
    q2 = np.empty(n); qd2 = np.empty(n); tau2 = np.empty(n); ctrl2 = np.empty(n)
    for k in range(n):
        ctrl2[k], q2[k], qd2[k], tau2[k] = step_once(info, res["ref_rad"], k, ankle_hold)

    dq = float(np.max(np.abs(q2 - tr["q"])))
    dv = float(np.max(np.abs(qd2 - tr["qd"])))
    dt_ = float(np.max(np.abs(tau2 - tr["tau"])))
    dc = float(np.max(np.abs(ctrl2 - tr["ctrl"])))
    print(f"\n    max |delta| over {n} steps:  angle {dq:.3e} rad   vel {dv:.3e} rad/s"
          f"\n                                torque {dt_:.3e} N.m   command {dc:.3e} rad")
    ok = max(dq, dv, dt_, dc) <= 1e-12
    print(f"    -> {'IDENTICAL' if max(dq, dv, dt_, dc) == 0.0 else 'within 1e-12'}"
          if ok else "    -> MISMATCH")
    if not ok:
        return None

    # the live numbers the viewer will reproduce
    err2 = np.degrees(q2 - res["ref_rad"])
    print(f"\n    replay metrics: RMS {float(np.sqrt(np.mean(err2 ** 2))):.4f} deg"
          f"   peak {float(np.max(np.abs(err2))):.4f} deg"
          f"   peak |tau| {float(np.max(np.abs(tau2))):.4f} N.m"
          f"   sat {M['sat_pct']:.4f} %")

    # cross-check against the CSV already on disk, if the quantitative run has been done
    if os.path.isfile(METRICS_CSV):
        try:
            with open(METRICS_CSV, newline="") as fh:
                rows = [r for r in csv.reader(fh) if r and not r[0].lstrip().startswith("#")]
            disk = {r[0].strip(): r[1].strip() for r in rows if len(r) >= 2}
            print(f"\n    cross-check against {os.path.relpath(METRICS_CSV, ROOT)}")
            differ = False
            for key, live in (("rms_err_deg", M["rms_err_deg"]),
                              ("peak_err_deg", M["peak_err_deg"]),
                              ("peak_tau_Nm", M["peak_tau_Nm"]),
                              ("sat_pct", M["sat_pct"])):
                if key in disk:
                    try:
                        d = float(disk[key])
                    except ValueError:
                        continue
                    same = abs(d - live) <= 1e-6 * max(1.0, abs(d))
                    differ |= not same
                    print(f"      {key:<14} on disk {d:.4f}   live {live:.4f}   "
                          f"{'match' if same else 'DIFFER'}")
            if differ:
                print("      NOTE: a DIFFER here does NOT invalidate the identity proof "
                      "above -- that compared\n            two runs in THIS process. It "
                      "means the CSV on disk was produced with different\n            "
                      "settings (kp/kv, --linear, or an older model). Re-run "
                      "bench_track_ab19.py to resync.")
        except Exception as exc:                              # never block the demo on this
            print(f"    (metrics cross-check skipped: {exc})")
    else:
        print(f"\n    ({os.path.relpath(METRICS_CSV, ROOT)} not found -- run "
              f"bench_track_ab19.py to enable the on-disk cross-check)")

    # ---- what happens at the loop wrap?  The resampled grid is [0, T), so the last
    # sample sits at 100*(T-dt)/T %, not at 100 %.  The wrap therefore steps the command
    # by (first - last), which is NOT the audit's cycle-closure figure (last knot - first
    # knot at the true 100 %).  Report the step that actually happens, and say what it costs.
    last_ph = float(res["gait_phase_percent"][-1])
    step_deg = float(np.degrees(res["ref_rad"][0] - res["ref_rad"][-1]))
    closure_deg = float(ref_closure(res))
    pk_cycle = float(np.max(np.abs(tau2)))
    tau3 = []
    for k in range(120):                                  # continue straight into cycle 2
        _, _, _, tt = step_once(info, res["ref_rad"], k % n, ankle_hold)
        tau3.append(tt)
    pk_wrap = float(np.max(np.abs(np.asarray(tau3)[:60])))
    print(f"\n    cycle wrap: the reference does not close on itself. Looping from the last "
          f"grid sample\n      ({last_ph:.4f} %) back to 0 % steps the command by "
          f"{step_deg:+.3f} deg; the audit's cycle-closure gap\n      at the true 100 % knot "
          f"is {closure_deg:+.3f} deg. Measured peak |tau| in the 60 steps after the\n      "
          f"wrap: {pk_wrap:.3f} N.m vs {pk_cycle:.3f} N.m within the cycle "
          f"({100 * pk_wrap / info['forcerange0'][0][1]:.2f} % of authority), so continuous\n"
          f"      looping is safe. But cycle 2+ are NOT bit-identical to cycle 1: they start "
          f"from the\n      previous cycle's end state, not the seeded initial condition.")

    return M


def ref_closure(res):
    """Cycle-closure gap at the true 100 % knot, in degrees, as the audit reports it.
    The resampled grid stops at T-dt, so recover the 100 % value by extending one step."""
    dphi = float(res["gait_phase_percent"][-1] - res["gait_phase_percent"][-2])
    # linear extension of the last interval to phase 100 %; the reference is smooth there
    slope = float(res["ref_rad"][-1] - res["ref_rad"][-2])
    at100 = float(res["ref_rad"][-1]) + slope * (100.0 - res["gait_phase_percent"][-1]) / dphi
    return math.degrees(at100 - float(res["ref_rad"][0]))


# ================================================================= [D] derived camera
def derive_camera(info, res, args):
    """Requirement 13: aim at the knee/shank, in the sagittal plane, with flexion
    swinging toward screen-right.  The azimuth is DERIVED (by probing which way the
    shank actually moves under flexion) rather than assumed from a convention."""
    model, data = info["model"], info["data"]
    kq = info["knee_qpos"]

    q0 = float(res["ref_rad"][0])
    data.qpos[kq] = q0
    mujoco.mj_forward(model, data)
    knee_p = np.array(data.xanchor[info["kj"]], float)
    ankle_a = np.array(data.xanchor[info["aj"]], float)

    data.qpos[kq] = min(q0 + math.radians(30.0), info["ctrlrange"][1])
    mujoco.mj_forward(model, data)
    ankle_b = np.array(data.xanchor[info["aj"]], float)
    dx = float(ankle_b[0] - ankle_a[0])

    # screen-right at azimuth 270 is -X; at azimuth 90 it is +X.  Pick the one that puts
    # the direction the shank actually swings under flexion toward screen-right.
    az = (270.0 if dx < 0 else 90.0) if args.azimuth is None else args.azimuth
    anterior = "-X" if dx < 0 else "+X"

    span = float(np.linalg.norm(ankle_a - knee_p))
    lookat = 0.5 * (knee_p + ankle_a)
    dist = args.distance if args.distance else max(0.85, 2.6 * span)

    data.qpos[kq] = q0
    mujoco.mj_forward(model, data)

    hr("[D] CAMERA (derived from the model, not hardcoded)")
    print(f"    knee axis  at ({knee_p[0]:+.4f}, {knee_p[1]:+.4f}, {knee_p[2]:+.4f}) m")
    print(f"    ankle axis at ({ankle_a[0]:+.4f}, {ankle_a[1]:+.4f}, {ankle_a[2]:+.4f}) m"
          f"   -> shank length {span:.4f} m")
    print(f"    +30 deg of knee flexion moves the ankle {dx:+.4f} m in X, so flexion is "
          f"toward {anterior}")
    print(f"    sagittal view: azimuth {az:.0f}, elevation {args.elevation:.0f}, "
          f"distance {dist:.3f} m, lookat mid-shank")
    print(f"    -> flexion swings the shank toward SCREEN-RIGHT; the knee sits at the top "
          f"of the frame")
    return lookat, dist, az


# ===================================================================== [E] live viewer
def run_viewer(info, res, n, args, ankle_hold, cam):
    model, data = info["model"], info["data"]
    dt = float(model.opt.timestep)
    lim = info["forcerange0"][0][1]
    ref = res["ref_rad"]
    phase = res["gait_phase_percent"]
    lookat, dist, az = cam

    # physics steps per rendered frame; --speed scales this, never dt
    spf = max(1, int(round(args.speed * (1.0 / FRAME_HZ) / dt)))

    state = {"paused": False, "reset": False}

    def key_callback(keycode):
        if keycode == K_SPACE:
            state["paused"] = not state["paused"]
            print(f"  [{'PAUSED' if state['paused'] else 'RESUMED'}]")
        elif keycode in (K_R_UPPER, K_R_LOWER):
            state["reset"] = True

    hr("[E] LIVE VIEWER")
    print(f"    {spf} physics steps per frame at {FRAME_HZ:.0f} Hz -> "
          f"{args.speed:.2f}x real time (dt is unchanged at {dt * 1e3:.3f} ms)")
    print("    Space = pause/resume    R = restart at 0 %    Esc = close")
    print(f"    printing every {args.print_every:.0f} % of gait phase\n")
    print(f"    {'phase %':>8} {'ref deg':>9} {'sim deg':>9} {'err deg':>9} "
          f"{'tau N.m':>9} {'% auth':>7}")

    k = 0
    cyc = 1
    err_acc = np.zeros(n); tau_acc = np.zeros(n)
    next_print = 0.0
    wall0 = time.perf_counter()
    sim0 = float(data.time)

    with mujoco.viewer.launch_passive(model, data,
                                      key_callback=key_callback,
                                      show_left_ui=False, show_right_ui=False) as viewer:
        with viewer.lock():
            viewer.cam.lookat[:] = lookat
            viewer.cam.distance = dist
            viewer.cam.azimuth = az
            viewer.cam.elevation = args.elevation
            if args.show_joints:
                viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_JOINT] = True

        while viewer.is_running():
            if state["reset"]:
                state["reset"] = False
                init_state(info, res, args.kp, args.kv)
                k, cyc = 0, 1
                err_acc[:] = 0.0; tau_acc[:] = 0.0
                next_print = 0.0
                wall0, sim0 = time.perf_counter(), float(data.time)
                print("  [RESTART at 0 % gait phase]")

            if state["paused"]:
                # keep rendering (so orbit/zoom still work) but advance no physics, and
                # re-baseline the pacing clock so resuming does not trigger a catch-up burst
                viewer.sync()
                time.sleep(1.0 / FRAME_HZ)
                wall0, sim0 = time.perf_counter(), float(data.time)
                continue

            for _ in range(spf):
                _cmd, q, _qd, tau = step_once(info, ref, k, ankle_hold)
                err_acc[k] = math.degrees(q - ref[k])
                tau_acc[k] = tau

                if phase[k] >= next_print:
                    print(f"    {phase[k]:8.2f} {math.degrees(ref[k]):9.3f} "
                          f"{math.degrees(q):9.3f} {math.degrees(q - ref[k]):9.3f} "
                          f"{tau:9.3f} {100.0 * abs(tau) / lim:7.2f}")
                    next_print += args.print_every

                k += 1
                if k >= n:                            # ---- one gait cycle completed
                    rms = float(np.sqrt(np.mean(err_acc ** 2)))
                    pkerr = float(np.max(np.abs(err_acc)))
                    pktau = float(np.max(np.abs(tau_acc)))
                    el = time.perf_counter() - wall0
                    simel = float(data.time) - sim0
                    print(f"    -- cycle {cyc} complete: RMS {rms:.4f} deg   "
                          f"peak err {pkerr:.4f} deg   peak |tau| {pktau:.4f} N.m "
                          f"({100 * pktau / lim:.2f} % auth)   "
                          f"{simel / el if el > 0 else 0:.2f}x real time")
                    k = 0
                    cyc += 1
                    next_print = 0.0
                    if args.max_cycles and cyc > args.max_cycles:
                        print(f"    -- reached --max-cycles {args.max_cycles}, closing")
                        return

            viewer.sync()

            # pace to wall clock from the sim clock, so slow frames self-correct
            target = wall0 + (float(data.time) - sim0) / max(args.speed, 1e-6)
            slack = target - time.perf_counter()
            if slack > 0:
                time.sleep(slack)
            elif slack < -0.25:                # fell far behind: re-baseline, no catch-up burst
                wall0, sim0 = time.perf_counter(), float(data.time)


# ============================================================================== main
def main() -> int:
    ap = argparse.ArgumentParser(
        description="Live MuJoCo view of the AB19 bench knee tracking experiment "
                    "(fixed-base bench -- NOT human walking).")
    ap.add_argument("--csv", default=bt.DEFAULT_CSV, help="AB19 reference CSV")
    ap.add_argument("--kp", type=float, default=bt.KP)
    ap.add_argument("--kv", type=float, default=bt.KV)
    ap.add_argument("--speed", type=float, default=1.0,
                    help="playback rate; 1.0 = real time, 0.35 = slow motion for a demo")
    ap.add_argument("--print-every", type=float, default=10.0,
                    help="terminal print interval in %% gait phase")
    ap.add_argument("--max-cycles", type=int, default=0, help="0 = loop forever")
    ap.add_argument("--azimuth", type=float, default=None, help="override derived azimuth")
    ap.add_argument("--elevation", type=float, default=-8.0)
    ap.add_argument("--distance", type=float, default=None)
    ap.add_argument("--show-joints", action="store_true",
                    help="draw MuJoCo joint markers on the knee and ankle")
    ap.add_argument("--linear", action="store_true",
                    help="linear instead of cubic reference interpolation (diagnostic)")
    args = ap.parse_args()

    # ------------------------------------------------------------------ [1] banner (16)
    print("=" * 78)
    print("  OSL V2 BENCH -- LIVE AB19 KNEE-ANGLE TRACKING")
    print("=" * 78)
    print(f"  subject      = {bt.SUBJECT}")
    print(f"  trial        = levelground / ccw / normal / 01_01")
    print(f"  gait cycle   = 1.205 s")
    print(f"  kp           = {args.kp:g} N.m/rad")
    print(f"  kv           = {args.kv:g} N.m.s/rad")
    print(f"  model        = models/osl_v2_bench.xml   (NEVER modified; gains are set "
          f"in the compiled model at runtime)")
    print(f"  playback     = {args.speed:g}x real time, looping 0 % -> 100 % -> 0 %")
    print()
    print("  THIS IS A FIXED-BASE BENCH, NOT A WALKING HUMAN SIMULATION.")
    print("  It shows the OSL knee tracking a human-DERIVED reference trajectory.")
    print("  There is no pelvis, no ground contact and no body weight, so the torque")
    print("  shown is bench actuator torque and is NOT a human knee moment.")
    print("=" * 78)

    if mujoco is None or gsb is None:
        return fatal("mujoco is not importable in this interpreter"
                     + (f" ({_GSB_EXIT})" if _GSB_EXIT else "")
                     + ".\n       Use the venv that runs the quantitative experiment:\n"
                       "         .venv\\Scripts\\python.exe experiments\\view_bench_ab19.py\n"
                       "       (.venv-analysis has matplotlib but no mujoco -- that one is "
                       "only for --plot-from in\n       bench_track_ab19.py.)")

    # ---------------------------------------------- [2] model + gsb's preflight checks
    hr("[B] MODEL")
    info = gsb.load_and_verify()
    if gsb.FAILS:
        return fatal(f"{len(gsb.FAILS)} model precondition(s) failed -- the bench model "
                     f"is not what the experiment was validated against:\n    "
                     + "\n    ".join(gsb.FAILS))
    model = info["model"]
    dt = float(model.opt.timestep)
    if abs(dt - bt.DT_EXPECT) > 1e-12:
        return fatal(f"timestep is {dt} s, expected {bt.DT_EXPECT} s -- the quantitative "
                     f"run's step grid would not match")
    print(f"    {os.path.relpath(info['path'], ROOT)}  dt {dt * 1e3:.3f} ms   "
          f"knee forcerange {info['forcerange0'][0]} N.m (untouched)")

    # ------------------------------------------------------- [3] reference, same as bt
    if not os.path.isfile(args.csv):
        return fatal(f"reference CSV not found: {args.csv}")
    ref = bt.load_reference(args.csv)
    cr = info["ctrlrange"]
    bt.audit_reference(ref, math.degrees(cr[0]), math.degrees(cr[1]))
    res, n = bt.resample(ref, dt, 1, "linear" if args.linear else "cubic")

    # ------------------------------------------------------------ [4] prove it matches
    M = consistency_check(info, res, n, args.kp, args.kv)
    if M is None:
        return fatal("the live control path does NOT reproduce bench_track_ab19.py. "
                     "Refusing to open the viewer: a demonstration that shows different "
                     "physics from the CSV would be misleading.")

    # -------------------------------------------------------------- [5] camera + go
    cam = derive_camera(info, res, args)
    ankle_hold = init_state(info, res, args.kp, args.kv)
    print(f"\n    seeded at 0 % gait phase: knee "
          f"{math.degrees(res['ref_rad'][0]):.3f} deg, qvel 0, ankle held at "
          f"{math.degrees(ankle_hold):.3f} deg")
    if bt.FLAGS:
        print(f"\n    NOTE: {len(bt.FLAGS)} reference-quality flag(s) from the audit above "
              f"(all concern the\n    moment/power columns, which this visualisation does "
              f"not use -- it is angle-driven only).")

    run_viewer(info, res, n, args, ankle_hold, cam)

    # ------------------------------------------------- [6] the model is still authored
    fr_k = tuple(float(x) for x in model.actuator_forcerange[info["ka"]])
    cr_k = tuple(float(x) for x in model.actuator_ctrlrange[info["ka"]])
    hr("[F] POST-RUN ASSERTION")
    ok = (abs(fr_k[0] - info["forcerange0"][0][0]) < 1e-9
          and abs(fr_k[1] - info["forcerange0"][0][1]) < 1e-9
          and abs(cr_k[0] - cr[0]) < 1e-12 and abs(cr_k[1] - cr[1]) < 1e-12)
    print(f"    knee forcerange {fr_k} and ctrlrange {cr_k} unchanged: {ok}")
    print("    the XML on disk was never opened for writing.")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
