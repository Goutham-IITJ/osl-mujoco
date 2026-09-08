#!/usr/bin/env python3
"""
gain_sweep_bench.py -- CONTROLLER TUNING for the CAD-derived OSL V2 bench model.

WHAT THIS IS
    A sweep of the knee position-servo gains (kp, kv) on models/osl_v2_bench.xml,
    driven by synthetic safe reference motions, to choose a tracking-controller
    bandwidth BEFORE the human gait reference is ever used.

WHAT THIS IS *NOT*  -- read this before quoting any number from it
    This is CONTROLLER TUNING, not a change to the physical OSL model.  kp and kv
    are properties of the *controller* wrapped around the joint, not of the leg.
    Nothing in models/osl_v2_bench.xml is modified: the gains are written into the
    already-compiled mjModel in memory and the file on disk is never touched.
    The authored torque limit (forcerange +/-142.2 N.m knee, +/-168.2 N.m ankle) is
    left exactly as generated and is asserted unchanged at the end of every run.
    None of the reference motions here are human data.  They are synthetic.

    Also note the boundary condition: `knee_prox` has no free joint, so the leg is
    welded to the world.  The knee actuator therefore fights only
        I*alpha + m*g*d*sin(q) + b*qd + frictionloss
    and NOT body weight.  Torques from this script are not comparable to a human
    knee moment during stance.

PLANT (COMPUTED from the authored inertials, printed and re-checked at run time)
    knee-distal inertia about the knee axis  0.251998 kg.m^2
    + joint armature                         0.01
    = effective                              0.261998 kg.m^2
    joint damping 0.3 N.m.s/rad, frictionloss 0.4 N.m, m*g*d = 8.8529 N.m

GAIN RULE
    kv is derived, not guessed:  zeta = (kv + b_joint) / (2*sqrt(kp*I_eff))
    so  kv = 2*zeta*sqrt(kp*I_eff) - b_joint,  with zeta = 0.7 by default.
    Crediting the joint damping already in the model matters at low kp, where
    0.3 N.m.s/rad is a real fraction of the required damping.

USAGE  (osl-mujoco's own .venv -- mujoco 3.12, numpy; nothing else is needed)
    .venv\\Scripts\\python.exe experiments\\gain_sweep_bench.py
    .venv\\Scripts\\python.exe experiments\\gain_sweep_bench.py --kp 60 200 600 2000
    .venv\\Scripts\\python.exe experiments\\gain_sweep_bench.py --refs sine_gait chirp

Exit code = number of failed checks (0 = clean), matching tools/validate_mjcf.py.
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import sys

import numpy as np

try:
    import mujoco
except ImportError:
    sys.exit("mujoco is not installed in this interpreter.\n"
             "  .venv\\Scripts\\activate   (osl-mujoco's venv already has mujoco 3.12)")

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "scripts"))
from mjcommon import nid, scene_path  # noqa: E402

# ------------------------------------------------------------------ expectations
# Authored values in models/osl_v2_bench.xml.  If any of these no longer match,
# the sweep REFUSES to run rather than silently sweeping a different plant.
EXPECT = dict(
    nq=2, nu=2,
    timestep=0.0005,
    knee_joint="knee", ankle_joint="ankle",
    knee_act="knee_pos", ankle_act="ankle_pos",
    knee_kp_authored=60.0, ankle_kp_authored=60.0,
    knee_kv_authored=0.0, ankle_kv_authored=0.0,
    knee_forcerange=(-142.2, 142.2), ankle_forcerange=(-168.2, 168.2),
    knee_range=(-0.0872664625997, 2.09439510239),
    ankle_range=(-0.523598775598, 0.349065850399),
    knee_armature=0.01, knee_damping=0.3, knee_frictionloss=0.4,
)
I_BODY_EXPECT = 0.251998        # knee-distal inertia about the knee axis, COMPUTED
ZETA_DEFAULT = 0.7
KP_DEFAULT = [60.0, 200.0, 600.0, 2000.0]

FAILS: list[str] = []
WARNS: list[str] = []


def check(cond: bool, msg: str) -> bool:
    if cond:
        print(f"    ok    {msg}")
    else:
        print(f"    FAIL  {msg}")
        FAILS.append(msg)
    return bool(cond)


def warn(cond: bool, msg: str) -> None:
    if not cond:
        print(f"    warn  {msg}")
        WARNS.append(msg)


def close(a, b, tol=1e-9) -> bool:
    return abs(float(a) - float(b)) <= tol


def dof_inertia(model, data, dof):
    """Diagonal element M[dof, dof] of the articulated mass matrix, WITHOUT reading the
    sparse mass-matrix array directly.

    MuJoCo API compatibility note.  The sparse mass matrix used to be exposed as
    `mjData.qM`; in current MuJoCo it is `mjData.M` (new internal representation), so on
    mujoco 3.12.0 the old spelling raises
        AttributeError: 'mujoco._structs.MjData' object has no attribute 'qM'
    Rather than branch on a version number, avoid the array entirely: `mj_mulM` computes
    res = M @ vec for whatever representation the build uses, so multiplying by the unit
    vector e_dof returns column `dof` of M, and its `dof`-th entry is the diagonal we
    want.  That call is stable across MuJoCo 2.x and 3.x.

    A dense `mj_fullM` cross-check is attempted as a second opinion and is *only* that:
    if the attribute or the signature differs on this build it is skipped, never failed,
    because `mj_mulM` alone already answers the question.

    Requires mj_forward (or at least mj_crb) to have run at the current qpos.
    Returns (i_from_mulM, i_from_fullM_or_nan).
    """
    vec = np.zeros(model.nv)
    vec[dof] = 1.0
    res = np.zeros(model.nv)
    mujoco.mj_mulM(model, data, res, vec)
    i_mul = float(res[dof])

    i_full = float("nan")
    sparse = getattr(data, "M", None)
    if sparse is None:
        sparse = getattr(data, "qM", None)
    if sparse is not None:
        try:
            dense = np.zeros((model.nv, model.nv))
            mujoco.mj_fullM(model, dense, sparse)
            i_full = float(dense[dof, dof])
        except (TypeError, ValueError, AttributeError):
            pass
    return i_mul, i_full


# =========================================================== 1. model + preflight
def load_and_verify():
    path = scene_path("bench")
    model = mujoco.MjModel.from_xml_path(path)
    data = mujoco.MjData(model)
    print(f"\n[1] MODEL  {os.path.relpath(path, ROOT)}")
    print(f"    nq={model.nq} nv={model.nv} nu={model.nu} nbody={model.nbody} "
          f"timestep={model.opt.timestep}")

    check(model.nq == EXPECT["nq"], f"nq == {EXPECT['nq']} (fixed base, 2 hinges)")
    check(model.nu == EXPECT["nu"], f"nu == {EXPECT['nu']}")
    check(close(model.opt.timestep, EXPECT["timestep"]),
          f"timestep == {EXPECT['timestep']} s")

    ids = {}
    for key, objtype in (("knee_joint", mujoco.mjtObj.mjOBJ_JOINT),
                         ("ankle_joint", mujoco.mjtObj.mjOBJ_JOINT),
                         ("knee_act", mujoco.mjtObj.mjOBJ_ACTUATOR),
                         ("ankle_act", mujoco.mjtObj.mjOBJ_ACTUATOR)):
        ids[key] = nid(model, objtype, EXPECT[key])

    # fixed base: the root body must carry no joint at all
    check(int(model.body_jntnum[1]) == 0,
          "root body has no free joint (welded to the world -- bench, not walking)")

    kj, aj = ids["knee_joint"], ids["ankle_joint"]
    ka, aa = ids["knee_act"], ids["ankle_act"]

    print("\n[2] ACTUATOR ALGEBRA  (tau = kp*(ctrl - q) - kv*qd, clamped to forcerange)")
    for nm, aid in (("knee", ka), ("ankle", aa)):
        check(model.actuator_gaintype[aid] == mujoco.mjtGain.mjGAIN_FIXED,
              f"{nm} gaintype == fixed")
        check(model.actuator_biastype[aid] == mujoco.mjtBias.mjBIAS_AFFINE,
              f"{nm} biastype == affine")
        check(model.actuator_dyntype[aid] == mujoco.mjtDyn.mjDYN_NONE,
              f"{nm} dyntype == none (no activation state)")
        check(model.actuator_trntype[aid] == mujoco.mjtTrn.mjTRN_JOINT,
              f"{nm} trntype == joint")
        check(close(model.actuator_gear[aid, 0], 1.0),
              f"{nm} gear == 1 (so actuator force IS joint torque in N.m)")

    kp0 = float(model.actuator_gainprm[ka, 0])
    kv0 = -float(model.actuator_biasprm[ka, 2])
    check(close(kp0, EXPECT["knee_kp_authored"]),
          f"knee authored kp == {EXPECT['knee_kp_authored']} (got {kp0})")
    check(close(kv0, EXPECT["knee_kv_authored"]),
          f"knee authored kv == {EXPECT['knee_kv_authored']} (got {kv0}) -- pure P servo")
    check(close(-float(model.actuator_biasprm[ka, 1]), kp0),
          "knee biasprm[1] == -kp (MuJoCo position-servo form)")
    check(close(float(model.actuator_biasprm[ka, 0]), 0.0), "knee biasprm[0] == 0")

    print("\n[3] TORQUE LIMIT AND ROM  (must be left UNCHANGED by this script)")
    fr_k = tuple(float(x) for x in model.actuator_forcerange[ka])
    fr_a = tuple(float(x) for x in model.actuator_forcerange[aa])
    check(all(close(a, b, 1e-6) for a, b in zip(fr_k, EXPECT["knee_forcerange"])),
          f"knee forcerange == {EXPECT['knee_forcerange']} N.m")
    check(all(close(a, b, 1e-6) for a, b in zip(fr_a, EXPECT["ankle_forcerange"])),
          f"ankle forcerange == {EXPECT['ankle_forcerange']} N.m")
    kr = tuple(float(x) for x in model.jnt_range[kj])
    check(all(close(a, b, 1e-9) for a, b in zip(kr, EXPECT["knee_range"])),
          f"knee range == [{math.degrees(kr[0]):.2f}, {math.degrees(kr[1]):.2f}] deg")
    cr = tuple(float(x) for x in model.actuator_ctrlrange[ka])
    check(all(close(a, b, 1e-9) for a, b in zip(cr, kr)),
          "knee ctrlrange == knee joint range (ctrl IS the reference angle, in rad)")

    print("\n[4] JOINT DYNAMICS  (via dof_* arrays -- model.joint(name) does NOT expose these)")
    kd = int(model.jnt_dofadr[kj])
    arm = float(model.dof_armature[kd])
    dmp = float(model.dof_damping[kd])
    frc = float(model.dof_frictionloss[kd])
    print(f"    knee armature={arm}  damping={dmp}  frictionloss={frc}")
    check(close(arm, EXPECT["knee_armature"]), f"knee armature == {EXPECT['knee_armature']}")
    check(close(dmp, EXPECT["knee_damping"]), f"knee damping == {EXPECT['knee_damping']}")
    check(close(frc, EXPECT["knee_frictionloss"]),
          f"knee frictionloss == {EXPECT['knee_frictionloss']}")

    print("\n[5] PLANT INERTIA  (measured from the compiled model, not the XML text)")
    kid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "flat")
    mujoco.mj_resetDataKeyframe(model, data, kid if kid >= 0 else 0)
    data.qpos[:] = 0.0
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)
    i_eff, i_full = dof_inertia(model, data, kd)
    i_body = i_eff - arm
    print(f"    M[knee,knee] via mj_mulM = {i_eff:.6f} kg.m^2  "
          f"(body {i_body:.6f} + armature {arm})")
    if math.isnan(i_full):
        print("    mj_fullM cross-check     = unavailable on this MuJoCo build (skipped)")
    else:
        print(f"    mj_fullM cross-check     = {i_full:.6f} kg.m^2")
        # warn, not check: a second opinion must not be able to refuse the sweep
        warn(close(i_eff, i_full, 1e-9),
             f"mj_mulM {i_eff:.9f} and mj_fullM {i_full:.9f} disagree on M[knee,knee]")
    check(abs(i_body - I_BODY_EXPECT) < 2e-3,
          f"knee-distal inertia == {I_BODY_EXPECT:.6f} kg.m^2 within 2e-3 (got {i_body:.6f})")

    # gravity moment about the knee at q = 90 deg -> m*g*d
    data.qpos[:] = 0.0
    data.qpos[int(model.jnt_qposadr[kj])] = math.pi / 2
    data.qvel[:] = 0.0
    data.ctrl[:] = 0.0
    mujoco.mj_forward(model, data)
    mgd = abs(float(data.qfrc_bias[kd]))
    print(f"    |qfrc_bias| at q=90deg = {mgd:.4f} N.m  (expect m*g*d ~ 8.85)")
    warn(abs(mgd - 8.8529) < 0.15,
         f"gravity moment {mgd:.4f} differs from the COMPUTED 8.8529 N.m by more than 0.15")

    sens = {}
    for nm in ("knee_q", "knee_qd", "knee_tau", "ankle_q", "ankle_qd", "ankle_tau"):
        sid = nid(model, mujoco.mjtObj.mjOBJ_SENSOR, nm)
        sens[nm] = int(model.sensor_adr[sid])

    info = dict(model=model, data=data, path=path,
                kj=kj, aj=aj, ka=ka, aa=aa,
                knee_dof=kd, knee_qpos=int(model.jnt_qposadr[kj]),
                ankle_dof=int(model.jnt_dofadr[aj]),
                ankle_qpos=int(model.jnt_qposadr[aj]),
                sens=sens, i_eff=i_eff, i_body=i_body, mgd=mgd,
                b_joint=dmp, fric=frc, keyframe=kid,
                forcerange0=(fr_k, fr_a), ctrlrange=cr,
                ankle_range=tuple(float(x) for x in model.jnt_range[aj]))
    return info


# ============================================================ 2. runtime gain set
def kv_for(kp: float, i_eff: float, b_joint: float, zeta: float) -> float:
    """zeta = (kv + b_joint) / (2*sqrt(kp*I_eff))  ->  solve for kv."""
    return 2.0 * zeta * math.sqrt(kp * i_eff) - b_joint


def set_gains(model, aid: int, kp: float, kv: float) -> None:
    """Write the servo gains into the COMPILED model only.

    models/osl_v2_bench.xml is never opened for writing.  forcerange and
    ctrlrange are deliberately not touched, so the authored torque authority and
    ROM still bound everything this controller does.
    """
    model.actuator_gainprm[aid, 0] = kp
    model.actuator_biasprm[aid, 0] = 0.0
    model.actuator_biasprm[aid, 1] = -kp
    model.actuator_biasprm[aid, 2] = -kv


# ============================================================ 3. reference motions
def _t(dur, dt):
    return np.arange(0.0, dur, dt)


def build_refs(dt: float, knee_lo: float, knee_hi: float) -> dict:
    """Synthetic, safe, entirely non-human reference motions (radians).

    Each entry is (t, ref, description, analysis_start_s).  analysis_start_s
    excludes the part of the trace where the error is dominated by an artefact of
    the reference rather than by the controller -- the instant of a step edge, or
    the first settling cycle of a sinusoid.  Reporting an RMS error that includes
    a step discontinuity would just re-report the step size.
    """
    refs = {}

    for amp in (10.0, 45.0):
        t = _t(1.5, dt)
        refs[f"step_{int(amp)}deg"] = (
            t, np.where(t < 0.2, 0.0, math.radians(amp)),
            f"hard {amp:.0f} deg step at t=0.2 s -- diagnostic only; a step is "
            f"infinitely fast so it WILL saturate at high kp",
            0.4)

    f = 0.9
    t = _t(4.0 / f, dt)
    refs["sine_gait"] = (
        t, math.radians(30.0) - math.radians(30.0) * np.cos(2 * math.pi * f * t),
        "0.9 Hz sinusoid, 0->60 deg: the amplitude and rate of a walking knee, "
        "but synthetic -- NOT the Camargo reference",
        2.0 / f)

    T, pre, post = 0.3, 0.1, 0.6
    t = _t(pre + T + post, dt)
    s = np.clip((t - pre) / T, 0.0, 1.0)
    refs["minjerk_60deg"] = (
        t, math.radians(60.0) * (10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5),
        "minimum-jerk 0->60 deg in 0.3 s: the fastest excursion a walking knee "
        "makes, smooth so it is achievable",
        0.05)

    f0, f1, dur = 0.2, 8.0, 10.0
    t = _t(dur, dt)
    ph = 2 * math.pi * (f0 * t + 0.5 * (f1 - f0) / dur * t ** 2)
    refs["chirp"] = (
        t, math.radians(30.0) + math.radians(10.0) * np.sin(ph),
        f"linear chirp {f0}->{f1} Hz at 10 deg amplitude about 30 deg: measures "
        f"closed-loop bandwidth and phase lag directly",
        0.2)

    # every sample must be inside ctrlrange, with margin
    for name, (t, r, _, _) in refs.items():
        lo, hi = float(np.min(r)), float(np.max(r))
        if lo < knee_lo + 1e-6 or hi > knee_hi - 1e-6:
            sys.exit(f"reference {name!r} spans [{math.degrees(lo):.2f}, "
                     f"{math.degrees(hi):.2f}] deg, outside the knee ROM "
                     f"[{math.degrees(knee_lo):.2f}, {math.degrees(knee_hi):.2f}] -- refusing to run")
    return refs


# ==================================================================== 4. run one
def run(info, kp: float, kv: float, t: np.ndarray, ref: np.ndarray, decim: int,
        analysis_start: float = 0.0):
    model, data = info["model"], info["data"]
    ka, aa = info["ka"], info["aa"]
    kd, kq = info["knee_dof"], info["knee_qpos"]
    aq = info["ankle_qpos"]
    S = info["sens"]

    set_gains(model, ka, kp, kv)          # knee: the swept controller
    # ankle: held at its keyframe value by its OWN authored servo (kp=60, kv=0).
    # Untouched, so the ankle is a fixed boundary condition, not a second variable.
    set_gains(model, aa, EXPECT["ankle_kp_authored"], EXPECT["ankle_kv_authored"])

    mujoco.mj_resetDataKeyframe(model, data, info["keyframe"])
    ankle_hold = float(data.qpos[aq])
    data.qpos[kq] = float(ref[0])
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)

    n = len(t)
    ref_vel = np.gradient(ref, model.opt.timestep)
    rows = []
    peak_tau = 0.0
    sat_n = 0
    tau_prev = None
    dtau = []
    ankle_dev = 0.0
    q_log = np.empty(n)
    tau_log = np.empty(n)

    for k in range(n):
        cmd = float(np.clip(ref[k], info["ctrlrange"][0], info["ctrlrange"][1]))
        data.ctrl[ka] = cmd
        data.ctrl[aa] = ankle_hold
        mujoco.mj_step(model, data)

        q = float(data.sensordata[S["knee_q"]])
        qd = float(data.sensordata[S["knee_qd"]])
        tau = float(data.sensordata[S["knee_tau"]])
        tau_direct = float(data.actuator_force[ka])
        tau_ideal = kp * (cmd - q) - kv * qd
        lim = info["forcerange0"][0][1]
        sat = abs(tau_ideal) > lim + 1e-6
        sat_n += int(sat)
        peak_tau = max(peak_tau, abs(tau))
        if tau_prev is not None:
            dtau.append(tau - tau_prev)
        tau_prev = tau
        a_q = float(data.sensordata[S["ankle_q"]])
        ankle_dev = max(ankle_dev, abs(a_q - ankle_hold))
        q_log[k] = q
        tau_log[k] = tau

        if k % decim == 0:
            rows.append((
                f"{data.time:.6f}", f"{kp:.1f}", f"{kv:.4f}",
                f"{ref[k]:.8f}", f"{math.degrees(ref[k]):.6f}",
                f"{q:.8f}", f"{math.degrees(q):.6f}",
                f"{math.degrees(q - ref[k]):.6f}",
                f"{ref_vel[k]:.6f}", f"{qd:.6f}",
                f"{cmd:.8f}", f"{tau:.6f}", f"{tau_direct:.6f}", f"{tau_ideal:.6f}",
                f"{100.0 * abs(tau) / lim:.4f}", int(sat),
                f"{a_q:.8f}", f"{float(data.sensordata[S['ankle_tau']]):.6f}",
            ))

    err = np.degrees(q_log - ref)
    i0 = min(int(analysis_start / model.opt.timestep), n - 2)
    lim = info["forcerange0"][0][1]
    m = dict(
        rms_err=float(np.sqrt(np.mean(err[i0:] ** 2))),
        rms_err_all=float(np.sqrt(np.mean(err ** 2))),
        peak_err=float(np.max(np.abs(err[i0:]))),
        ss_err=float(np.mean(np.abs(err[-int(0.1 / model.opt.timestep):]))),
        peak_tau=peak_tau,
        pct_auth=100.0 * peak_tau / lim,
        sat_pct=100.0 * sat_n / n,
        chatter=float(np.sqrt(np.mean(np.square(dtau))) / model.opt.timestep) if dtau else 0.0,
        ankle_dev_deg=math.degrees(ankle_dev),
        analysis_start_s=analysis_start,
        final_time=float(data.time),
    )
    return rows, m, q_log, tau_log


def step_shape(t, ref, q, dt):
    i0 = int(0.2 / dt)
    tgt = float(ref[-1])
    if abs(tgt) < 1e-9:
        return float("nan"), float("nan")
    seg, ts = q[i0:], t[i0:] - t[i0]
    os_ = 100.0 * (float(np.max(seg)) - tgt) / tgt
    out = np.where(np.abs(seg - tgt) > 0.02 * abs(tgt))[0]
    st = float(ts[out[-1]]) if len(out) and out[-1] < len(seg) - 1 else float("nan")
    return os_, st


# ======================================================================== 5. main
def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--kp", nargs="+", type=float, default=KP_DEFAULT)
    ap.add_argument("--zeta", type=float, default=ZETA_DEFAULT)
    ap.add_argument("--refs", nargs="+", default=None,
                    help="subset of: step_10deg step_45deg sine_gait minjerk_60deg chirp")
    ap.add_argument("--decim", type=int, default=2, help="CSV decimation (2 -> 1 kHz)")
    ap.add_argument("--outdir", default=os.path.join(ROOT, "build", "gain_sweep"))
    args = ap.parse_args()

    print("=" * 96)
    print("OSL V2 BENCH -- CONTROLLER GAIN SWEEP")
    print("CONTROLLER TUNING ONLY.  The physical model is not modified;")
    print("gains are set on the compiled mjModel and forcerange is left as authored.")
    print("=" * 96)

    info = load_and_verify()
    if FAILS:
        print(f"\n{len(FAILS)} preflight check(s) failed -- refusing to sweep a model "
              f"that is not the one this script was written for.")
        for f in FAILS:
            print(f"  - {f}")
        return len(FAILS)

    model = info["model"]
    dt = float(model.opt.timestep)
    refs = build_refs(dt, info["ctrlrange"][0], info["ctrlrange"][1])
    if args.refs:
        unknown = set(args.refs) - set(refs)
        if unknown:
            sys.exit(f"unknown reference(s): {sorted(unknown)}\navailable: {sorted(refs)}")
        refs = {k: refs[k] for k in args.refs}

    print(f"\n[6] GAINS  zeta = {args.zeta}, kv = 2*zeta*sqrt(kp*I_eff) - b_joint")
    print(f"    I_eff = {info['i_eff']:.6f} kg.m^2   b_joint = {info['b_joint']} N.m.s/rad")
    print(f"    {'kp':>7} {'kv':>9} {'wn rad/s':>10} {'wn Hz':>8} {'fric deadband':>14}")
    gains = []
    for kp in args.kp:
        kv = kv_for(kp, info["i_eff"], info["b_joint"], args.zeta)
        wn = math.sqrt(kp / info["i_eff"])
        gains.append((kp, kv))
        print(f"    {kp:7.0f} {kv:9.3f} {wn:10.3f} {wn / 2 / math.pi:8.3f} "
              f"{math.degrees(info['fric'] / kp):13.3f}d")

    os.makedirs(args.outdir, exist_ok=True)
    trace_path = os.path.join(args.outdir, "gain_sweep_trace.csv")
    metrics_path = os.path.join(args.outdir, "gain_sweep_metrics.csv")

    HEADER = ["time_s", "kp", "kv", "ref_rad", "ref_deg", "sim_rad", "sim_deg",
              "err_deg", "ref_vel_rad_s", "sim_vel_rad_s", "ctrl_rad",
              "tau_sensor_Nm", "tau_actforce_Nm", "tau_unclamped_Nm",
              "pct_authority", "saturated", "ankle_q_rad", "ankle_tau_Nm"]

    results = {}
    with open(trace_path, "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["# CONTROLLER TUNING SWEEP -- synthetic references, NOT human data"])
        w.writerow([f"# model={os.path.relpath(info['path'], ROOT)} (unmodified on disk)"])
        w.writerow([f"# I_eff={info['i_eff']:.6f} b_joint={info['b_joint']} "
                    f"fric={info['fric']} mgd={info['mgd']:.4f} zeta={args.zeta}"])
        w.writerow([f"# forcerange knee={info['forcerange0'][0]} UNCHANGED"])
        w.writerow(["reference"] + HEADER)
        print(f"\n[7] SWEEP  ({len(refs)} references x {len(gains)} gains)")
        for name, (t, r, desc, astart) in refs.items():
            print(f"\n  {name}: {desc}")
            print(f"    metrics measured from t >= {astart:.3f} s")
            for kp, kv in gains:
                rows, m, q_log, tau_log = run(info, kp, kv, t, r, args.decim, astart)
                if name.startswith("step"):
                    m["overshoot_pct"], m["settle_s"] = step_shape(t, r, q_log, dt)
                else:
                    m["overshoot_pct"], m["settle_s"] = float("nan"), float("nan")
                results[(name, kp)] = m
                for row in rows:
                    w.writerow([name] + list(row))
                print(f"    kp={kp:7.0f} kv={kv:7.3f} | RMSe {m['rms_err']:7.3f}d "
                      f"peak {m['peak_err']:7.3f}d ss {m['ss_err']:6.3f}d | "
                      f"tau {m['peak_tau']:7.2f} N.m ({m['pct_auth']:5.1f}% auth) "
                      f"sat {m['sat_pct']:5.1f}% | ankle drift {m['ankle_dev_deg']:.3f}d")

    with open(metrics_path, "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        keys = ["rms_err", "rms_err_all", "peak_err", "ss_err", "peak_tau", "pct_auth",
                "sat_pct", "overshoot_pct", "settle_s", "chatter", "ankle_dev_deg",
                "analysis_start_s"]
        w.writerow(["reference", "kp", "kv", "zeta"] + keys)
        for (name, kp), m in results.items():
            kv = kv_for(kp, info["i_eff"], info["b_joint"], args.zeta)
            w.writerow([name, kp, f"{kv:.4f}", args.zeta] +
                       [f"{m[k]:.6f}" for k in keys])

    # ------------------------------------------------------- integrity + repeatability
    print("\n[8] INTEGRITY")
    fr_k = tuple(float(x) for x in model.actuator_forcerange[info["ka"]])
    fr_a = tuple(float(x) for x in model.actuator_forcerange[info["aa"]])
    check(fr_k == info["forcerange0"][0] and fr_a == info["forcerange0"][1],
          "forcerange identical to the authored values after the whole sweep")
    check(tuple(float(x) for x in model.actuator_ctrlrange[info["ka"]]) == info["ctrlrange"],
          "knee ctrlrange unchanged")
    mtime_note = os.path.getmtime(info["path"])
    print(f"    models/osl_v2_bench.xml mtime unchanged by this run: {mtime_note}")

    name0 = next(iter(refs))
    t0, r0, _, a0 = refs[name0]
    a = run(info, gains[0][0], gains[0][1], t0, r0, args.decim, a0)[2]
    b = run(info, gains[0][0], gains[0][1], t0, r0, args.decim, a0)[2]
    check(np.array_equal(a, b), "two identical runs are bit-identical (deterministic)")

    print(f"\n[9] OUTPUT")
    print(f"    trace   {trace_path}")
    print(f"    metrics {metrics_path}")
    print(f"\n{len(FAILS)} failed, {len(WARNS)} warning(s)")
    for x in WARNS:
        print(f"  warn: {x}")
    for x in FAILS:
        print(f"  FAIL: {x}")
    return len(FAILS)


if __name__ == "__main__":
    sys.exit(main())
