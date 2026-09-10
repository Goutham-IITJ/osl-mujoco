"""
oslbench.model -- load models/osl_v2_bench.xml, resolve the joint/actuator/sensor IDs,
verify the compiled model is the one the experiment was validated against, and measure
the plant.

WHAT THE BENCH MODEL IS
    A FIXED-BASE test bench.  The root body `knee_prox` carries no joint, so the
    device is bolted to the world: no pelvis, no torso, no ground contact, no body
    weight.  nq = nv = nu = 2 (knee hinge + ankle hinge).  The knee actuator fights
        I*alpha + m*g*d*sin(q) + b*qdot + frictionloss
    and nothing else, which is why bench actuator torque is NOT a human knee moment.

WHAT THIS FILE DOES NOT DO
    No control law (see controller.py), no stepping (see simulation.py), no metrics.
    It never opens the XML for writing.  `load_bench_model` prints its own check
    results and nothing else; every value it prints is read back out of the COMPILED
    model, not out of the XML text.

THE NINE PRECONDITIONS
    If any authored value has drifted, `BenchModel.failures` is non-empty and the
    entry points refuse to run rather than silently measuring a different plant.
"""

from __future__ import annotations

import math
import os
import sys

import numpy as np

try:
    import mujoco
except ImportError:                                          # pragma: no cover
    mujoco = None

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MODEL_DIR = os.path.join(ROOT, "models")
BENCH_XML = os.path.join(MODEL_DIR, "osl_v2_bench.xml")

# ------------------------------------------------------------------ expectations
# Authored values in models/osl_v2_bench.xml.  If any of these no longer match, the
# experiment REFUSES to run rather than silently simulating a different plant.
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
MGD_EXPECT = 8.8529             # N.m, gravity moment about the knee at q = 90 deg
SENSOR_NAMES = ("knee_q", "knee_qd", "knee_tau", "ankle_q", "ankle_qd", "ankle_tau")


def name_id(model, objtype, name: str) -> int:
    """mj_name2id with a fatal error instead of a silent -1.

    A silent -1 is dangerous here: with exactly two actuators, a mistyped "knee_pos"
    would command the ankle and every number printed afterwards would still look
    plausible.
    """
    i = mujoco.mj_name2id(model, objtype, name)
    if i < 0:
        sys.exit(f"FATAL: {name!r} not found in the model (type {objtype})")
    return int(i)


def close(a, b, tol: float = 1e-9) -> bool:
    return abs(float(a) - float(b)) <= tol


def dof_inertia(model, data, dof: int):
    """Diagonal element M[dof, dof] of the articulated mass matrix, WITHOUT reading the
    sparse mass-matrix array directly.

    MuJoCo API compatibility note.  The sparse mass matrix used to be exposed as
    `mjData.qM`; in current MuJoCo it is `mjData.M`, so on mujoco 3.12.0 the old
    spelling raises AttributeError.  Rather than branch on a version number, avoid the
    array entirely: `mj_mulM` computes res = M @ vec for whatever representation the
    build uses, so multiplying by the unit vector e_dof returns column `dof` of M and
    its `dof`-th entry is the diagonal we want.  That call is stable across 2.x/3.x.

    A dense `mj_fullM` cross-check is attempted as a second opinion and is only that.
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


class BenchModel:
    """The compiled bench model plus everything downstream needs to address it.

    Attributes (all read back out of the compiled model):
        model, data, path              mjModel, mjData, absolute path to the XML
        timestep                       s, authored 0.0005
        knee_joint / ankle_joint       joint ids
        knee_act / ankle_act           actuator ids
        knee_dof / knee_qpos           address of the knee in qvel / qpos
        ankle_dof / ankle_qpos         address of the ankle in qvel / qpos
        sensors                        {name: sensordata index}
        keyframe                       id of the "flat" keyframe (-1 if absent)
        knee_range / knee_ctrlrange    rad
        knee_forcerange                (lo, hi) N.m, authored +/-142.2
        ankle_range / ankle_ctrlrange  rad
        ankle_forcerange               (lo, hi) N.m, authored +/-168.2
        i_body / i_eff                 kg.m^2, knee-distal inertia and + armature
        armature / b_joint / frictionloss   knee dof properties
        mgd                            N.m, |gravity moment| about the knee at 90 deg
        failures / warnings            list[str], empty on a healthy model
    """

    def __init__(self, model, data, path):
        self.model, self.data, self.path = model, data, path
        self.failures: list[str] = []
        self.warnings: list[str] = []

    # -------------------------------------------------------------- check helpers
    def _check(self, cond: bool, msg: str, verbose: bool) -> bool:
        if cond:
            if verbose:
                print(f"    ok    {msg}")
        else:
            if verbose:
                print(f"    FAIL  {msg}")
            self.failures.append(msg)
        return bool(cond)

    def _warn(self, cond: bool, msg: str, verbose: bool) -> None:
        if not cond:
            if verbose:
                print(f"    warn  {msg}")
            self.warnings.append(msg)

    # -------------------------------------------------------------- introspection
    @property
    def force_limit(self) -> float:
        """The knee's symmetric torque authority in N.m (142.2)."""
        return float(self.knee_forcerange[1])

    def limits_unchanged(self) -> bool:
        """Re-read forcerange and ctrlrange out of the compiled model and compare them
        with the values captured at load time.  Called after every run: if a controller
        had quietly widened the torque authority, every torque number would be void."""
        m = self.model
        fr_k = tuple(float(x) for x in m.actuator_forcerange[self.knee_act])
        fr_a = tuple(float(x) for x in m.actuator_forcerange[self.ankle_act])
        cr_k = tuple(float(x) for x in m.actuator_ctrlrange[self.knee_act])
        return (fr_k == self.knee_forcerange and fr_a == self.ankle_forcerange
                and cr_k == self.knee_ctrlrange)

    def keyframe_id(self) -> int:
        """The "flat" keyframe, or 0 if it was renamed (mj_resetDataKeyframe(-1) raises)."""
        return self.keyframe if self.keyframe >= 0 else 0

    def relpath(self) -> str:
        return os.path.relpath(self.path, ROOT)


def load_bench_model(path: str | None = None, verbose: bool = True) -> BenchModel:
    """Compile models/osl_v2_bench.xml and verify it is the validated bench.

    Returns a BenchModel whose `.failures` list is empty on a healthy model.  The
    caller decides what to do about failures; this function never raises for a
    precondition, so the full report is always printed.
    """
    if mujoco is None:
        sys.exit("FATAL: mujoco is not importable in this interpreter.\n"
                 "  Use osl-mujoco's own venv:  .venv\\Scripts\\python.exe ...")
    path = os.path.abspath(path or BENCH_XML)
    if not os.path.isfile(path):
        sys.exit(f"FATAL: model not found: {path}")

    model = mujoco.MjModel.from_xml_path(path)
    data = mujoco.MjData(model)
    bm = BenchModel(model, data, path)
    chk = lambda c, m: bm._check(c, m, verbose)          # noqa: E731  local shorthand
    wrn = lambda c, m: bm._warn(c, m, verbose)           # noqa: E731

    if verbose:
        print(f"\n[1] MODEL  {os.path.relpath(path, ROOT)}")
        print(f"    nq={model.nq} nv={model.nv} nu={model.nu} nbody={model.nbody} "
              f"timestep={model.opt.timestep}")

    chk(model.nq == EXPECT["nq"], f"nq == {EXPECT['nq']} (fixed base, 2 hinges)")
    chk(model.nu == EXPECT["nu"], f"nu == {EXPECT['nu']}")
    chk(close(model.opt.timestep, EXPECT["timestep"]),
        f"timestep == {EXPECT['timestep']} s")

    ids = {}
    for key, objtype in (("knee_joint", mujoco.mjtObj.mjOBJ_JOINT),
                         ("ankle_joint", mujoco.mjtObj.mjOBJ_JOINT),
                         ("knee_act", mujoco.mjtObj.mjOBJ_ACTUATOR),
                         ("ankle_act", mujoco.mjtObj.mjOBJ_ACTUATOR)):
        ids[key] = name_id(model, objtype, EXPECT[key])

    # fixed base: the root body must carry no joint at all
    chk(int(model.body_jntnum[1]) == 0,
        "root body has no free joint (welded to the world -- bench, not walking)")

    kj, aj = ids["knee_joint"], ids["ankle_joint"]
    ka, aa = ids["knee_act"], ids["ankle_act"]
    bm.knee_joint, bm.ankle_joint, bm.knee_act, bm.ankle_act = kj, aj, ka, aa
    bm.timestep = float(model.opt.timestep)

    if verbose:
        print("\n[2] ACTUATOR ALGEBRA  (tau = kp*(ctrl - q) - kv*qd, clamped to forcerange)")
    for nm, aid in (("knee", ka), ("ankle", aa)):
        chk(model.actuator_gaintype[aid] == mujoco.mjtGain.mjGAIN_FIXED,
            f"{nm} gaintype == fixed")
        chk(model.actuator_biastype[aid] == mujoco.mjtBias.mjBIAS_AFFINE,
            f"{nm} biastype == affine")
        chk(model.actuator_dyntype[aid] == mujoco.mjtDyn.mjDYN_NONE,
            f"{nm} dyntype == none (no activation state)")
        chk(model.actuator_trntype[aid] == mujoco.mjtTrn.mjTRN_JOINT,
            f"{nm} trntype == joint")
        chk(close(model.actuator_gear[aid, 0], 1.0),
            f"{nm} gear == 1 (so actuator force IS joint torque in N.m)")

    kp0 = float(model.actuator_gainprm[ka, 0])
    kv0 = -float(model.actuator_biasprm[ka, 2])
    chk(close(kp0, EXPECT["knee_kp_authored"]),
        f"knee authored kp == {EXPECT['knee_kp_authored']} (got {kp0})")
    chk(close(kv0, EXPECT["knee_kv_authored"]),
        f"knee authored kv == {EXPECT['knee_kv_authored']} (got {kv0}) -- pure P servo")
    chk(close(-float(model.actuator_biasprm[ka, 1]), kp0),
        "knee biasprm[1] == -kp (MuJoCo position-servo form)")
    chk(close(float(model.actuator_biasprm[ka, 0]), 0.0), "knee biasprm[0] == 0")

    if verbose:
        print("\n[3] TORQUE LIMIT AND ROM  (must be left UNCHANGED by the controller)")
    fr_k = tuple(float(x) for x in model.actuator_forcerange[ka])
    fr_a = tuple(float(x) for x in model.actuator_forcerange[aa])
    chk(all(close(a, b, 1e-6) for a, b in zip(fr_k, EXPECT["knee_forcerange"])),
        f"knee forcerange == {EXPECT['knee_forcerange']} N.m")
    chk(all(close(a, b, 1e-6) for a, b in zip(fr_a, EXPECT["ankle_forcerange"])),
        f"ankle forcerange == {EXPECT['ankle_forcerange']} N.m")
    kr = tuple(float(x) for x in model.jnt_range[kj])
    chk(all(close(a, b, 1e-9) for a, b in zip(kr, EXPECT["knee_range"])),
        f"knee range == [{math.degrees(kr[0]):.2f}, {math.degrees(kr[1]):.2f}] deg")
    cr = tuple(float(x) for x in model.actuator_ctrlrange[ka])
    chk(all(close(a, b, 1e-9) for a, b in zip(cr, kr)),
        "knee ctrlrange == knee joint range (ctrl IS the reference angle, in rad)")

    bm.knee_range, bm.knee_ctrlrange = kr, cr
    bm.knee_forcerange, bm.ankle_forcerange = fr_k, fr_a
    bm.ankle_range = tuple(float(x) for x in model.jnt_range[aj])
    bm.ankle_ctrlrange = tuple(float(x) for x in model.actuator_ctrlrange[aa])

    if verbose:
        print("\n[4] JOINT DYNAMICS  (via dof_* arrays -- model.joint(name) hides these)")
    kd = int(model.jnt_dofadr[kj])
    arm = float(model.dof_armature[kd])
    dmp = float(model.dof_damping[kd])
    frc = float(model.dof_frictionloss[kd])
    if verbose:
        print(f"    knee armature={arm}  damping={dmp}  frictionloss={frc}")
    chk(close(arm, EXPECT["knee_armature"]), f"knee armature == {EXPECT['knee_armature']}")
    chk(close(dmp, EXPECT["knee_damping"]), f"knee damping == {EXPECT['knee_damping']}")
    chk(close(frc, EXPECT["knee_frictionloss"]),
        f"knee frictionloss == {EXPECT['knee_frictionloss']}")

    if verbose:
        print("\n[5] PLANT INERTIA  (measured from the compiled model, not the XML text)")
    kid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "flat")
    mujoco.mj_resetDataKeyframe(model, data, kid if kid >= 0 else 0)
    data.qpos[:] = 0.0
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)
    i_eff, i_full = dof_inertia(model, data, kd)
    i_body = i_eff - arm
    if verbose:
        print(f"    M[knee,knee] via mj_mulM = {i_eff:.6f} kg.m^2  "
              f"(body {i_body:.6f} + armature {arm})")
    if math.isnan(i_full):
        if verbose:
            print("    mj_fullM cross-check     = unavailable on this MuJoCo build (skipped)")
    else:
        if verbose:
            print(f"    mj_fullM cross-check     = {i_full:.6f} kg.m^2")
        # warn, not check: a second opinion must not be able to refuse the experiment
        wrn(close(i_eff, i_full, 1e-9),
            f"mj_mulM {i_eff:.9f} and mj_fullM {i_full:.9f} disagree on M[knee,knee]")
    chk(abs(i_body - I_BODY_EXPECT) < 2e-3,
        f"knee-distal inertia == {I_BODY_EXPECT:.6f} kg.m^2 within 2e-3 (got {i_body:.6f})")

    # gravity moment about the knee at q = 90 deg -> m*g*d
    data.qpos[:] = 0.0
    data.qpos[int(model.jnt_qposadr[kj])] = math.pi / 2
    data.qvel[:] = 0.0
    data.ctrl[:] = 0.0
    mujoco.mj_forward(model, data)
    mgd = abs(float(data.qfrc_bias[kd]))
    if verbose:
        print(f"    |qfrc_bias| at q=90deg = {mgd:.4f} N.m  (expect m*g*d ~ {MGD_EXPECT})")
    wrn(abs(mgd - MGD_EXPECT) < 0.15,
        f"gravity moment {mgd:.4f} differs from the COMPUTED {MGD_EXPECT} N.m by "
        f"more than 0.15")

    bm.sensors = {nm: int(model.sensor_adr[name_id(model, mujoco.mjtObj.mjOBJ_SENSOR, nm)])
                  for nm in SENSOR_NAMES}
    bm.knee_dof, bm.knee_qpos = kd, int(model.jnt_qposadr[kj])
    bm.ankle_dof, bm.ankle_qpos = int(model.jnt_dofadr[aj]), int(model.jnt_qposadr[aj])
    bm.armature, bm.b_joint, bm.frictionloss = arm, dmp, frc
    bm.i_eff, bm.i_body, bm.mgd = i_eff, i_body, mgd
    bm.keyframe = int(kid)
    return bm
