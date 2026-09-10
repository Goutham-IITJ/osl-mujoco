"""
tests/stub_mujoco.py -- a stand-in for MuJoCo, so the tests can run WITHOUT MuJoCo.

READ THIS BEFORE TRUSTING ANY NUMBER THAT CAME OUT OF IT
    This is NOT MuJoCo and it is NOT the validated physics.  It is a small, deterministic
    2-DOF plant with the same interface, whose only job is to make the WIRING testable:
    that the reference reaches the controller, that the controller's gains reach the
    actuator, that the ROM clip and the forcerange clamp are applied, that the logger
    records the values the step returned, and that every entry point runs end to end.

    The validated physics numbers come from REAL MuJoCo only.  The check that defends
    them is `experiments/verify_against_oracle.py`, which compares a real run against the
    frozen tests/oracle/ result.  Nothing in this file can substitute for that.

WHY IT EXISTS ANYWAY
    Two reasons.  A reviewer can clone the repo, run `python tests/run_tests.py` with
    nothing but numpy installed, and see the control law and the plumbing verified.  And
    a mistake in the plumbing is caught here, where it costs seconds, instead of during a
    demo.

WHAT THE PLANT IS
    knee    I*qdd = tau - m*g*d*sin(q) - b*qd - frictionloss*sign(qd)
    ankle   the same form with its own (much smaller) inertia
    semi-implicit Euler at 0.5 ms, matching the authored timestep.  The inertia, damping,
    friction and gravity constants are the ones MEASURED from the compiled model, so the
    stub is in the right ballpark -- but "the right ballpark" is not the experiment.
"""

from __future__ import annotations

import math
import sys
import types

import numpy as np

# ---- constants measured from the compiled models/osl_v2_bench.xml -------------------
DT = 0.0005
I_BODY, ARMATURE = 0.251998, 0.01           # kg.m^2   knee-distal inertia + armature
I_KNEE = I_BODY + ARMATURE                  # 0.261998
B_KNEE, FRIC_KNEE = 0.3, 0.4                # N.m.s/rad, N.m
MGD = 8.8529                                # N.m, gravity moment about the knee at 90 deg
I_ANKLE, B_ANKLE, FRIC_ANKLE = 0.0080, 0.05, 0.05
MGD_ANKLE = 0.35

KNEE_LIM, ANKLE_LIM = 142.2, 168.2          # N.m, authored forcerange
KNEE_RANGE = (-0.0872664625997, 2.09439510239)
ANKLE_RANGE = (-0.523598775598, 0.349065850399)
ANKLE_KEYFRAME = 0.041837154678             # rad, the "flat" keyframe value
AUTHORED_KP, AUTHORED_KV = 60.0, 0.0

KNEE, ANKLE = 0, 1
JOINT_NAMES = ("knee", "ankle")
ACT_NAMES = ("knee_pos", "ankle_pos")
SENSOR_NAMES = ("knee_q", "knee_qd", "knee_tau", "ankle_q", "ankle_qd", "ankle_tau")


class _Opt:
    def __init__(self):
        self.timestep = DT
        self.gravity = np.array([0.0, 0.0, -9.81])
        self.flags = {}


class StubModel:
    """The same arrays oslbench.model reads, filled with the AUTHORED values."""

    def __init__(self, path=""):
        self.path = path
        self.opt = _Opt()
        self.nq = self.nv = self.nu = 2
        self.nbody = 4
        self.body_jntnum = np.array([0, 0, 1, 1])       # body 1 has no joint: fixed base
        self.jnt_range = np.array([KNEE_RANGE, ANKLE_RANGE])
        self.jnt_dofadr = np.array([0, 1])
        self.jnt_qposadr = np.array([0, 1])
        self.dof_armature = np.array([ARMATURE, 0.0])
        self.dof_damping = np.array([B_KNEE, B_ANKLE])
        self.dof_frictionloss = np.array([FRIC_KNEE, FRIC_ANKLE])
        self.actuator_forcerange = np.array([[-KNEE_LIM, KNEE_LIM],
                                             [-ANKLE_LIM, ANKLE_LIM]])
        self.actuator_ctrlrange = np.array([KNEE_RANGE, ANKLE_RANGE])
        self.actuator_gear = np.ones((2, 6))
        self.actuator_gaintype = np.zeros(2, int)       # mjGAIN_FIXED
        self.actuator_biastype = np.full(2, 2, int)     # mjBIAS_AFFINE
        self.actuator_dyntype = np.zeros(2, int)        # mjDYN_NONE
        self.actuator_trntype = np.zeros(2, int)        # mjTRN_JOINT
        self.actuator_gainprm = np.zeros((2, 10))
        self.actuator_biasprm = np.zeros((2, 10))
        for a in (KNEE, ANKLE):                         # the MJCF-authored servo
            self.actuator_gainprm[a, 0] = AUTHORED_KP
            self.actuator_biasprm[a, 1] = -AUTHORED_KP
            self.actuator_biasprm[a, 2] = -AUTHORED_KV
        self.sensor_adr = np.arange(6)

    @staticmethod
    def from_xml_path(path):
        return StubModel(path)


class StubData:
    def __init__(self, model):
        self._m = model
        self.qpos = np.zeros(2)
        self.qvel = np.zeros(2)
        self.qacc = np.zeros(2)
        self.ctrl = np.zeros(2)
        self.qfrc_bias = np.zeros(2)
        self.sensordata = np.zeros(6)
        self.actuator_force = np.zeros(2)
        self.xanchor = np.zeros((4, 3))
        self.M = np.array([I_KNEE, I_ANKLE])            # diagonal, as a sparse stand-in
        self.time = 0.0


# ------------------------------------------------------------------- the plant
def _actuator_torque(model, data, a, q, qd):
    """Exactly the algebra a MuJoCo position actuator applies, including the clamp."""
    tau = (float(model.actuator_gainprm[a, 0]) * float(data.ctrl[a])
           + float(model.actuator_biasprm[a, 0])
           + float(model.actuator_biasprm[a, 1]) * q
           + float(model.actuator_biasprm[a, 2]) * qd)
    lo, hi = model.actuator_forcerange[a]
    return float(min(max(tau, lo), hi))


def _bias(q, mgd):
    return -mgd * math.sin(q)


def _refresh(model, data, applied=None):
    """Recompute the reported quantities for the CURRENT state.

    `applied` is MuJoCo's convention, and it matters: after mj_step, both
    data.actuator_force and the actuatorfrc sensors report the force that was USED to
    take the step -- i.e. evaluated on the PRE-step state -- not a fresh evaluation on
    the state the step landed on.  Passing the applied torques in reproduces that.
    Leaving it None (mj_forward) evaluates on the current state, which is also what
    MuJoCo does there.
    """
    for a in (KNEE, ANKLE):
        data.actuator_force[a] = (
            float(applied[a]) if applied is not None else
            _actuator_torque(model, data, a, float(data.qpos[a]), float(data.qvel[a])))
    data.qfrc_bias[0] = _bias(float(data.qpos[0]), MGD)
    data.qfrc_bias[1] = _bias(float(data.qpos[1]), MGD_ANKLE)
    data.sensordata[:] = (data.qpos[0], data.qvel[0], data.actuator_force[KNEE],
                          data.qpos[1], data.qvel[1], data.actuator_force[ANKLE])
    # crude forward kinematics, only so viewer.derive_camera has anchors to look at
    q = float(data.qpos[0])
    data.xanchor[1] = (0.0, 0.0, 0.0)
    data.xanchor[2] = (-0.42 * math.sin(q), 0.0, -0.42 * math.cos(q))


def mj_forward(model, data):
    _refresh(model, data)


def _advance(q, qd, tau, inertia, b, fric, mgd, lo, hi):
    net = tau + _bias(q, mgd) - b * qd
    if abs(qd) < 1e-9 and abs(net) <= fric:
        qd_new = 0.0                                    # stiction
    else:
        f = -fric * math.copysign(1.0, qd if abs(qd) > 1e-12 else net)
        qd_new = qd + (net + f) / inertia * DT
        if qd != 0.0 and qd_new * qd < 0.0 and abs(net) <= fric:
            qd_new = 0.0
    q_raw = q + qd_new * DT
    q_new = min(max(q_raw, lo), hi)
    if q_new != q_raw:                                  # hit a joint limit: stop there
        qd_new = 0.0
    return q_new, qd_new


def mj_step(model, data):
    applied = [0.0, 0.0]
    for a, inertia, b, fric, mgd, rng in ((KNEE, I_KNEE, B_KNEE, FRIC_KNEE, MGD,
                                           KNEE_RANGE),
                                          (ANKLE, I_ANKLE, B_ANKLE, FRIC_ANKLE,
                                           MGD_ANKLE, ANKLE_RANGE)):
        q, qd = float(data.qpos[a]), float(data.qvel[a])
        tau = _actuator_torque(model, data, a, q, qd)
        applied[a] = tau
        data.qpos[a], data.qvel[a] = _advance(q, qd, tau, inertia, b, fric, mgd,
                                              rng[0], rng[1])
    data.time += DT
    _refresh(model, data, applied)


def mj_resetDataKeyframe(model, data, kid):
    data.qpos[:] = (0.0, ANKLE_KEYFRAME)
    data.qvel[:] = 0.0
    data.ctrl[:] = 0.0
    data.time = 0.0
    _refresh(model, data)


def mj_mulM(model, data, res, vec):
    res[:] = np.asarray(data.M) * np.asarray(vec)


def mj_fullM(model, dense, sparse):
    dense[:] = np.diag(np.asarray(sparse))


def mj_name2id(model, objtype, name):
    table = {"joint": JOINT_NAMES, "actuator": ACT_NAMES, "sensor": SENSOR_NAMES,
             "key": ("flat",)}
    names = table.get(str(objtype), ())
    return names.index(name) if name in names else -1


class _MjtObj:
    mjOBJ_JOINT, mjOBJ_ACTUATOR = "joint", "actuator"
    mjOBJ_SENSOR, mjOBJ_KEY, mjOBJ_BODY = "sensor", "key", "body"


class _MjtGain:
    mjGAIN_FIXED = 0


class _MjtBias:
    mjBIAS_AFFINE = 2


class _MjtDyn:
    mjDYN_NONE = 0


class _MjtTrn:
    mjTRN_JOINT = 0


class _MjtVisFlag:
    mjVIS_JOINT = "mjVIS_JOINT"


class _MjvCamera:
    def __init__(self):
        self.lookat = np.zeros(3)
        self.distance = self.azimuth = self.elevation = 0.0
        self.type = 0


class _Renderer:
    def __init__(self, model, height, width):
        self.h, self.w, self.scenes = height, width, 0

    def update_scene(self, data, cam=None):
        self.scenes += 1

    def render(self):
        return np.zeros((self.h, self.w, 3), np.uint8)

    def close(self):
        pass


def install():
    """Put the stub in sys.modules as `mujoco`.  Returns the module.

    Real MuJoCo wins if it is importable: a test run inside .venv exercises the real
    engine, and the stub is only a fallback for an environment that has none.
    """
    try:
        import mujoco                                   # noqa: F401
        return mujoco
    except ImportError:
        pass

    m = types.ModuleType("mujoco")
    m.MjModel, m.MjData = StubModel, StubData
    m.mj_step, m.mj_forward = mj_step, mj_forward
    m.mj_resetDataKeyframe = mj_resetDataKeyframe
    m.mj_mulM, m.mj_fullM, m.mj_name2id = mj_mulM, mj_fullM, mj_name2id
    m.mjtObj, m.mjtGain, m.mjtBias = _MjtObj, _MjtGain, _MjtBias
    m.mjtDyn, m.mjtTrn, m.mjtVisFlag = _MjtDyn, _MjtTrn, _MjtVisFlag
    m.MjvCamera, m.Renderer = _MjvCamera, _Renderer
    m.mjv_defaultFreeCamera = lambda model, cam: None
    v = types.ModuleType("mujoco.viewer")
    v.launch_passive = None                             # the live demo needs real MuJoCo
    m.viewer = v
    m.__stub__ = True
    sys.modules["mujoco"], sys.modules["mujoco.viewer"] = m, v
    return m


def is_stub(mod=None) -> bool:
    mod = mod or sys.modules.get("mujoco")
    return bool(getattr(mod, "__stub__", False))
