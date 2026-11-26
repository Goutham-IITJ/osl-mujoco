"""One-joint plant adapter; controller and compliant drivetrain remain shared."""
from pathlib import Path
from types import SimpleNamespace
import mujoco


def load_fixture_model():
    path = Path(__file__).resolve().parents[2] / 'models/osl_v2_knee_fixture.xml'
    model = mujoco.MjModel.from_xml_path(str(path))
    if (model.nq, model.nv, model.nu) != (1, 1, 1):
        raise ValueError('Fixture must contain exactly one knee joint and torque actuator')
    joint = model.joint('knee')
    if abs(model.opt.timestep - .0005) > 1e-12:
        raise ValueError('Fixture replay requires 0.5 ms physics timestep')
    return SimpleNamespace(model=model, data=mujoco.MjData(model), path=str(path),
        knee_qpos=int(joint.qposadr[0]), knee_dof=int(joint.dofadr[0]),
        timestep=float(model.opt.timestep), failures=[])


class FixtureSimulation:
    def __init__(self, bench, layer, current_source, joint_name='knee'):
        self.bench, self.layer, self.current_source = bench, layer, current_source
        self.servo_connected = False
        self.last_layer = None
        joint=bench.model.joint(joint_name)
        self.qpos=int(joint.qposadr[0])
        self.dof=int(joint.dofadr[0])

    def reset(self, q0_rad, ref_vel0=None):
        b = self.bench
        mujoco.mj_resetData(b.model, b.data)
        b.data.qpos[self.qpos] = q0_rad
        mujoco.mj_forward(b.model, b.data)
        self.layer.seed_rigid(float(q0_rad), 0.)
        self.layer.steps = 0
        self.last_layer = None

    def step(self, q_ref, k):
        b = self.bench
        q, qd = float(b.data.qpos[self.qpos]), float(b.data.qvel[self.dof])
        current = float(self.current_source(k, q, qd))
        self.last_layer = self.layer.advance(q, current, b.timestep)
        b.data.ctrl[:] = 0.  # No second torque source alongside the belt.
        b.data.qfrc_applied[self.dof] = self.last_layer.tau_j_mean
        mujoco.mj_step(b.model, b.data)

    def knee_servo_force(self):
        return float(self.bench.data.actuator_force[0])
