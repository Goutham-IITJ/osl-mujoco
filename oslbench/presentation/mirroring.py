"""D1: display a calibrated measured knee angle, without physics or control.

The caller owns a separate display model/data and its knee qpos index. No encoder
transport or calibration is inferred here; IMU observations are not joint angles.
"""
from oslbench.contracts import Sample


def mirror_measured_knee(sample, model, data, knee_qpos, *, kinematics):
    if not isinstance(sample, Sample) or not sample.valid:
        raise ValueError("A valid measured joint sample is required")
    if (sample.source, sample.unit, sample.frame, sample.stage) != (
            "measured", "rad", "knee_joint", "acquisition"):
        raise ValueError("Requires acquired encoder position in calibrated knee coordinates")
    if type(knee_qpos) is not int or not 0 <= knee_qpos < len(data.qpos):
        raise ValueError("Invalid knee qpos index")
    data.qpos[knee_qpos] = sample.value
    # Supply mujoco.mj_kinematics: no mj_step, mj_forward or command output.
    kinematics(model, data)
