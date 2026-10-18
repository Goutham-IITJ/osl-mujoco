"""Shared observation envelope and channel declarations; no replay or transport."""
from dataclasses import dataclass
from types import MappingProxyType

from .contracts import Sample

# Version 1 supports only the state quantities with established contract semantics.
CHANNELS = MappingProxyType({
    "knee_angle": ("angle", "rad", "knee_joint"),
    "knee_velocity": ("angular_velocity", "rad/s", "knee_joint"),
    "actuator_output_angle": ("angle", "rad", "actuator_output"),
    "actuator_output_velocity": ("angular_velocity", "rad/s", "actuator_output"),
    "electromagnetic_output_torque": ("torque", "Nm", "actuator_output"),
    "belt_joint_torque": ("torque", "Nm", "knee_joint"),
    "belt_deflection": ("angle", "rad", "knee_joint"),
})

# CHANNELS remains the frozen Phase 6D replay-v1 registry. Only live ingestion
# uses this additive registry; an IMU is never represented as a knee encoder.
IMU_CHANNELS = MappingProxyType({
    **{f"imu_{axis}": ("angle", "rad", "imu_filter_ned")
       for axis in ("roll", "pitch", "yaw")},
    **{f"imu_q{i}": ("quaternion_component", "1", "imu_filter_ned")
       for i in range(4)},
})
OBSERVATION_CHANNELS = MappingProxyType({**CHANNELS, **IMU_CHANNELS})


@dataclass(frozen=True)
class ReplayRecord:
    """Envelope preserves sequence/group/clock, even when a sample is missing."""
    sequence: int
    snapshot_id: int
    channel: str
    time_s: float
    clock_id: str
    sample: Sample | None
    source: str
    unit: str
    frame: str
    stage: str
