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
