"""D2 handoff: explicit estimator/reference generation, with no default mapping.

An application must supply a validated estimator and separately schedule the
returned reference in a simulation. This function neither steps nor commands it.
"""
from oslbench.contracts import PositionCommand, finite
from oslbench.telemetry_schema import IMU_CHANNELS, ReplayRecord


def prepare_imu_reference(records, time_s, *, estimate_reference):
    finite(time_s)
    if time_s < 0 or not callable(estimate_reference):
        raise ValueError("Explicit estimator and nonnegative simulation time required")
    records = tuple(records)
    if not records:
        raise ValueError("Measured IMU observations required")
    for record in records:
        if not isinstance(record, ReplayRecord) or record.channel not in IMU_CHANNELS:
            raise ValueError("Expected an IMU observation")
        sample = record.sample
        _, unit, frame = IMU_CHANNELS[record.channel]
        if sample is None or not sample.valid or (
                sample.source, sample.unit, sample.frame, sample.stage) != (
                "measured", unit, frame, "acquisition"):
            raise ValueError("Valid measured IMU observation required")
        if (record.source, record.unit, record.frame, record.stage, record.time_s) != (
                sample.source, sample.unit, sample.frame, sample.stage, sample.time_s):
            raise ValueError("Observation envelope disagrees with sample")
    reference = estimate_reference(records, time_s)
    if not isinstance(reference, PositionCommand) or reference.coordinate != "knee_joint":
        raise ValueError("Estimator must explicitly return a knee-joint position reference")
    return reference
