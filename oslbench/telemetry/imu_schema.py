"""Versioned read-only IMU HTTP payload. Not the Phase 6D replay format."""
import json
import math

from oslbench.telemetry_schema import IMU_CHANNELS

VERSION = "osl-imu-http/1"
NAMES = tuple(IMU_CHANNELS)
MAX_BODY = 65536
STATES = ("OPEN", "WAITING", "STALE", "DISCONNECTED", "CLOSED")


def number(value, name, nonnegative=True):
    if (type(value) not in (int, float) or not math.isfinite(value)
            or (nonnegative and value < 0)):
        raise ValueError(f"Invalid {name}")
    return value


def integer(value, name):
    if type(value) is not int or value < 0:
        raise ValueError(f"Invalid {name}")
    return value


def metadata(session, provenance, mounting):
    if provenance not in ("measured", "synthetic_mock"):
        raise ValueError("Unknown provenance")
    if not isinstance(mounting, str) or not mounting.strip():
        raise ValueError("Explicit mounting description required (unknown is allowed)")
    return dict(dataset_id=session, source=provenance,
        clock=dict(id=session + ":pi_receipt", unit="s"),
        channels={name: dict(quantity=q, unit=u, frame=f, stage="acquisition",
            source=provenance, timestamp="Pi monotonic receipt seconds since service start",
            sign="Device filter convention; no knee mapping or mounting correction")
            for name, (q, u, f) in IMU_CHANNELS.items()},
        identity=dict(device="MicroStrain 3DM-GV7-AR (operator must verify)",
            orientation="Filter 0x82/0x05 radians, 3-2-1 yaw-pitch-roll; 0x82/0x03 q0,q1,q2,q3",
            reference_frame="NED local-level; device/vehicle mounting transform unverified",
            mounting=mounting, synthetic=provenance == "synthetic_mock",
            timing="time_s is Pi receipt, NOT sensor sample time. Device timestamps remain separate.",
            delivery="Latest value per channel; sequence gaps expose skipped packets. Not a recorder."))


def timestamps(values):
    if not isinstance(values, dict) or set(values) - {"reference_ns", "gps_unix_ns", "external_ns", "legacy_unix_ns"}:
        raise ValueError("Unknown device timestamp representation")
    for clock, item in values.items():
        if not isinstance(item, dict) or set(item) != {"value", "valid"} or type(item['valid']) is not bool:
            raise ValueError("Malformed device timestamp")
        if item['value'] is not None:
            integer(item['value'], clock)
        elif item['valid']:
            raise ValueError("Valid device timestamp needs a value")


def observation(item):
    if not isinstance(item, dict) or set(item) != {"sequence", "time_s", "value", "valid", "device_timestamps", "reason"}:
        raise ValueError("Malformed observation")
    integer(item['sequence'], 'sequence')
    number(item['time_s'], 'Pi receipt time')
    if type(item['valid']) is not bool or not isinstance(item['reason'], str):
        raise ValueError("Explicit validity and reason required")
    if item['value'] is None:
        if item['valid']:
            raise ValueError("Missing value cannot be valid")
    else:
        number(item['value'], 'orientation', nonnegative=False)
    timestamps(item['device_timestamps'])


def validate(payload):
    if not isinstance(payload, dict) or set(payload) != {"schema", "session_id", "metadata", "status", "error", "server_time_s", "stale_after_s", "samples"}:
        raise ValueError("Unexpected IMU payload fields")
    if payload['schema'] != VERSION or payload['status'] not in STATES:
        raise ValueError("Unsupported IMU version/status")
    sid = payload['session_id']
    if not isinstance(sid, str) or not sid or len(sid) > 128:
        raise ValueError("Invalid session identity")
    m = payload['metadata']
    if not isinstance(m, dict) or set(m) != {'dataset_id', 'source', 'clock', 'channels', 'identity'}:
        raise ValueError("Malformed metadata")
    identity = m.get('identity')
    if not isinstance(identity, dict):
        raise ValueError("Malformed identity")
    expected = metadata(sid, m['source'], identity.get('mounting'))
    if m != expected:
        raise ValueError("Unverified units, frame, clock, ordering or provenance")
    if not isinstance(payload['error'], str):
        raise ValueError("Error must be text")
    now = number(payload['server_time_s'], 'server time')
    if number(payload['stale_after_s'], 'stale threshold') <= 0:
        raise ValueError("Stale threshold must be positive")
    samples = payload['samples']
    if not isinstance(samples, dict) or set(samples) - set(NAMES):
        raise ValueError("Unknown IMU channel")
    for item in samples.values():
        observation(item)
        if item['time_s'] > now:
            raise ValueError("Observation after server clock")
    return payload


def encode(payload):
    return json.dumps(validate(payload), allow_nan=False, separators=(',', ':')).encode('utf-8')


def decode(data):
    if len(data) > MAX_BODY:
        raise ValueError("IMU payload too large")
    def pairs(items):
        result = {}
        for k, v in items:
            if k in result:
                raise ValueError("Duplicate JSON key")
            result[k] = v
        return result
    return validate(json.loads(data, object_pairs_hook=pairs))
