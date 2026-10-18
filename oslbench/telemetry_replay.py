"""Deterministic, data-only CSV/JSON replay. No physics or command imports."""
from dataclasses import dataclass
from pathlib import Path
from types import MappingProxyType
import csv
import hashlib
import json
import math
import re

from .contracts import Sample, RobotState

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
REQUIRED_STATE = tuple(CHANNELS)[:4]
PROVENANCE = ("simulation", "measured", "estimated", "derived", "replayed")
HEADER = ("sequence", "snapshot_id", "channel", "time_s", "value", "valid")


class DatasetError(ValueError):
    """The dataset has no unambiguous version-1 interpretation."""


def _require(ok, message):
    if not ok:
        raise DatasetError(message)


def _number(text, label):
    try:
        value = float(text)
    except (ValueError, TypeError):
        raise DatasetError(label + ": expected finite number") from None
    _require(math.isfinite(value), label + ": non-finite value")
    return value


def _integer(text, label):
    _require(isinstance(text, str) and re.fullmatch(r"0|[1-9][0-9]*", text) is not None,
             label + ": expected nonnegative integer")
    return int(text)


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        _require(key not in result, "Duplicate JSON key: " + key)
        result[key] = value
    return result


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


class ReplaySource:
    """Validated immutable records plus a resettable virtual delivery clock.

    next() advances clock to the next record. advance_to(t) drains records <= t.
    robot_state(id) is explicit random access, not a live state accumulator.
    """
    def __init__(self, manifest_path):
        path = Path(manifest_path)
        try:
            m = json.loads(path.read_text(encoding="utf-8"), object_pairs_hook=_unique_object)
        except (OSError, UnicodeError, json.JSONDecodeError) as exc:
            raise DatasetError(f"Manifest: {exc}") from exc
        _require(isinstance(m, dict), "Manifest must be an object")
        required = {"schema_version", "dataset_id", "source", "clock", "sequence", "channels",
                    "data_file", "record_count", "data_sha256"}
        _require(required <= m.keys() and m.keys() <= required | {"identity"}, "Manifest fields mismatch")
        _require(type(m["schema_version"]) is int and m["schema_version"] == 1, "Unsupported schema_version")
        _require(isinstance(m["dataset_id"], str) and bool(m["dataset_id"].strip()), "dataset_id required")
        _require(m["source"] in PROVENANCE, "Unknown source provenance")
        clock = m["clock"]
        _require(isinstance(clock, dict) and set(clock) == {"id", "unit"}, "clock requires id/unit")
        _require(isinstance(clock["id"], str) and bool(clock["id"].strip()) and clock["unit"] == "s", "Clock must have identity and seconds unit")
        _require(m["sequence"] == "contiguous_zero_based", "Unsupported sequence semantics")
        _require(type(m["record_count"]) is int and m["record_count"] >= 0, "Invalid record_count")
        _require(isinstance(m["data_sha256"], str) and re.fullmatch('[0-9a-f]{64}',m["data_sha256"]) is not None, "Invalid data_sha256")
        _require(isinstance(m["data_file"], str) and m["data_file"] == Path(m["data_file"]).name and m["data_file"] not in ('', '.', '..'), "data_file must be a sibling filename")
        channels = m["channels"]
        _require(isinstance(channels, dict) and bool(channels), "Nonempty channels required")
        for name, meta in channels.items():
            _require(name in CHANNELS, "Unknown channel: " + name)
            _require(isinstance(meta, dict) and set(meta) == {"quantity", "unit", "frame", "stage", "source", "timestamp", "sign"}, "Channel metadata fields: " + name)
            _require(tuple(meta[k] for k in ("quantity", "unit", "frame")) == CHANNELS[name], "Incompatible quantity/unit/frame: " + name)
            _require(meta["timestamp"] == "time_s" and meta["source"] in PROVENANCE, "Invalid timestamp/source: " + name)
            _require(isinstance(meta["sign"], str) and bool(meta["sign"].strip()), "Sign convention required: " + name)
            try:
                Sample(0., meta["unit"], meta["frame"], 0., meta["stage"], meta["source"])
            except ValueError as exc:
                raise DatasetError(f"Channel {name}: {exc}") from exc
        try:
            data = (path.parent / m["data_file"]).read_bytes()
        except OSError as exc:
            raise DatasetError(f"Data file: {exc}") from exc
        _require(hashlib.sha256(data).hexdigest() == m["data_sha256"], "Data SHA-256 mismatch (changed/truncated file)")
        import io
        try:
            reader = csv.DictReader(io.StringIO(data.decode("utf-8"), newline=""), strict=True)
            _require(reader.fieldnames == list(HEADER), "CSV header mismatch")
            records, groups, seen = [], {}, set()
            previous_time = -1.
            for line, row in enumerate(reader, 2):
                label = f"CSV line {line}"
                _require(set(row) == set(HEADER) and all(v is not None for v in row.values()), label + ": malformed columns")
                seq = _integer(row["sequence"], label + " sequence")
                _require(seq == len(records), label + ": duplicate, missing or out-of-order sequence")
                group = _integer(row["snapshot_id"], label + " snapshot_id")
                name = row["channel"]
                _require(name in channels, label + ": undeclared channel " + name)
                _require((group, name) not in seen, label + ": duplicate snapshot/channel")
                seen.add((group, name))
                t = _number(row["time_s"], label + " time_s")
                _require(t >= 0 and t >= previous_time, label + ": out-of-order timestamp")
                previous_time = t
                _require(row["valid"] in ("0", "1"), label + ": valid must be 0 or 1")
                valid = row["valid"] == "1"
                meta = channels[name]
                if row["value"] == "":
                    _require(not valid, label + ": missing value marked valid")
                    sample = None
                else:
                    sample = Sample(_number(row["value"], label + " value"), meta["unit"], meta["frame"], t, meta["stage"], meta["source"], valid)
                record = ReplayRecord(seq, group, name, t, clock["id"], sample,
                                      meta["source"], meta["unit"], meta["frame"], meta["stage"])
                records.append(record)
                groups.setdefault(group, {})[name] = sample
            _require(len(records) == m["record_count"], "record_count mismatch (truncated or extra data)")
        except (UnicodeError, csv.Error) as exc:
            raise DatasetError(f"CSV: {exc}") from exc
        _require("identity" not in m or isinstance(m["identity"], dict), "identity must be an object")
        self._manifest_json = json.dumps(m)
        self.dataset_id = m["dataset_id"]
        self.clock_id = clock["id"]
        self.records = tuple(records)
        self._groups = groups
        self.reset()

    @property
    def manifest(self):
        """Return a detached metadata copy; original provenance remains intact."""
        return json.loads(self._manifest_json)

    def reset(self):
        self._cursor = 0
        self.time_s = 0.

    @property
    def ended(self):
        return self._cursor == len(self.records)

    def __iter__(self):
        return self

    def __next__(self):
        if self.ended:
            raise StopIteration
        record = self.records[self._cursor]
        self._cursor += 1
        self.time_s = max(self.time_s, record.time_s)
        return record

    def advance_to(self, time_s):
        t = _number(time_s, "Replay clock")
        _require(t >= self.time_s, "Replay clock cannot go backwards; reset first")
        emitted = []
        while not self.ended and self.records[self._cursor].time_s <= t:
            emitted.append(next(self))
        self.time_s = t
        return tuple(emitted)

    def robot_state(self, snapshot_id):
        """Explicit group lookup: never interpolate, carry forward, or fill zeros.

        Missing/invalid required kinematics -> None. Optional invalid samples keep
        their validity; missing optional samples remain None. Unknown group -> KeyError.
        """
        samples = self._groups[snapshot_id]
        if any(samples.get(k) is None or not samples[k].valid for k in REQUIRED_STATE):
            return None
        return RobotState(**{k: samples.get(k) for k in CHANNELS})
