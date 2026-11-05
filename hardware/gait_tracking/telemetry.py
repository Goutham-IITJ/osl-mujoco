"""Copy this file to the hardware PC/Pi beside its controller. Standard library only.

One control-thread producer; networking never runs in publish(). This is soft
real-time Python: a nonblocking buffer does not guarantee hard real-time timing.
"""
import argparse
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import hmac
import json
import math
import os
import threading
import time
from urllib.parse import parse_qs, urlsplit
import uuid

SCHEMA = "osl-joint-telemetry-v1"
NUMBERS = ("time_s", "dt_s", "reference_deg", "actual_deg",
           "requested_current_A", "commanded_current_A")


def validate_sample(row):
    if not isinstance(row, dict):
        raise ValueError("sample must be an object")
    for key in NUMBERS:
        value = row.get(key)
        if type(value) not in (int, float) or not math.isfinite(value):
            raise ValueError("invalid " + key)
    if row["time_s"] < 0 or row["dt_s"] <= 0:
        raise ValueError("invalid source time/dt")
    if type(row.get("sequence")) is not int or row["sequence"] < 0:
        raise ValueError("invalid sequence")
    if type(row.get("encoder_valid")) is not bool:
        raise ValueError("encoder_valid must be boolean")
    if row.get("joint") not in ("knee", "ankle"):
        raise ValueError("invalid joint")
    for key in ("joint_sequence", "control_tick"):
        if type(row.get(key)) is not int or row[key] < 0:
            raise ValueError("invalid " + key)
    for key in ("reference_time_s", "reference_velocity_deg_s", "p_term_A", "i_term_A", "d_term_A"):
        if type(row.get(key)) not in (int, float) or not math.isfinite(row[key]):
            raise ValueError("invalid " + key)
    if not 0 <= row["reference_time_s"] <= row["time_s"]:
        raise ValueError("invalid reference time")
    if not isinstance(row.get("status"), str) or len(row["status"]) > 128:
        raise ValueError("status must be a short string")


class TrackingTelemetryServer:
    def __init__(self, host="127.0.0.1", port=8794, *, token=None,
                 capacity=8000, source="physical", angle_sign=-1):
        if host not in ("127.0.0.1", "localhost", "::1") and not token:
            raise ValueError("LAN binding requires a nonempty token")
        if capacity < 1 or angle_sign not in (-1, 1):
            raise ValueError("invalid capacity/sign")
        self.token = token
        self._lock = threading.Lock()
        self._rows = deque(maxlen=capacity)
        self._sequence = 0
        self._last_time = -1.0
        self._last_publish = None
        self.dropped = 0
        self.rejected = 0
        self.metadata = dict(schema=SCHEMA, session_id=str(uuid.uuid4()),
            source=source, clock="controller monotonic elapsed seconds",
            angle_unit="degree", angle_frame="calibrated knee encoder",
            flexion_positive_multiplier=angle_sign,
            current_unit="A", current_frame="driver command; shaft/sign unverified",
            measured_current_available=False, torque_constant_verified=False,
            sample_stage="encoder read then current command in same control tick",
            capacity=capacity)
        self.metadata.update(joints={"knee": {"angle_multiplier": -1, "current_multiplier": 1},
                                     "ankle": {"angle_multiplier": 1, "current_multiplier": -1}},
                             producer_complete=False,
                             reference_clock="reference_time_s is shared loop-start time; time_s is encoder-read time")
        self.metadata.pop("flexion_positive_multiplier", None)
        self.metadata["angle_frame"] = "raw calibrated per-joint encoder, before CSV cosmetic sign"
        self._joint_sequences = {"knee": 0, "ankle": 0}
        owner = self

        class Handler(BaseHTTPRequestHandler):
            def setup(self):
                super().setup()
                self.connection.settimeout(2.0)

            def log_message(self, *args):
                pass

            def do_GET(self):
                if owner.token and not hmac.compare_digest(
                        self.headers.get("Authorization", ""), "Bearer " + owner.token):
                    self.send_error(401)
                    return
                path = urlsplit(self.path)
                if path.path == "/v1/metadata":
                    result = owner.metadata
                elif path.path == "/v1/samples":
                    try:
                        query = parse_qs(path.query)
                        after = int(query.get("after", ["-1"])[0])
                        limit = int(query.get("limit", ["1000"])[0])
                        if after < -1 or not 1 <= limit <= 1000:
                            raise ValueError()
                    except ValueError:
                        self.send_error(400)
                        return
                    result = owner.batch(after, limit)
                else:
                    self.send_error(404)
                    return
                payload = json.dumps(result, allow_nan=False).encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(payload)))
                self.end_headers()
                self.wfile.write(payload)

        self.server = ThreadingHTTPServer((host, port), Handler)
        self.server.daemon_threads = True
        self.thread = threading.Thread(target=self.server.serve_forever,
                                       kwargs={"poll_interval": .05}, daemon=True)
        self.thread.start()

    @property
    def port(self):
        return self.server.server_address[1]

    def publish(self, *, time_s, dt_s, reference_deg, actual_deg,
                requested_current_A, commanded_current_A,
                encoder_valid, joint, control_tick, reference_time_s, reference_velocity_deg_s,
                p_term_A, i_term_A, d_term_A, status="running"):
        """Return False for a dropped/rejected tick. Never wait for network/lock.

        Call from exactly one producer thread. Numbers use original controller
        signs; the receiver performs explicit display-only sign conversion.
        """
        if joint not in self._joint_sequences:
            raise ValueError("Unknown joint")
        joint_sequence = self._joint_sequences[joint]
        self._joint_sequences[joint] += 1
        sequence = self._sequence
        self._sequence += 1
        row = dict(sequence=sequence, time_s=time_s, dt_s=dt_s,
                   reference_deg=reference_deg, actual_deg=actual_deg,
                   requested_current_A=requested_current_A,
                   commanded_current_A=commanded_current_A,
                   encoder_valid=encoder_valid, status=status, joint=joint, joint_sequence=joint_sequence,
                   control_tick=control_tick, reference_time_s=reference_time_s,
                   reference_velocity_deg_s=reference_velocity_deg_s, p_term_A=p_term_A, i_term_A=i_term_A, d_term_A=d_term_A)
        try:
            validate_sample(row)
            if time_s <= self._last_time:
                raise ValueError("nonmonotonic timestamp")
        except (ValueError, TypeError):
            self.rejected += 1
            return False
        self._last_time = time_s
        if not self._lock.acquire(blocking=False):
            self.dropped += 1
            return False
        try:
            self._rows.append(row)
            self._last_publish = time.monotonic()
        finally:
            self._lock.release()
        return True

    def batch(self, after=-1, limit=1000):
        with self._lock:
            rows = list(self._rows)
            last = self._last_publish
        return dict(schema=SCHEMA, session_id=self.metadata["session_id"],
                    samples=[r for r in rows if r["sequence"] > after][:limit],
                    oldest_sequence=rows[0]["sequence"] if rows else None,
                    newest_sequence=rows[-1]["sequence"] if rows else None,
                    producer_age_s=None if last is None else time.monotonic()-last,
                    dropped_publish_count=self.dropped, rejected_count=self.rejected)

    def close(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=2)


    def finish(self):
        """After motor cleanup, allow the laptop a bounded final drain interval."""
        self.metadata["final_sequence"] = self._sequence - 1
        self.metadata["producer_complete"] = True
        time.sleep(2.)
        self.close()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Synthetic two-joint telemetry only; no motor access")
    parser.add_argument("--seconds", type=float, default=30.)
    parser.add_argument("--port", type=int, default=8794)
    args = parser.parse_args()
    server = TrackingTelemetryServer("127.0.0.1", args.port, source="synthetic",
                                    token=os.environ.get("OSL_TELEMETRY_TOKEN"))
    start=time.perf_counter()
    tick=0
    try:
        while time.perf_counter()-start < args.seconds:
            t=time.perf_counter()-start
            for joint in ("knee", "ankle"):
                server.publish(joint=joint,control_tick=tick,reference_time_s=t,time_s=time.perf_counter()-start,
                    dt_s=.005,reference_deg=10*math.sin(t),actual_deg=9*math.sin(t-.05),
                    reference_velocity_deg_s=10*math.cos(t),p_term_A=0.,i_term_A=0.,d_term_A=0.,
                    requested_current_A=0.,commanded_current_A=0.,encoder_valid=True)
            tick+=1
            time.sleep(max(0.,start+tick*.005-time.perf_counter()))
    finally:
        server.finish()
