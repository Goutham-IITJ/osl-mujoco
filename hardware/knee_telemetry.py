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

SCHEMA = "osl-knee-telemetry-v1"
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
    if not isinstance(row.get("status"), str) or len(row["status"]) > 128:
        raise ValueError("status must be a short string")


class KneeTelemetryServer:
    def __init__(self, host="127.0.0.1", port=8794, *, token=None,
                 capacity=4000, source="physical", angle_sign=-1):
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
                encoder_valid, status="running"):
        """Return False for a dropped/rejected tick. Never wait for network/lock.

        Call from exactly one producer thread. Numbers use original controller
        signs; the receiver performs explicit display-only sign conversion.
        """
        sequence = self._sequence
        self._sequence += 1
        row = dict(sequence=sequence, time_s=time_s, dt_s=dt_s,
                   reference_deg=reference_deg, actual_deg=actual_deg,
                   requested_current_A=requested_current_A,
                   commanded_current_A=commanded_current_A,
                   encoder_valid=encoder_valid, status=status)
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


def main():
    parser = argparse.ArgumentParser(description="Synthetic knee telemetry; no hardware access")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8794)
    parser.add_argument("--seconds", type=float, default=60)
    args = parser.parse_args()
    server = KneeTelemetryServer(args.host, args.port,
        token=os.environ.get("OSL_TELEMETRY_TOKEN"), source="synthetic")
    print(f"Synthetic telemetry listening on {args.host}:{server.port}", flush=True)
    start = time.monotonic()
    tick = 0
    try:
        while time.monotonic()-start < args.seconds:
            t = tick / 200
            server.publish(time_s=t, dt_s=.005,
                reference_deg=-30-20*math.sin(2*math.pi*t),
                actual_deg=-30-18*math.sin(2*math.pi*(t-.04)),
                requested_current_A=2*math.sin(2*math.pi*t),
                commanded_current_A=2*math.sin(2*math.pi*t), encoder_valid=True,
                status="synthetic")
            tick += 1
            time.sleep(max(0, start+tick/200-time.monotonic()))
    except KeyboardInterrupt:
        pass
    finally:
        server.close()


if __name__ == "__main__":
    main()
