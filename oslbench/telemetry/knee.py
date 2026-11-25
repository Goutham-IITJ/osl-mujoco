"""Buffered knee telemetry receiver. Source timestamps are never arrival times."""
import json
import math
from urllib.request import Request, build_opener, ProxyHandler, HTTPRedirectHandler

from hardware.knee_telemetry import SCHEMA, validate_sample


class NoRedirect(HTTPRedirectHandler):
    def redirect_request(self, *args, **kwargs):
        return None


class KneeReceiver:
    def __init__(self, host, port=8794, token=None, timeout=1):
        # A host, not a URL: never send credentials to an injected path/redirect.
        if not host or any(c not in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789.-" for c in host):
            raise ValueError("host must be an IPv4 address or hostname")
        self.url = f"http://{host}:{port}"
        self.token = token
        self.timeout = timeout
        self.opener = build_opener(ProxyHandler({}), NoRedirect())
        self.metadata = self.get("/v1/metadata")
        meta = self.metadata
        if (meta.get("schema") != SCHEMA or not isinstance(meta.get("session_id"), str)
                or meta.get("angle_unit") != "degree" or meta.get("current_unit") != "A"
                or meta.get("flexion_positive_multiplier") not in (-1, 1)):
            raise ValueError("unsupported metadata")
        self.sequence = -1
        self.time_s = -1.0
        self.missing = 0

    def get(self, path):
        headers = {"Authorization": "Bearer " + self.token} if self.token else {}
        with self.opener.open(Request(self.url + path, headers=headers), timeout=self.timeout) as response:
            raw = response.read(2_000_001)
        if len(raw) > 2_000_000:
            raise ValueError("oversized response")
        return json.loads(raw)

    def poll(self):
        batch = self.get(f"/v1/samples?after={self.sequence}&limit=1000")
        if batch.get("schema") != SCHEMA or batch.get("session_id") != self.metadata["session_id"]:
            raise ValueError("hardware session changed; start a new recording")
        rows = batch.get("samples")
        age = batch.get("producer_age_s")
        if not isinstance(rows, list) or len(rows) > 1000:
            raise ValueError("invalid sample batch")
        if age is not None and (type(age) not in (float, int) or not math.isfinite(age) or age < 0):
            raise ValueError("invalid producer age")
        seq, timestamp, missing = self.sequence, self.time_s, 0
        for row in rows:
            validate_sample(row)
            if row["sequence"] <= seq or row["time_s"] <= timestamp:
                raise ValueError("nonmonotonic source samples")
            missing += row["sequence"]-seq-1
            seq, timestamp = row["sequence"], row["time_s"]
        self.sequence, self.time_s = seq, timestamp
        self.missing += missing
        batch["missing_this_batch"] = missing
        return batch
