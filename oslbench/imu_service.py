"""One acquisition worker and a read-only, latest-channel HTTP service."""
import copy
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import ipaddress
import json
import secrets
import threading
import time
import uuid

from .imu_telemetry import VERSION, NAMES, metadata, number, observation, encode


class Acquisition:
    def __init__(self, reader, mounting, *, provenance='measured', stale_after=0.5, clock=time.monotonic):
        if number(stale_after, 'stale threshold') <= 0:
            raise ValueError('Stale threshold must be positive')
        self.reader, self.clock, self.stale_after = reader, clock, stale_after
        self.session = uuid.uuid4().hex
        self.metadata = metadata(self.session, provenance, mounting)
        self._start = clock()
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._samples = {}
        self._sequence = 0
        self._status, self._error = 'WAITING', ''
        self._reference_ns = None
        self._thread = None

    def start(self):
        if self._thread is not None or self._stop.is_set():
            raise RuntimeError('Acquisition can only start once')
        self._thread = threading.Thread(target=self._run, name='osl-imu-reader', daemon=True)
        self._thread.start()

    def _publish(self, packet):
        now = self.clock() - self._start
        if not isinstance(packet, dict) or set(packet) != {'values', 'device_timestamps'}:
            raise ValueError('Malformed reader packet')
        values = packet['values']
        if not isinstance(values, dict) or not values or set(values) - set(NAMES):
            raise ValueError('Unknown or empty reader channel set')
        pending = {}
        for name, value in values.items():
            if not isinstance(value, dict) or set(value) != {'value', 'valid', 'reason'}:
                raise ValueError('Malformed reader value')
            item = dict(sequence=self._sequence, time_s=now, **value,
                        device_timestamps=copy.deepcopy(packet['device_timestamps']))
            observation(item)
            pending[name] = item
        reference = packet['device_timestamps'].get('reference_ns')
        if reference and reference['valid']:
            ns = reference['value']
            if self._reference_ns is not None and ns < self._reference_ns:
                raise ValueError('Device reference clock reset; restart service for a new session')
            self._reference_ns = ns
        with self._lock:
            self._samples.update(pending)
            self._sequence += 1
            self._status, self._error = 'OPEN', ''

    def _run(self):
        try:
            while not self._stop.is_set():
                for packet in self.reader.read(100):
                    if self._stop.is_set():
                        break
                    self._publish(packet)
                self._stop.wait(.001)
        except Exception as exc:
            with self._lock:
                self._status, self._error = 'DISCONNECTED', str(exc)
        finally:
            try:
                self.reader.close()
            except Exception as exc:
                with self._lock:
                    self._status, self._error = 'DISCONNECTED', 'Reader close failed: ' + str(exc)

    def snapshot(self):
        with self._lock:
            samples = copy.deepcopy(self._samples)
            now = self.clock() - self._start
            state = self._status
            if state == 'OPEN' and all(now - s['time_s'] > self.stale_after for s in samples.values()):
                state = 'STALE'
            return dict(schema=VERSION, session_id=self.session, metadata=copy.deepcopy(self.metadata),
                        status=state, error=self._error, server_time_s=now,
                        stale_after_s=self.stale_after, samples=samples)

    def close(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(2.)
            if self._thread.is_alive():
                raise RuntimeError('Reader did not honor its bounded read timeout')
        else:
            self.reader.close()
        with self._lock:
            self._status = 'CLOSED'


def check_bind(host, token):
    address = ipaddress.ip_address(host)
    private_lan = any(address in net for net in (
        ipaddress.ip_network('10.0.0.0/8'), ipaddress.ip_network('172.16.0.0/12'),
        ipaddress.ip_network('192.168.0.0/16'))) if address.version == 4 else False
    if address.version != 4 or (not address.is_loopback and not private_lan):
        raise ValueError('Bind to explicit IPv4 loopback or RFC1918 LAN address; wildcard/public binding refused')
    if not address.is_loopback and (not token or len(token) < 24):
        raise ValueError('LAN binding requires a bearer token of at least 24 characters')


def make_server(acquisition, host='127.0.0.1', port=8793, token=None):
    check_bind(host, token)
    class Handler(BaseHTTPRequestHandler):
        def setup(self):
            super().setup()
            self.connection.settimeout(2.)

        def do_GET(self):
            if token and not secrets.compare_digest(self.headers.get('Authorization', ''), 'Bearer ' + token):
                self.send_error(401)
                return
            if self.path not in ('/v1/latest', '/health'):
                self.send_error(404)
                return
            snapshot = acquisition.snapshot()
            body = encode(snapshot) if self.path == '/v1/latest' else json.dumps({
                k: snapshot[k] for k in ('schema', 'session_id', 'status', 'error', 'server_time_s', 'stale_after_s')}).encode()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError, TimeoutError):
                pass

        def log_message(self, *args):
            pass  # Never log authorization headers or tokens.
    server = ThreadingHTTPServer((host, port), Handler)
    server.daemon_threads = True
    return server
