"""Bounded background HTTP ingestion implementing the existing TelemetrySource."""
import copy
import ipaddress
import threading
import time
from urllib.request import build_opener, ProxyHandler, HTTPRedirectHandler, Request

from .contracts import Sample
from .imu_telemetry import decode, NAMES, MAX_BODY, number
from .telemetry_schema import ReplayRecord
from .telemetry_source import local_time
from .telemetry_presentation import TelemetryPresentation


class NoRedirect(HTTPRedirectHandler):
    def redirect_request(self, *args, **kwargs):
        raise ValueError('Telemetry redirects are refused')


class HttpTelemetrySource:
    def __init__(self, host, port=8793, *, interval=.05, timeout=1., token=None):
        address = ipaddress.ip_address(host)  # no DNS/proxy ambiguity or credential-bearing URLs
        if address.version != 4:
            raise ValueError('Use an IPv4 address or an SSH loopback tunnel')
        if type(port) is not int or not 1 <= port <= 65535:
            raise ValueError('Invalid port')
        for value, name in ((interval, 'poll interval'), (timeout, 'timeout')):
            if not 0 < number(value, name) <= 60:
                raise ValueError(name + ' must be in (0, 60] seconds')
        self.url = f'http://{address}:{port}/v1/latest'
        self.interval, self.timeout, self.token = interval, timeout, token
        self._lock, self._stop = threading.Lock(), threading.Event()
        self._metadata = None
        self._status, self._error = 'CONNECTING', ''
        self._session, self._server_time = None, None
        self._last = {}
        self._pending = {}
        self._details = {}
        self._now = 0.
        self._thread = threading.Thread(target=self._run, name='osl-imu-http', daemon=True)
        self._thread.start()

    @property
    def metadata(self):
        with self._lock:
            if self._metadata is None:
                raise RuntimeError('No verified metadata received yet')
            return copy.deepcopy(self._metadata)

    @property
    def ready(self):
        with self._lock:
            return self._metadata is not None

    @property
    def status(self):
        with self._lock:
            return self._status

    @property
    def diagnostics(self):
        with self._lock:
            details = copy.deepcopy(self._details)
            if details:
                elapsed = time.monotonic() - details.pop('_received_monotonic')
                details['seconds_since_http_receipt'] = elapsed
                for item in details['channels'].values():
                    item['pi_receipt_age_s'] += elapsed
            details['error'] = self._error
            return details

    def _fetch(self, opener):
        headers = {'Accept': 'application/json'}
        if self.token:
            headers['Authorization'] = 'Bearer ' + self.token
        started = time.monotonic()
        with opener.open(Request(self.url, headers=headers, method='GET'), timeout=self.timeout) as response:
            if response.status != 200 or response.headers.get_content_type() != 'application/json':
                raise ValueError('Expected HTTP 200 application/json')
            parts, size = [], 0
            while True:
                if self._stop.is_set() or time.monotonic() - started > self.timeout:
                    raise TimeoutError('HTTP response deadline exceeded')
                chunk = response.read1(min(4096, MAX_BODY + 1 - size))
                if not chunk:
                    break
                parts.append(chunk)
                size += len(chunk)
                if size > MAX_BODY:
                    raise ValueError('HTTP payload too large')
        return decode(b''.join(parts)), time.monotonic() - started

    def _accept(self, payload, transit):
        now = time.monotonic()
        with self._lock:
            if self._session is not None and payload['session_id'] != self._session:
                self._status, self._error = 'SESSION_CHANGED', 'Pi restarted; launch a new client session'
                self._pending.clear()
                self._stop.set()
                return
            if self._metadata is not None and payload['metadata'] != self._metadata:
                raise ValueError('Session metadata changed')
            if self._server_time is not None and payload['server_time_s'] < self._server_time:
                raise ValueError('Pi receipt clock reversed')
            if set(self._last) - set(payload['samples']):
                raise ValueError('Latest-channel cache lost observations within a session')
            pending, last = {}, dict(self._last)
            m = payload['metadata']
            for index, name in enumerate(NAMES):
                if name not in payload['samples']:
                    continue
                item = payload['samples'][name]
                previous = last.get(name)
                if previous:
                    if item['sequence'] < previous['sequence'] or item['time_s'] < previous['time_s']:
                        raise ValueError('Per-channel sequence or receipt time reversed')
                    if item['sequence'] == previous['sequence']:
                        if item != previous:
                            raise ValueError('Observation mutated without a new sequence')
                        continue
                channel = m['channels'][name]
                sample = None if item['value'] is None else Sample(item['value'], channel['unit'],
                    channel['frame'], item['time_s'], 'acquisition', m['source'], item['valid'])
                record = ReplayRecord(item['sequence'] * len(NAMES) + index, item['sequence'], name,
                    item['time_s'], m['clock']['id'], sample, m['source'], channel['unit'],
                    channel['frame'], 'acquisition')
                pending[name] = record
                last[name] = copy.deepcopy(item)
            self._session, self._metadata = payload['session_id'], copy.deepcopy(m)
            self._server_time = payload['server_time_s']
            self._last = last
            self._pending.update(pending)  # bounded latest-per-channel handoff
            self._status, self._error = payload['status'], payload['error']
            self._details = dict(pi_status=payload['status'], stale_after_s=payload['stale_after_s'],
                _received_monotonic=now, http_round_trip_s=transit,
                clock_note='Pi host receipt age plus HTTP round trip: conservative host-age estimate, not sensor age',
                channels={name: dict(device_timestamps=copy.deepcopy(item['device_timestamps']),
                    pi_received_time_s=item['time_s'], pi_receipt_age_s=payload['server_time_s']-item['time_s']+transit,
                    packet_sequence=item['sequence'], reason=item['reason'])
                    for name, item in payload['samples'].items()})

    def _run(self):
        opener = build_opener(ProxyHandler({}), NoRedirect())
        while not self._stop.is_set():
            try:
                payload, transit = self._fetch(opener)
                if not self._stop.is_set():
                    self._accept(payload, transit)
            except Exception as exc:
                with self._lock:
                    if not self._stop.is_set():
                        self._status, self._error = 'DISCONNECTED', type(exc).__name__ + ': ' + str(exc)
            self._stop.wait(self.interval)

    def poll(self, local_time_s):
        now = local_time(local_time_s)
        if now < self._now:
            raise ValueError('Local receipt clock cannot reverse')
        self._now = now
        with self._lock:
            if self._status in ('CLOSED', 'SESSION_CHANGED'):
                return ()
            result = tuple(sorted(self._pending.values(), key=lambda r: r.sequence))
            self._pending.clear()
            return result

    def close(self):
        self._stop.set()
        self._thread.join(2 * self.timeout + 1.)
        if self._thread.is_alive():
            raise RuntimeError('HTTP worker did not stop within its bounded deadline')
        with self._lock:
            self._status = 'CLOSED'
            self._pending.clear()


class HttpTelemetryPresentation(TelemetryPresentation):
    """Existing presentation plus transport diagnostics; no second dashboard."""
    def snapshot(self):
        result = super().snapshot()
        details = self.source.diagnostics
        result['identity']['transport'] = details
        # Never call old producer data fresh merely because HTTP succeeded.
        for name, item in details.get('channels', {}).items():
            if item['pi_receipt_age_s'] > details['stale_after_s']:
                result['channels'][name]['freshness'] = 'STALE'
        return result
