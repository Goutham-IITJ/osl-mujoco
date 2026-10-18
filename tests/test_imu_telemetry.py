"""No-device IMU pipeline tests: fake decoded readers and loopback HTTP only."""
import copy
from contextlib import contextmanager
import json
from pathlib import Path
import queue
import socket
import subprocess
import sys
import threading
import time
from urllib.request import Request, urlopen
from urllib.error import HTTPError

from oslbench.imu_telemetry import encode, decode, NAMES
from oslbench.imu_service import Acquisition, make_server, check_bind
from oslbench.http_telemetry_source import HttpTelemetrySource, HttpTelemetryPresentation
from oslbench.microstrain_reader import MsclReader


def rejects(call):
    try:
        call()
    except (ValueError, TypeError, RuntimeError):
        return
    raise AssertionError('Invalid input accepted')


def wait_for(predicate, timeout=3.):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(.01)
    raise AssertionError('Timed out waiting for test worker')


class Reader:
    def __init__(self):
        self.queue = queue.Queue()
        self.closed = False
        self.calls = 0

    def read(self, timeout_ms):
        self.calls += 1
        try:
            item = self.queue.get(timeout=timeout_ms / 1000.)
        except queue.Empty:
            return []
        if isinstance(item, Exception):
            raise item
        return [item]

    def close(self):
        self.closed = True


def packet(value=.2, reference=1000000000):
    return dict(values={name: dict(value=value if i < 3 else (1. if i == 3 else 0.),
                                  valid=True, reason='') for i, name in enumerate(NAMES)},
                device_timestamps={'reference_ns': dict(value=reference, valid=True)})


@contextmanager
def service(stale_after=.5, token=None):
    reader = Reader()
    acquisition = Acquisition(reader, 'SYNTHETIC test fixture; no mounted sensor',
                              provenance='synthetic_mock', stale_after=stale_after)
    server = make_server(acquisition, port=0, token=token)
    worker = threading.Thread(target=server.serve_forever, kwargs={'poll_interval': .02}, daemon=True)
    acquisition.start()
    worker.start()
    try:
        yield reader, acquisition, server
    finally:
        server.shutdown()
        server.server_close()
        worker.join(1.)
        acquisition.close()
        assert not worker.is_alive() and reader.closed


def test_imu_reader_sdk_decoding_and_validity():
    class Vector:
        def size(self): return 4
        def as_floatAt(self, i): return (1., .1, .2, .3)[i]
    class Point:
        def __init__(self, field, qualifier, value=.2, valid=True):
            self.f, self.q, self.v, self.ok = field, qualifier, value, valid
        def field(self): return self.f
        def qualifier(self): return self.q
        def as_float(self): return self.v
        def as_Vector(self): return Vector()
        def hasValidFlag(self): return True
        def valid(self): return self.ok
    class Shared:
        def hasReferenceTime(self): return True
        def referenceTime(self): return 1234567890
        def hasGpsTimestamp(self): return False
        def hasExternalTimestamp(self): return False
    class Packet:
        def data(self): return [Point(0x8205, 6), Point(0x8205, 7, .3, False),
                               Point(0x8205, 8, float('nan')), Point(0x8203, 5),
                               Point(0x800C, 6, 999.)]
        def shared(self): return Shared()
        def hasDeviceTime(self): return False
    decoded = MsclReader.decode_packet(Packet())
    assert decoded['values']['imu_roll']['value'] == .2  # sensor-class field ignored
    assert decoded['values']['imu_pitch'] == dict(value=.3, valid=False, reason='device_invalid_or_validity_flag_absent')
    assert decoded['values']['imu_yaw']['value'] is None and not decoded['values']['imu_yaw']['valid']
    assert [decoded['values']['imu_q'+str(i)]['value'] for i in range(4)] == [1., .1, .2, .3]
    assert decoded['device_timestamps']['reference_ns']['value'] == 1234567890
    if sys.platform != 'linux':
        rejects(lambda: MsclReader('/dev/nonexistent'))


def test_imu_payload_roundtrip_and_strict_metadata():
    a = Acquisition(Reader(), 'synthetic fixture', provenance='synthetic_mock')
    a._publish(packet())
    payload = a.snapshot()
    assert decode(encode(payload)) == payload
    changes = [lambda p: p['metadata']['channels']['imu_roll'].update(unit='deg'),
               lambda p: p['metadata']['channels']['imu_q0'].update(frame='knee_joint'),
               lambda p: p['samples']['imu_roll'].update(value=float('nan')),
               lambda p: p['samples']['imu_roll'].update(value=None, valid=True),
               lambda p: p['samples']['imu_roll'].update(sequence=True),
               lambda p: p.update(schema='other')]
    for change in changes:
        bad = copy.deepcopy(payload)
        change(bad)
        rejects(lambda: decode(json.dumps(bad).encode()))
    rejects(lambda: decode(b'{"schema":1,"schema":2}'))
    rejects(lambda: decode(b' ' * 65537))
    a.close()


def test_imu_missing_invalid_and_no_zero_fill():
    a = Acquisition(Reader(), 'synthetic', provenance='synthetic_mock')
    p = packet()
    p['values'] = {'imu_roll': dict(value=None, valid=False, reason='missing'),
                   'imu_pitch': dict(value=.7, valid=False, reason='device_invalid')}
    p['device_timestamps'] = {}
    a._publish(p)
    payload = decode(encode(a.snapshot()))
    assert set(payload['samples']) == {'imu_roll', 'imu_pitch'}
    assert payload['samples']['imu_roll']['value'] is None
    assert payload['samples']['imu_pitch']['value'] == .7
    assert not payload['samples']['imu_pitch']['valid']
    assert payload['samples']['imu_roll']['device_timestamps'] == {}
    a.close()


def test_imu_end_to_end_repeated_http_is_not_fresh_data():
    with service(stale_after=.08) as (r, a, server):
        source = HttpTelemetrySource('127.0.0.1', server.server_port, interval=.01, timeout=.2)
        try:
            wait_for(lambda: source.ready)
            p = HttpTelemetryPresentation(source)
            assert all(c['quality'] == 'UNAVAILABLE' for c in p.snapshot()['channels'].values())
            r.queue.put(packet())
            wait_for(lambda: source.diagnostics.get('channels', {}).get('imu_roll'))
            records = p.update(0.)
            assert len(records) == 7 and p.delivered == 7
            assert records[0].clock_id.endswith(':pi_receipt')
            wait_for(lambda: source.status == 'STALE')
            assert p.update(.2) == () and p.delivered == 7
            snap = p.snapshot()
            assert snap['channels']['imu_roll']['freshness'] == 'STALE'
            assert snap['channels']['imu_roll']['held']
            assert snap['identity']['transport']['channels']['imu_roll']['device_timestamps']['reference_ns']['value'] == 1000000000
            assert snap['channels']['imu_roll']['source'] == 'synthetic_mock'
        finally:
            source.close()
        assert source.poll(1.) == () and source.status == 'CLOSED'


def test_imu_latest_delivery_gaps_and_nonblocking_poll():
    with service() as (r, a, server):
        for i in range(5):
            r.queue.put(packet(float(i), 1000000000 + i))
        wait_for(lambda: a.snapshot()['samples'].get('imu_roll', {}).get('sequence') == 4)
        source = HttpTelemetrySource('127.0.0.1', server.server_port, interval=.02, timeout=.2)
        try:
            wait_for(lambda: source.ready)
            started = time.monotonic()
            records = source.poll(0.)
            assert time.monotonic() - started < .1
            assert len(records) == 7 and records[0].snapshot_id == 4 and records[0].sequence == 28
            assert records[0].sample.value == 4.
        finally:
            source.close()


def test_imu_existing_browser_dashboard_and_disconnect():
    from oslbench.replay_presentation import ReplayWebDashboard
    with service() as (r, a, server):
        r.queue.put(packet())
        source = HttpTelemetrySource('127.0.0.1', server.server_port, interval=.01, timeout=.2)
        dashboard = None
        try:
            wait_for(lambda: bool(source.diagnostics.get('channels')))
            presentation = HttpTelemetryPresentation(source)
            presentation.update(0.)
            dashboard = ReplayWebDashboard(presentation, port=0, open_browser=False)
            with urlopen(dashboard.url + 'data', timeout=1) as response:
                body = json.load(response)
            assert body['channels']['imu_q0']['unit'] == '1'
            assert body['channels']['imu_roll']['value'] == .2
            assert 'SYNTHETIC' in body['label']
            r.queue.put(OSError('device unplugged'))
            wait_for(lambda: source.status == 'DISCONNECTED')
            presentation.update(.3)
            assert presentation.snapshot()['channels']['imu_roll']['value'] == .2
            assert 'unplugged' in source.diagnostics['error']
        finally:
            if dashboard:
                dashboard.close()
            source.close()


def test_imu_acquisition_failure_and_device_clock_reset():
    with service() as (r, a, server):
        r.queue.put(packet(reference=100))
        wait_for(lambda: bool(a.snapshot()['samples']))
        r.queue.put(packet(reference=99))
        wait_for(lambda: a.snapshot()['status'] == 'DISCONNECTED')
        assert 'clock reset' in a.snapshot()['error']
        assert a.snapshot()['samples']['imu_roll']['device_timestamps']['reference_ns']['value'] == 100
    with service() as (r, a, server):
        r.queue.put(OSError('USB disconnected'))
        wait_for(lambda: a.snapshot()['status'] == 'DISCONNECTED')
        assert 'USB disconnected' in a.snapshot()['error']


def test_imu_concurrent_snapshots_and_single_worker():
    with service() as (r, a, server):
        rejects(a.start)
        for i in range(60):
            r.queue.put(packet(reference=1000+i))
        for _ in range(100):
            payload = decode(encode(a.snapshot()))
            if payload['samples']:
                assert len({v['sequence'] for v in payload['samples'].values()}) == 1
        wait_for(lambda: a.snapshot()['samples'].get('imu_roll', {}).get('sequence') == 59)


def test_imu_http_read_only_auth_and_health():
    rejects(lambda: check_bind('0.0.0.0', 'x'*24))
    rejects(lambda: check_bind('8.8.8.8', 'x'*24))
    rejects(lambda: check_bind('192.168.1.2', None))
    check_bind('192.168.1.2', 'x'*24)
    with service(token='x'*24) as (r, a, server):
        url = f'http://127.0.0.1:{server.server_port}'
        try:
            urlopen(url + '/health', timeout=1)
        except HTTPError as exc:
            assert exc.code == 401
        else:
            raise AssertionError('Unauthenticated request accepted')
        with urlopen(Request(url+'/health', headers={'Authorization':'Bearer '+'x'*24}), timeout=1) as response:
            assert json.load(response)['status'] == 'WAITING'
        try:
            urlopen(Request(url+'/v1/latest', method='POST'), timeout=1)
        except HTTPError as exc:
            assert exc.code == 501
        else:
            raise AssertionError('Write method accepted')


def test_imu_transport_timeout_malformed_and_recovery():
    with service() as (r, a, server):
        original = a.snapshot
        mode = ['malformed']
        def snapshot():
            if mode[0] == 'slow':
                time.sleep(.25)
            payload = original()
            if mode[0] == 'malformed':
                # Health/status vocabulary invalid but JSON itself remains valid.
                payload['error'] = 12
            return payload
        # A separate tiny HTTP server can deliberately emit invalid JSON/schema.
        from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                body = json.dumps(snapshot()).encode()
                try:
                    self.send_response(200)
                    self.send_header('Content-Type','application/json')
                    self.send_header('Content-Length',str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                except OSError:
                    pass
            def log_message(self,*args): pass
        bad = ThreadingHTTPServer(('127.0.0.1',0), Handler)
        worker = threading.Thread(target=bad.serve_forever, kwargs={'poll_interval':.02}, daemon=True)
        worker.start()
        source = HttpTelemetrySource('127.0.0.1', bad.server_port, interval=.01, timeout=.05)
        try:
            wait_for(lambda: source.status == 'DISCONNECTED')
            assert not source.ready
            mode[0] = 'slow'
            wait_for(lambda: 'timed out' in source.diagnostics.get('error','').lower())
            mode[0] = 'good'
            r.queue.put(packet())
            wait_for(lambda: source.ready and source.status == 'OPEN')
            assert len(source.poll(0.)) == 7
            # Same endpoint with a new session must never silently join clocks.
            a.session = 'new_session'
            from oslbench.imu_telemetry import metadata
            a.metadata = metadata(a.session,'synthetic_mock','test')
            wait_for(lambda: source.status == 'SESSION_CHANGED')
            assert source.poll(1.) == ()
        finally:
            source.close()
            bad.shutdown()
            bad.server_close()
            worker.join(1.)


def test_imu_unavailable_server_and_close():
    with socket.socket() as sock:
        sock.bind(('127.0.0.1',0))
        port = sock.getsockname()[1]
    source = HttpTelemetrySource('127.0.0.1', port, interval=.01, timeout=.05)
    try:
        wait_for(lambda: source.status == 'DISCONNECTED')
        assert not source.ready and source.poll(0.) == ()
    finally:
        source.close()
    source.close()
    assert source.status == 'CLOSED' and not source._thread.is_alive()


def test_imu_imports_no_physics_or_device_library_and_replay_schema_frozen():
    code = '''
import sys, importlib.abc
class Block(importlib.abc.MetaPathFinder):
 def find_spec(self, fullname, path=None, target=None):
  if fullname.split('.')[0] in ('mujoco','numpy','mscl','MSCL') or fullname in ('oslbench.controller','oslbench.simulation','oslbench.joint_current_control','oslbench.drivetrain_sim'):
   raise AssertionError('Forbidden dependency: '+fullname)
sys.meta_path.insert(0,Block())
import oslbench.imu_service, oslbench.microstrain_reader, oslbench.http_telemetry_source
import oslbench.contracts as contracts
def forbidden(*args, **kwargs): raise AssertionError('Actuator command construction')
contracts.PositionCommand = contracts.TorqueCommand = contracts.CurrentCommand = forbidden
from oslbench.telemetry_replay import CHANNELS, ReplaySource
from oslbench.telemetry_schema import IMU_CHANNELS
assert len(CHANNELS)==7 and not set(CHANNELS).intersection(IMU_CHANNELS)
assert ReplaySource('tests/fixtures/telemetry_replay/manifest.json').records
sys.path.insert(0,'tests')
from test_imu_telemetry import test_imu_end_to_end_repeated_http_is_not_fresh_data
test_imu_end_to_end_repeated_http_is_not_fresh_data()
'''
    result = subprocess.run([sys.executable,'-B','-c',code], cwd=Path(__file__).resolve().parents[1],
                            text=True,capture_output=True)
    assert result.returncode == 0, result.stderr
