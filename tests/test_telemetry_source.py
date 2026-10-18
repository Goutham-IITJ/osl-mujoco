"""Hardware-independent source/lifecycle tests; synthetic contracts only."""
from dataclasses import replace
import json
from pathlib import Path
import subprocess
import sys
from urllib.request import urlopen

from oslbench.telemetry_source import MockTelemetrySource,validate_observation
from oslbench.telemetry_presentation import TelemetryPresentation
from oslbench.replay_presentation import ReplayWebDashboard


def reject(call):
    try:call()
    except (ValueError,TypeError):return
    raise AssertionError('Malformed observation accepted')


def test_telemetry_source_deterministic_asynchronous_records():
    a,b=MockTelemetrySource(),MockTelemetrySource()
    first=[]
    for t in (0.,.25,.75,1.,1.25,1.5):first.extend(a.poll(t))
    second=b.poll(2.)
    assert tuple(first)==second and len(first)==6
    assert [r.sequence for r in first]==list(range(6))
    assert first[0].time_s==100. and first[1].time_s==99.9
    assert all(r.source=='synthetic_mock' and r.clock_id=='synthetic_observation_clock' for r in first)
    assert a.status==b.status=='EXHAUSTED' and a.poll(2.)==()
    assert a.metadata['identity']['synthetic']
    m=a.metadata;m['channels'].clear();assert a.metadata['channels']


def test_telemetry_source_missing_invalid_stale_and_unavailable():
    p=TelemetryPresentation(MockTelemetrySource(),stale_after_s=.5)
    assert all(c['quality']=='UNAVAILABLE' for c in p.snapshot()['channels'].values())
    p.update(0.);p.update(.25)
    s=p.snapshot()['channels']['knee_angle']
    assert s['time_s']==100. and s['received_time_s']==0. and s['age_s']==.25
    p.update(.6);s=p.snapshot()['channels']['knee_angle']
    assert s['quality']=='VALID' and s['freshness']=='STALE' and s['value']==.2
    p.update(.75);s=p.snapshot()['channels']['knee_angle']
    assert s['quality']=='MISSING' and s['value'] is None and s['time_s']==100.4
    p.update(1.25);s=p.snapshot()['channels']
    assert s['belt_joint_torque']['quality']=='INVALID'
    assert s['belt_joint_torque']['points'][-1]==[100.2,None]
    assert s['actuator_output_angle']['quality']=='UNAVAILABLE'
    p.update(3.);s=p.snapshot()
    assert s['status']=='DISCONNECTED' and s['delivered']==6
    # Arrival at this poll is fresh despite old independent observation time.
    assert s['channels']['knee_angle']['received_time_s']==3.
    p.update(4.);assert p.snapshot()['channels']['knee_angle']['freshness']=='STALE'


def test_telemetry_source_closure_disconnect_and_clock_errors():
    s=MockTelemetrySource();s.poll(0.);s.disconnect()
    assert s.status=='DISCONNECTED' and s.poll(5.)==()
    s.close();s.close();assert s.status=='CLOSED' and s.poll(6.)==()
    s.disconnect();assert s.status=='CLOSED'
    s=MockTelemetrySource();s.close();assert s.poll(0.)==()
    reject(lambda:s.poll(-1.));reject(lambda:s.poll(float('nan')))
    s=MockTelemetrySource();s.poll(1.);reject(lambda:s.poll(.5))
    p=TelemetryPresentation(MockTelemetrySource());p.update(1.);reject(lambda:p.update(.5))
    reject(lambda:TelemetryPresentation(MockTelemetrySource(),0.))


def test_telemetry_source_rejects_incompatible_envelopes():
    s=MockTelemetrySource();r=s.poll(0.)[0];m=s.metadata
    for bad in (replace(r,unit='degree'),replace(r,frame='rotor'),replace(r,clock_id='wrong'),
                replace(r,source='measured'),replace(r,sequence=-1),replace(r,time_s=101.)):
        reject(lambda:validate_observation(bad,m))
    reject(lambda:validate_observation(3.,m))
    class Duplicate:
        metadata=m
        status='OPEN'
        def poll(self,now):return (r,r)
        def close(self):pass
    p=TelemetryPresentation(Duplicate());reject(lambda:p.update(0.))
    assert p.delivered==0 and not p.latest


def test_telemetry_source_dashboard_and_local_receipt_clock():
    p=TelemetryPresentation(MockTelemetrySource());d=ReplayWebDashboard(p,port=0,open_browser=False)
    try:
        p.update(.25);d.update(p.snapshot())
        with urlopen(d.url+'data',timeout=2) as r:body=json.load(r)
        assert body['mode']=='mock' and 'SYNTHETIC MOCK' in body['label']
        assert body['time_s']==.25 and body['channels']['knee_angle']['time_s']==100.
        assert body['channels']['knee_angle']['received_time_s']==.25
        with urlopen(d.url+'key?k=escape',timeout=2) as r:r.read()
        d.pump();assert p.source.status=='CLOSED'
    finally:d.close()


def test_telemetry_source_replay_unchanged_and_no_physics_import():
    code="""
import sys,importlib.abc
class Block(importlib.abc.MetaPathFinder):
 def find_spec(self,fullname,path=None,target=None):
  if fullname.split('.')[0] in ('mujoco','numpy') or fullname in ('oslbench.controller','oslbench.simulation','oslbench.drivetrain_sim','oslbench.ankle_current_control','oslbench.joint_current_control'):
   raise AssertionError('Forbidden dependency '+fullname)
sys.meta_path.insert(0,Block())
import oslbench.contracts as c
def forbidden(*a,**k):raise AssertionError('Command creation')
c.CurrentCommand=c.PositionCommand=c.TorqueCommand=forbidden
from oslbench.telemetry_source import MockTelemetrySource
from oslbench.telemetry_presentation import TelemetryPresentation
from oslbench.replay_presentation import ReplayPresentation,ReplayWebDashboard
from oslbench.telemetry_replay import ReplaySource
p=TelemetryPresentation(MockTelemetrySource());d=ReplayWebDashboard(p,port=0,open_browser=False)
try:
 for t in (0.,.25,.75,1.,1.25,1.5,3.):p.update(t);d.update(p.snapshot())
 assert p.source.status=='DISCONNECTED'
 p.source.close()
finally:d.close()
r=ReplaySource('tests/fixtures/telemetry_replay/manifest.json');expected=r.records
v=ReplayPresentation(r);v.key('space');assert v.advance(1.)==expected and v.status=='EOF'
assert 'mujoco' not in sys.modules
"""
    result=subprocess.run([sys.executable,'-B','-c',code],cwd=Path(__file__).resolve().parents[1],text=True,capture_output=True)
    assert result.returncode==0,result.stderr


def test_telemetry_source_neutral_schema_compatibility():
    from dataclasses import FrozenInstanceError
    import pickle
    from oslbench import telemetry_schema as schema, telemetry_replay as replay
    from oslbench import observation_presentation as formatting, replay_presentation as ui
    assert replay.ReplayRecord is schema.ReplayRecord
    assert replay.CHANNELS is schema.CHANNELS
    assert ui.observation_channels is formatting.observation_channels
    assert ui.MAX_PLOT_PTS == formatting.MAX_PLOT_PTS == 320
    record = MockTelemetrySource().poll(0.)[0]
    assert type(record) is schema.ReplayRecord
    assert pickle.loads(pickle.dumps(record)) == record
    assert replace(record, sample=None).sample is None
    try:
        record.sequence = 10
    except FrozenInstanceError:
        pass
    else:
        raise AssertionError('Observation envelope became mutable')
    try:
        schema.CHANNELS['new'] = ('angle', 'rad', 'knee_joint')
    except TypeError:
        pass
    else:
        raise AssertionError('Channel registry became mutable')


def test_telemetry_source_neutral_imports_and_formatting():
    code = """
import sys, importlib.abc
class Block(importlib.abc.MetaPathFinder):
 def find_spec(self, fullname, path=None, target=None):
  if fullname.split('.')[0] in ('mujoco', 'numpy') or fullname in (
   'oslbench.telemetry_replay', 'oslbench.replay_presentation',
   'oslbench.drivetrain_dashboard', 'oslbench.controller', 'oslbench.simulation'):
   raise AssertionError('Forbidden dependency ' + fullname)
sys.meta_path.insert(0, Block())
from dataclasses import replace
from oslbench.telemetry_schema import ReplayRecord, CHANNELS
from oslbench.observation_presentation import observation_channels
from oslbench.telemetry_source import MockTelemetrySource
from oslbench.telemetry_presentation import TelemetryPresentation
p = TelemetryPresentation(MockTelemetrySource())
assert all(c['quality'] == 'UNAVAILABLE' for c in p.snapshot()['channels'].values())
p.update(0.)
p.update(.5)
c = p.snapshot()['channels']['knee_angle']
assert c == dict(unit='rad', frame='knee_joint', stage='post_step',
 sign=p.metadata['channels']['knee_angle']['sign'], source='synthetic_mock',
 quality='VALID', value=.2, time_s=100., age_s=.5, held=True,
 sequence=0, snapshot_id=0, points=[[100., .2]], received_time_s=0., freshness='RECENT')
p.update(.6)
assert p.snapshot()['channels']['knee_angle']['freshness'] == 'STALE'
p.update(.75)
assert p.snapshot()['channels']['knee_angle']['quality'] == 'MISSING'
p.update(1.25)
assert p.snapshot()['channels']['belt_joint_torque']['quality'] == 'INVALID'
# Exercise the pure formatter's replay-age and decimation branches without replay.
s = MockTelemetrySource(); r = s.poll(0.)[0]; name = r.channel
m = {'channels': {name: s.metadata['channels'][name]}}
rows = [replace(r, sequence=i, time_s=float(i)) for i in range(400)]
c = observation_channels(m, {name: rows[-1]}, {name: rows}, 400.)[name]
assert c['age_s'] == 1. and c['held'] and c['time_s'] == 399.
assert c['points'] == [[float(i*399//319), .2] for i in range(320)]
assert 'received_time_s' not in c and 'freshness' not in c
"""
    result = subprocess.run([sys.executable, '-B', '-c', code],
                            cwd=Path(__file__).resolve().parents[1],
                            text=True, capture_output=True)
    assert result.returncode == 0, result.stderr
