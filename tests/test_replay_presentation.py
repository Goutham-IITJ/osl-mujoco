"""Replay UI lifecycle and observation isolation; never starts a simulation."""
import json
from pathlib import Path
import subprocess
import sys
from urllib.request import urlopen
from oslbench.telemetry_replay import ReplaySource
from oslbench.replay_presentation import ReplayPresentation, ReplayWebDashboard

FIXTURE=Path(__file__).parent/'fixtures/telemetry_replay/manifest.json'


def make():
    return ReplayPresentation(ReplaySource(FIXTURE))


def test_replay_ui_initial_pause_resume_and_speed():
    p=make(); initial=p.snapshot()
    assert initial['status']=='PAUSED' and initial['delivered']==0
    assert all(c['quality']=='UNAVAILABLE' for c in initial['channels'].values())
    assert p.advance(10)==() and p.snapshot()==initial
    p.key('speed0.5');p.key('space');p.advance(.001)
    assert p.source.time_s==.0005 and p.delivered==7
    p.key('space');paused=p.snapshot();p.advance(10);assert p.snapshot()==paused
    p.key('speed2');p.key('space');p.advance(.001)
    assert p.source.time_s==.0025
    for value in (0.,float('nan'),True,3.):
        try:p.set_speed(value)
        except ValueError:pass
        else:assert False


def test_replay_ui_step_reset_and_eof():
    p=make();first=p.step()
    assert len(first)==3 and p.source.time_s==0 and p.status=='PAUSED'
    assert len(p.step())==4 and p.source.time_s==.0005
    p.key('r');assert p.delivered==0 and p.source.time_s==0
    assert all(not h for h in p.history.values())
    p.key('space');p.advance(10)
    assert p.status=='EOF' and p.source.ended and p.delivered==9
    end=p.snapshot();p.key('space');p.step();p.advance(100);assert p.snapshot()==end
    p.key('r');assert p.status=='PAUSED' and not p.source.ended


def test_replay_ui_mixed_times_missing_invalid_and_retained_age():
    p=make();p.key('space');events=p.advance(.002)
    assert [e.sequence for e in events]==list(range(7))
    s=p.snapshot()['channels']
    assert s['belt_deflection']['time_s']==0 and s['knee_angle']['time_s']==.0005
    assert s['knee_angle']['age_s']==.0015 and s['knee_angle']['held']
    assert s['knee_angle']['value']==.2 and s['knee_angle']['source']=='simulation'
    p.advance(.001)
    s=p.snapshot()['channels']['knee_angle']
    assert s['quality']=='MISSING' and s['value'] is None and s['sequence']==7
    p.advance(.001)
    s=p.snapshot()['channels']['knee_velocity']
    assert s['quality']=='INVALID' and s['points'][-1]==[.004,None]
    assert sum(len(h) for h in p.history.values())==9


def test_replay_ui_frame_rate_and_speed_preserve_records():
    original=ReplaySource(FIXTURE).records
    for speed in (.25,.5,1.,2.):
        p=ReplayPresentation(ReplaySource(FIXTURE),speed);p.key('space')
        delivered=[]
        for _ in range(100):
            delivered.extend(p.advance(.0007))
            if p.status=='EOF':break
        assert tuple(delivered)==original
        assert p.source.time_s==original[-1].time_s


def test_replay_ui_http_controls_publish_same_clock():
    p=make();dashboard=ReplayWebDashboard(p,port=0,open_browser=False)
    def fetch(path):
        with urlopen(dashboard.url+path,timeout=2) as response:return response.read()
    try:
        assert b'OSL telemetry replay' in fetch('')
        for key in ('space','speed2'):
            fetch('key?k='+key);dashboard.pump()
        p.advance(.00025);dashboard.update(p.snapshot())
        data=json.loads(fetch('data'))
        assert data['time_s']==p.source.time_s==.0005 and data['delivered']==7
        fetch('key?k=space');dashboard.pump();old=p.snapshot();p.advance(1);assert p.snapshot()==old
        fetch('key?k=r');dashboard.pump();assert p.source.time_s==0 and p.delivered==0
        fetch('key?k=step');dashboard.pump();assert p.delivered==3
        fetch('key?k=escape');dashboard.pump();assert p.status=='STOPPED'
    finally:dashboard.close()


def test_replay_ui_no_physics_controller_or_command_dependency():
    code="""
import sys,importlib.abc,json
class Block(importlib.abc.MetaPathFinder):
 def find_spec(self,fullname,path=None,target=None):
  if fullname.split('.')[0] in ('mujoco','numpy') or fullname in ('oslbench.controller','oslbench.simulation','oslbench.drivetrain_sim','oslbench.ankle_current_control','oslbench.joint_current_control'):
   raise AssertionError('Forbidden replay dependency '+fullname)
sys.meta_path.insert(0,Block())
import oslbench.contracts as c
def forbidden(*a,**k):raise AssertionError('Actuator command constructed')
c.CurrentCommand=c.PositionCommand=c.TorqueCommand=forbidden
from oslbench.telemetry_replay import ReplaySource
from oslbench.replay_presentation import ReplayPresentation,ReplayWebDashboard
p=ReplayPresentation(ReplaySource(sys.argv[1]));d=ReplayWebDashboard(p,port=0,open_browser=False)
try:
 p.key('space');p.advance(10);d.update(p.snapshot());assert p.status=='EOF'
 assert 'mujoco' not in sys.modules
finally:d.close()
"""
    r=subprocess.run([sys.executable,'-B','-c',code,str(FIXTURE.resolve())],cwd=FIXTURE.parents[3],capture_output=True,text=True)
    assert r.returncode==0,r.stderr


def test_replay_ui_source_files_unchanged_and_snapshot_no_mutation():
    import hashlib
    paths=[FIXTURE,FIXTURE.with_name('samples.csv')]
    before=[hashlib.sha256(p.read_bytes()).hexdigest() for p in paths]
    p=make();p.key('space');p.advance(10)
    snapshot=p.snapshot();snapshot['channels']['knee_angle']['value']=42
    assert p.snapshot()['channels']['knee_angle']['value'] is None
    assert before==[hashlib.sha256(p.read_bytes()).hexdigest() for p in paths]
