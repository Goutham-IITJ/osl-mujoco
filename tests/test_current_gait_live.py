"""Selection, presentation wiring and lifecycle without a second gait simulation."""
from types import SimpleNamespace
import json
import numpy as np
from oslbench.joint_current_control import make_gait_source,GaitCurrentSource
from oslbench.feedback_location_diagnostic import ActuatorDifferenceSource
from oslbench.current_gait_live import LiveState,snapshot
from test_gait_feedback_location_ab import setup_source


def test_current_live_selection_default_and_C():
    old,layer=setup_source('B')
    args=(old.cfg,old.audit,old.ref,2,layer)
    assert type(make_gait_source(*args)) is GaitCurrentSource
    c=make_gait_source(*args,mode='actuator-difference')
    assert type(c) is ActuatorDifferenceSource and c.layer is layer
    try:make_gait_source(*args,mode='bad')
    except ValueError:pass
    else:assert False


def test_current_live_lifecycle():
    s=LiveState(True);assert s.status=='paused'
    s.key('space');assert s.status=='active'
    s.key('space');assert s.status=='paused'
    s.status='complete';s.key('space');assert s.status=='complete'
    s.key('r');assert s.restart
    s.key('escape');assert s.status=='stopped'
    s.key('space');assert s.status=='stopped'


def test_current_live_initial_snapshot_is_finite_and_identified():
    old,layer=setup_source('B')
    ctx=dict(dt=.0005,bench=SimpleNamespace(data=SimpleNamespace(qpos=[.2]),knee_qpos=0),
        sim=SimpleNamespace(layer=layer),res={'ref_rad':[.2]})
    args=SimpleNamespace(feedback_mode='actuator-difference',kp=.75,kd=.03,ki=0)
    from oslbench.drivetrain_dashboard import SCALARS
    s=snapshot([],ctx,'paused',args)
    assert all(np.isfinite(s[k]) for k in SCALARS)
    assert 'actuator-difference' in s['status'] and 'PAUSED' in s['status']


def test_current_live_empty_attempt_saves_status_without_steps():
    import tempfile
    from pathlib import Path
    from unittest.mock import patch
    from oslbench.current_gait_live import save_attempt
    from oslbench.drivetrain_dashboard import NoDashboard
    from experiments import run_current_controlled_gait as gait
    ctx=dict(cfg=SimpleNamespace(current_loop_tau_s=0),src=SimpleNamespace(updates=0),dt=.0005)
    args=SimpleNamespace(feedback_mode='actuator-difference')
    with tempfile.TemporaryDirectory() as folder, patch.object(gait,'metadata',return_value={}):
        p=save_attempt(folder,1,[],ctx,args,'stopped',NoDashboard())
        md=json.loads((p/'run_metadata.json').read_text())
        assert md['recorded_physics_steps']==0 and md['run_status']=='stopped'
        try:save_attempt(folder,1,[],ctx,args,'stopped',NoDashboard())
        except FileExistsError:pass
        else:assert False, 'must refuse to overwrite an attempt'


def test_current_live_snapshot_uses_logged_current_and_torques():
    from experiments import run_current_controlled_gait as g
    row=np.arange(len(g.CSV_HEADER),dtype=float)
    ctx=dict(dt=.0005,audit=SimpleNamespace(belt_ratio=50/11))
    args=SimpleNamespace(feedback_mode='actuator-difference',kp=.75,kd=.03,ki=0)
    s=snapshot([row],ctx,'paused',args)
    get=lambda name:row[g.CSV_HEADER.index(name)]
    assert s['y_actual'][0]==get('current_actual_modelled_A')
    assert s['y_applied'][0]==get('current_applied_A')
    assert s['y_em'][0]==get('estimated_actuator_output_torque_Nm')
    assert s['y_belt_actuator'][0]==get('belt_torque_actuator_side_Nm')
    assert 'PAUSED' in s['status'] and 'actuator-difference' in s['status']


def test_current_live_dashboard_extended_snapshot_and_keys():
    from oslbench.drivetrain_dashboard import DrivetrainWebDashboard,SCALARS,TRACES
    limits=dict(ang=(-10,125),ths=(-.07,.07),gap=(-5,5),tau=(-160,160),iq=(-60,60),fit_edge=.055,stance_end=60)
    d=DrivetrainWebDashboard(dict(subject='AB19',trial='test',current_controlled=True),([0,100],[0,0]),limits,port=0,open_browser=False)
    try:
        s={k:0 for k in SCALARS};s.update({k:[0,1] for k in TRACES})
        s.update(y_em=[2,3],y_actual=[4,5],status='COMPLETE')
        d.update(s);body=json.loads(d._body)
        assert body['y_em']==[2,3] and body['y_actual']==[4,5]
        assert body['status']=='COMPLETE' and not body['nonfinite']
        events=[];d.bind_keys(events.append);d._keys.extend(['space','r','escape']);d.pump()
        assert events==['space','r','escape']
        assert b'CURRENT-CONTROLLED GAIT' in d.html and b'c7' in d.html
    finally:d.close()
    assert not d.alive
