"""Contract failures and exact compatibility against the original sources."""
from dataclasses import replace
from types import SimpleNamespace
from oslbench.contracts import (Sample, RobotState, PositionCommand, TorqueCommand,
    CurrentCommand, CurrentState, GaitContractAdapter, layer_current)
from oslbench.joint_current_control import make_gait_source
from test_gait_feedback_location_ab import setup_source


def rejects(fn, error=ValueError):
    try:
        fn()
    except error:
        return
    raise AssertionError("Malformed contract accepted")


def pair(mode):
    base, layer = setup_source('B')
    src = make_gait_source(base.cfg, base.audit, base.ref, 2, layer, mode)
    src.dt = .0005
    return src, layer


def test_contracts_units_frames_and_validity():
    s = Sample(.2, 'rad', 'knee_joint', 0., 'pre_step')
    rejects(lambda: replace(s, value=float('nan')))
    rejects(lambda: replace(s, value=True))
    rejects(lambda: replace(s, unit='degree'))
    rejects(lambda: replace(s, time_s=-1))
    rejects(lambda: replace(s, frame='unknown'))
    rejects(lambda: replace(s, valid='yes'))
    rejects(lambda: RobotState(s, s, s, s))
    rejects(lambda: PositionCommand('knee', .2, 'unknown'))
    rejects(lambda: TorqueCommand('knee', float('inf'), 'rotor'))
    rejects(lambda: CurrentCommand('ankle', 1., 'applied', 0.))


def test_contracts_current_boundary_rejects_other_meanings():
    for other in (1., PositionCommand('knee', .1, 'knee_joint'),
                  TorqueCommand('knee', 1., 'actuator_output')):
        rejects(lambda: layer_current(other), TypeError)
    for convention in ('requested', 'applied', 'modelled'):
        rejects(lambda: layer_current(CurrentCommand('knee', 2., convention, 0.)))
    command = CurrentCommand('knee', 2., 'layer_rotor_equivalent', 0.)
    assert layer_current(command) == 2.
    rejects(lambda: CurrentState(command, command, command, command))


def test_contracts_adapter_exact_modes_schedule_and_single_conversion():
    for mode in ('legacy', 'actuator-difference'):
        src, layer = pair(mode)
        original, original_layer = pair(mode)
        adapter = GaitContractAdapter(src, layer)
        for k in range(6):
            layer.theta_a = original_layer.theta_a = .1 + k*.01
            layer.theta_a_dot = original_layer.theta_a_dot = .2+k
            expected = original(k, .15+k*.001, .4)
            actual = adapter(k, .15+k*.001, .4)
            assert actual == expected and src.last == original.last
            assert adapter.state.knee_angle.value == .15+k*.001
            assert adapter.state.actuator_output_angle.value == layer.theta_a
            assert adapter.state.knee_velocity.unit == 'rad/s'
            assert adapter.state.knee_angle.source == 'simulation'
            c = adapter.currents
            assert c.requested.current_A == src.last['current_request_A']
            assert c.applied.current_A == src.last['current_applied_A']
            assert c.modelled.current_A == src.last['current_actual_modelled_A']
            assert actual == src.audit.layer_current_for(c.modelled.current_A)
            assert c.requested.time_s == (k//2)*.001
            assert c.modelled.time_s == k*.0005
        assert src.updates == original.updates == 3
        src.reset(); original.reset()
        assert adapter(0, .2, .1) == original(0, .2, .1)
        assert src.last['derivative_used'] == 0


def test_contracts_adapter_rejects_stale_invalid_and_wrong_layer():
    src, layer = pair('actuator-difference')
    adapter = GaitContractAdapter(src, layer); adapter(0, .1, .2)
    state = adapter.state
    for bad in (replace(state.knee_angle, valid=False),
                replace(state.knee_angle, time_s=.1),
                replace(state.knee_angle, source='measured'),
                replace(state.knee_angle, stage='post_step')):
        rejects(lambda: adapter.update(0, replace(state, knee_angle=bad)))
    rejects(lambda: adapter.update(0, replace(state, actuator_output_angle=replace(state.actuator_output_angle,value=1.))))
    _, other = pair('legacy')
    rejects(lambda: GaitContractAdapter(src, other))


def test_contracts_post_step_preserves_mixed_timing_and_torque_frames():
    src, layer = pair('legacy'); adapter=GaitContractAdapter(src,layer)
    record=SimpleNamespace(theta_a=.4,theta_a_dot=.5,tau_m=6.,tau_j=7.,theta_s=.01)
    state=adapter.post_step(2,.2,.3,record)
    assert state.knee_angle.time_s == .0015
    assert state.electromagnetic_output_torque.time_s == .001
    assert state.belt_deflection.time_s == .001
    assert state.electromagnetic_output_torque.frame == 'actuator_output'
    assert state.electromagnetic_output_torque.value == 6.
    assert state.belt_joint_torque.value == 7.
    assert state.actuator_output_angle.value == .4


def test_contracts_clipping_and_lag_remain_distinct():
    src, layer=pair('legacy')
    src.ref=[10.]*6
    # Use actual CurrentLoopModel constructor rather than assuming its internal field.
    from oslbench.ankle_current_control import CurrentLoopModel
    src.loop=CurrentLoopModel(.01)
    adapter=GaitContractAdapter(src,layer); adapter(0,0.,0.)
    c=adapter.currents
    assert c.requested.current_A > c.applied.current_A
    assert c.applied.current_A > c.modelled.current_A > 0
    assert c.layer_rotor_equivalent.current_A == src.audit.layer_current_for(c.modelled.current_A)
