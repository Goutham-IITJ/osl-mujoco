"""C's controller-sampled angle differences, signs, hold and reset contract."""
from dataclasses import replace
import math
from oslbench.ankle_current_control import CurrentPID, CurrentLimits
from oslbench.joint_current_control import KNEE_GAIT_CONFIG, GaitCurrentSource
from oslbench.feedback_location_diagnostic import (ActuatorDifferencePID,
    ActuatorDifferenceSource, FeedbackLocationPID, referred_angle)
from test_gait_feedback_location_ab import setup_source


def test_feedback_location_C_output_angle_units_and_ratio_once():
    nt=50/11
    for sign in (-1,1):
        for angle in (-12.,0.,17.):
            assert abs(referred_angle(nt*math.radians(angle),nt,sign)-sign*angle)<1e-12


def test_feedback_location_C_known_sequence_direct_calculation():
    pid=ActuatorDifferencePID(KNEE_GAIT_CONFIG.gains,CurrentLimits())
    old=None
    for ref,meas,act in [(10.,9.,10.),(10.1,9.1,10.06),(10.2,9.4,10.18),(10.15,9.5,10.16)]:
        pid.actuator_angle_deg=act
        command=pid.update(ref,meas,.001)
        vr=(ref-old[0])/.001 if old else 0.
        va=(act-old[1])/.001 if old else 0.
        expected=.75*(ref-meas)+.03*(vr-va)
        assert abs(command-expected)<1e-12
        assert pid.signals['actuator_difference_velocity_deg_s']==va
        old=(ref,act)


def test_feedback_location_C_uses_actual_controller_period():
    pid=ActuatorDifferencePID(KNEE_GAIT_CONFIG.gains,CurrentLimits())
    pid.actuator_angle_deg=1.;pid.update(1.,1.,.001)
    pid.actuator_angle_deg=1.2;pid.update(1.4,1.,.002)
    assert abs(pid.last['d_A']-3.)<1e-12
    assert pid.signals['controller_period_s']==.002


def test_feedback_location_C_preserves_reference_term():
    a=FeedbackLocationPID(KNEE_GAIT_CONFIG.gains,CurrentLimits(),'A')
    b=FeedbackLocationPID(KNEE_GAIT_CONFIG.gains,CurrentLimits(),'B')
    c=ActuatorDifferencePID(KNEE_GAIT_CONFIG.gains,CurrentLimits())
    for i,ref in enumerate((12.,12.1,12.4,12.2)):
        c.actuator_angle_deg=11.+i*.04;b.actuator_velocity_deg_s=-13.
        for pid,meas in [(a,11.+i*.03),(b,10.),(c,10.5)]:pid.update(ref,meas,.001)
        assert a.signals['reference_derivative_current_A']==b.signals['reference_derivative_current_A']==c.signals['reference_derivative_current_A']


def test_feedback_location_C_first_update_and_reset():
    pid=ActuatorDifferencePID(KNEE_GAIT_CONFIG.gains,CurrentLimits())
    for _ in range(2):
        pid.actuator_angle_deg=1000.
        pid.update(15.,10.,.001)
        assert pid.last['d_A']==0 and not pid.last['derivative_used']
        assert pid.signals['actuator_difference_velocity_deg_s']==0
        pid.actuator_angle_deg=1000.1;pid.update(15.,10.,.001)
        assert pid.last['derivative_used']
        pid.reset()


def test_feedback_location_C_measurement_motor_signs():
    for measurement_sign in (-1,1):
        for motor_sign in (-1,1):
            g=replace(KNEE_GAIT_CONFIG.gains,encoder_to_motor_sign=motor_sign)
            pid=ActuatorDifferencePID(g,CurrentLimits())
            for ref,meas,act in [(10.,9.,10.),(10.1,9.1,10.06)]:
                pid.actuator_angle_deg=referred_angle((50/11)*math.radians(act),50/11,measurement_sign)
                command=pid.update(measurement_sign*ref,measurement_sign*meas,.001)
            assert abs(command-motor_sign*measurement_sign*(.75+1.2))<1e-10


def test_feedback_location_C_samples_only_updates_and_holds():
    old,layer=setup_source('B')
    src=ActuatorDifferenceSource(old.cfg,old.audit,old.ref,2,layer);src.dt=.0005
    commands=[]
    for k,act in enumerate((10.,999.,10.06,-999.,10.18,888.)):
        layer.theta_a=(50/11)*math.radians(act)
        layer.theta_a_dot=1000+k # Must not drive C's derivative.
        commands.append(src(k,math.radians(9.),0.))
        if k%2:
            assert commands[-1]==commands[-2]
            assert src.last['actuator_equivalent_sample_deg']!=act
        else:
            assert abs(src.last['actuator_equivalent_sample_deg']-act)<1e-12
    assert abs(src.last['actuator_difference_velocity_deg_s']-120)<1e-9
    assert src.updates==3


def test_feedback_location_C_does_not_change_AB_or_default_construction():
    for mode in ('A','B'):
        source,_=setup_source(mode)
        assert type(source.pid) is FeedbackLocationPID and source.pid.mode==mode
        original=GaitCurrentSource(source.cfg,source.audit,source.ref,2)
        assert type(original.pid) is CurrentPID and original.pid.derivative_on=='error'


def test_feedback_location_C_refined_scheduler_and_hold():
    old,layer=setup_source('B')
    src=ActuatorDifferenceSource(old.cfg,old.audit,old.ref,4,layer)
    src.dt=.00025
    commands=[]
    for k in range(12):
        act=10.+.06*(k//4) if k%4==0 else 999.+k
        layer.theta_a=(50/11)*math.radians(act)
        layer.theta_a_dot=-9999.
        commands.append(src(k,math.radians(9.),0.))
        assert src.last['controller_period_s']==.001
        assert src.last['controller_updated']==int(k%4==0)
        if k%4:
            assert commands[-1]==commands[-2]
        if k==0:
            assert src.last['d_A']==0 and not src.last['derivative_used']
        if k==4:
            assert abs(src.last['actuator_difference_velocity_deg_s']-60.)<1e-9
    assert src.updates==3
