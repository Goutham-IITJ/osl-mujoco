"""Focused diagnostic tests; no experiment files or simulation runs are generated."""
import copy
import math
from types import SimpleNamespace
from unittest.mock import patch

from oslbench.ankle_current_control import CurrentPID, PIDGains, CurrentLimits, ConversionAudit
from oslbench.drivetrain import PAPER
from oslbench.drivetrain_sim import DrivetrainLayer
from oslbench.joint_current_control import GaitCurrentSource, KNEE_GAIT_CONFIG
from oslbench.feedback_location_diagnostic import (FeedbackLocationPID,
    FeedbackLocationSource, referred_velocity)
from experiments import run_gait_feedback_location_ab as runner


def setup_source(mode):
    cfg = copy.deepcopy(KNEE_GAIT_CONFIG)
    p = PAPER.with_belt_ratio(50/11)
    audit = ConversionAudit(.855, 'manual_rotor', 'diagnostic test', p.n_t, 'CAD', p)
    layer = DrivetrainLayer(p, enabled=True)
    ref = [math.radians(x) for x in (10,11,12,13,14,15)]
    src = FeedbackLocationSource(cfg, audit, ref, 2, layer, mode)
    src.dt = .0005
    return src, layer


def test_feedback_location_units_sign_and_single_belt_ratio():
    for sign in (-1, 1):
        assert abs(referred_velocity((50/11)*math.pi, 50/11, sign)-sign*180) < 1e-12
        assert abs(referred_velocity(-(50/11)*math.pi, 50/11, sign)+sign*180) < 1e-12
    # A rigid shaft and joint have the same controller velocity, with no n_a factor.
    assert referred_velocity(0, 50/11, 1) == 0


def test_feedback_location_A_exact_production_equivalence():
    for sign in (-1, 1):
        g = PIDGains(kp_A_per_deg=.75, ki_A_per_deg_s=0,
                     kd_A_per_deg_per_s=.03, encoder_to_motor_sign=sign)
        a = FeedbackLocationPID(g, CurrentLimits(), 'A')
        original = CurrentPID(g, CurrentLimits(), derivative_on='error')
        # Include clipping, negative commands and changing dt, not just near zero.
        for ref, meas, dt in [(12,12,.001),(13,10,.001),(-100,30,.002),
                               (80,-30,.001),(14,13,.001)]:
            assert a.update(ref, meas, dt) == original.update(ref, meas, dt)
            assert a.last == original.last
            assert abs(a.signals['derivative_decomposition_residual_A']) < 1e-11


def test_feedback_location_reference_contribution_preserved():
    g = copy.deepcopy(KNEE_GAIT_CONFIG.gains)
    a, b = [FeedbackLocationPID(g, CurrentLimits(), mode) for mode in ('A','B')]
    a.update(10,9,.001)
    b.update(10,-3,.001)
    b.actuator_velocity_deg_s = -30
    a.update(10.1,9.03,.001)
    b.update(10.1,4,.001)
    assert a.signals['reference_derivative_current_A'] == b.signals['reference_derivative_current_A']
    assert abs(b.last['d_A'] - .03*(100+30)) < 1e-12
    assert b.last['p_A'] == .75*(10.1-4)


def test_feedback_location_first_update_and_reset_suppression():
    for mode in ('A','B'):
        pid = FeedbackLocationPID(KNEE_GAIT_CONFIG.gains, CurrentLimits(), mode)
        for repeat in range(2):
            pid.actuator_velocity_deg_s = 1000
            pid.update(20,10,.001)
            assert pid.last['d_A'] == 0 and not pid.last['derivative_used']
            assert pid.signals['reference_derivative_current_A'] == 0
            assert pid.signals['measurement_derivative_current_A'] == 0
            pid.update(20,11,.001)
            assert pid.last['derivative_used']
            pid.reset()


def test_feedback_location_schedule_and_hold_both_modes():
    for mode in ('A','B'):
        src, layer = setup_source(mode)
        calls = []
        for k in range(6):
            layer.theta_a_dot = (k+1)*.5
            calls.append(src(k, math.radians(10+k*.1), 999))
            assert src.last['control_update'] == k//2
            assert src.last['controller_updated'] == int(k%2 == 0)
            assert src.last['current_actual_modelled_A'] == src.last['current_applied_A']
            if k%2:
                assert calls[-1] == calls[-2]
            if mode == 'B' and k%2 == 0:
                assert src.last['derivative_feedback_velocity_deg_s'] == referred_velocity(
                    layer.theta_a_dot, layer.p.n_t, 1)
        assert src.updates == 3


def test_feedback_location_default_path_unchanged():
    src, layer = setup_source('A')
    original = GaitCurrentSource(src.cfg, src.audit, src.ref, 2)
    original.dt = src.dt
    assert type(original.pid) is CurrentPID and original.pid.derivative_on == 'error'
    for k in range(6):
        q = math.radians(9+k*.12)
        assert src(k,q,-333) == original(k,q,777)
        for key,value in original.last.items():
            assert src.last[key] == value, key


def test_feedback_location_isolation_configuration_and_backend():
    a = runner.arguments()
    assert (a.kp,a.kd,a.ki,a.kt,a.belt,a.current_loop,a.init,a.cycles) == (
        .75,.03,0.,'manual_rotor','cad','ideal','aligned',1)
    with patch.object(runner.Path, 'exists', return_value=False):
        assert runner.variant_path('A').parent == runner.OUT.resolve()
        assert runner.variant_path('B') != runner.variant_path('A')
    with patch.object(runner.Path, 'exists', return_value=True):
        try:
            runner.variant_path('A')
        except FileExistsError:
            pass
        else:
            raise AssertionError('Must refuse existing results')
    try:
        runner.variant_path('A', runner.OLD)
    except ValueError:
        pass
    else:
        raise AssertionError('Must reject alternate output roots')
    try:
        runner.require_real(SimpleNamespace(__stub__=True, __version__='fake'))
    except RuntimeError:
        pass
    else:
        raise AssertionError('Must reject stub')


def test_feedback_location_historical_metadata_rows():
    values = runner.baseline_values({'rms_error_deg':4.4}, {'experiment':'original', 'cycles':1})
    assert values == {'rms_error_deg':4.4, 'meta_experiment':'original', 'meta_cycles':1}
