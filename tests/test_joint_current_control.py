#!/usr/bin/env python3
"""
test_joint_current_control.py -- unit tests for joint_current_control.py and gait integration.

Covers:
  - Current-controller units and output for known errors.
  - Derivative initialisation and controller rate-holding (1 kHz on 2 kHz physics).
  - Knee reference and measurement sign conventions (negative feedback).
  - Current conversion and single application of the planetary gear ratio.
  - Verification that the legacy knee position servo is disconnected.
  - Regression test: single execution per trajectory in both aligned and zero init modes.
  - Consistency of logged tracking error and plot conventions.
"""

from __future__ import annotations

import argparse
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
for _p in (ROOT, HERE):
    if _p not in sys.path:
        sys.path.insert(0, _p)


class Skip(Exception):
    """Raised to skip a test when preconditions are not met."""


try:
    import stub_mujoco
    stub_mujoco.install()
except Exception:                                               # noqa: BLE001
    pass

from oslbench.ankle_current_control import (CAD_BELT_RATIO, ConversionAudit,
                                            CurrentLimits, CurrentPID,
                                            PIDGains, TORQUE_CONSTANT_CHOICES,
                                            torque_constant)
from oslbench.controller import PDController
from oslbench.drivetrain import PAPER, DrivetrainParameters
from oslbench.drivetrain_sim import DrivetrainBenchSimulation, DrivetrainLayer
from oslbench.joint_current_control import (GAIN_SOURCE_NOTE,
                                            GaitCurrentSource,
                                            JointCurrentConfig,
                                            KNEE_GAIT_CONFIG)
from oslbench.model import load_bench_model
from experiments.run_current_controlled_gait import (CSV_HEADER, build, metrics,
                                                     require_backend, run)


def test_current_controller_units_and_output_for_known_errors():
    """CurrentPID accepts degrees and produces Amperes with the configured gains."""
    gains = PIDGains(kp_A_per_deg=0.75, ki_A_per_deg_s=0.0, kd_A_per_deg_per_s=0.03,
                     encoder_to_motor_sign=+1)
    limits = CurrentLimits(sim_clip_A=60.0)
    pid = CurrentPID(gains, limits, derivative_on="error")

    # 1. Zero error -> zero current
    cmd0 = pid.update(ref_deg=10.0, meas_deg=10.0, dt=0.001)
    assert abs(cmd0) < 1e-12, f"Expected 0.0 A, got {cmd0}"
    assert pid.last["p_A"] == 0.0
    assert pid.last["d_A"] == 0.0

    # 2. Known positive error: ref=14.0 deg, meas=10.0 deg -> error = 4.0 deg
    # On first call where error changes, derivative is evaluated
    cmd1 = pid.update(ref_deg=14.0, meas_deg=10.0, dt=0.001)
    # p = 0.75 * 4.0 = 3.0 A; d = 0.03 * (4.0 - 0.0) / 0.001 = 120.0 A -> raw = 123 A -> clipped to 60 A
    assert pid.last["p_A"] == 3.0, f"Expected p_A = 3.0 A, got {pid.last['p_A']}"
    assert pid.last["saturated"] is True
    assert cmd1 == 60.0

    # 3. Known error without derivative: reset and test first update
    pid.reset()
    cmd_first = pid.update(ref_deg=14.0, meas_deg=10.0, dt=0.001)
    # First update suppresses derivative: p = 0.75 * 4.0 = 3.0 A, d = 0.0 A
    assert pid.last["derivative_used"] is False
    assert pid.last["d_A"] == 0.0
    assert abs(cmd_first - 3.0) < 1e-12, f"Expected 3.0 A, got {cmd_first}"

    # 4. Known negative error on first update: ref=8.0 deg, meas=10.0 deg -> error = -2.0 deg
    pid.reset()
    cmd_neg = pid.update(ref_deg=8.0, meas_deg=10.0, dt=0.001)
    # p = 0.75 * (-2.0) = -1.5 A
    assert abs(cmd_neg - (-1.5)) < 1e-12, f"Expected -1.5 A, got {cmd_neg}"


def test_derivative_initialisation_and_controller_rate_holding():
    """First update suppresses derivative, and 1 kHz command is held at 2 kHz physics."""
    cfg = KNEE_GAIT_CONFIG
    audit = ConversionAudit(
        k_t_output_Nm_per_A=0.855, k_t_output_choice="manual_rotor",
        k_t_output_provenance="test", belt_ratio=CAD_BELT_RATIO,
        belt_ratio_source="test", params=PAPER, limits=CurrentLimits())

    # Reference array for 4 physics steps (2 controller updates at steps_per_update=2)
    ref_rad = [math.radians(10.0), math.radians(10.0), math.radians(12.0), math.radians(12.0)]
    steps_per_update = 2
    src = GaitCurrentSource(cfg, audit, ref_rad, steps_per_update)
    src.dt = 0.0005  # 2 kHz physics -> 0.5 ms

    # Step 0 (k=0): 1st update -> PID evaluates, derivative suppressed
    i0 = src(0, theta_j=math.radians(5.0), theta_j_dot=0.0)
    assert src.updates == 1
    assert src.last["derivative_used"] == 0
    assert src.last["error_deg"] == 5.0
    held_cmd_0 = src.held_cmd_A

    # Step 1 (k=1): Between updates -> PID does NOT evaluate; held_cmd_A is unchanged
    i1 = src(1, theta_j=math.radians(5.5), theta_j_dot=0.1)
    assert src.updates == 1, "Controller should not update on odd step"
    assert src.held_cmd_A == held_cmd_0, "Command must be held across control interval"

    # Step 2 (k=2): 2nd update -> PID evaluates again with new reference and joint angle
    i2 = src(2, theta_j=math.radians(6.0), theta_j_dot=0.1)
    assert src.updates == 2, "Controller must update on step multiple"
    assert src.last["derivative_used"] == 1, "Derivative should now be active"


def test_knee_reference_and_measurement_sign_conventions():
    """Knee configuration enforces negative feedback in the shared model frame."""
    cfg = KNEE_GAIT_CONFIG
    assert cfg.joint == "knee"
    assert cfg.measurement_sign == +1, "Knee reference and measurement share the MuJoCo frame (+1)"
    assert cfg.gains.encoder_to_motor_sign == +1, "Motor sign +1 required for negative feedback"
    assert cfg.loop_product == +1, "Loop product must be +1 for negative feedback"
    assert cfg.negative_feedback is True

    # Test physical sign propagation:
    # A positive tracking error (ref > meas) produces a positive current command
    # I_q > 0 -> tau_m > 0 -> theta_a accelerates positive -> theta_s < 0 -> tau_j > 0
    # Positive tau_j on MuJoCo knee hinge accelerates the knee in positive direction (flexion)
    dt = 0.0005
    layer = DrivetrainLayer(PAPER, enabled=True)
    layer.seed_rigid(0.0)
    step1 = layer.advance(theta_j=0.0, i_q=5.0, dt=dt)
    assert step1.tau_m > 0.0, "Positive current must produce positive motor torque"
    assert step1.theta_a > 0.0, "Positive motor torque must advance actuator shaft positive"
    # In semi-implicit Euler, the spring force uses the pre-step configuration.
    # On the second step, the advanced actuator position causes theta_s < 0 and tau_j > 0.
    step2 = layer.advance(theta_j=0.0, i_q=5.0, dt=dt)
    assert step2.theta_s < 0.0, "Positive actuator shaft must produce negative belt deflection"
    assert step2.tau_j_mean > 0.0, "Negative belt deflection must apply positive joint torque"


def test_current_conversion_applies_planetary_ratio_once():
    """layer_current_for correctly rescales amps so n_a is applied exactly once inside the layer."""
    kt_ao = 0.855  # N.m/A at actuator output
    audit = ConversionAudit(
        k_t_output_Nm_per_A=kt_ao, k_t_output_choice="manual_rotor",
        k_t_output_provenance="test", belt_ratio=CAD_BELT_RATIO,
        belt_ratio_source="test", params=PAPER, limits=CurrentLimits())

    i_cmd = 10.0  # A
    i_rotor = audit.layer_current_for(i_cmd)

    # In DrivetrainLayer: tau_m = i_rotor * PAPER.k_t * PAPER.n_a
    tau_m = i_rotor * PAPER.k_t * PAPER.n_a
    expected_tau_m = i_cmd * kt_ao
    assert abs(tau_m - expected_tau_m) < 1e-12, (
        f"tau_m ({tau_m:.6f}) must equal i_cmd * kt_ao ({expected_tau_m:.6f})")


def test_old_knee_position_servo_is_disconnected():
    """Disconnecting the servo sets gains to zero in compiled model and yields 0 N.m actuator force."""
    bench = load_bench_model(verbose=False)
    if bench.failures:
        raise Skip(f"Bench model failed preconditions: {bench.failures}")

    layer = DrivetrainLayer(PAPER, enabled=True)
    pd = PDController.knee(bench, 600.0, 17.253)
    sim = DrivetrainBenchSimulation(bench, pd, layer=layer, disconnect_servo=True)

    assert sim.servo_connected is False
    assert bench.model.actuator_gainprm[bench.knee_act, 0] == 0.0
    assert bench.model.actuator_biasprm[bench.knee_act, 1] == 0.0

    # Step with a large reference command
    sim.reset(0.0)
    sim.step(math.radians(20.0), 0)
    assert sim.knee_servo_force() == 0.0, "Disconnected knee servo must deliver exactly 0.0 N.m"


def test_single_execution_per_trajectory_in_both_init_modes():
    """Regression test: run() executes the simulation exactly once for both aligned and zero init."""
    bench = load_bench_model(verbose=False)
    if bench.failures:
        raise Skip(f"Bench model failed preconditions: {bench.failures}")

    dt = bench.timestep
    n = 20  # short trajectory
    ref_rad = np.linspace(math.radians(12.0), math.radians(20.0), n)
    phase = np.linspace(0.0, 100.0, n)
    ref_vel = np.zeros(n)
    res = dict(ref_rad=ref_rad, gait_phase_percent=phase, ref_vel_rad_s=ref_vel, n=n)

    audit = ConversionAudit(
        k_t_output_Nm_per_A=0.855, k_t_output_choice="manual_rotor",
        k_t_output_provenance="test", belt_ratio=CAD_BELT_RATIO,
        belt_ratio_source="test", params=PAPER, limits=CurrentLimits())
    cfg = KNEE_GAIT_CONFIG
    src = GaitCurrentSource(cfg, audit, ref_rad, steps_per_update=2)
    src.dt = dt

    # Test mode 1: aligned
    layer1 = DrivetrainLayer(PAPER, enabled=True)
    pd1 = PDController.knee(bench, 600.0, 17.253)
    sim1 = DrivetrainBenchSimulation(bench, pd1, layer=layer1, current_source=src, disconnect_servo=True)
    step_calls_aligned = [0]
    orig_step1 = sim1.step

    def count_step1(q_ref, k):
        step_calls_aligned[0] += 1
        return orig_step1(q_ref, k)

    sim1.step = count_step1
    ctx1 = dict(bench=bench, sim=sim1, src=src, res=res, n=n, dt=dt)
    args_aligned = argparse.Namespace(init="aligned")
    rows_aligned = run(ctx1, args_aligned)

    assert step_calls_aligned[0] == n, f"Expected {n} steps for 'aligned', got {step_calls_aligned[0]}"
    assert len(rows_aligned) == n

    # Test mode 2: zero (REGRESSION TEST: previously ran 2*n steps)
    layer2 = DrivetrainLayer(PAPER, enabled=True)
    pd2 = PDController.knee(bench, 600.0, 17.253)
    sim2 = DrivetrainBenchSimulation(bench, pd2, layer=layer2, current_source=src, disconnect_servo=True)
    step_calls_zero = [0]
    orig_step2 = sim2.step

    def count_step2(q_ref, k):
        step_calls_zero[0] += 1
        return orig_step2(q_ref, k)

    sim2.step = count_step2
    ctx2 = dict(bench=bench, sim=sim2, src=src, res=res, n=n, dt=dt)
    args_zero = argparse.Namespace(init="zero")
    rows_zero = run(ctx2, args_zero)

    assert step_calls_zero[0] == n, (
        f"REGRESSION: 'zero' init executed {step_calls_zero[0]} steps; must execute exactly {n} steps")
    assert len(rows_zero) == n


def test_tracking_error_consistency_and_plot_convention():
    """Logged tracking_error_deg is reference minus measured, and final_error matches."""
    bench = load_bench_model(verbose=False)
    if bench.failures:
        raise Skip(f"Bench model failed preconditions: {bench.failures}")

    assert CSV_HEADER[5] == "tracking_error_deg", (
        f"Expected column 5 to be 'tracking_error_deg', got {CSV_HEADER[5]}")

    dt = bench.timestep
    n = 150  # >= 100 steps so 50 ms start transient is well within range
    ref_rad = np.linspace(math.radians(10.0), math.radians(20.0), n)
    phase = np.linspace(0.0, 100.0, n)
    ref_vel = np.zeros(n)
    res = dict(ref_rad=ref_rad, gait_phase_percent=phase, ref_vel_rad_s=ref_vel, n=n)

    audit = ConversionAudit(
        k_t_output_Nm_per_A=0.855, k_t_output_choice="manual_rotor",
        k_t_output_provenance="test", belt_ratio=CAD_BELT_RATIO,
        belt_ratio_source="test", params=PAPER, limits=CurrentLimits())
    cfg = KNEE_GAIT_CONFIG
    src = GaitCurrentSource(cfg, audit, ref_rad, steps_per_update=2)
    src.dt = dt
    layer = DrivetrainLayer(PAPER, enabled=True)
    pd = PDController.knee(bench, 600.0, 17.253)
    sim = DrivetrainBenchSimulation(bench, pd, layer=layer, current_source=src, disconnect_servo=True)

    ctx = dict(bench=bench, sim=sim, src=src, res=res, n=n, dt=dt, audit=audit)
    args = argparse.Namespace(init="aligned")
    rows = run(ctx, args)

    for r in rows:
        r_deg = r[3]
        q_deg = r[4]
        err_deg = r[5]
        assert abs(err_deg - (r_deg - q_deg)) < 1e-10, (
            f"tracking_error_deg ({err_deg}) must equal ref_deg ({r_deg}) - knee_angle_deg ({q_deg})")

    m = metrics(rows, ctx, args)
    expected_final_err = rows[-1][3] - rows[-1][4]
    assert abs(m["final_error_deg"] - expected_final_err) < 1e-10, (
        f"final_error_deg ({m['final_error_deg']}) must equal final reference - actual ({expected_final_err})")

    # Verify that RMS and MAE error do not depend on the sign of error
    # Create reversed rows
    col_err = CSV_HEADER.index("tracking_error_deg")
    reversed_rows = []
    for r in rows:
        r_list = list(r)
        r_list[col_err] = -r_list[col_err]
        reversed_rows.append(tuple(r_list))
    m_rev = metrics(reversed_rows, ctx, args)
    assert abs(m["rms_error_deg"] - m_rev["rms_error_deg"]) < 1e-12, "RMS error must be sign-invariant"
    assert abs(m["mae_error_deg"] - m_rev["mae_error_deg"]) < 1e-12, "MAE error must be sign-invariant"
    assert abs(m["final_error_deg"] + m_rev["final_error_deg"]) < 1e-12, "Final error must flip sign"


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    passed = 0
    for t in tests:
        t()
        passed += 1
        print(f"ok {t.__name__}")
    print(f"\n{passed} passed")


def test_gait_current_loop_selection_hold_and_modelled_torque():
    """C must hold commands while lag evolves and reaches the plant each step."""
    from experiments.run_gait_current_loop_sensitivity import arguments
    from oslbench.sim import current_gait as gait
    import mujoco
    for tau in (0., 2.):
        args = arguments(tau)
        ctx = gait.build(args, mujoco, False)
        assert ctx['cfg'].current_loop_tau_s == tau*.001
        assert ctx['src'].mode == 'C'
        ctx['n'] = 12
        rows = np.asarray(gait.run(ctx, args))
        col = lambda name: rows[:,gait.CSV_HEADER.index(name)]
        command, actual = col('current_applied_A'), col('current_actual_modelled_A')
        assert np.array_equal(command[::2],command[1::2])
        assert ctx['src'].updates == 6 and ctx['src'].loop.steps == 12
        assert col('pid_derivative_used')[0] == 0
        expected = command if tau == 0 else command+(np.r_[0.,actual[:-1]]-command)*math.exp(-.0005/.002)
        assert np.allclose(actual,expected,rtol=1e-12,atol=1e-12)
        assert np.allclose(col('estimated_actuator_output_torque_Nm'),actual*.855,rtol=1e-12,atol=1e-12)
        if tau:
            assert np.any(actual[1::2] != actual[::2])
        else:
            assert np.array_equal(command,actual)
        ctx['src'].reset()
        assert ctx['src'].loop.i_actual == 0 and ctx['src'].loop.steps == 0
