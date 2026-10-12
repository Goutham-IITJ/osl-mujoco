# OSL V2 ankle parameter sensitivity

This report contains six **real-MuJoCo simulation predictions**, not hardware measurements. Controller gains, clipping, timing, initialisation, compiled MJCF and encoder sign were held constant.

## Official manufacturer specification

The official CubeMars AK80-9 V3.0 product page was retrieved on **2026-10-12**:

<https://www.cubemars.com/product/ak80-9-v3-0-robotic-actuator.html>

It lists a torque constant of **0.095 N.m/A**, an internal reduction of **9:1**, rated
torque of **9 N.m**, peak torque of **22 N.m**, rated current of **12 A**, and peak
current of **28 A**. The page does not explicitly state whether the 0.095 N.m/A constant is rotor-side or
actuator-output-side, and it does not resolve whether the current ratings refer to the
motor, driver, or protocol command. My current understanding is that the constant is
rotor-side, before the internal gearbox; this is **not independently verified hardware
documentation**. Under that provisional interpretation, `0.095 × 9 = 0.855 N.m/A` is
the ideal actuator-output equivalent when losses are neglected.

The product comparison separately identifies the AK80-6 as a **6:1** actuator. Thus
`0.095 × 6 ≈ 0.5701` is a plausible lead about the existing script value, not evidence
of its provenance.

## Parameter meanings

- `script`: 0.5701 N.m/A, reported as an actuator-output constant from the hardware script; its shaft provenance remains unresolved.
- `paper`: 0.9972 N.m/A, derived from the paper rotor constant 0.1108 N.m/A times the 9:1 planetary ratio.
- `manual_rotor`: 0.8550 N.m/A, derived as `0.095 × 9` under the provisional rotor-side interpretation and neglecting losses; this is the preferred working hypothesis, not a manufacturer-confirmed output constant.
- `cad`: belt ratio 50/11 = 4.5454545 from the MJCF pulley tooth counts.
- `paper`: belt ratio 4.61 from Best et al. 2025.

The selected output-side torque constant is converted to the rotor-current input expected by `DrivetrainLayer`; the planetary ratio is then applied once inside that layer. The belt ratio enters once through `theta_s = theta_j - theta_a/n_t`. No rigid joint-torque scaling replaces the compliant belt. The manufacturer-derived `manual_rotor` candidate is an ideal actuator-output equivalent, not a direct manufacturer-listed output constant. It is retained as the preferred working hypothesis for interpretation only; the simulations still preserve all three candidates and their existing results.

## Results

The complete machine-readable comparison is in `build/ankle_sensitivity/sensitivity_summary.csv`. Settling uses the same definition as the ankle experiment: the first sample after the last excursion outside a ±0.1° band around the 5° target.

- Successful runs: 6/6.
- Final-angle range: 4.949342° to 4.987075°.
- Rise-time range: 0.088500 s to 0.105000 s.
- Peak applied-current range: 3.750000 A to 3.750000 A.
- Peak joint-side belt-torque range: 1.218884 to 1.893103 N.m.
- Maximum belt-deflection range: 0.001359935 to 0.002086932 rad.

## Interpretation

The torque-constant choice changes the current-to-output-torque conversion and therefore changes the simulated plant response, transmitted torque and belt deflection. It does not change the configured current clip or the controller gains. The belt-ratio choice changes the actuator/joint kinematic relationship and the compliant belt loading, so it can change both dynamics and the joint-side torque even when the output torque constant is fixed.

The six traces should be read as a range of plausible predictions, not as a ranking or a fit to the informal lab estimate. Tracking, settling and saturation are robust only to the extent shown by the six rows; no candidate is preferred without primary hardware evidence. Differences combine torque scaling with belt-stiffness interpretation through the changed deflection kinematics.

The official page resolves the named AK80-9's nominal 9:1 reduction and lists its
0.095 N.m/A constant. My provisional rotor-side interpretation makes 0.8550 N.m/A
the preferred working hypothesis, but the shaft reference remains unconfirmed.
The existing script value is still not reconciled with the AK80-9 specification; the
6:1 coincidence remains only a hypothesis. Direct manufacturer clarification of the
constant's shaft, the exact actuator variant installed on the rig, and measured current
plus a named torque shaft would further test the ideal-current-loop assumption. The
three torque-constant candidates remain in the study, with the existing results
unchanged; the provisional preference is not a selection based on matching the
informal lab torque estimate.

Equation residuals, backend/model identity, update counts, servo checks and per-run warnings are recorded in `sensitivity_summary.csv`; generated plots are `angle_comparison.png` and `current_joint_torque_comparison.png`.

## Supplemental 20° ankle step

The ankle current experiment now accepts `--step-deg` while retaining its historical
5° default. A separate 20° run was completed with the same default `script` torque
constant, CAD belt ratio, gains, timing, current clip and real-MuJoCo model:

```text
experiments\run_ankle_current_step.py --step-deg 20 --outdir build\ankle_current_step_20deg
```

This is an additional current-controlled Option-C drivetrain simulation, not a
replacement for the six 5° sensitivity cases and not a hardware result. It reached
`20.2079801°` with `5.35332773%` overshoot, `0.089 s` 10–90% rise time, `1.5745 s`
settling time using the same ±2% step band (±0.4° for a 20° step), and `15 A` peak
requested/applied current without current clipping. The output files are
`build/ankle_current_step_20deg/ankle_current_step.csv`,
`ankle_current_step_metrics.csv`, and `ankle_current_step.png`. The run used
`ngeom=217`, `nmesh=80`, 8,000 physics steps, 4,000 control updates, zero ankle-servo
force, and the geometry-derived encoder sign. Its peak applied current is above the
reported 12 A rated-current figure but below the reported 28 A peak-current figure;
those manufacturer ratings and their exact electrical reference remain unresolved.

The ankle plot also includes angular velocity for gain-comparison work. The CSV
already records both `ankle_velocity_rad_s` (MuJoCo joint-coordinate velocity) and
`encoder_rate_deg_s` (the same velocity mapped into the encoder frame). The plot
shows both signals in a dedicated angular-velocity panel. Controller gains can be
varied without editing code, for example:

```text
experiments\run_ankle_current_step.py --step-deg 20 --kp 1.0 --kd 0.05 --ki 0 --outdir build\ankle_current_step_20deg_kp1_kd005
```

Use a separate output directory for each gain combination so the baseline and prior
comparison results remain intact.
