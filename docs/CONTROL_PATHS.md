# Authoritative control-path reference

Current implementation inventory, 2026-10-18. Path labels A/B/C below describe formulations; feedback experiment variants A/B/C are a separate naming system. All commands run from the repository root. None were executed for Phase 6B.

## A. Original position-servo benchmark

`PDController` uses Kp=600 Nm/rad and Kd=17.253 Nm s/rad. Joint-frame reference/position are radians and velocity is rad/s. The law is `Kp*(clamped_reference-q)-Kd*qdot`; `write_to_model` installs it into the compiled MuJoCo position actuator. Authored command and torque limits remain active. The reference is supplied each physics step (canonical 0.5 ms); MuJoCo evaluates the actuator law.

```powershell
.\.venv\Scripts\python.exe -B experiments\run_bench_ab19.py
.\.venv\Scripts\python.exe -B experiments\run_live_demo.py
```

Sources: [controller](../oslbench/controller.py), [simulation](../oslbench/simulation.py). Tests: [bench suite](../tests/test_oslbench.py), [frozen oracle](../tests/oracle/), [oracle verifier](../experiments/verify_against_oracle.py). Evidence: [benchmark](../build/bench_track_ab19/), [gain sweep](../build/gain_sweep/), [validation record](VALIDATION.md).

Status: protected numerical regression reference. Tracking human kinematics is not validation of physical motor/drivetrain dynamics.

## B. Torque-domain PD through the compliant drivetrain

[PDCurrentSource](../oslbench/drivetrain_sim.py) uses the benchmark torque gains by default. Its position feedback is joint angle; derivative feedback is instantaneous actuator-output velocity divided by external belt ratio `n_t`. The actuator output shaft is after the internal gearbox and before the belt. Torque request is `Kp*(clamped_reference-q_joint)-Kd*(theta_a_dot/n_t)`, converted to layer rotor current by dividing by `n_t*k_t*n_a`.

The source is evaluated every physics step before advancing the external actuator/belt layer. Belt deflection is `q_joint-theta_a/n_t`; joint torque reaches MuJoCo through `qfrc_applied`. The knee position servo is disconnected in memory. This source uses `torque_unclamped`: the disconnected servo's force limit is not a drive-current limit, and this path assumes no current clipping. Paper-derived parameters are not calibrated hardware parameters.

Actual consumers: [benchmark comparison](../experiments/run_bench_ab19_drivetrain.py), [live drivetrain demo](../experiments/run_live_drivetrain_demo.py), and [instability diagnostic](../experiments/diagnose_drivetrain_instability.py). The comparison exposes `--kp`/`--kv`; the live demo uses the benchmark gains without offering those overrides.

```powershell
.\.venv\Scripts\python.exe -B experiments\run_bench_ab19_drivetrain.py
.\.venv\Scripts\python.exe -B experiments\run_live_drivetrain_demo.py
```

Tests: [drivetrain algebra](../tests/test_drivetrain.py), [integration/feedback](../tests/test_drivetrain_sim.py). Comparison writes `bench_track_ab19_servo.csv` and `bench_track_ab19_drivetrain.csv` under [benchmark outputs](../build/bench_track_ab19/). Other evidence: [instability results](../build/drivetrain_instability/RESULTS.md), [diagnosis](DRIVETRAIN_INSTABILITY_DIAGNOSIS.md), [parameter provenance](DRIVETRAIN_PARAMETERS.md). These are simulation comparisons, not physical validation. Inspect output destinations before rerunning historical experiments.

## C. Current-domain knee gait

[CurrentPID](../oslbench/ankle_current_control.py), [GaitCurrentSource](../oslbench/joint_current_control.py) and [ActuatorDifferenceSource](../oslbench/feedback_location_diagnostic.py) implement the current-domain path. Kp=0.75 A/degree, Kd=0.03 A/(degree/second), Ki=0. These gains came from the reported ankle step-test configuration and remain provisional for the knee.

The knee reference and measurement are MuJoCo coordinates converted to degrees with +1 measurement sign. The motor sign is +1 by simulation wiring; neither establishes a physical encoder convention.

- `legacy` (default): sampled error derivative; a measurement-derivative option also exists.
- `actuator-difference` (Variant C): sampled reference derivative minus the finite difference of actuator-output angle referred through `n_t` to knee-equivalent degrees. Requires Ki=0 and derivative-on=error; retains first-update derivative suppression and reset semantics.

Controller updates occur every 1 ms, with held commands between updates. The current-loop model advances every 0.5 ms physics step. Canonical settings use the ideal loop, 0.855 Nm/A output-equivalent `manual_rotor`, CAD belt ratio 50/11 and +/-60 A simulation clipping. `ConversionAudit` maps modelled current into the layer's rotor-current convention with gearing applied once. Requested, clipped/applied, modelled actual and layer-equivalent currents are distinct signals, not hardware measurements.

The compliant plant is `DrivetrainBenchSimulation`; the knee position servo is disconnected. A base-class `PDController` remains for bookkeeping and does not drive this knee.

```powershell
.\.venv\Scripts\python.exe -B experiments\run_current_controlled_gait.py --view --feedback-mode actuator-difference --start-paused --speed 0.25 --zoom 1.2
.\.venv\Scripts\python.exe -B experiments\run_current_controlled_gait.py --feedback-mode legacy
```

[Live adapter](../oslbench/current_gait_live.py) and [dashboard](../oslbench/drivetrain_dashboard.py) consume the runner's shared execution. Browser buttons control simulation lifecycle only. See [live guide](LIVE_DEMO.md).

Tests: [joint current](../tests/test_joint_current_control.py), [feedback A/B](../tests/test_gait_feedback_location_ab.py), [estimator comparison](../tests/test_gait_feedback_location_estimator.py), [live integration](../tests/test_current_gait_live.py). Evidence: [current gait](../build/current_controlled_gait/), [feedback/estimator report](../build/gait_feedback_location_estimator_ab/FEEDBACK_LOCATION_ESTIMATOR_COMPARISON.md), [convergence report](../build/gait_feedback_location_timestep_convergence/TIMESTEP_CONVERGENCE_REPORT.md), [Phase 5 report](../build/integrated_current_gait_phase5/PHASE5_REPORT.md).

Status: implemented live simulation with recorded diagnostic parity and timestep evidence. Historical CSV rows mix pre-step reference and post-step knee timing; retain that convention when comparing old metrics. No knee hardware validation is established. See [research index](RESEARCH_INDEX.md) and [architecture](ARCHITECTURE.md).
