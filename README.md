# OSL V2 in MuJoCo

Implemented 2026-10-19: `oslbench/control/`, `sim/`, `telemetry/` and `presentation/` now own the four responsibility domains; shared contracts remain at package root and `apps/` composes workflows. Experiment commands remain available. Root-level module redirectors have been retired; use the canonical modules described in the architecture guide. The current-gait viewer uses the library runtime directly. See [architecture](docs/ARCHITECTURE.md) for D1 measured-joint mirroring and D2 explicit IMU-estimator handoff. Encoder calibration/transport and a validated IMU estimator remain pending.

CAD-derived Open-Source Leg V2 simulation for fixed-base knee tracking, compliant-drivetrain research and eventual sim-to-real comparison. The current live application runs current-controlled gait with a native MuJoCo viewer, browser dashboard and saved traces from the same execution.

This is a simulation research platform. Deterministic file replay is implemented for the seven established state channels; browser replay displays those recorded observations with explicit timestamps and validity. A passive MSCL IMU reader and read-only Pi-to-laptop HTTP pipeline are implemented with hardware-free tests; physical deployment remains unvalidated. IMU-driven simulation, full-pose replay and predictive hardware comparison remain future work. AB19 human gait is a reference, not a measurement of this robot.

## Start here

- [Live instructions and controls](docs/LIVE_DEMO.md)
- [Recorded telemetry playback, schema and API](docs/TELEMETRY_REPLAY.md)
- [Synthetic mock telemetry and read-only source boundary](docs/TELEMETRY_SOURCE.md)
- [Pi IMU service and laptop client: deployment and limitations](docs/IMU_TELEMETRY.md)
- [Authoritative control paths](docs/CONTROL_PATHS.md)
- [Current architecture and proposed boundaries](docs/ARCHITECTURE.md)
- [Research and evidence index](docs/RESEARCH_INDEX.md)
- [Authoritative roadmap](docs/NEXT_PHASE_PROPOSAL.md)
- [Working baseline and provenance](docs/WORKING_BASELINE.md)
- [Original benchmark validation record](docs/VALIDATION.md) (dated evidence; see the research index)

## Three control paths

| Path | Controller and gains | Plant / purpose |
|---|---|---|
| A: original position servo | `PDController`: Kp 600 Nm/rad, Kd 17.253 Nm s/rad | Direct MuJoCo position actuator; frozen regression oracle |
| B: torque PD through drivetrain | `PDCurrentSource`: torque-domain PD converted to current | External actuator/belt dynamics and joint `qfrc_applied`; comparison/demo |
| C: current-domain gait | Kp 0.75 A/degree, Kd 0.03 A/(degree/second), Ki 0 | Current-loop model and compliant drivetrain; canonical live application |

These gains have different units and are not interchangeable. Path C's gains were provisionally transferred from the reported ankle step-test configuration; they are not confirmed knee hardware tuning.

## Environments

Commands below use Windows PowerShell from the repository root. Use the existing environments if available; do not recreate or upgrade an environment used for recorded results.

The observed simulation environment has Python 3.10.7, MuJoCo 3.12.0, NumPy 2.2.6 and Matplotlib 3.10.9. The analysis environment has Python 3.12.10, NumPy, Matplotlib, SciPy and h5py. Full observed package metadata is in the [baseline manifest](docs/baselines/phase6b_20261018.json). Unlike the older README, the current simulation environment **does include plotting**.

For a new installation only, the existing setup approach is:

```powershell
py -3.10 -m venv .venv
.\.venv\Scripts\python.exe -m pip install mujoco==3.12.0 numpy==2.2.6 matplotlib==3.10.9 glfw==2.10.2 PyOpenGL==3.1.10
py -3.12 -m venv .venv-analysis
.\.venv-analysis\Scripts\python.exe -m pip install numpy matplotlib scipy h5py
```

These are dependency installation instructions, not a tested clean-install recipe or lockfile. No dependency manifest currently captures all transitive dependencies. Dataset/reference availability must also be checked: the runners use `build/AB19_knee_gait_reference.csv`, which may be absent from a fresh checkout. Do not regenerate or replace an existing reference; verify it against the baseline manifest. MyoAssist environments are separate and parked.

## Run the canonical live simulation

```powershell
.\.venv\Scripts\python.exe -B experiments\run_current_controlled_gait.py --view --feedback-mode actuator-difference --start-paused --speed 0.25 --zoom 1.2
```

Variant C is selected explicitly. Use `--feedback-mode legacy`, or omit feedback selection, for the unchanged default. Remove `--view` for headless execution. Variant C requires Ki=0 and derivative-on=error.

Canonical settings: 0.855 Nm/A output-equivalent `manual_rotor`, external belt ratio 50/11, ideal current loop, +/-60 A simulation clip, 0.5 ms physics and 1 ms control. These are simulation assumptions, not hardware safety limits. Speed and zoom affect presentation only.

Space pauses/resumes; R saves the attempt and restarts; Esc saves/exits. The browser offers equivalent lifecycle controls. See the [live guide](docs/LIVE_DEMO.md) for outputs and completion behavior.

## Replay saved observations (no physics)

```powershell
.\.venv\Scripts\python.exe -B experiments\run_telemetry_replay.py --manifest build\phase6d_replay\phase5_dataset\manifest.json --speed 0.25
```

The browser starts paused. Space plays/pauses, R resets, N steps one timestamp; speed
buttons change pacing. EOF holds the final display. This is recorded simulation
telemetry, not hardware data. Each channel retains its source time/validity; no full
robot pose is invented. See the [replay guide](docs/TELEMETRY_REPLAY.md) for controls.

## Tests and original oracle

For developer use; these commands execute tests/simulation and were not run during this documentation milestone:

```powershell
.\.venv\Scripts\python.exe -B tests\run_tests.py
.\.venv\Scripts\python.exe -B experiments\verify_against_oracle.py
```

The test runner can use a stub when MuJoCo is unavailable; inspect its engine banner. A stub result is not a physics verification. The oracle command compares a new direct-servo benchmark against the frozen trace and metrics in `tests/oracle/`. Never update that oracle to hide a regression.

## Outputs and evidence

- Live gait: `build/current_controlled_gait/live_<mode>_<timestamp>/attempt_001/`, including CSV, metrics, metadata, plots and dashboard snapshot. Subsequent attempts are preserved separately.
- Headless gait: tagged directories under `build/current_controlled_gait/`.
- Original benchmark: `build/bench_track_ab19/`; oracle verification: `build/verify_oracle/`.
- Completed Phase 5 evidence: [report](build/integrated_current_gait_phase5/PHASE5_REPORT.md) and [live attempt](build/integrated_current_gait_phase5/visible_live_run/attempt_001/).

`build/` contains evidence-bearing outputs and also the reference input. Preserve it; a fresh Git checkout may not include it. Historical runs, including partial/stub runs, must retain their identities.

Saved Phase 5 records report exact live-versus-diagnostic C parity across 29 channels and 2,410 steps. The original oracle protects Path A; it does not establish Path C hardware fidelity. Numerical convergence and passing tests are not physical validation. The [research index](docs/RESEARCH_INDEX.md) separates completed simulation evidence from proposals.

Detailed CAD derivations, original setup rationale and earlier benchmark narrative remain in the [preserved historical README](docs/README_PRE_PHASE6B.md), [CAD findings](docs/CAD_FINDINGS.md) and [benchmark reference](docs/BENCH_EXPERIMENT.md).
