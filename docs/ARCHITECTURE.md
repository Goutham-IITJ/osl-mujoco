# Architecture: implemented behavior and proposed boundaries

Current reference, 2026-10-18. Implementation order is maintained only in the [roadmap](NEXT_PHASE_PROPOSAL.md); this document describes ownership, not a competing plan.

## Existing execution

`reference.py` loads/audits/resamples the reference. `controller.py` and the current-control modules implement the three [control paths](CONTROL_PATHS.md). `model.py` owns bench model access. `BenchSimulation` steps the direct-servo plant; `DrivetrainBenchSimulation` adds the external actuator/belt state and applied joint torque. `drivetrain.py` contains parameter provenance and equations.

The [current gait runner](../experiments/run_current_controlled_gait.py) constructs configuration and the plant, owns `iter_run`, metrics and output functions. Both headless and live execution use that iterator. The [live adapter](../oslbench/current_gait_live.py) currently imports the experiment runner; this dependency should be wrapped and then inverted incrementally, not resolved by copying equations. Shared current primitives currently reside in `ankle_current_control.py`; their location does not make knee settings hardware-confirmed.

The native viewer displays the same model/data that physics advances. Dashboard snapshots are decimated presentation data; CSV records retain the physics-step data. Lifecycle events are distinct from actuator commands. Rendering speed does not set physics/control periods.

## Implemented Phase 6C contracts

[contracts.py](../oslbench/contracts.py) provides immutable `Sample`, `RobotState`,
`PositionCommand`, `TorqueCommand`, `CurrentCommand` and `CurrentState` types.
Sample values declare rad, rad/s, Nm or A, a knee_joint/actuator_output/rotor frame,
relative simulation-clock seconds, acquisition stage, source and validity. Positive
coordinates follow the existing model; hardware encoder polarity is not inferred.
RobotState validates the units/frames of each named field. Optional forces remain
absent before they are evaluated; invalid/non-finite controller inputs are rejected.

The production gait `build` wraps either selected source in `GaitContractAdapter`.
Its callable captures pre-step kinematics and routes RobotState through `update` to
the unchanged source. The source still owns PID scheduling, held output, loop and
conversion. Four separately tagged CurrentCommand values record requested and applied
current at the last controller update, modelled current at this physics step, and
already-converted layer-equivalent rotor current. These are simulated values, never
hardware measurements. `layer_current` rejects position/torque commands and all
non-layer current conventions, then unwraps without another conversion.

The adapter requires the actual feedback layer, positive dt and one layer substep.
After stepping, the iterator exposes `ctx['typed_state']` and `ctx['typed_currents']`.
Joint/shaft kinematics carry post-step time; electromagnetic output torque, belt
joint torque and joint-frame deflection carry step-input time. Existing CSV columns
and their historical timing conventions are unchanged. Electromagnetic torque is
read directly from LayerStep.tau_m in actuator-output Nm, without a ratio conversion.

The original controller classes, MuJoCo simulation, viewer and dashboards remain in
place. The simulation still consumes a scalar through its historical callable API;
the typed boundary is inside the production gait adapter. Direct iterator callers
with an unwrapped source remain supported. Position/torque command types are defined
but not wired into the original servo or torque-domain control paths. The live module
still imports its experiment runner; the controller still reads actuator feedback
from the existing layer. This is a compatibility slice, not backend independence.

## Implemented Phase 6D replay and remaining interfaces

[telemetry_replay.py](../oslbench/telemetry_replay.py) loads a versioned CSV/JSON
observation dataset using the existing Sample/RobotState types. ReplayRecord is an
envelope for sequence, group and clock identity, including explicit missing samples.
The core has deterministic iteration, reset, explicit group lookup and a virtual clock;
it has no physics, controller, command or visualization dependency. Seven existing
RobotState channels are supported; current/IMU observations are not yet in schema v1.
See [schema and API](TELEMETRY_REPLAY.md).

The package initializer now lazily resolves its existing public exports so data-only
imports do not trigger MuJoCo. No controller, simulation or contract equations changed.
An explicit [offline exporter](../tools/export_gait_telemetry.py) reads saved gait data,
writes only a new build directory and preserves pre/post-step times with source hashes.
No original trace is rewritten. Source provenance stays simulation when replayed.

Phase 6E adds browser observation playback through ReplayPresentation and a small
ReplayWebDashboard subclass reusing the existing server/event plumbing. One ReplaySource
clock drives table/plots, with explicit per-channel time, age and validity. Start/pause,
reset, timestamp-step, speed and EOF are presentation lifecycle only. The dataset lacks
a complete robot pose, so native MuJoCo viewing is deliberately not connected. No live
simulation/viewer/dashboard implementation changes are required. File ordering alone
does not establish synchronization.
Read-only hardware ingestion still needs actual protocols, acquisition-stage semantics,
calibration, clock alignment and fault definitions. The current controller adapter
continues to reject non-simulation inputs. No hardware command capability is introduced.

## Implemented Phase 6F-A source foundation

[TelemetrySource](../oslbench/telemetry_source.py) defines read-only metadata/status,
nonblocking poll(local_time_s) and close. It reuses the existing ReplayRecord/Sample
envelope and channel declarations; deterministic replay is not forced into this lifecycle.
MockTelemetrySource produces finite, explicitly synthetic observations with independent
source/arrival times, missing/invalid data and a scripted disconnection.

[TelemetryPresentation](../oslbench/telemetry_presentation.py) consumes arrival batches,
validates declarations and preserves Sample provenance. Shared channel formatting and
the existing browser page/server show source time and local receipt time separately.
Receipt-age STALE is an explicit configurable display policy, not synchronized source
age or hardware safety logic. No controller, command or simulation is involved. Replay
retains its original clock/lifecycle; only common display formatting is shared.

See [source API and mock demo](TELEMETRY_SOURCE.md). This is not a Raspberry Pi adapter:
transport, actual hardware channels, calibration, acquisition stages and synchronization
remain unresolved. No complete robot pose is synthesized from the mock observations.

## Separate modes

1. **Offline simulation, existing:** reference -> scheduled controller -> simulated plant -> MuJoCo -> full-rate records/metrics.
2. **Live simulation, existing:** the same execution -> native viewer and browser snapshots; UI events -> lifecycle coordinator.
3. **Telemetry replay, implemented data source:** timestamped file -> schema/unit/frame validation -> typed observations and virtual clock. Phase 6E browser dashboard consumes the same clock and observations; no native pose reconstruction or hardware commands.
4. **Hardware mirroring, future:** sensors -> read-only acquisition -> timestamps/calibration -> validated measurements -> display-only MuJoCo state/logging. Displaying measurements is not prediction.
5. **Predictive twin, future:** known inputs -> independently evolved simulation; synchronized observations -> residual comparison. Do not overwrite predicted state with each observation and call the result validation.

Replay/hardware boundaries must detect gaps, out-of-order samples, stale data, clock resets/skew, invalid units/frames and device faults. Hardware thresholds require actual acquisition requirements. Any future command transport must be separate from read-only acquisition, with documented arming, watchdog, stop and limit behavior.

The earlier audit found no IMU acquisition implementation. Phase 6F-B now adds a
passive MSCL filter reader and HTTP ingestion, tested without hardware; see below.
Physical encoder acquisition, serial/CAN actuator transport and hardware-to-MuJoCo
bridging remain absent. MyoAssist is parked. See [baseline provenance](WORKING_BASELINE.md).

## Phase 6F-B software boundary (hardware validation pending)

`microstrain_reader.py` uses official decoded MSCL filter fields. `imu_service.py`
owns a single reader worker, locked latest-channel cache and read-only HTTP endpoints.
`imu_telemetry.py` validates a separate HTTP schema. `HttpTelemetrySource` performs
network I/O in a worker and implements the existing nonblocking source protocol;
`HttpTelemetryPresentation` adds transport diagnostics to the existing browser UI.
No physics, command transport or second dashboard is introduced.

`telemetry_schema.CHANNELS` stays frozen for replay v1. A separate live registry
adds seven orientation components. Sample admits an additive, narrowly scoped
`imu_filter_ned`/`acquisition` case; RobotState and commands retain their original
constraints. Device timestamps remain separate from Pi and laptop receipt clocks.
The Pi deployment, MSCL native binding, actual stream, mounting and synchronization
still require physical checks. See [IMU deployment guide](IMU_TELEMETRY.md).
