# Deterministic timestamped telemetry replay

Phase 6D implements a data source, not a simulation or hardware controller. It uses
existing `Sample` and `RobotState` contracts. It neither imports MuJoCo nor advances
physics, calls controllers, issues commands or changes the source files.

## Version 1 dataset

A directory contains `samples.csv` and `manifest.json`. See the complete
[synthetic manifest](../tests/fixtures/telemetry_replay/manifest.json) and
[synthetic CSV](../tests/fixtures/telemetry_replay/samples.csv).

The JSON sidecar requires:

| Field | Meaning |
|---|---|
| `schema_version` | Integer `1`; other versions rejected |
| `dataset_id` | Nonempty dataset/run label |
| `source` | `simulation`, `measured`, `estimated`, `derived` or `replayed` |
| `clock` | `{ "id": "<actual source clock identity>", "unit": "s" }`; one clock per dataset |
| `sequence` | Literal `contiguous_zero_based` |
| `data_file` | Sibling CSV filename; no parent/absolute paths |
| `record_count`, `data_sha256` | Expected row count and exact CSV byte hash; detect truncation/change |
| `channels` | Declared channel metadata, described below |
| `identity` | Optional object containing actual source/input/calibration identity; never fabricated |

Channel metadata contains `quantity`, `unit`, `frame`, `stage`, `source`,
`timestamp` (literal `time_s`) and a nonempty `sign` convention description.
The reader validates the version-1 channel registry:

| Channel | Quantity | Unit | Frame |
|---|---|---|---|
| `knee_angle` | angle | rad | knee_joint |
| `knee_velocity` | angular_velocity | rad/s | knee_joint |
| `actuator_output_angle` | angle | rad | actuator_output |
| `actuator_output_velocity` | angular_velocity | rad/s | actuator_output |
| `electromagnetic_output_torque` | torque | Nm | actuator_output |
| `belt_joint_torque` | torque | Nm | knee_joint |
| `belt_deflection` | angle | rad | knee_joint |

These are the existing RobotState fields. Version 1 deliberately does not include
current observations, reference trajectories, IMU signals or command output. Future
channels need explicit documented semantics; unknown names are rejected. Channel
provenance can differ from the dataset's umbrella provenance. Replaying a simulation
retains `simulation`; playback does not turn it into a hardware measurement.

CSV columns, in exact order, and an example synthetic record:

```csv
sequence,snapshot_id,channel,time_s,value,valid
0,0,belt_deflection,0.0,0.01,1
```

Every row is one observation, with its own source timestamp in seconds. `sequence`
is a contiguous zero-based **file record number**, not an invented hardware packet
counter. `snapshot_id` is a nonnegative integer grouping related observations; a group
need not be simultaneous. A snapshot/channel pair must be unique. Sequence and group
IDs are retained in `ReplayRecord` envelopes around existing Samples.

`stage` uses the existing Sample contract: `pre_step`, `post_step`, `held_command` or
`step_input`. A source with different acquisition semantics needs an explicit additive
contract/schema extension; do not relabel it to fit. Clock metadata is retained in the
record envelope because Sample itself has no clock-ID field. The manifest property
returns a detached metadata copy preserving signs, provenance and source identities.

## Data quality policy

- Timestamps must be finite, nonnegative and globally nondecreasing in file order.
  Equal timestamps are allowed, including distinct signals; sequence breaks ties.
  Timestamp order does **not** prove physical clock synchronization.
- Duplicate, skipped or out-of-order file sequences are rejected. Physical gaps are
  allowed and remain gaps; no nominal sensor rate is inferred.
- Valid values are finite numbers and `valid=1`. A finite but invalid observation uses
  `valid=0` and produces a Sample with `valid=False`.
- A missing value is exactly an empty `value` cell with `valid=0`. Its record preserves
  timestamp/channel metadata but has `sample=None`. No fictitious zero is constructed.
- Missing timestamps, NaN/infinity (even when invalid), unknown channels, incompatible
  units/frames, malformed CSV/JSON, duplicate JSON keys, count/hash mismatch and unknown
  schema fields are rejected with `DatasetError`.
- A dataset may declare only a subset of supported channels. Missing channels remain
  absent. Zero-row datasets are valid and immediately ended if count/hash agree.
- No interpolation, forward-fill, calibration conversion, automatic sorting or dropping
  of invalid input occurs in the reader. The explicit gait exporter sorts its newly
  constructed observations; the reader never repairs an out-of-order dataset.

## API and virtual clock

From the repository root, with Python on the environment path:

```python
from oslbench.telemetry_replay import ReplaySource

replay = ReplaySource("tests/fixtures/telemetry_replay/manifest.json")
for record in replay:
    print(record.sequence, record.channel, record.time_s, record.sample)
assert replay.ended
replay.reset()                         # cursor and virtual clock return to zero
batch = replay.advance_to(0.001)        # all not-yet-emitted records <= this time
state = replay.robot_state(0)           # explicit random access by group ID
```

Iteration advances the virtual delivery clock to the next observation without sleep.
`next` raises `StopIteration` at end; `advance_to` returns an empty tuple if nothing is
due, including after end. Clock reversal is rejected; use reset. A reset emits the same
records in the same order. Source timestamps never change with delivery-clock time.

`robot_state(id)` does not use the replay cursor: it is explicit whole-dataset access.
It returns None if any required kinematic sample in that group is missing/invalid,
raises KeyError for an unknown group, and otherwise returns RobotState with each
field's original timestamp. Optional missing signals remain None; optional invalid
samples retain their validity. It never borrows a signal from another group. A future
streaming viewer must choose delivery/availability policy explicitly rather than using
random access as though all observations had already arrived.

## Offline export of existing gait evidence

The exporter reads an existing full-precision trace and its metadata. Use a NEW output
directory under `build/`; existing directories are refused:

```powershell
.\.venv\Scripts\python.exe -B tools\export_gait_telemetry.py --trace build\integrated_current_gait_phase5\visible_live_run\attempt_001\gait_trace_full_precision.csv --metadata build\integrated_current_gait_phase5\visible_live_run\attempt_001\run_metadata.json --outdir build\my_new_gait_replay
```

The exporter never starts MuJoCo. It converts knee degrees to radians, retains the other
six values without rescaling, and records original file hashes and run metadata.
Knee/actuator kinematics have physical time `row.time_s + physics_dt_s`; force and
belt-deflection samples have `row.time_s`. Actuator-output electromagnetic torque is
already output-side Nm and is not multiplied or divided by gearing. The original CSV
and its historically mixed timing convention remain untouched. Snapshot IDs are source
row indices; exported sequence numbers reflect chronological observation order.

The Phase 6D [converted dataset](../build/phase6d_replay/phase5_dataset/manifest.json)
contains 16,870 observations and reconstructs 2,410 states from saved Phase 5 evidence.
[Verification](../build/phase6d_replay/saved_trace_verification.json) checks all seven
channels exactly after the explicit angle conversion, mixed timestamps and reset
repeatability. This is simulation evidence, not physical telemetry validation.

## Tests and presentation boundary

```powershell
.\.venv\Scripts\python.exe -B tests\run_tests.py -k replay
.\.venv\Scripts\python.exe -B tests\run_tests.py
```

Tests use the synthetic fixture plus temporary malformed variants. A fresh-process
import test blocks even attempted imports of MuJoCo and NumPy while loading/iterating
replay. Package-level historical exports now resolve lazily; a compatibility test
checks their object identities. Controller and simulation modules are unchanged.

The source eagerly loads/validates the dataset in memory; it is not a streaming parser,
wall-clock scheduler or multi-clock synchronizer. Phase 6E adds the separate browser
presentation scheduler described below; the replay core remains unchanged. Hardware
acquisition, physical calibration and predictive comparison remain future work. The
[roadmap](NEXT_PHASE_PROPOSAL.md) is authoritative.

## Phase 6E browser replay

From the repository root:

```powershell
.\.venv\Scripts\python.exe -B experiments\run_telemetry_replay.py --manifest build\phase6d_replay\phase5_dataset\manifest.json --speed 0.25
```

This opens the browser dashboard on loopback port 8791, initially paused. Use `--port 0`
for a free port, `--no-browser` to open the printed URL manually, or `--autoplay` to
start immediately. `--exit-on-end` closes the server at EOF; by default it stays open
at the final recorded state so you can inspect it or reset. Closing only the browser
does not stop the process. Use Stop server or Ctrl+C to exit. Optional `--outdir` must
name a NEW directory below `build/phase6e_replay/`; it saves launch identity, HTML page
source and final presentation JSON, not a new simulation trace or standalone offline player.

| Control | Behavior |
|---|---|
| Play / pause, Space | Start/resume or freeze the single virtual clock and displayed state |
| Reset, R | Reset ReplaySource, clear displayed history/values, return paused at time zero |
| Next timestamp, N | While paused, deliver every observation at the next timestamp; no interpolation |
| 0.25x / 0.5x / 1x / 2x | Change wall-time-to-virtual-time pacing only |
| Stop server | Stop and close the presentation service |

`ReplayPresentation` wraps the existing ReplaySource. Its `advance(elapsed_s)` uses
`ReplaySource.time_s` as the only authoritative virtual time. The launcher supplies
monotonic wall-clock deltas and processes elapsed time before queued UI actions.
The dashboard consumes the same immutable-record-derived presentation snapshot for
its table and all plots; it never runs a separate replay engine. Changing browser
poll/render frequency cannot create, reorder or rewrite observations. EOF clamps
virtual time to the final source timestamp and never automatically restarts.

The table shows all declared channels, original units/frame, provenance, stage,
sequence/snapshot ID, source timestamp and age relative to virtual time. No second
angle or gearing conversion occurs. UNAVAILABLE means nothing has arrived; MISSING
means the latest explicit observation has no sample; INVALID retains a recorded
invalid value with its quality label but plots no valid point. A retained older value
is labelled HELD with its original timestamp/age. No unverified freshness threshold
is invented and no retained value becomes a new observation. Reset clears retention.

Plots use dots at actual valid observations in each channel's native unit versus
source seconds. At most 320 recorded observations per channel are selected for display,
including endpoints; no line interpolation or smoothing is used. Full event delivery
and latest-value semantics are independent of plot decimation. Source/run metadata is
available in the expandable identity panel. The saved Phase 6D dataset is explicitly
labelled RECORDED SIMULATION TELEMETRY, NOT PHYSICAL HARDWARE DATA. Other dataset
provenance is shown as declared and is not asserted to be hardware-verified.

The existing DrivetrainWebDashboard HTTP server, event queue, browser opening and
shutdown methods are reused through ReplayWebDashboard. Live dashboard pages/update
logic and the canonical current-gait launch remain unchanged. A replay-specific page
is needed to represent missing values and distinct source timestamps honestly.

This phase deliberately uses the browser surface, not the native MuJoCo viewer. The
seven-channel dataset has no ankle pose or full robot configuration; actuator-output
shaft state also does not identify an independently rendered shaft joint in the bench
model. The native live viewer is coupled to simulation stepping. No full pose is
fabricated, no MuJoCo model is loaded, and no mj_step/kinematic refresh or controller
is called. Current observations, reference angle/error, ankle/IMU data and physical
hardware remain unsupported rather than synthesized.

Focused integration checks are included in `tests/run_tests.py -k replay` (or use
`-k replay_ui` for just the seven presentation tests). See the
[Phase 6E report](../build/phase6e_replay/REPORT.md) for actual launch/control checks,
preservation evidence and full-suite results. Next work is the hardware-interface and
calibration specification needed for a read-only acquisition adapter; no physical
connection or predictive comparison is implemented here.
