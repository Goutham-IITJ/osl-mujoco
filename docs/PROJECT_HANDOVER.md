# Project handover — OSL V2 MuJoCo

## Latest handoff milestone: 4 November 2026 — dual-joint telemetry package

User confirmed the continuous tracking recordings used the FULLY ASSEMBLED leg.
Do not use independent fixed knee/ankle fixtures as the matching gait plant.
User requested a bounded hardware-code handoff first, mindful of usage budget.
Deliverable ZIP: `build/osl_gait_tracking_telemetry_20261104.zip` (18 files,
CRC checked); final synthetic sample count was 599 per joint.
New `hardware/gait_tracking/`: all five supplied trajectory scripts with marked
telemetry startup/publish/shutdown additions, one shared standalone `telemetry.py`,
all reference CSV dependencies, README and source-hash manifest. Portable normal-
walk dataset paths are the only non-telemetry source changes; gains, guards,
timing settings, motion and logging algorithms preserved. Old knee module untouched.
Builder: `tools/build_tracking_telemetry_package.py` (does not overwrite README).
Protocol is `osl-joint-telemetry-v1` with both joint tags, global/per-joint sequences,
control tick, raw controller signs and separate reference/sensor-read times.
Use new `experiments/record_tracking_telemetry.py`; old knee receiver/simulation
cannot consume this stream. Recorder has no dashboards or simulation yet.
Local auth/channel/drop tests and all-five-controller source-preservation test
passed; synthetic end-to-end COMPLETE with 599 samples/joint in
`build/tracking_received_20261104_001414_067189`. First demo attempt failed due to
Windows coarse monotonic timestamps; synthetic generator corrected to perf_counter
and retested. Physical scripts already use perf_counter and were not affected.
No hardware script executed. Tests require loopback network outside sandbox.
Next: operator lab telemetry check, then assembled-leg replay/controller adapter
and dashboards. Ankle coordinate/range and actual assembled mount orientation
still need resolution before quantitative comparison. No model tuning/production
controller change/commit/push performed.

## Latest inspection: 3 November 2026 — continuous tracking archive

Inspected `Downloads/Tracking Results & Codes & Dataset.zip` statically; no supplied
hardware script imported/executed, no simulation/tuning. Report:
`TRACKING_ARCHIVE_INSPECTION_2026-11-03.md`. Evidence:
`build/tracking_inspection_20261103/` (original code/CSV copies, inventory/hashes,
recording summary). Reproducible tool `tools/inspect_tracking_archive.py`.
Five code snapshots, five AB01 angle/GRF pairs, six paired knee/ankle result runs:
normal walk at two speeds, ramps up/down, stairs up/down. All logs nominal 200 Hz;
folder suffixes `0.6ms` etc are not control dt. Fixed confirmed gains and logged
PID terms match within 3.56e-15 A; no recorded clipping flags. Distinguish hardware
target errors from sim-to-real errors; no tracking simulation results yet.
Blocking interpretation questions: separate fixtures versus assembled mechanical
leg (asked user), ankle zero/direction/range (several targets outside CAD +20 deg),
source snapshot/run time-scale correspondence. Preserve loop-start vs per-joint
measurement clocks, mixed exported column signs, nonzero homed initial pose.
Next: resolve setup/ranges and run one controlled normal-walking replay before
expanding to all conditions. No model defaults or hardware code changed.

## Paper preparation milestone: 2 November 2026 — Section 3 scope

Construction audit completed as a separate working document:
`PAPER_CAD_TO_MUJOCO_AUDIT.md`; numerical evidence and source hashes in
`baselines/paper_model_construction_20261102.json`. Recomputed segment aggregation
with a centroid-centred summation and compiled the current bench/knee/ankle XMLs
using MuJoCo 3.12.0 with `mj_forward` only. Tensors agree to numerical precision;
none of the three segments needed the builder's repair fallback. Rechecked local
MyoAssist XML mass totals with/without socket. Source/model hashes unchanged.
No full tests, physics stepping, model regeneration, tuning or hardware action.
Remaining: recover CAD version, verify actual lab component correspondence and
primary-paper comparisons; then prepare subsection 3.2 prose and audit drivetrain
parameter sources. Earlier statements below describing this audit as next work
are superseded by this entry.

Follow-up: user accepted the five-subsection outline in the working notes.
Reviewed the historical prior-art/research-direction notes and all three pages
of `Downloads/ICORR_2027.pdf`, including rendered pages. The PDF is an early
T-Motor-actuated hardware draft with template content; software Section 3 has
not yet been inserted. Prior-art map explicitly records a failed search and
cannot substantiate novelty. Added a proposed outcome-independent gain-evaluation
protocol: distinguish tuning from plant identification; fixed-gain recordings
cannot establish hardware-optimal gains. No sweep, simulation, primary-literature
search or hardware operation performed. Existing PDF/Overleaf content untouched.

User owns Section 3, Software and Sim-to-Real Framework, and may revise its
subheadings. Paper preparation is a separate chat from ongoing testing/analysis;
proceed gradually with implementation/source verification before manuscript prose.
See `PAPER_SECTION3_WORKING_NOTES.md` for the tentative structure, initial evidence,
open provenance questions and next bounded task. Read-only XML checks confirmed
raw URDF mass/inertia problems, current bench/fixture masses and both fixture
source hashes matching the current bench. Code inspection found the custom CAD
lineage in current step-replay paths and an implemented optional first-order
current-loop approximation; hardware identification remains unestablished.
No simulation, tests, model regeneration, tuning or primary-PDF verification was
performed in this paper-preparation pass. Next: detailed CAD pipeline audit and
equivalent-component MyoAssist cross-check before drafting subsection 3.2.

## Latest replay milestone: 2 November 2026 — ankle steps

Completed six ankle replays (5/10/15 degrees, joint-error D and C), confirmed
gains 2/0/.075. Report `ANKLE_MULTISTEP_2026-11-02.md`; outputs
`build/ankle_multistep_20261102_results/`. Primary RMS discrepancies:
0.143611/0.237994/0.379364 degrees; simulated rise 78.07/72.67/70.33 ms vs
hardware 52.82/49.59/48.63 ms. No clipping or belt-fit exceedance.
Ankle CSV current sign is -1 relative to displayed angle PID domain; normalized
term arithmetic matches within 1.78e-15 A. Original CSVs retained intact.
Shared stepping and fixture adapter now support joint selection without renamed
knee channels in ankle outputs. Confirmed gains are explicit in ankle context;
historical ankle profile remains unchanged. 17 tests passed. Knee regression
reproduced all 12 prior comparison/simulation CSV files byte for byte.
Next: physical ankle encoder zero/direction, measured load/current conversion,
and continuous trajectory validation. Transferred drivetrain and joint losses
remain provisional; positive CSV angle mapped to positive CAD coordinate is an
explicit assumption. Earlier pending-ankle-replay statements below are superseded.

## Latest model milestone: 2 November 2026 — ankle fixture

User confirmed ankle housing clamped above, prosthetic foot attached below,
normal leg orientation. Built `models/osl_v2_ankle_fixture_with_foot.xml` with
one ankle joint, no knee, fixed housing and full CAD foot/output segment.
Moving CAD mass 0.915159 kg; inherited joint losses remain provisional.
Guide `ANKLE_FIXTURE.md`; builder `tools/build_ankle_fixture.py --load with-foot`.
Shared preview supports `--model ankle-with-foot`. Render inspected at
`build/ankle_fixture_preview/neutral.png`; two ankle tests passed, including
inertia/rotation preservation and one-second finite physics stepping.
An unused output-only diagnostic variant was also built; it is not the lab setup.
Earlier requests for mounting information are superseded by this clarification;
physical encoder zero/sign and exact moving mass still require verification.
Next: ankle controller/drivetrain adapter and 5/10/15-degree replay with confirmed
2.0/0/.075 gains. No ankle controller replay or live validation completed yet.

## Latest milestone: 2 November 2026 — knee multi-amplitude replay

Completed fixed-model knee 5/10/15-degree comparison from `Downloads/All_Step_Results.zip`.
See `KNEE_MULTISTEP_2026-11-02.md`; reproducible command is
`experiments/compare_knee_step_archive.py`. Evidence: `build/knee_multistep_20261102/`.
All 199 rows per file retained; both C and knee-error D replayed, no parameter fitting.
Knee-error D RMS differences: 0.194868, 0.326615, 0.475019 degrees. Simulated rises
82.17/74.78/72.82 ms versus hardware 53.95/55.04/52.96 ms. New 15-degree trial
differs from the original physical recording; do not attribute different RMS to
a model improvement. Five fixture tests passed; plot inspected. CSVs lack original
telemetry validity/session flags; timing reconstructed from cumulative dt.
Confirmed gains: knee 1.5/.15/.06, ankle 2.0/0/.075. Ankle mounting photo pending.
Earlier table's statement that multi-amplitude comparison is pending is superseded
by this milestone. Next: verify physical parameters, continuous-trajectory validation;
no new parameter set adopted, defaults and historical recordings preserved.

Last reviewed: **2 November 2026 (Asia/Calcutta)**. This is the current continuity
entry point for a new agent/account. It supersedes older documents' project-status
and next-step claims, not their historical results. Recheck the working tree and
newer user instructions before acting. This is a handover, not a chat export.

## Purpose and present stage

We built a CAD-derived OSL V2 simulation, current-domain controller and compliant
actuator/belt model, then a read-only hardware telemetry pipeline to compare a
physical knee response against an independent MuJoCo response. The immediate work
is **offline comparison of recorded steps, investigating remaining discrepancies,
and preparing evidence for a research paper**. Whole-body walking and IMU-derived
gait control are not the current completed result.

The user is a **co-author**. On 2 November they clarified that they have the paper
structure but **have not started their own section**. No manuscript was identified
in this audit. Obtain the structure, assigned section and latest author guidance
before drafting. `PAPER_ARCHITECTURE_REPLY_2026-11-02.md` is a draft explanatory
reply and figure notes, not the manuscript. Sending any reply is a separate action.

Working repository: `C:/Users/Goutham A.S/Documents/GitHub/osl-mujoco`.
Related folders: sibling `AB19` (human reference data), `OSL Papers` (literature
PDFs), and `myo_folder` (parked human/MyoAssist work). The lab collaborator named
in recent discussion is Preetam; correspondence was being prepared for Saurav sir.

## Read in this order

1. This file: present state and next work.
2. [Hardware integration](../hardware/README.md) and [live comparison](LIVE_KNEE_COMPARISON.md).
3. [Knee fixture](KNEE_FIXTURE.md), [derivative audit](KNEE_DERIVATIVE_AUDIT_2026-10-31.md),
   and [parameter sensitivity](KNEE_FIXTURE_SENSITIVITY_2026-10-31.md).
4. [Paper architecture notes](PAPER_ARCHITECTURE_REPLY_2026-11-02.md) and
   [research briefing](RESEARCH_BRIEFING_2026-10-30.md) for explanation/history.
5. [Control paths](CONTROL_PATHS.md), [architecture](ARCHITECTURE.md), and
   [documentation audit](DOCUMENTATION_AUDIT_2026-11-02.md) as needed.

## What is established, and what remains open

| Work | Status and evidence |
|---|---|
| Original position-servo and AB19 benchmark | Preserved regression baseline; `tests/oracle/`. Human gait reference, not robot measurements. |
| Current-domain controller/drivetrain and dashboards | Implemented; older simulation reports indexed in `RESEARCH_INDEX.md`. |
| Physical knee telemetry | A 15-degree isolated-knee step was received, compared live and saved on 30 October. `build/hardware_step_20261030_134549_560/metadata.json` identifies source `physical`; 199 paired samples are in the saved comparison. This supersedes older blanket statements that no physical test exists. |
| Mounting correction | User confirmed on 31 October: lower knee plate fixed to desk, upper thigh attachment moving; no attached thigh, ankle or foot. Separate knee-only fixture created. Its CAD mass/inertia and inherited losses remain provisional. |
| Fixture comparison | Offline replay of the same 199 samples: RMS difference 1.195285 degrees (full leg) versus 0.511719 degrees (fixture); peak 5.211969 versus 2.329077 degrees. Saved in `build/knee_fixture_replay_20261031_results/`. Historical full-leg simulated angles reproduced exactly. |
| Remaining discrepancy | Fixture rise time 72.70 ms versus recorded hardware 50.97 ms. Fixture tail RMS 0.2520 degrees versus full-leg 0.1710: not every metric improved. Single step evidence, not gait validation. |
| Derivative audit | Fixture C versus knee-error D gave RMS 0.511719 versus 0.511868 degrees. Recorded P/I/D arithmetic matched to 3.55e-15 A in the audit. Does not establish identical independent dynamic responses. `build/knee_derivative_audit_20261031/`. |
| Parameter sensitivity | Ten baseline/one-at-a-time cases completed; no new parameter set adopted. `build/knee_fixture_sensitivity_20261031/`. |
| New step archive | Received 2 November: knee and ankle 5, 10, 15 degrees; 199 rows in each of six CSVs. No 20-degree CSV. Contents inspected; multi-amplitude simulation comparison not yet completed in the reviewed evidence. |
| IMU path | Separate passive acquisition implementation and hardware-free tests. No physical deployment validation established by this audit; do not confuse it with the successful knee encoder stream. |
| Paper | Structure exists with user; assigned section and structure location still need to be supplied. Architecture PNG/SVG/PDF are under `docs/assets/research_briefing/`. |

Numbers above were checked against saved reports/results, not recomputed in this
documentation audit. A 57% reduction in one RMS metric is not general model validation.

## Hardware telemetry: what actually runs

```text
Hardware host (Pi in the reported test)
  target -> physical PID -> CAN current command -> motor/knee
                 ^                                  |
                 +---------- I2C encoder ------------+
  existing loop values -> knee_telemetry.py bounded buffer -> HTTP JSON batches
                                                            |
Laptop: receiver -> raw recording ---------------------------+
        target -> independent PID -> actuator/belt -> MuJoCo
        encoder -> one-time angle initialization + comparison only
        paired source-time errors -> dashboards and CSV recordings
```

- `hardware/knee_step_test_PID_control_me.py`: complete supplied physical step
  controller with marked telemetry additions. Startup, per-tick publish and
  shutdown hooks; do not insert them a second time. It executes physical homing
  and motor control, so it is not a laptop test script.
- `hardware/knee_telemetry.py`: standard-library HTTP server inside that process,
  port **8794**, bounded **4000-tick** ring, source/session/sequence metadata.
  Publishes existing readings; no second encoder reader or motor-command endpoint.
- `oslbench/telemetry/knee.py`: laptop batch receiver. Polling approximately every
  **50 ms** retrieves multiple samples from a nominal **200 Hz** controller.
- `oslbench/apps/knee_telemetry.py`: hardware-only receiver/recorder, dashboard 8795.
- `oslbench/apps/live_knee_comparison.py`: integrated comparison; dashboards
  **8796** (motion/error) and **8797** (simulation diagnostics), optional native viewer.
- `oslbench/sim/live_knee.py`: source-time stepping using shared simulation/control.
  `oslbench/sim/knee_fixture.py`: separate one-joint plant adapter.

The physical loop provides source timestamp, actual dt, target, measured angle,
encoder validity, requested/clamped current and status. The initial telemetry
metadata uses a -1 multiplier to present flexion-positive angles; do not blindly
apply this again to exported CSVs that already use display signs. Commanded current
is not measured motor current, and reported torque columns are not torque sensors.

Live simulation uses a **0.5 ms physics step**, **200 Hz knee controller**, default
**100 ms delivery buffer**, and compares angles at original source timestamps.
Targets are held between source samples; angles are interpolated for comparison.
The hardware never waits for the laptop. No cross-machine clock synchronization or
hard real-time lockstep is claimed. Initial velocity/belt deflection are assumed zero.
Gaps after initialization, invalid sequence/session behavior, excessive sample
intervals or backlog cause explicit failure; missing target history is not invented.
Initial truncation and pending samples at exit must be checked in run metadata.

Pause display affects a browser view only. Stop laptop run saves/stops the laptop;
it **does not stop the physical controller**. Motor shutdown belongs to the hardware
operator/controller. A single successful test does not establish timing neutrality
of the telemetry thread, completeness of every recording, or calibrated plant fidelity.

Live comparison still selects the **full-leg plant**. The **knee-only fixture has
been compared offline**; there is no fixture-selection CLI flag in the current live
application. Do not describe the offline fixture result as a live fixture experiment.

The knee profile is Kp=1.5, Ki=0.15, Kd=0.06 (current/degree units), integral limit
50 degree-seconds, C feedback, 200 Hz, provisional 0.5701 Nm/A. Hardware uses
knee-error derivative; C uses actuator-side finite differences. The preserved ankle
profile on the knee bench is different (0.75/0/0.03, 1 kHz, 0.855 Nm/A). Selecting
`--controller ankle` does not create an ankle plant. Simulation clips are not physical
safety limits. Torque/current shaft meaning, moving inertia, friction and actual
variable hardware dt remain sources of uncertainty.

## Next work: new multi-amplitude data

Original archive: `C:/Users/Goutham A.S/Downloads/All_Step_Results.zip`.
A byte-identical continuity copy is under `data/hardware/2026-11-02/` with
`manifest.json` (archive and CSV hashes, row counts, provenance). Preserve originals.
This archive's 15-degree recording is not automatically the same trial as the
30 October live recording; compare hashes/timestamps and provenance before claiming so.

1. Validate all six files: timestamps/dt, signs/zeros, clipping, controller terms,
   test identity, sample coverage and physical setup. Ankle P/error magnitudes
   indicate Kp=2 A/degree, different from the preserved ankle-derived profile;
   verify the full controller rather than guessing Ki/Kd or current polarity.
2. Implement a documented CSV-to-replay input adapter for the three knee files.
   Existing `experiments/replay_knee_fixture.py` expects `metadata.json`,
   `hardware_batches.jsonl` and an original `comparison.csv`; it cannot consume
   this ZIP or a plain hardware CSV directly. CSVs lack telemetry session/sequence
   and validity fields: any row indices or validity assumptions must be labelled
   as import metadata, not fabricated hardware observations.
3. Generalize analysis before using it on 5/10-degree steps: the existing replay
   metrics hardcode **1.5 and 13.5 degrees** for rise time and a **0.5 s** tail.
   Derive thresholds from each step's initial/final target, identify step onset,
   and explicitly handle missing crossings. Keep the original 15-degree result.
4. First compare all knee recordings with one unchanged fixture/controller baseline.
   Report RMS/peak difference, rise time, overshoot, final/tail error, current and
   timing. Label C vs hardware-style derivative separately. Preserve raw timing;
   do not shift curves to improve apparent agreement.
5. Confirm ankle fixture, moving load, coordinates and controller before ankle
   simulation. The implemented live packet/receiver is knee-specific; dual-joint
   live comparison has not been demonstrated.
6. If calibration is justified, decide calibration/validation allocation before
   tuning. The historical 15-degree test has already informed model choices, so
   it is not untouched validation data. Reserve a genuinely independent trial
   or obtain another one for a strong validation claim.

User preference expressed in the recent chat: use recorded data for fast, repeatable
model iterations; use live hardware for new trajectories/setup changes and final
validation. Do not start hardware merely to recreate an existing recording.

## Resume commands and checks

Use existing `.venv` from repository root in PowerShell. Do not upgrade it while
reproducing evidence. The README contains environment versions and setup limitations.

```powershell
git status --short
.venv\Scripts\python.exe -B experiments\replay_knee_fixture.py --help
.venv\Scripts\python.exe -B experiments\replay_knee_fixture.py
# Optional controlled derivative comparison, in a separate new output directory:
.venv\Scripts\python.exe -B experiments\replay_knee_fixture.py --feedback legacy
```

Replay defaults to the preserved 30 October recording and creates a new timestamped
folder. It does not process the new ZIP. Hardware/synthetic launch procedures are
in `LIVE_KNEE_COMPARISON.md`; start the laptop first and wait for READY. Use fresh
output folders. Never use a real token in checked-in instructions.

For implementation changes, relevant checks include:

```powershell
.venv\Scripts\python.exe -B -m unittest discover -s tests -p test_knee_fixture.py -v
.venv\Scripts\python.exe -B -m unittest discover -s tests -p test_knee_live_connection.py -v
.venv\Scripts\python.exe -B -m unittest discover -s tests -p test_live_knee_comparison.py -v
.venv\Scripts\python.exe -B tests\run_tests.py
```

Inspect the engine banner: a stub pass is not MuJoCo physics verification.
These are resume instructions, not a claim that the suites ran during this audit.

## Preservation before changing accounts

At audit start HEAD was `4f9eb68` on the local working repository, with numerous
existing modified/deleted/untracked files. Important fixture, live comparison,
telemetry and test files were untracked. **HEAD alone does not represent this work.**
Retired root redirectors were already deleted; preserve that in-progress state.
See `docs/baselines/continuity_20261102.json` for the captured status and file hashes.

- Retain the complete working folder, including untracked code/models and Git
  history. Review/commit the intended working changes after coordinating with
  active development chats; this audit does not stage, commit or push them.
- Make a separate backup of `build/`, especially the physical recording, fixture
  replay, derivative/sensitivity evidence and `AB19_knee_gait_reference.csv`.
  They are generally Git-ignored. **A manifest is not a backup.**
- Retain CAD/mesh assets (`osl_v2_0_assembly/`, `models/`), oracle, external data,
  original hardware CSVs/controller version, and paper structure/draft when supplied.
- The archive copy is inside the working folder, not a second-device backup.
  Verify external backup contents and hashes before relying on it. No account
  migration or external backup was performed by this documentation task.

Suggested first prompt on the new account:

> Read AGENTS.md and docs/PROJECT_HANDOVER.md, check the current Git status and
> available evidence, and summarize the current stage and unresolved assumptions.
> We are continuing recorded knee-step comparison and paper preparation. Preserve
> existing work and historical outputs. Then follow my next instruction.

Keep this file current as work proceeds. Documentation preserves technical context;
it does not guarantee that original app chats will be visible after account switching.
