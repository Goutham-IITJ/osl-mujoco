# Tracking archive: initial static inspection

Source: `Downloads/Tracking Results & Codes & Dataset.zip` (11,552,867 bytes,
89 members). Archive hash, member list, source settings, CSV quality checks and
per-recording summaries: `build/tracking_inspection_20261103/inventory.json`.
Original Python/CSV members copied byte for byte under that folder's `originals/`.
No supplied Python was imported or executed, no simulation or hardware action
was performed, and existing models/controllers/results were not changed.

## Contents

- Five hardware tracking scripts: normal walking, ramp ascent, ramp descent,
  stairs down and stairs up. These contain homing and physical motor commands.
- Five AB01 reference datasets, each with an angle CSV and GRF CSV. These are
  desired human motion inputs, not measured robot output. They are distinct from
  the historical AB19 reference used in the simulation benchmark.
- Six result folders, each with knee and ankle trajectory CSVs and four combined
  plot PNGs. Normal walking has two recorded speed conditions but only one supplied
  code snapshot. Overall: 12 robot-result CSVs and 24 result PNGs.

## Recorded hardware tracking, not simulation agreement

| Result-folder suffix | Samples per joint | Last logged time (s) | Knee target RMS (deg) | Ankle target RMS (deg) |
|---|---:|---:|---:|---:|
| normal_walk_0.6ms | 2173 | 10.865 | 1.201 | 0.558 |
| normal_walk_1.2ms | 1086 | 5.430 | 4.566 | 1.616 |
| ramp_ascent_1ms | 744 | 3.720 | 3.307 | 1.610 |
| ramp_descent_1ms | 587 | 2.935 | 4.293 | 1.602 |
| stairs_down_0.35ms | 791 | 3.955 | 2.561 | 1.315 |
| stairs_up_0.31ms | 827 | 4.135 | 2.519 | 0.772 |

RMS computed from exported target minus actual angles. These are hardware-to-target
errors and must not be presented as hardware-to-simulation errors. Intervals are
approximately 5 ms, nominal 200 Hz; folder suffixes do not describe controller dt.
Both joints have identical loop-start time columns per run but different actual
sensor-read dt. Longest loop-start interval observed: approximately 9.20 ms.
Finite numeric result columns and strictly increasing time were checked. No
hardware clipping flags are set in any recording. Sensor validity, terminal abort
reason, sequence counters and measured current are absent; completeness cannot be
proved just from a monotonic CSV.

## Controller and reference details

All five source snapshots use knee gains 1.5/.15/.06, ankle 2/0/.075,
integral limit 50 degree-seconds, 200 Hz, KT .5701 and belt torque multiplier
4.61. Shared PID arithmetic using recorded targets/measurements and per-joint dt
reproduced logged P/I/D terms across all 12 files within 3.56e-15 A after sign
normalization. Clipping limit is explicitly 60 A in the supplied scripts. The
CSV torque column matches .5701 times commanded current; it is a derived value.
Torque constant shaft meaning and actual current dynamics remain unverified.

Normal/ramp scripts select `knee_angle_l` and `ankle_angle_l`. References undergo
stride selection, sign corrections, initial-angle shifts, optional fitting to
joint limits and cubic-spline interpolation. Stairs use explicit time windows
and a knee-maximum starting-phase option. The reference velocity is logged but
is not a feedforward input to the PID. Thus raw dataset columns are insufficient
to reproduce the exact motor command reference without reconstructing shaping.
For the first replay, the recorded `reference_deg` is the authoritative target.

Knee exported reference/actual angles are negated to display flexion-positive;
the exported error and reference velocity retain their internal signs. Ankle
angle columns retain their internal signs; motor/PID current terms use -1 sign.
Do not apply one blanket sign transform to every CSV column. Reference velocity
can be normalized independently for analysis; hardware measured velocity is not
included.

The normal-walking snapshot has TIME_SCALE=2 and five strides, consistent with
the slower recording. The faster recording needs its run-specific setting
documented rather than assuming this snapshot is identical. Stairs-up snapshot
also has TIME_SCALE=2, but the 4.135 s recording calls for a duration/stride-window
reconciliation before declaring exact source-to-run correspondence. Recorded
targets allow replay even while this provenance question is open.

GRF data contain NaNs, especially all-NaN stairs files. GRF is reference-processing
context, not a measured load from the OSL rig. Stairs scripts use explicit angle
windows rather than GRF heel-strike detection. Do not impose dataset GRF forces
on the physical-fixture simulation without a matching load experiment.

## Questions and constraints before replay

1. Physical arrangement: separate fixed knee/ankle fixtures, or an assembled leg?
   Both controller enable switches are on and the code addresses both motors,
   which does not establish their mechanical coupling. User clarification pending.
2. Ankle range/zero/direction: the CAD fixture range is -30 to +20 degrees, whereas
   several exported references exceed +20 (normal walk reaches +27.13; stairs down
   spans approximately -30.99 to +28.42). The supplied stairs-down safety limits
   are ±31. Resolve anatomical/encoder zero and true usable range. Do not silently
   clip references or broaden the physical model limits to hide the discrepancy.
3. Source timing: target is evaluated at shared loop-start time, measurement comes
   later within each joint's dt. Preserve both clocks and quantify alignment
   assumptions rather than relabeling loop-start time as encoder-read time.
4. Initial state: homing/hold precede CSV logging. Each starts at a nonzero angle;
   neither moving speed nor belt preload is measured. Initialization choices must
   be explicit and startup metrics separated from subsequent tracking.

## Bounded next work

First confirm mounting and ankle coordinate mapping. Then replay one normal-walk
condition using recorded target histories, unchanged gains, joint-error derivative,
and own simulation feedback; retain C as a diagnostic. Validate timing and ranges
before expanding to the six-condition matrix. Save overlays, raw timing errors,
phase lag separately from any lag-compensated errors, currents, torque and belt
fit-range flags. Preserve fixed-model results before any identification/tuning.
Use held-out recordings for later calibration validation. Full gait/terrain names
describe reference trajectories, not proof that terrain contact was tested.

Reproduce inspection (new timestamped output folder):

```powershell
.venv\Scripts\python.exe -B tools\inspect_tracking_archive.py
```

Inspection validation consisted of ZIP/member parsing, AST-only source parsing,
numeric CSV checks, time checks, PID-term reconstruction and torque-scale checks.
No simulation tests or hardware execution are claimed by this inspection.
