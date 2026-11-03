# Ankle fixture replay: 5, 10 and 15 degrees

The ankle-only housing is fixed above the attached foot, per user clarification.
No shin rod, knee or ground contact is included. The fixed housing is rigid;
clamp flexibility is not represented. The complete CAD foot/output mass is
0.915159 kg. Neither moving mass nor friction has been measured on the lab rig.

## Fixed experiment settings

- Confirmed ankle gains: Kp 2 A/degree, Ki 0, Kd 0.075 A/(degree/second).
- 200 Hz controller inferred from recorded approximately 5 ms intervals; fixed
  0.5 ms physics step and 10 physics steps per controller update.
- Joint-error derivative is the primary comparison, with first derivative suppressed.
  Configuration C (actuator-angle differences) is a separate diagnostic.
- Shared CurrentPID, GaitCurrentSource, current conversion and DrivetrainLayer;
  no separate PID or belt algorithm was introduced. Shared FixtureSimulation and
  source-time stepping now select joint addresses and output labels explicitly.
- Provisional 0.5701 Nm/A actuator-output interpretation; ideal current loop;
  60 A simulation clip is not an established hardware limit.
- CAD belt ratio 50/11. Hardware CSV equivalent-torque convention uses 4.61;
  these are not silently conflated. CSV torque is derived, not measured.
- Existing paper-derived actuator inertia, friction and belt fit transferred to
  this ankle fixture; not experimentally identified here. MuJoCo joint armature
  0.008 kg m², damping 0.25 Nm s/rad, frictionloss 0.3 Nm retained.
- Original ankle-derived profile (.75/0/.03) remains unchanged. New confirmed
  gains apply only to this explicit recorded-data ankle experiment.

## Results: joint-error derivative

| Step | RMS sim/hardware difference (deg) | Peak difference (deg) | Rise hardware / sim (ms) | Peak ankle drive / actuator-side belt torque (Nm) | Peak requested/applied current (A) |
|---|---:|---:|---|---|---:|
| 5° | 0.1436 | 0.5690 | 52.82 / 78.07 | 3.05 / 0.67 | 9.88 |
| 10° | 0.2380 | 1.1087 | 49.59 / 72.67 | 5.64 / 1.24 | 19.75 |
| 15° | 0.3794 | 1.7308 | 48.63 / 70.33 | 8.35 / 1.84 | 29.92 |

Configuration C RMS values: 0.1567, 0.2409 and 0.3881 degrees. It produces faster
rises (71.84, 67.97, 66.20 ms) and smaller peak discrepancies, but worse tail
agreement in all three tests. Neither choice removes the transient discrepancy.
No new tuning or physical parameter optimization was performed.

Six replays retained all 199 rows per recording. No simulation current clipping
or belt-fit exceedance; all recorded hardware clipping flags false. All torque
results are model predictions. No periodic phase lag is assigned to a single step.

## Sign, timing and provenance

The ankle CSV angle is already display-positive. Its logged P/I/D currents use
the opposite sign: multiplying each by -1 reproduces shared PID terms to a maximum
error of 1.78e-15 A using the actual recorded dt. The same -1 conversion is applied
to recorded requested/commanded currents for comparison in the positive simulation
coordinate. Originals are copied intact. This arithmetic check supports the gain/
current convention; physical encoder zero and anatomical motion polarity still
need verification against a rig photo/video or controller source.

Hardware time is reconstructed from cumulative dt, as in the knee CSV replay.
Original telemetry timestamps/session/sequence/validity are absent. Finite encoder
values are assumed usable and row order supplies synthetic sequence numbers.
Only the first measured angle initializes the simulated joint; subsequent hardware
angles never close its control loop. Velocity and belt deflection initialize to
zero. The controller uses fixed 5 ms updates rather than hardware timing jitter.
The positive steps are assumed to map to positive CAD ankle dorsiflexion; if the
physical zero/polarity differs, its gravity loading must be rechecked.

## Reproduce and evidence

```powershell
.venv\Scripts\python.exe -B experiments\compare_knee_step_archive.py --joint ankle
```

The existing experiment accepts joint selection to keep CSV ingestion, arithmetic
checks, plots and metrics common. Each invocation creates a new output folder.
Results: `build/ankle_multistep_20261102_results/`, including summary CSV/JSON,
comparison.png, original CSV copies and per-feedback full-rate simulation,
comparison and provenance metadata. Earlier first-pass outputs remain intact.

Validation: 3 ankle tests, 5 knee fixture tests and 9 live-comparison tests passed
(17 total). The full knee multi-step replay was repeated into a new directory;
all 12 knee comparison/simulation CSV files reproduce historical bytes exactly.
The six-panel ankle figure was visually inspected and its joint label corrected.
No physical hardware commands were executed and no production default changed.

Next: confirm ankle zero/direction and exact moving assembly, verify torque/current
and drivetrain parameters, and compare continuous trajectory recordings. Strong
steady agreement alone does not resolve the slower simulated rise or validate
the model's predicted torque/current dynamics.
