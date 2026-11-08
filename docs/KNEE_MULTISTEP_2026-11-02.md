# Knee validation at 5, 10 and 15 degrees

All three knee CSVs in `All_Step_Results.zip` were replayed with the same
knee-only plant. No mechanical parameters or gains were fitted. Confirmed
gains: Kp 1.5 A/degree, Ki 0.15 A/(degree second), Kd 0.06 A/(degree/second).
Controller rate 200 Hz, physics timestep 0.5 ms, provisional constant
0.5701 Nm/A, integral limit 50 degree-seconds, ideal simulation current loop.

| Step | RMS difference, knee-error D / C (deg) | Peak difference, knee-error D (deg) | Rise time, hardware / simulation knee-error D (ms) |
|---|---|---|---|
| 5° | 0.1949 / 0.2004 | 0.8712 | 53.95 / 82.17 |
| 10° | 0.3266 / 0.3245 | 1.5556 | 55.04 / 74.78 |
| 15° | 0.4750 / 0.4755 | 2.3720 | 52.96 / 72.82 |

The same provisional model gives bounded responses across these three amplitudes,
but consistently rises more slowly than hardware. This is evidence beyond the
original 15-degree recording; it is not full plant or gait validation. The new
15-degree CSV is a different trial from the original telemetry run: its RMS
must not be presented as an improvement caused by another model change.

There are 199 samples per trial, all retained in each of six replays (two feedback
choices). No simulated clipping or belt-fit exceedance occurred; recorded
hardware clipping flags are all false. Shared PID arithmetic fed recorded
angles and dt reproduced each file's P/I/D terms to within 3.56e-15 A. This is
separate from the dynamic simulation, which uses its own feedback.

Timing uses cumulative actual dt to reconstruct encoder-read times; CSV time_s
is the earlier loop-start timestamp in the supplied implementation. Exact original
telemetry timestamps/session/sequence/validity are absent. Row order supplies
synthetic sequence numbers; finite CSV angles are assumed usable, not sensor-
validated. Initial pose uses only the first measured angle; velocity and belt
deflection start at zero. No pre-step baseline was fabricated. Rise time uses
the first crossings of 10% and 90% of the nominal target, assuming a zero starting
reference; no periodic phase lag is claimed.

Outputs: `build/knee_multistep_20261102/`: original knee CSV copies, each feedback
case's full-rate simulation and comparison CSVs, metadata/provenance, summary
CSV/JSON and `comparison.png`. Source archive is read without extraction or edits.

```powershell
.venv\Scripts\python.exe -B experiments\compare_knee_step_archive.py
```

Every invocation writes a new timestamped folder. `--archive` selects a ZIP and
`--outdir` must be new. Existing 15-degree history and production defaults remain
unchanged. Five fixture tests passed. Full output summaries were inspected and
the six-panel comparison figure was visually checked.

Next: retain the fixed baseline while confirming physical inertia/current
conversion and obtaining continuous trajectory data. Use separate calibration
and held-out recordings if parameters are fitted later. Ankle work awaits mounting
information; confirmed ankle gains are 2.0, 0.0, 0.075, distinct from the preserved
historical ankle-derived profile.
