# Knee fixture: inherited joint-parameter sensitivity

Ten offline replays used the same 199 physical samples, knee gains,
configuration C, 200 Hz control rate, 0.5 ms physics step and drivetrain.
One MuJoCo joint parameter was changed at a time to zero, half or twice its
inherited value. Zero is a diagnostic removal, not a claim of zero physical
loss or rotor inertia. Explicit drivetrain inertia/friction were retained.
Only in-memory models were changed; XML hashes were checked unchanged.

| Case | RMS difference (deg) | Rise time (ms) | Tail RMS difference (deg) | Peak knee drive torque (Nm) |
|---|---:|---:|---:|---:|
| Baseline | 0.5117 | 72.70 | 0.2520 | 5.91 |
| Joint armature zero | 0.4825 | 74.11 | 0.2080 | 1.58 |
| Joint armature half | 0.4919 | 73.81 | 0.2199 | 3.65 |
| Joint armature doubled | 0.5434 | 68.83 | 0.2945 | 9.79 |
| Joint damping zero | 0.5022 | 68.65 | 0.3022 | 5.59 |
| Joint damping half | 0.5090 | 70.48 | 0.2810 | 5.74 |
| Joint damping doubled | 0.5329 | 75.11 | 0.2105 | 6.25 |
| Joint frictionloss zero | 0.5097 | 71.13 | 0.2818 | 5.19 |
| Joint frictionloss half | 0.5124 | 71.90 | 0.2707 | 5.56 |
| Joint frictionloss doubled | 0.5135 | 74.33 | 0.2143 | 6.59 |

Baseline armature: 0.01 kg m²; damping: 0.3 Nm s/rad; frictionloss:
0.4 Nm. Rise time is the first interpolated 1.5-to-13.5-degree crossing;
tail starts at source time 0.5 s. Torque is simulated, not measured.

All ten runs completed with no current clipping and no belt fit-range
exceedances. The baseline paired and full-rate simulation CSVs reproduce
the prior knee-fixture replay byte for byte. Model XML remains unchanged.

Removing joint armature reduced RMS by about 5.7%, but increased rise time.
None of these individual changes approached the measured 50.97 ms rise time.
Therefore these tests do not justify adopting a new parameter set. They also
do not establish that a combination of changes or other parameter values
would have no effect. No parameter optimization or physical calibration was
performed.

Next priorities: confirm actuator-output inertia and torque/current conversion
against hardware documentation, verify actual moving components and mass,
and obtain additional step amplitudes/directions with recorded current and
timing. Investigate timing and current dynamics separately from mechanical
parameters. Keep baseline settings until those checks support a change.

## Reproduce

```powershell
.venv\Scripts\python.exe -B experiments\knee_fixture_sensitivity.py
```

Each invocation creates a new folder. `--recording` selects the saved telemetry
directory; `--outdir` must name a new directory. Results for this study are in
`build/knee_fixture_sensitivity_20261031`: summary CSV/JSON plus each case's
comparison CSV, full-rate simulation CSV and metadata with exact runtime
parameter values and source/model hashes. No hardware connection is needed.
