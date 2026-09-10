# Validation: does the reorganised code still produce the validated result?

A reorganisation that changes the numbers is not a reorganisation, it is a new experiment.
This file records how that is checked, what the check has established, and the one part of
it that still has to be run on the Windows machine.

## The claim being defended

The benchmark result, produced by the original single-file implementation
(`experiments/bench_track_ab19.py`, now deleted; recoverable from git at `0cd33a5`):

| Metric | Value |
| --- | --- |
| RMS tracking error | 4.361613372292228 ° |
| peak tracking error | 9.090662089393883 ° |
| mean absolute error | 3.2798529494340207 ° |
| peak actuator torque | 16.004120587370423 N·m |
| torque authority used | 11.25465582796795 % |
| torque saturation | 0.0 % |
| peak knee velocity | 5.055899819703143 rad/s |
| steps | 2410 |
| reference clipped into ROM | 0 steps |

## How the comparison works

The old implementation's **actual output files** were frozen, not its summary numbers:

```
tests/oracle/bench_track_ab19.csv          2410 rows x 21 columns -- the full trace
tests/oracle/bench_track_ab19_metrics.csv  every metric, the audit fields, all 5 flags
```

`experiments/verify_against_oracle.py` re-runs the pipeline as it stands today, then
compares every column of every one of the 2410 rows, and every metric, against those files.
So the comparison is not "the headline numbers still look right" — it is a row-by-row,
column-by-column identity check on reference angle, actual angle, error, velocity, command,
torque, unclamped torque, percent authority, saturation flag and power.

The oracle lives in `tests/`, not `build/`, on purpose: `build/` is gitignored and is
overwritten by every run, so an oracle stored there would be destroyed by the first re-run.

**Tolerance.** Each column is compared at the resolution the CSV is *written* at — a
disagreement smaller than the last printed digit is not detectable in the file at all, so
that is the only defensible tolerance for a file comparison. In practice this is 5e-7 for
the degree and torque columns and 5e-9 for the radian columns, and the saturation flag must
match exactly. Metrics are compared at 1e-9 relative. The expectation is not "within
tolerance" but *bit-identical*: the reorganisation moved code between files and did not
change the order of a single MuJoCo call.

## The command

```powershell
.\.venv\Scripts\python.exe experiments\verify_against_oracle.py
```

It exits non-zero on any disagreement and prints which column or metric differs, at which
step, by how much. `--update-oracle` re-freezes the oracle and should only ever be used
deliberately, after a change that is *intended* to move the numbers.

## What has been established, and by what

**The reference half of the pipeline is confirmed identical.** This was checked by running
the current code against the oracle with the physics deliberately replaced by
`tests/stub_mujoco.py`, a small 2-DOF stand-in plant. Under that substitution the columns
that cannot depend on the plant came out matching the oracle exactly —

`time_s`, `gait_phase_percent`, `ref_rad`, `ref_deg`, `ref_vel_rad_s`, `ctrl_rad`,
`saturated`, `human_knee_angle_deg`, `human_knee_moment`, `human_knee_power`, every
`ref_*` audit field, and all five audit flags in their original order

— while the eleven plant-dependent columns and fifteen plant-dependent metrics differed, as
they must. That is a useful result in both directions: it confirms that CSV loading, column
validation, the audit, the cubic-spline resample, the phase computation, the ROM clip and
the writer all survived the move byte-for-byte, and it confirms that the comparison has
teeth rather than passing trivially. (The stub landed about 1.9 % away on RMS error —
4.2791° against 4.3616° — which is a sane ballpark and unmistakably not the experiment.)

**The control law and the wiring are confirmed by the test suite.** `tests/run_tests.py`,
44 tests, checks the PD equation against arithmetic performed independently inside the
test, that `KD` really is the ζ = 0.7 value for `KP = 600`, that both saturations fire in
both directions and that their flags agree with their values, that the four gain writes
reach the compiled model while `forcerange` and `ctrlrange` are left untouched, that the
step identity `tau_actuator == controller.torque(q_ref, q_pre, qdot_pre)` holds, and that
the reference contract holds (101 rows, phase in `[0, 100)`, period 1.2050 s, 2410 uniform
resamples). Result: 43 passed, 0 failed, 1 skipped — the skip is deliberate, a guard that
refuses to let a stub run be mistaken for validation.

**The protected artefacts are confirmed unchanged**, by sha256 asserted in the tests:

```
models/osl_v2_bench.xml            c417e691ff90a6f4a47ef581e6caaa341b0e1c829038f6d3e6740927ae69053d
build/AB19_knee_gait_reference.csv 83fed583b80c61c5bcd7978f2f2dd0870fe3c1efa25db68dbe9606b20da8a00d
```

No MyoAssist or `OpenSourceLeg_KA_L1` file was read or written by any of this, which the
tests also check by parsing every Python file in the repository and looking for such an
import or path.

## What still has to be run

**The full oracle comparison has not been executed against real MuJoCo.** The environment
this refactor was carried out in has no MuJoCo installed and no network to install it, so
the plant-dependent half of the comparison — the eleven columns and fifteen metrics that
depend on the physics engine, including every headline number in the table at the top of
this file — is checked by a script that has been written, exercised and shown to fail
correctly, but not yet seen to pass.

That gap closes with one command, run on the Windows machine in `.venv`:

```powershell
.\.venv\Scripts\python.exe experiments\verify_against_oracle.py
```

Until that has been run, the honest statement is: *the reference pipeline is proven
identical, the control law is proven correct, the artefacts are proven unchanged, and the
physics is expected to be bit-identical because no MuJoCo call moved — but expected is not
measured.*
