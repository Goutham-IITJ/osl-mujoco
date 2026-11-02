# Recorded knee step: derivative and parameter audit

The same 199 physical samples were replayed with the same gains (1.5, 0.15,
0.06), 200 Hz controller, 0.5 ms physics step, 0.5701 N m/A assumption,
initialization, and mechanical parameters. Only the derivative feedback signal
changed. Existing recordings, default controller and mechanical settings were
not changed.

| Knee-only fixture metric | C: actuator-angle differences | Knee-error differences |
|---|---:|---:|
| RMS simulation minus hardware (degrees) | 0.511719 | 0.511868 |
| Peak absolute difference (degrees) | 2.3291 | 2.3318 |
| 10–90% rise time (ms) | 72.697 | 72.492 |
| Peak angle (degrees) | 14.9973 | 15.0013 |
| Peak knee drive torque (N m, simulated) | 5.9109 | 6.2010 |
| Peak requested/applied current (A) | 22.2694 | 22.2694 |
| Clipping / belt fit exceedances | 0 / 0 | 0 / 0 |

The derivative choice does not explain the remaining discrepancy for this
fixture and this step. Hardware rise time was approximately 50.97 ms. No
periodic phase-lag metric is claimed for a single step.

## Controller arithmetic verification

As a separate offline arithmetic check, the shared CurrentPID was fed the
hardware CSV's recorded target, encoder angle and actual dt, with the CSV's
display-angle/sign convention accounted for. Maximum P/I/D-term discrepancy
over all 199 rows was 3.55e-15 A. This verifies the recorded controller law;
it does not mean the independent simulation uses measured angles as feedback.

The dynamic replay still uses fixed 5 ms controller intervals rather than the
hardware's slightly variable dt, and initializes only at the first recorded
encoder sample. These timing/initial-condition limitations remain explicit.

## Full-leg diagnostic

Knee-error derivative on the full-leg model produced oscillations, 640 clipped
physics steps and 1,317 physics steps outside the belt model's fitted deflection
range. Its RMS discrepancy was 4.655 degrees. Large torques from that run are
out-of-fit extrapolations, not credible hardware torque predictions. This is
not a reason to change hardware feedback; it highlights the dependence on
plant configuration. The C full-leg baseline remains preserved.

## Inertia and friction audit

| Quantity | Current value | Interpretation |
|---|---:|---|
| CAD moving-body inertia about knee | 0.00048137 kg m² | Reconstructed, not measured |
| Additional MuJoCo knee armature | 0.01 kg m² | Inherited assumption; about 21 times CAD body inertia |
| Drivetrain actuator-output inertia | 0.00983 kg m² | Explicit separate actuator state |
| Drivetrain inertia reflected to joint | 0.20310 kg m² | Rigid-belt limit only |
| MuJoCo joint damping | 0.3 N m s/rad | Inherited |
| Drivetrain reflected damping | 1.25207 N m s/rad | Rigid-belt limit only |
| MuJoCo joint friction loss | 0.4 N m | Inherited |
| Drivetrain reflected constant friction | 0.77727 N m | Excludes current-dependent friction |

These terms coexist in the equations. This is possible physical overlap, not
proof of double counting: joint losses could represent separate output-bearing
losses. Likewise, explicit actuator inertia and physical output assembly inertia
are distinct; the inherited armature's provenance needs clarification. Do not
remove terms merely to improve this one recording. The reflected values do not
replace the compliant two-state drivetrain equations.

Recommended next experiment: an isolated, documented sensitivity comparison of
the unverified joint armature/damping/friction, retaining the same controller
and physical recording. Use results to identify which measurements are needed,
not to declare a fitted parameter set validated. Confirm shaft meaning of the
torque constant and actual moving components before calibration.

## Reproduce

```powershell
.venv\Scripts\python.exe -B experiments\replay_knee_fixture.py --feedback legacy
```

`legacy` is the existing code name for knee-error derivative feedback, not a
request to use ankle gains. The experiment still selects knee gains. Omitting
`--feedback` uses C. Each invocation creates a new output directory.

Saved results: `build/knee_derivative_audit_20261031`; prior C results:
`build/knee_fixture_replay_20261031_results`. Parameter and arithmetic audit:
`build/knee_derivative_audit_20261031/parameter_audit.json`.
Five fixture tests passed after the experiment. No production default, hardware
file, physical parameter, or historical recording was changed.
