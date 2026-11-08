# Assembled-leg continuous tracking replay

Completed all six recordings with both joints on one coupled fixed-base model.
Gains unchanged: knee 1.5/.15/.06, ankle 2/0/.075; 200 Hz control, 0.5 ms physics,
provisional .5701 Nm/A and existing drivetrain parameters. Configuration C is used
for the completed results. No physical parameter fitting or phase shifting of
the plotted traces was performed.

| Recording | Pairs per joint | RMS knee / ankle (deg) | Peak knee / ankle (deg) | Exploratory best-fit lag knee / ankle (ms) |
|---|---:|---|---|---|
| Slow walking | 2173 | 0.762 / 0.237 | 2.213 / 1.220 | 10 / 0 |
| Faster walking | 1086 | 2.272 / 0.662 | 6.093 / 2.948 | 10 / 0 |
| Ramp ascent | 744 | 1.351 / 0.505 | 3.546 / 1.833 | 5 / 0 |
| Ramp descent | 587 | 2.381 / 0.691 | 7.793 / 2.358 | 5 / 0 |
| Stairs down | 791 | 1.277 / 0.675 | 3.515 / 2.212 | 5 / 5 |
| Stairs up | 827 | 1.090 / 0.330 | 3.282 / 1.270 | 5 / 0 |

These are simulation-versus-recorded-hardware differences, not errors relative
to the target. All six C runs are complete with no simulated current clipping
or belt-fit exceedance. Positive lag means simulated motion later than hardware;
lag uses a fixed-window exploratory fit at 5 ms resolution, not a network delay
measurement. Raw errors are not lag-compensated. Faster reference motion gives
larger discrepancies, particularly at the knee.

## Conditions on interpretation

The ankle joint range was explicitly expanded in memory to ±31 degrees using
`--ankle-range hardware`, because the recorded references exceed the authored
CAD -30/+20 range. This is an unverified operating-envelope/coordinate assumption,
not an identified correction. No source XML was edited. The nominal upper knee
body is fixed to world and no ground/GRF loading is applied; physical root mount,
anatomical zero and rotation polarity still need confirmation. Initial velocities
and belt preload are assumed zero. CSV sensor timing is reconstructed from actual
per-joint dt while reference timing retains loop-start timestamps. Finite CSV
angles are assumed usable because original validity flags are absent.

The physical PID uses joint-error derivative. C uses sampled actuator-side
differences. Thus these are matching-gain, different-feedback comparisons. The
corrected hardware-style D slow-walking diagnostic diverged around 0.58 s and
was stopped at 0.15 rad belt deflection; it is incomplete and outside fitted
belt behavior. Its extrapolated torques are not credible hardware predictions.
Do not present C's agreement as proof of a fully validated identical-controller
digital twin. That discrepancy remains an investigation task.

Early development artifacts `assembled_first_*`/`assembled_fine_*` had a new
adapter motor-sign error and are excluded from model conclusions. The sign was
corrected explicitly and covered by a negative-feedback regression test before
the completed C matrix was generated. A smaller-timestep failed development run
does not establish numerical convergence.

## Artifacts

`build/assembled_matrix_20261104_C/` contains each condition's comparison PNG,
paired comparison CSV, full-rate simulated-variable CSV, summary and metadata.
An aggregate `matrix_summary.csv` and supplemental `study_manifest.json` record
settings, archive hash and limitations. `docs/ASSEMBLED_TRACKING.md` contains
commands for all conditions, live two-joint input, native viewer and dashboard.

Normal walking:

![Slow walking](../build/assembled_matrix_20261104_C/knee_ankle_normal_walk_0.6ms/comparison.png)

## Application validation

Four runtime tests passed, covering independent controller/layer objects,
negative feedback and both servo disconnections, own simulation feedback,
explicit range handling, invalid initial encoder rejection and CSV timing.
The executable Node dashboard test passed for fixed cards, pause/resume,
joint/category selection, axis labels and stop request. HTTP page/data/Stop
endpoints passed. Two synthetic source-to-live-simulation tests, including native
viewer startup/close, each finished COMPLETE with 60 pairs per joint. Latest:
`build/assembled_validation_20261104/live_result_1793734555609325200/`.
No physical motor commands or supplied hardware controllers were run.

Next: verify physical angle/mount/range, investigate joint-error-D instability
and initial preload/timing, then perform a short physical two-joint streaming
check. Preserve these fixed-model comparisons before any calibration.
