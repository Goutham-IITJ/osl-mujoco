# Refactor map: what the code did before, and where each piece went

This file exists because the bench experiment was originally written as two large
scripts that each did seven jobs at once. It records the state of the code *before*
the reorganisation, the dependency and data-flow graph that had to be preserved, and
the exact old -> new mapping. Nothing about the physics, the gains, the model or the
AB19 reference was changed by the move; `docs/VALIDATION.md` records the numerical
proof of that.

Read `docs/CODE_MAP.md` if all you want is "where is the controller".

## 1. The old dependency graph

Five files, one linear import chain, and every one of them physics-aware:

```
experiments/demo_bench_ab19.py        presentation demo, argparse, recording
        |  imports  demo_dashboard  (leaf: drawing only, no physics)
        v
experiments/view_bench_ab19.py        live viewer, init_state(), step_once(),
        |                             consistency_check(), derive_camera()
        v
experiments/bench_track_ab19.py       reference loading, audit, spline resample,
        |                             simulate(), metrics, lag diagnostic, CSV, plots
        v
experiments/gain_sweep_bench.py       model load + 9 preflight checks, set_gains(),
        |                             kv_for(), synthetic references, sweep loop
        v
scripts/mjcommon.py                   nid(), scene_path()   (shared with check_model,
                                      view_osl, demo_sweep -- NOT part of the bench)
```

The chain worked, and it did guarantee one control law, but it had three problems the
professor was right about:

1. To read the controller you had to open `bench_track_ab19.py`, scroll 320 lines into
   `simulate()`, and understand that the gains were actually being written by
   `gain_sweep_bench.set_gains()` two files away.
2. `bench_track_ab19.py` was imported *for its side effects on the module-level lists*
   `NOTES` and `FLAGS`, so the demo depended on the experiment's global state.
3. Importing the demo transitively imported a gain sweep. Anything that wanted the
   reference loader also got matplotlib-dependent plotting code and an argparse CLI.

## 2. The old data flow

One pass, top to bottom, with no seam between stages:

```
build/AB19_knee_gait_reference.csv
   -> load_reference()        101 rows, 5 columns, verbatim              [bench_track]
   -> audit_reference()       R1 grid / R2 ROM / R3 sign / R4 kinetics   [bench_track]
   -> resample()              NaturalCubic -> 2410 samples at dt         [bench_track]
   -> res["ref_rad"]          the reference angle in radians
   -> np.clip(ref, ctrlrange) command clamp                              [bench_track]
   -> data.ctrl[knee_act]     MuJoCo position servo
   -> mujoco.mj_step()        tau = kp*(ctrl-q) - kv*qd, clamped to forcerange
   -> data.sensordata[...]    q, qd, tau back out
   -> rows.append(...)        21 formatted strings per step              [bench_track]
   -> M = dict(...)           14 metrics + 4 lag-diagnostic entries      [bench_track]
   -> bench_track_ab19.csv    3 provenance lines + header + 2410 rows    [bench_track]
   -> make_plots()            re-reads its own CSV, draws 2 figures      [bench_track]
```

`view_bench_ab19.py` re-entered that flow at `res["ref_rad"]` through `step_once()`,
and `demo_bench_ab19.py` re-entered `view_bench_ab19.py`. The gain sweep ran a
*parallel* copy of the `clip -> ctrl -> mj_step -> sensordata` loop inside
`gain_sweep_bench.run()`, with its own 18-column trace and its own metric names.

## 3. Who owned which responsibility, before

| Responsibility | Old owner |
| --- | --- |
| load the MJCF, resolve IDs, verify the model | `gain_sweep_bench.load_and_verify` |
| plant properties (I_eff, m*g*d, damping) | `gain_sweep_bench.dof_inertia`, `load_and_verify` |
| PD gains, `kv` rule | `gain_sweep_bench.kv_for`, `set_gains`; constants `KP`/`KV` in `bench_track_ab19` |
| the control law itself | inlined in `bench_track_ab19.simulate`, `gain_sweep_bench.run`, `view_bench_ab19.step_once` |
| command clamp to `ctrlrange` | inlined in all three loops above |
| torque clamp to `forcerange` | MuJoCo, plus an inlined `tau_ideal` saturation flag in all three loops |
| load / validate the AB19 CSV | `bench_track_ab19.load_reference`, `audit_reference` |
| interpolation and resampling | `bench_track_ab19.NaturalCubic`, `resample` |
| synthetic reference motions | `gain_sweep_bench.build_refs` |
| reset / seed / step MuJoCo | `bench_track_ab19.simulate`, `gain_sweep_bench.run`, `view_bench_ab19.init_state` |
| per-step logging | inlined `rows.append(...)` in `simulate` and `run` |
| CSV writing | `bench_track_ab19.main`, `gain_sweep_bench.main` |
| metric computation | inlined at the end of `simulate` and `run` |
| lag diagnostic | inlined at the end of `simulate` |
| offline figures | `bench_track_ab19.make_plots` |
| live visualisation | `view_bench_ab19.derive_camera`, `run_viewer` |
| identity proof (live == quantitative) | `view_bench_ab19.consistency_check` |
| presentation demo, recording | `demo_bench_ab19.verify`, `record`, `run_demo` |
| second window | `demo_dashboard.py` (already clean: no physics) |

## 4. The new structure

Library code in `oslbench/`, one job per module, no argparse and no `print` banners:

```
oslbench/
  model.py       load models/osl_v2_bench.xml, resolve IDs, verify structure,
                 measure the plant (I_eff, m*g*d, damping, friction).  No control.
  controller.py  Kp, Kd, error = q_ref - q, tau = Kp*error - Kd*qdot,
                 the ctrlrange command clamp, the forcerange torque clamp,
                 the zeta -> Kd rule, and how those gains reach mjModel.  No MuJoCo
                 stepping, no file I/O.
  reference.py   AB19 CSV loading, column and phase validation, the four-part audit,
                 the natural cubic spline, resampling onto dt, and the synthetic
                 sweep references.  No MuJoCo.
  simulation.py  create/reset/seed MjData, one step() that reads reference ->
                 controller -> ctrl -> mj_step -> measurements, and run() over a
                 whole reference.  No plotting, no metrics.
  logging.py     the per-step table (21 columns for the benchmark, 18 for the sweep),
                 the CSV writers with their provenance headers, and the reader used
                 by the offline plots.  Knows nothing about physics.
  metrics.py     RMS / MAE / peak error, peak torque, % authority, saturation, peak
                 velocity, power, the servo-lag diagnostic, and the sweep metrics.
                 No plotting.
  plotting.py    the six-panel results figure and the separate human-kinetics figure,
                 built from a logged CSV.  Never runs MuJoCo.
  viewer.py      camera derivation, the live passive-viewer loop, the headless
                 recorder, and the consistency proof.  Adds visualisation only: the
                 joint is moved exclusively by simulation.step().
  dashboard.py   the browser / tkinter / terminal second window.  A consumer of
                 published state; it has no model and no controller.
```

Entry points in `experiments/`, deliberately thin -- load model, load reference, make
controller, run, save, measure, plot:

```
experiments/run_bench_ab19.py       the validated AB19 benchmark          (was bench_track_ab19.py)
experiments/run_gain_sweep.py       the kp/kv sweep                        (was gain_sweep_bench.py)
experiments/run_live_demo.py        viewer + dashboard + --record          (was demo_bench_ab19.py
                                                                            + view_bench_ab19.py)
experiments/plot_bench_results.py   offline figures from a logged CSV       (was --plot-from)
experiments/verify_against_oracle.py  fresh run vs the frozen validated CSV (new)
tests/test_oslbench.py              lightweight assert-based tests          (new)
```

## 5. Old -> new, line by line

| Old | New |
| --- | --- |
| `gain_sweep_bench.EXPECT`, `I_BODY_EXPECT` | `oslbench.model.EXPECT`, `model.I_BODY_EXPECT` |
| `gain_sweep_bench.check/warn/close` | `oslbench.model._Checks` (same messages, collected on the returned object instead of module globals) |
| `gain_sweep_bench.dof_inertia` | `oslbench.model.dof_inertia` (unchanged) |
| `gain_sweep_bench.load_and_verify` | `oslbench.model.load_bench_model` -> `BenchModel` (same nine checks, same order, same text; the `info` dict became named attributes) |
| `gain_sweep_bench.kv_for` | `oslbench.controller.kd_for_damping_ratio` (same formula; `kv_for` kept as an alias) |
| `gain_sweep_bench.set_gains` | `oslbench.controller.PDController.write_to_model` (same four array writes) |
| `gain_sweep_bench.build_refs` | `oslbench.reference.synthetic_references` (unchanged) |
| `gain_sweep_bench.run` | `oslbench.simulation.BenchSimulation.run_sweep` + `oslbench.metrics.sweep_metrics` + `oslbench.logging.SweepLog` |
| `gain_sweep_bench.step_shape` | `oslbench.metrics.step_shape` (unchanged) |
| `gain_sweep_bench.main` | `experiments/run_gain_sweep.py` |
| `bench_track_ab19.KP`, `KV` | `oslbench.controller.KP`, `controller.KD` (`KV` kept as an alias) |
| `bench_track_ab19.DT_EXPECT` | `oslbench.model.EXPECT["timestep"]` |
| `bench_track_ab19.SUBJECT`, `TRIAL`, `MASS_KG`, `STANCE_END`, `DEFAULT_CSV` | `oslbench.reference.SUBJECT`, `TRIAL`, `SUBJECT_MASS_KG`, `STANCE_END`, `DEFAULT_REFERENCE_CSV` |
| `bench_track_ab19.load_reference` | `oslbench.reference.load_reference` -> `GaitReference` |
| `bench_track_ab19.audit_reference` | `oslbench.reference.audit_reference` -> `ReferenceAudit` (same checks, same order, same flag text; `NOTES`/`FLAGS` globals became fields on the returned object) |
| `bench_track_ab19.NaturalCubic` | `oslbench.reference.NaturalCubic` (unchanged) |
| `bench_track_ab19.resample` | `oslbench.reference.resample_reference` -> `ResampledReference` |
| `bench_track_ab19.simulate` -- seeding | `oslbench.simulation.BenchSimulation.reset` |
| `bench_track_ab19.simulate` -- inner loop | `oslbench.simulation.BenchSimulation.step` |
| `bench_track_ab19.simulate` -- `rows.append` | `oslbench.logging.BenchLog.append` / `.rows()` |
| `bench_track_ab19.simulate` -- `M = dict(...)` | `oslbench.metrics.tracking_metrics` |
| `bench_track_ab19.simulate` -- lag search | `oslbench.metrics.lag_diagnostic` |
| `bench_track_ab19.CSV_HEADER` | `oslbench.logging.BENCH_CSV_HEADER` |
| `bench_track_ab19.main` -- CSV writing | `oslbench.logging.write_bench_csv`, `write_metrics_csv` |
| `bench_track_ab19.make_plots` | `oslbench.plotting.plot_bench_results` (+ `oslbench.logging.read_bench_csv` for the parsing half) |
| `bench_track_ab19.main` -- report | `experiments/run_bench_ab19.py` |
| `view_bench_ab19.init_state` | `oslbench.simulation.BenchSimulation.reset` (one implementation now, not two) |
| `view_bench_ab19.step_once` | `oslbench.simulation.BenchSimulation.step` (one implementation now, not three) |
| `view_bench_ab19.consistency_check` | `oslbench.viewer.consistency_check` |
| `view_bench_ab19.ref_closure` | `oslbench.reference.cycle_closure_at_100pct` |
| `view_bench_ab19.derive_camera` | `oslbench.viewer.derive_camera` |
| `view_bench_ab19.run_viewer` | folded into `oslbench.viewer.run_live_view` (the demo loop superseded it: same loop, plus the dashboard) |
| `view_bench_ab19.FRAME_HZ`, `K_SPACE`, `K_R_*` | `oslbench.viewer.FRAME_HZ`, `KEY_SPACE`, `KEY_R_UPPER`, `KEY_R_LOWER` |
| `demo_bench_ab19.verify` | `oslbench.viewer.preflight` |
| `demo_bench_ab19.open_sink`, `record` | `oslbench.viewer.open_video_sink`, `record_video` |
| `demo_bench_ab19.cycle_metrics` | `oslbench.metrics.cycle_metrics` |
| `demo_bench_ab19.run_demo` | `oslbench.viewer.run_live_view` |
| `demo_bench_ab19.TITLE`, `SUBTITLE`, `CYCLE_S`, `MAX_PLOT_PTS` | `oslbench.viewer.TITLE`, `TITLE_TTY`, `SUBTITLE`, `CYCLE_S`, `MAX_PLOT_PTS` |
| `demo_bench_ab19.main` | `experiments/run_live_demo.py` |
| `demo_dashboard.py` | `oslbench/dashboard.py` (moved verbatim; run it alone with `python -m oslbench.dashboard`) |

Deleted after migration, recoverable from git at `0cd33a5`:
`experiments/bench_track_ab19.py`, `experiments/gain_sweep_bench.py`,
`experiments/view_bench_ab19.py`, `experiments/demo_bench_ab19.py`,
`experiments/demo_dashboard.py`.

Untouched by the refactor: `models/osl_v2_bench.xml`,
`build/AB19_knee_gait_reference.csv`, `experiments/gain_sweep_analytic.py`,
`experiments/camargo_probe.py`, everything in `tools/`, everything in `scripts/`,
and every MyoAssist / OpenSourceLeg_KA_L1 file.

The two exceptions are prose only, not code: `experiments/gain_sweep_analytic.py` and
`BENCH_TUNING_AND_DATASET.md` each named `gain_sweep_bench.py` in a comment or a command
line, and now name `experiments/run_gain_sweep.py` instead. No number, formula or
argument in either file changed.

## 6. One deliberate duplication

`scripts/mjcommon.py` still owns `nid()` and `scene_path()` for
`scripts/check_model.py`, `scripts/view_osl.py` and `scripts/demo_sweep.py`, which are
not part of the bench experiment. `oslbench/model.py` resolves names with its own
three-line `name_id()` instead of importing from `scripts/`, so that the library is a
self-contained package with no `sys.path` surgery. That is the only intentional
overlap, it is six lines, and neither copy can drift into a different *physics*
decision because neither one touches gains, limits or the plant.
