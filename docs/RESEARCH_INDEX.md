# Research and evidence index

Navigation verified against the local workspace on 2026-10-18. Linked build outputs are local, generally ignored by Git, and may be absent in a fresh checkout. Preserve their locations and contents. This index does not certify hardware fidelity or rerun any experiment.

## Research stages

| Stage | Implementation / procedure | Existing evidence and interpretation |
|---|---|---|
| 1. Original position servo | [benchmark](../experiments/run_bench_ab19.py), [benchmark guide](BENCH_EXPERIMENT.md), [oracle verifier](../experiments/verify_against_oracle.py) | [benchmark outputs](../build/bench_track_ab19/), [frozen oracle](../tests/oracle/), [gain sweep](../build/gain_sweep/), [step response](../build/step_response/), [armature pilot](ARMATURE_PILOT.md). Completed simulation work; original regression formulation |
| 2. Drivetrain and instability | [comparison runner](../experiments/run_bench_ab19_drivetrain.py), [live demo](../experiments/run_live_drivetrain_demo.py), [parameters](DRIVETRAIN_PARAMETERS.md) | [diagnosis](DRIVETRAIN_INSTABILITY_DIAGNOSIS.md), [results](../build/drivetrain_instability/RESULTS.md), [layer dynamics](../build/layer_dynamic_test/), [null test](../build/layer_null_test/), [paper parameters](../build/paper_parameters/). Implemented external-layer coupling; paper-derived simulation |
| 3. Ankle current control | [alignment](PHYSICAL_ANKLE_CONTROL_ALIGNMENT.md), [step runner](../experiments/run_ankle_current_step.py) | [step outputs](../build/ankle_current_step/), [parameter sensitivity](ANKLE_PARAMETER_SENSITIVITY.md), [sensitivity outputs](../build/ankle_sensitivity/), [current-loop report](../build/ankle_current_loop_sensitivity/CURRENT_LOOP_SENSITIVITY.md). Completed simulated alignment/sensitivity, not measured actuator validation |
| 4. Current-controlled knee baseline | [gait runner](../experiments/run_current_controlled_gait.py), [control paths](CONTROL_PATHS.md) | [gait outputs](../build/current_controlled_gait/), [sensitivity report](../build/gait_investigation/GAIT_SENSITIVITY_COMPARISON.md). Provisional ankle-to-knee gains |
| 5. Feedback location and estimator | [A/B runner](../experiments/run_gait_feedback_location_ab.py), [estimator runner](../experiments/run_gait_feedback_location_estimator.py) | [A/B report](../build/gait_feedback_location_ab/FEEDBACK_LOCATION_COMPARISON.md), [estimator/Variant C report](../build/gait_feedback_location_estimator_ab/FEEDBACK_LOCATION_ESTIMATOR_COMPARISON.md). Later estimator comparison refines attribution of the earlier location comparison; preserve both |
| 6. Timestep convergence | [convergence runner](../experiments/run_gait_feedback_location_timestep.py) | [report](../build/gait_feedback_location_timestep_convergence/TIMESTEP_CONVERGENCE_REPORT.md). Numerical refinement evidence for Variant C, not hardware validation |
| 7. Integrated Variant C | [live guide](LIVE_DEMO.md), [live adapter](../oslbench/current_gait_live.py) | [Phase 5 report](../build/integrated_current_gait_phase5/PHASE5_REPORT.md), [completed live attempt](../build/integrated_current_gait_phase5/visible_live_run/attempt_001/), [earlier zero-step attempt](../build/integrated_current_gait_phase5/live_run/attempt_001/), [saved check records](../build/integrated_current_gait_phase5/final_tests_execution.json). Completed simulation integration; partial and complete attempts remain distinct |
| 8. Deferred work | [architecture](ARCHITECTURE.md), [authoritative roadmap](NEXT_PHASE_PROPOSAL.md) | Contract/adapters, telemetry replay, read-only acquisition, mirroring and predictive validation are planned. Current-loop refinement is optional and evidence-driven. No physical validation dataset is established by this inventory |

AB19 is human gait data used as a reference. Papers/manuals and reported ankle-controller settings supply provenance, not measurements validating this exact knee build. Simulated current is not measured drive current. Stub runs and incomplete attempts must not be pooled with complete real-MuJoCo runs.

## Documentation categories and historical status

No bulk moves are needed. The following labels establish navigation while preserving original filenames and scientific conclusions.

| Existing document | Category / treatment |
|---|---|
| [README](../README.md) | Current user entry point; detailed former narrative preserved below |
| [Historical README](README_PRE_PHASE6B.md) | Historical snapshot, including old setup/capability claims; current guide supersedes operational instructions |
| [Live demo](LIVE_DEMO.md) | Current procedure at top; separately labelled historical position-servo guide below |
| [Roadmap](NEXT_PHASE_PROPOSAL.md) | Sole authoritative current plan at top; all older proposals retained as historical |
| [Control paths](CONTROL_PATHS.md) | Current controller reference |
| [Architecture](ARCHITECTURE.md) | Current ownership and explicitly proposed interfaces |
| [Working baseline](WORKING_BASELINE.md) | Current inventory method and snapshot provenance |
| [Code map](CODE_MAP.md) | Older architecture map; incomplete for current paths/test suites (including its 44-test count). Use control paths/architecture for present scope; update later |
| [Validation](VALIDATION.md) | Dated original-refactor evidence and oracle procedure; pending-run statements are historical, not a claim that later Phase 5 checks are absent |
| [Refactor map](REFACTOR_MAP.md) | Historical migration record; preserve |
| [Bench experiment](BENCH_EXPERIMENT.md) | Technical reference scoped to original direct-servo benchmark |
| [CAD findings](CAD_FINDINGS.md) | CAD geometry/inertia provenance reference |
| [Bench design](../BENCH_GAIT_TRACKING_DESIGN.md) | Historical preimplementation design; its DESIGN ONLY status describes that document's date |
| [Tuning and dataset](../BENCH_TUNING_AND_DATASET.md) | Historical tuning/acquisition report; statements that gait had not yet been used are dated |
| [Actuator dynamics](ACTUATOR_DYNAMICS_ANALYSIS.md) | Technical analysis plus historical implementation proposal; retain equations/provenance, defer any future split |
| [Drivetrain parameters](DRIVETRAIN_PARAMETERS.md) | Parameter/frame provenance; its open belt-ratio question does not override canonical current-gait CAD ratio 50/11 |
| [Integration options](DRIVETRAIN_INTEGRATION_OPTIONS.md) | Historical design comparison. Analysis-only/not-yet-integrated wording is superseded by existing DrivetrainBenchSimulation and Path B/C evidence |
| [Instability diagnosis](DRIVETRAIN_INSTABILITY_DIAGNOSIS.md) | Research evidence scoped to its torque-domain controller and feedback wiring; do not generalize directly to every current-domain experiment |
| [Armature pilot](ARMATURE_PILOT.md) | Completed simulation sensitivity report, not present canonical plant settings |
| [Ankle parameter sensitivity](ANKLE_PARAMETER_SENSITIVITY.md) | Completed scoped simulation research evidence |
| [Physical ankle alignment](PHYSICAL_ANKLE_CONTROL_ALIGNMENT.md) | Implementation/provenance and dated simulation comparisons; title does not establish physical validation |
| [MyoAssist findings](MYOASSIST_FINDINGS.md) | Dated supporting findings for parked work |
| [MyoAssist pivot](MYOASSIST_PIVOT.md) | Historical/deferred proposal, not active implementation plan |
| [Prior art and direction](PRIOR_ART_AND_DIRECTION.md) | Historical hypotheses and explicitly unverified literature map; no novelty claim follows without primary-source review |

All seven Markdown reports linked in stages 2–7 remain at their existing build locations. Later evidence refines earlier conclusions; it does not authorize replacing original reports or traces.

## Evidence registration convention

A result identity should include control path and diagnostic variant separately, commit plus dirty/untracked source hashes, model/reference hashes, environment versions, configuration, timing convention, execution engine, completion status, outputs and report. Record measured, simulated, derived and assumed quantities separately. Use a new inventory for later states; never overwrite historical manifests.

The [Phase 6B baseline](WORKING_BASELINE.md) records the working source and links to canonical results. It is an inventory, not a claim that all historical runs used the current HEAD, and not a new experimental result.
