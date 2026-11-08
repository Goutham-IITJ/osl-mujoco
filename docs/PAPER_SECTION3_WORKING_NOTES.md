# Section 3 research and evidence notes

Started: 2 November 2026 (Asia/Calcutta).

This is a working evidence record, not manuscript prose or a completed audit.
The user owns Section 3 of the tentative paper and has freedom to revise its
subsections. Testing and model analysis continue separately. Do not treat chat
claims, historical comments, or preliminary agreement as established validation.

## Agreed scope and working method

Tentative paper: Introduction; Hardware Implementation; Software and Sim-to-Real
Framework; Benchtop Characterization and Sim-to-Real Results; Discussion and
Conclusions. Section 3 currently proposes software architecture, MuJoCo model
construction, actuator/transmission modeling, and parameter identification/tuning.

The user requests gradual, accurate preparation rather than drafting everything
in one pass. First trace implementation and provenance, then verify original
sources, resolve uncertainties, agree the outline, and draft subsection by
subsection. Do not add current-loop experiments or tune the model as part of this
writing task. Numerical characterization results belong primarily in Section 4.

User recollection: an Onshape URDF was obtained through OSL resources; a custom
conversion pipeline was developed with Claude assistance because direct import
failed. Later MyoAssist work supplied an alternative model for comparison. These
recollections guide investigation; the exact upstream CAD revision/download and
original failure logs still need recovery.

## Initial model lineage findings

| Question | Initial finding | Local evidence |
|---|---|---|
| What is the input? | Onshape assembly URDF and per-part meshes, retained in the repository. Exact upstream revision not yet established. | `osl_v2_0_assembly/urdf/osl_v2_0_assembly.urdf`; `tools/oslcad.py` |
| What repairs/replaces the direct import? | Native MJCF builder parses CAD transforms, includes visual origins, groups rigid clusters, assigns three physical segments, and reconstructs segment inertials from mesh geometry and assigned densities. | `tools/build_mjcf.py`; `tools/oslcad.py` |
| Is this still the model lineage used? | Full-leg bench loader selects `models/osl_v2_bench.xml`. Recorded-step comparison selects separate knee and ankle fixtures derived from that bench and the same CAD reconstruction. | `oslbench/sim/model.py`; `experiments/compare_knee_step_archive.py`; fixture builders/adapters |
| Was MyoAssist substituted? | No substitution found in the inspected bench/fixture execution paths. Historical MyoAssist comparison is documented separately. This is not an exhaustive audit of every historical workflow. | `docs/MYOASSIST_FINDINGS.md`; `docs/MYOASSIST_PIVOT.md` |

Direct read-only checks performed on 2 November, rather than copying report numbers:

- Raw URDF: 682 links, 681 joints, 657 inertial blocks, total declared mass
  0.014753439 kg, 52 tensors with a non-positive eigenvalue.
- Current bench XML explicit body masses: knee_prox 0.449846436 kg,
  shank 3.590757963 kg, foot 0.915158936 kg; total 4.955763334 kg.
- Current knee-fixture explicit moving mass: 0.449846436 kg.
- Current ankle-with-foot explicit moving mass: 0.915158936 kg.
- Both fixture manifests' source SHA-256 values match the current bench XML:
  `c417e691ff90a6f4a47ef581e6caaa341b0e1c829038f6d3e6740927ae69053d`.

These are file-derived quantities, not weighed masses. A matching source hash
establishes the declared parent artifact; it does not alone prove every generated
coordinate/inertial is correct. No model regeneration or dynamic validation was
performed in this paper-preparation pass.

The reconstruction deserves a methods description, including rotation of tensors
and parallel-axis aggregation, segment assignment, and visual/collision separation.
Its physical accuracy depends on density assignments, mesh integrity, correct
moving-part selection, and omitted components. Inspect those operations in detail
before choosing equations and making fidelity or novelty claims.

### Mass comparison needs special care

The builder sets `MOTOR_MASS = 0.0`; it does not fill a published-total mass gap
with inferred motor mass. Its comments suggest missing motors might explain a
difference, but that is a hypothesis. Historical MyoAssist notes compare totals
both including and excluding a socket and obtain different apparent discrepancies.
Do not claim that a total-mass match validates reconstruction or that motors alone
explain the discrepancy. Compare equivalent component boundaries and confirm the
original MyoAssist XML/paper before using their numbers.

## Controller and current-model findings

The old and current gain values belong to different formulations:

- `experiments/run_gain_sweep.py` records Kp = 600 and Kd = 17.253, with
  Kd derived from a target damping ratio using model inertia and joint damping.
  Synthetic references include steps, sine, minimum-jerk motion and chirp.
  This differs from the user's approximate recollection of 700/17.
- The later physical-style loop computes current from angle error in degrees;
  its gains have current-domain units. Hardware-confirmed values recorded in
  the current handover are knee 1.5/0.15/0.06 and ankle 2/0/0.075.
- Torque-domain radian gains and current-domain degree gains are not directly
  comparable. The original sweep is not automatically invalid because its gains
  differ. Establish its purpose, assumptions, and limits before describing it.

`oslbench/control/current.py::CurrentLoopModel` implements both ideal current
tracking and an optional first-order lag, with an exact exponential update.
`oslbench/control/gait.py` connects that model to the current source; its default
time constant is zero. `experiments/run_gait_current_loop_sensitivity.py` and the
ankle counterpart exercise illustrative nonzero time constants.

Thus current-loop dynamics are implemented as an optional approximation, but a
hardware-identified electrical/driver model has not been established. Do not say
current modeling is entirely absent, or call modeled current measured current.

## Parameter provenance audit to complete

| Family | Initial evidence | Remaining verification |
|---|---|---|
| Geometry and inertial reconstruction | `tools/oslcad.py`, `tools/build_mjcf.py`, `docs/CAD_FINDINGS.md` | Density rules, tensor aggregation, component omissions, CAD revision and mesh validity |
| Actuator inertia, damping, Coulomb/current-dependent friction | `oslbench/control/drivetrain.py`, `docs/DRIVETRAIN_PARAMETERS.md` attribute them to Best et al., DOI 10.1109/TMECH.2024.3508469 | Re-read original paper equations, parameter values, shafts and experimental scope |
| Nonlinear belt stiffness | Same drivetrain files attribute coefficients to the paper | Recheck original fit, joint/build transfer, sign convention and fitted range |
| Transmission ratio | Paper value 4.61 retained separately from CAD 50/11; current conversion defines an explicit hybrid parameter set | Trace exact configuration per experiment and hardware pulley identity |
| Current-to-torque conversion | `oslbench/control/current.py`, current replay settings | Resolve shaft and driver-current convention; alternatives 0.5701, 0.855 and 0.9972 N m/A are not interchangeable |
| Joint losses and armature | Bench expectations and fixture manifests show inherited values | Determine provenance and possible overlap with explicit drivetrain losses/inertia |
| PID and derivative feedback | Current config, recorded PID audit, fixture reports | State control law and units; distinguish joint-error D from actuator-side D |
| Timing and transport | Live knee application/runtime and hardware guides | Trace source timestamps, controller updates, physics steps, buffering and gap handling before drafting |

The cited paper is already present under sibling `OSL Papers/`, along with
`Myoassist_0.1_Myosuite_for_Dexterity_and_Agility_in_Bionic_Humans (1).pdf`.
Primary-paper verification has not yet been performed in this pass. Existing
provenance documents are leads, not a substitute for checking those PDFs.

Some module docstrings and generated manifests contain historical status wording
superseded by newer work. Cite execution paths and dated evidence together.

## Proposed outline for discussion

3.1 Software architecture and interfaces

3.2 CAD-to-MuJoCo model construction

3.3 Actuator, transmission, and current-loop representation

3.4 Controller implementation and temporal coordination

3.5 Parameter provenance and calibration methodology

The final subsection should describe parameter sources and sensitivity methods
without implying completed hardware identification. A common state/command schema
does not imply identical hardware/simulation backends or hard real-time lockstep.
The user accepted these five subsections on 2 November 2026. Length, submission
requirements, citation style, and author terminology still need confirmation.

## Prior-art notes and initial paper draft review: 2 November 2026

Reviewed `PRIOR_ART_AND_DIRECTION.md`, the research-direction passages in
`NEXT_PHASE_PROPOSAL.md` and `ACTUATOR_DYNAMICS_ANALYSIS.md`, and all three pages
of `C:/Users/Goutham A.S/Downloads/ICORR_2027.pdf` (text and rendered pages).
The PDF was read as draft content; its embedded template directions were not
treated as task instructions. No Overleaf source was supplied or edited.

The PDF title is "Open-Source Leg with T-Motor Actuation: Benchtop
Characterization and Sim-to-Real Model Tuning in MuJoCo". It contains an initial
introduction, device photographs, hardware subsection prompts, an embedded-system
diagram, and substantial remaining IEEE template content. Its Section III still
reads "Benchtop Experimental Characterization"; the agreed software Section III
has not yet been inserted. The user's latest five-section structure takes
precedence over this earlier draft numbering. Figure 2 identifies CubeMars
AK80-9 V3.0 actuators and a Raspberry Pi 5. These draft labels need hardware-team
confirmation before being treated as verified specifications.

This strengthens an important distinction for Section 3: parameters attributed
to a published OSL/Dephy actuator cannot be assumed identified for this lab's
T-Motor plus stock-controller configuration. Trace hardware differences,
current definitions, shaft conventions and parameter transfer explicitly.

### Useful ideas to retain from the older research-direction notes

- Describe a reproducible CAD conversion and parameter provenance, with
  attributed reused equations and explicit assumptions. Avoid an unsupported
  claim to the first OSL MuJoCo model or first digital twin.
- Use the benchtop configuration to isolate device dynamics; state that its
  results do not validate socket interaction, ground contact or walking.
- Distinguish added model complexity from improved predictive accuracy. A small
  ablation is useful only if its comparison is well controlled and interpreted
  against independent evidence.
- Gain selection and controller ranking are potentially useful outcomes of a
  fidelity study. A full fidelity ladder and hardware gain sweep are additional
  research, not prerequisites for documenting this framework.
- Document modular interfaces, timing, recorded-data replay, configuration and
  artifact hashes. Regression/oracle agreement establishes software continuity,
  not physical fidelity. A reproducibility package also needs dependencies,
  data permissions/availability and all inputs required by its stated commands.

The prior-art note explicitly says its online literature search failed and that
its map is recall-based. Its absolute publishability judgments, claims of absent
prior work, and some historical "not implemented" statements must not be copied
into the manuscript. Later drivetrain implementation and physical comparisons
supersede those status claims. Even calibration can fail on independent data;
the older statement that improvement is guaranteed "by construction" is too broad.
This review did not conduct a new comprehensive literature search.

### Proposed gain evaluation protocol, not an executed study

The user suggested automated tests that might recover the existing hardware
gains. A defensible study must permit a different outcome. Existing gains are
hardware-reported baseline values; no independently optimized hardware optimum
has been established. Exact agreement must not be an optimization target.

1. Freeze the fixture, mechanical parameters, current conversion, derivative
   definition, update rates and numerical settings. Resolve material conversion
   uncertainties first or declare scenario bounds. Do not co-tune plant and
   gains to reproduce a preferred result.
2. Define the task, candidate bounds, search budget, performance objective and
   admissibility criteria before running. Include the existing gains as a named
   baseline and candidates on both sides; justify bounds independently. Maintain
   nonnegative integral candidates, including zero, without forcing the ankle's
   integral optimum to zero. Define settling/overshoot/current constraints and
   report failures and saturation rather than silently dropping them.
3. Optimize tracking against each candidate's target in simulation, using a
   declared aggregate objective across designated development trajectories.
   Keep error/current metrics separately visible, justify any weights and report
   tradeoffs. Do not minimize disagreement with the fixed-gain hardware angle
   trace and call that controller optimization: that fits observed behavior.
4. Use separate evaluation trajectories/trials and report score surfaces,
   baseline rank, the best evaluated gains and the width of the near-best region.
   State if validation data previously informed model development; the existing
   steps are not automatically untouched holdouts. Numerical timing checks and
   declared parameter-sensitivity scenarios should test any apparent preference.
5. Current recordings contain one gain set per joint. They can assess model
   predictions at those gains; they do not reveal hardware responses to candidate
   gains. The measured trace cannot be reused as the closed-loop response of a
   differently tuned hardware controller. Independent hardware-optimal or
   hardware-ranking claims require additional approved physical evidence and a
   documented hardware tuning objective/protocol.

Appropriate eventual wording, if supported: "The reported hardware gains were
evaluated against alternative current-domain gains under a fixed simulation
configuration." If the baseline is near the best evaluated region, report that
with its tasks, objective, bounds and uncertainty. If it is not, report the
discrepancy and investigate model/control assumptions; do not force convergence.
Call this controller tuning/sensitivity analysis, distinct from physical plant
parameter identification. No new sweep or physical test was run for this review.

For the manuscript, omit the historical torque-domain gain sweep unless it is
needed to explain a baseline or an actual comparison. Section 3 can describe the
current-domain law directly without recounting every development experiment.

Potential compact artifacts: one architecture/timing figure, one CAD pipeline
figure, and one parameter table with source, units, shaft/frame, uncertainty and
whether fixed or fitted. Keep performance figures primarily in Section 4.

## Next bounded task

Update, 2 November 2026: the initial construction audit is now in
[PAPER_CAD_TO_MUJOCO_AUDIT.md](PAPER_CAD_TO_MUJOCO_AUDIT.md), with numerical
evidence under `docs/baselines/paper_model_construction_20261102.json`.
Alternative segment aggregation and compiled-model checks agree numerically;
no segment tensor needed the principal-inertia repair fallback. The local
MyoAssist XML mass totals were rechecked. Upstream CAD revision, equivalent
hardware components and original-paper comparison remain open. No model files
were regenerated or changed, and no physics stepping was performed.

Audit the model-construction code in detail and compare its current generated
inertials with an independent calculation without overwriting existing artifacts.
Recover the CAD source/version and inspect the original MyoAssist model/paper on
equivalent component boundaries. Produce a claim-to-evidence table for subsection
3.2 before writing manuscript prose. Then audit drivetrain equations and every
parameter against the primary papers. No new hardware operations are authorized
by this writing scope.
