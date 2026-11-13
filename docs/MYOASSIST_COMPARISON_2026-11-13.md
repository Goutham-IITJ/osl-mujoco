# CAD-derived OSL V2 versus MyoAssist: verified comparison

13 November 2026. Prepared from the supplied ICORR 2025 paper, local model files, current OSL runtime, and preserved experiment records. No model parameters or controllers changed.

## Answer for the professor

Our model is a **CAD-derived rigid-body model of the OSL V2 prosthesis, coupled to an explicit compliant drivetrain and a current-control/replay framework**. MyoAssist's myoOSL is a **neuromusculoskeletal human–prosthesis environment for locomotion and human–device control research**. Both use MuJoCo and represent an OSL V2 knee–ankle device. Their principal difference is where they place modelling detail: our work resolves device geometry, mass-property construction, transmission dynamics and recorded hardware response; MyoAssist resolves human anatomy, muscle actuation, residual-limb interaction and terrain locomotion.

Calling our current bench model a “musculoskeletal leg model” would be misleading: it contains no biological muscles or tendons. The separate simplified walking scaffold also has no muscles. Neither more CAD meshes nor more equations alone establishes greater physical accuracy.

## 1. Keep these versions separate

| Version inspected | What it actually contains |
|---|---|
| Supplied **MyoAssist 0.1**, ICORR 2025, Section IV, printed pp. 440–441 | myoOSL with **54 muscles**, two prosthetic actuators and **four passive socket coordinates**; paper describes 23 internal DOFs and a 50% transfemoral amputation |
| Local `myo_sim/.../legacy/osl/myolegs_osl.xml` | Legacy implementation consistent with the paper's 54-muscle/four-socket-coordinate architecture; this checkout contains later maintenance changes, so it is not certified byte-identical to the 2025 release |
| Local modern `assist_sim` device `OpenSourceLeg_KA_L1` attached to `myolegs` | **65 remaining muscles**, two prosthetic actuators, rigid socket attachment; exported scene already present locally |
| Modern `OpenSourceLeg_KA_L1` attached to `myolegs22` | **15 remaining muscles**, two prosthetic actuators, rigid socket; lineage used for our later reflex/torque-bridge exploration |
| Our current assembled replay | Fixed-base, two-joint CAD leg, two current controllers and two compliant drivetrain states; no human, socket or ground loading |
| Our `osl_v2_walk.xml` | Separate simplified, fixed-pelvis scaffold with hip, contralateral hip, knee, ankle and four passive socket joints; not the plant used for the reported assembled hardware comparisons |

Thus **“MyoAssist has a rigid socket” is false for the supplied paper**. It is true of the newer local KA_L1 fragment. Likewise, 54, 65 and 15 are different model lineages, not inconsistent counts of one model.

## 2. Mechanics and detail

| Aspect | Our CAD model/current replay | MyoAssist 0.1 / local OSL models | Consequence |
|---|---|---|---|
| Anatomical scope | Prosthetic device only in the active comparison | Residual limb, pelvis/torso and intact contralateral leg, with muscle–tendon actuation | MyoAssist can study human compensation and balance; our bench cannot |
| Geometry construction | Onshape export: 682 links/681 joints reconstructed into three rigid device segments; visual-origin transforms repaired | Device represented by assembly meshes and authored rigid-body inertials | Our traceability is at CAD-part level; the exported CAD joint count is **not** the simulated mobility |
| Device mobility | Knee and ankle hinges; three rigid segments | Same two powered device hinges; modern device has seven XML bodies but three rigid groups | Seven versus three bodies does not imply more independent prosthetic motion |
| Mesh complexity | 260 source STL assets; compiled bench uses **80 mesh assets and 217 geoms** | Modern device declares five prosthesis meshes plus one residual-femur replacement mesh; compiled whole scenes include additional anatomy | “260 versus 5” compares source inventory with device assets, not solver DOFs or equivalent rendering cost |
| Inertia | Part volumes × assigned material densities, aggregated COM and inertia using rigid transforms/parallel-axis terms | Prescribed assembly mass, COM and diagonal inertia tensors; paper describes matching hardware mass and torque/ROM | Our construction is independently reproducible, but assigned densities and component accounting require physical confirmation |
| Joint torque production | Current input drives actuator-shaft inertia/friction, then an elastic belt drives the joint | OSL uses bounded algebraic torque actuators; no explicit compliant motor–belt shaft state in inspected OSL implementations | Our current drivetrain represents internal lag, losses and elastic energy absent from the bounded ideal torque path |
| Biological actuation | None | MuJoCo muscle activation dynamics, muscle force–length/velocity behaviour and tendon routing | MyoAssist is considerably richer in physiology; no claim of a detailed nervous-system model follows automatically |
| Socket | Absent from active bench; four spring-damped coordinates in separate walk scaffold | Paper/legacy: four passive coordinates; modern KA_L1: rigid | Compliance is not a unique contribution over MyoAssist 0.1 |
| Foot/ground | Active bench has no floor/contact loading; ground/walk variants use blade contact geometry and protective bounding boxes | Modern device has three capsule foot-contact geoms and foot/toe touch sites; locomotion environment adds terrain | Different contact abstractions; our blade geometry is rigid, not a validated flexible carbon-fibre foot |
| Control/task | Recorded angle references, 200 Hz current PID, 0.5 ms mechanics, time-aligned encoder comparisons | Paper: muscle policy plus selection among five OSL parameter sets, each using a four-phase impedance FSM | Tracking imposed gait angles is distinct from generating stable human–prosthesis locomotion |
| Evidence | Existing physical recordings compared offline to simulation | Paper presents locomotion environment; Section IV-E explicitly does not supply a complete myoOSL baseline controller | Neither task availability nor a visual animation establishes validated walking |

### Current drivetrain versus the old servo description

The September report compared the authored `<position kp="60">` actuators with MyoAssist's torque sources. Those position actuators still exist in the bench XML, but **the current assembled runtime zeros both gain and bias arrays and applies belt torque through `qfrc_applied`**. Therefore “our model is just a position servo” is outdated for the current experiment.

For each joint, the current mechanical structure is:

```text
angle reference -> sampled current PID -> commanded current
  -> actuator torque, shaft inertia and friction
  -> nonlinear belt compliance -> CAD joint/body motion
```

The belt deflection is `theta_s = theta_j - theta_a/n_t`, with restoring torque
`tau_j = -(p1*theta_s + p2*theta_s*abs(theta_s))`.
The actuator has its own angle and velocity, independent of joint motion. Two joints therefore add four continuous shaft-state values outside MuJoCo's `qpos/qvel`, plus controller memory. A count of `nq=2` alone omits this extra runtime state.

The current replay uses CAD belt ratio **50/11**, provisional actuator-output conversion **0.5701 Nm/A**, and literature-derived actuator/belt parameters. It is a documented hybrid, not a fully identified actuator. No electrical winding, thermal, backlash or belt-hysteresis model is claimed. The bench's authored joint armature/damping/friction placeholders also remain present in this runtime; their interpretation alongside the explicit drivetrain needs care before claiming every loss/inertia term is physically identified.

In modern KA_L1, the applied joint torque follows the numerical gains **49.4 and 58.4** times a command clipped to **±2.88**, giving **±142.272 and ±168.192 Nm**. These are model torque-scaling constants. They are **not established OSL V2 mechanical reduction ratios**. The OSL project documents 9 × 50/11 = **40.909** for its CAD chain, versus 9 × 4.61 = **41.49** for the cited drivetrain paper. The older local report's interpretation of 49.4/58.4 as verified V2 physical ratios must not be reused.

A further subtlety: the modern fragment does not explicitly author joint armature/damping. The saved 80-muscle-lineage scene compiles with OSL armature **0.01 kg m²**, damping **0.5 Nm s/rad** and zero frictionloss through host defaults. A fresh `myolegs22` composition compiles with **zero armature and zero damping**. These are configuration-dependent model properties, not measurements of hardware.

## 3. Quantified device differences

The following rigid-group comparison uses modern KA_L1. Legacy values differ only in the last decimal places for the two pylons.

| Rigid group / quantity | Our CAD device | MyoAssist KA_L1 |
|---|---:|---:|
| Proximal group, excluding socket | 0.449846 kg | 0.189058 kg (femoral pylon + proximal adapter) |
| Between knee and ankle | 3.590758 kg | 4.047425 kg (knee assembly + tibial pylon + ankle assembly) |
| Distal foot group | 0.915159 kg | 0.291000 kg |
| **Device excluding socket** | **4.955763 kg** | **4.527483 kg** |
| Socket | Absent from bench | 0.849991 kg |
| Device including its socket | Not applicable to bench | 5.377474 kg |
| Knee-to-ankle axis separation | 0.33184 m | 0.37610 m, derived from XML transforms |
| Authored knee range | −5° to +120° | 0° to +120° |
| Authored ankle range | −30° to +20° | approximately −30° to +30° |

Excluding the socket, our authored model is **0.428280 kg (9.46%) heavier**, not lighter. Its distal foot group is about **3.14 times** as heavy. Its knee–ankle separation is **44.26 mm shorter**. These differences affect gravity moments, swing inertia and coupling even with an identical controller.

This is a normalized model-accounting comparison, **not proof of an error in either model**: the foot/adapter boundaries, specific foot, battery inventory and physical hardware configuration have not been matched component by component. Our CAD includes the Variflex foot; the MyoAssist device uses an `osl_custom_foot_v2` asset. The current manuscript's SACH/off-board-battery description is also an unresolved correspondence issue. Equal total mass would not imply equal COM or inertia.

The assembled archive experiments explicitly override ankle range to **±31° in memory**. They do not validate the authored −30°/+20° range or change the XML default.

## 4. Complexity measured from compiled local models

Read-only compilation and `mj_forward` were performed on 13 November; no dynamic rollout or new hardware experiment was run.

| Model | qpos (`nq`) | Velocity coordinates (`nv`) | Actuators (`nu`) | Activation states (`na`) | Tendons | Equality constraints |
|---|---:|---:|---:|---:|---:|---:|
| CAD bench XML | 2 | 2 | 2 authored servos* | 0 | 0 | 0 |
| CAD walk scaffold | 8 | 8 | 4 | 0 | 0 | 0 |
| Legacy myoOSL | 30 | 29 | 56 = 54 muscles + 2 motors | 54 | 54 | 7 |
| Saved modern `myolegs` + KA_L1 scene | 26 | 25 | 67 = 65 muscles + 2 motors | 65 | 65 | 7 |
| Fresh modern `myolegs22` + KA_L1 | 30 | 30 | 17 = 15 muscles + 2 motors | 15 | 15 | 17 |

*Disabled in current assembled replay as explained above. Counts include raw compiled coordinates, including constrained coordinates; they are **not a rank analysis of independent mobility**. Free-root quaternions also make `nq` differ from `nv`. Do not interpret the reduced-muscle model's larger raw coordinate count as greater anatomical complexity. The paper's 23 internal coordinates exclude the six root velocity coordinates in the legacy count of 29.

The first four rows were checked using existing OSL MuJoCo 3.12.0 against saved XMLs. Fresh composition used the existing MyoAssist environment, MuJoCo 3.11.0, within that checkout's declared supported range. Compilation under 3.12 does not establish support for the full MyoAssist stack under that version. No wall-clock performance benchmark was performed: mesh counts alone cannot establish which complete workflow is faster.

## 5. What our earlier Myo exploration established

The local folder contains the three source repositories, exported human–OSL scene, separate environments, reflex bridge modifications and CSV traces. The older work is recoverable without access to the old account.

- `step0_output.txt` preserves **53 passed, 5 failed and 2 warnings**. The failures concern the report's incorrect predicted inherited joint defaults, later explained in its addendum. Do not re-label the raw run as an all-pass test.
- `step1_trace.csv` records a short reflex-path experiment with zero OSL motor torque, ending around **0.645 s** with pelvis height near **0.650 m**. It is not stable prosthetic walking evidence.
- `step2_actuator_sanity.csv` preserves diagnostic torque trials. The dedicated local torque interface is present. This establishes an implemented exploration/bridge, not a trained or validated locomotion controller.
- No evidence reviewed here establishes that our CAD device was substituted into MyoAssist and then walked successfully. The existing exported scene uses the official KA_L1 device.

## 6. Claims we can defend

**Suggested manuscript/meeting wording:**

> Unlike MyoAssist's myoOSL environment, which models neuromusculoskeletal coordination and human–prosthesis interaction during locomotion, our work focuses on the device-level dynamics of a CAD-derived OSL V2 assembly. We reconstruct rigid-body mass properties from component geometry and assigned material densities, couple the knee and ankle to explicit compliant drivetrain models, and compare simulated joint responses with recorded physical-device trajectories. These models provide complementary levels of detail: MyoAssist supplies human muscle and locomotion dynamics, whereas our framework supplies traceable device reconstruction and hardware-response analysis. The present evidence does not establish superior overall fidelity or validated human–prosthesis walking.

Existing assembled results report simulation-to-recorded-hardware RMS differences of **0.762–2.381° at the knee** and **0.237–0.691° at the ankle** across six recordings. These are archived results, not new tests in this audit. Their scope is fixed-base trajectory replay without ground loading. Configuration C uses actuator-side derivative feedback while hardware uses joint-error derivative; equal gains do not establish identical control laws. These results cannot be compared directly with MyoAssist's distance-walked task score.

To isolate a future fidelity comparison: attach both device variants to the **same human and socket**, normalize frames, component inventory, joint zeros/ranges, control law and contact conditions, then change geometry/inertia and drivetrain dynamics in separate experiments. Evaluate physical torque/current response, tracking transients, COM/inertia and contact behaviour with independent data. That controlled experiment has not yet been performed.

## 7. Evidence and reproducibility

Primary paper: `C:/Users/Goutham A.S/Downloads/Myoassist_0.1_Myosuite_for_Dexterity_and_Agility_in_Bionic_Humans.pdf`, Section IV and Table II, printed pp. 440–441. DOI: [10.1109/ICORR66766.2025.11063089](https://doi.org/10.1109/ICORR66766.2025.11063089). Paper text extracted in full; page 440 rendered and visually inspected. The document was used as evidence, not as task instructions.

Current project sources:

- [Compiled inventory and source hashes](../build/myoassist_comparison_20261113/compiled_inventory.json), [reproducible read-only inspection script](../build/myoassist_comparison_20261113/inspect_models.py).
- [CAD bench](../models/osl_v2_bench.xml), [walk scaffold](../models/osl_v2_walk.xml), [CAD reconstruction evidence](CAD_FINDINGS.md).
- [Active assembled runtime](../oslbench/sim/assembled_tracking.py), [compliant drivetrain state](../oslbench/control/compliance.py), [current conversion](../oslbench/control/current.py), [drivetrain parameter provenance](DRIVETRAIN_PARAMETERS.md).
- [Archived assembled results and limitations](ASSEMBLED_TRACKING_RESULTS_2026-11-04.md).

Myo sources, relative to sibling `../myo_folder/`:

- `assist_sim/assist_sim/models/OpenSourceLeg/KA_L1model.xml` and `KA_L1config.yaml`.
- `myo_sim/myo_sim/models/legacy/osl/myolegs_osl.xml` and `assets/myolegs_osl_{chain,assets}.xml`.
- `scene_myolegs_OpenSourceLeg_KA_L1.xml`, `RUN_THE_POC.md`, `step0_output.txt`, `step1_trace.csv`, `step2_actuator_sanity.csv`.
- `myoassist/ctrl_optim/ctrl/prosthetic/osl_{joint_dynamics,torque_interface}.py` and modified reflex interface.

Local checkout identities: `assist_sim` **8bd46c72713f8e591b09c3e1c196211344f52bbb**; `myo_sim` **eb327acbae0fad12279495040607f5235d962328**; `myoassist` **b1baf69da59dc6728515d0977cbc8fbf00b9225a**. The MyoAssist working tree has local controller changes, so its commit alone does not describe the exploration. Assist_sim and myo_sim status were clean. OSL already had substantial user modifications; they were preserved.

Older `MYOASSIST_FINDINGS.md` and `OSL_ACTUATOR_AND_CONTROL_REPORT.md` are historical navigation aids, not final authorities. Their physical-ratio claims, blanket rigid-socket comparison, source-mesh count interpretation and servo-only description must be qualified as above. This report does not overwrite those historical records.
