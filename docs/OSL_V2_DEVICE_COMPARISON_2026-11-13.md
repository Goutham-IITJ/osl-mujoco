# OSL V2 robotic leg only: our CAD model versus MyoAssist 1.0

13 November 2026. Scope: the powered knee–ankle prosthesis. The human model and its controller are outside this comparison.

Robbins et al., [MyoAssist 1.0](https://doi.org/10.64898/2026.08.25.746839), Section III-B (PDF p. 4), describes its device modelling as a simplified rigid-body, direct-torque representation and points to more detailed actuator dynamics as an extension. Section IV-B (PDF p. 6) describes a four-state impedance controller for powered prostheses; that is the control law, not motor or transmission physics. For exact device values, the primary implementation sources are the local `assist_sim/assist_sim/models/OpenSourceLeg/KA_L1model.xml` and `KA_L1config.yaml` at checkout `8bd46c72713f8e591b09c3e1c196211344f52bbb`. KA is the knee–ankle device; the A variant is ankle-only. Values below are for the inspected local implementation, not values tabulated in the 1.0 paper.

| Device feature | Our CAD OSL V2, current assembled replay | MyoAssist 1.0, local KA_L1 |
|---|---|---|
| Joints | Two powered hinges, knee and ankle | Same two powered hinges |
| Rigid groups | Three rigid groups built from CAD parts | Seven XML bodies including the socket, grouped into three rigid groups by fixed connections |
| Geometric detail | 80 compiled mesh assets and 217 placed visual geoms in the bench; original CAD source has 260 STL assets | Five device mesh assets including the socket, plus simple primitives such as the pylons; excludes the residual-femur mesh from the count |
| Mass and inertia | Part geometry and assigned material densities combined into group mass, centre of mass and inertia | Assembly-level inertial parameters authored in the XML |
| Actuation | Current command drives explicit actuator inertia, damping and friction, then a nonlinear compliant belt applies joint torque. The XML position servos are disabled in this replay. | Direct joint torque: gains 49.4 knee and 58.4 ankle times the clipped command (±2.88), giving ±142.272 and ±168.192 N·m respectively. No separate motor-side state or belt stretch. Gains are not verified physical reductions. |
| Joint limits as authored | Knee −5° to 120°; ankle −30° to 20°. The archived assembled replay used an explicit ±31° ankle override. | Knee 0° to 120°; ankle ±30° |
| Knee–ankle spacing | 331.84 mm | 376.10 mm |
| Foot | CAD Variflex foot. Bench replay has no ground contact; separate ground model uses rigid blade contact geometry. | Custom-foot mesh and three capsule contact geoms |

| Mass by rigid group | Our model | MyoAssist KA_L1 |
|---|---:|---:|
| Above knee, excluding socket | 0.450 kg | 0.189 kg |
| Between knee and ankle | 3.591 kg | 4.047 kg |
| Distal to ankle | 0.915 kg | 0.291 kg |
| **Total excluding socket** | **4.956 kg** | **4.527 kg** |
| Socket (excluded) | Absent from bench | 0.850 kg |

The model mass difference is 0.428 kg (+9.46% for ours), with more of the mass in our proximal and foot groups and less in our shank group. The models use different foot geometry and the physical component inventories have not been reconciled. This is a difference between authored models, not a measured hardware mass error.

The compiled models also let us quantify **rigid-body inertia about each powered joint**. At zero authored knee and ankle angles, with all other coordinates held fixed, the knee inertia is **0.2520 kg·m² for ours versus 0.2692 kg·m² for KA_L1**. At the ankle it is **0.003029 versus 0.002761 kg·m²**. Thus our distal group is about 3.14× heavier, but its inertia about the ankle is only about 10% greater. These are model-derived quantities; they exclude joint armature and external drivetrain state. Calculated from body COM Jacobians and inertial-frame tensors after `mj_forward` in MuJoCo 3.12 using `models/osl_v2_bench.xml` and the saved `scene_myolegs_OpenSourceLeg_KA_L1.xml`. Each joint's contribution is the sum over bodies of mass times squared COM speed plus angular velocity projected through each body inertia tensor, evaluated for unit velocity at that joint.

**Defensible comparison:** Both have the same two joint motions. Our model resolves more of the physical assembly geometry and adds motor-side mechanical states, transmission losses and belt deflection. KA_L1 uses assembly-level rigid bodies and applies bounded torque directly at the joints. Our added mechanics can represent transient effects that the direct-torque model omits. Greater physical accuracy would require testing both models against the same hardware, with matched component inventory, loads and inputs. No new dynamic or hardware test was run.

Local sources: [CAD findings](CAD_FINDINGS.md), [current replay implementation](../oslbench/sim/assembled_tracking.py), [drivetrain provenance](DRIVETRAIN_PARAMETERS.md), [broader version comparison and compiled inventory](MYOASSIST_COMPARISON_2026-11-13.md).
