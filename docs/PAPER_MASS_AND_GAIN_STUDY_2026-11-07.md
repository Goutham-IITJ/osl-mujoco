# Mass reconstruction and controller gain evaluation

## CAD mass reconstruction

The raw URDF declares 0.014753439 kg across its links. The builder replaces the
exported inertials with estimates from mesh volume and assigned density, then
aggregates the parts into three rigid segments. MuJoCo loads those explicit
inertials with `inertiafromgeom="false"`. The resulting difference is the
consequence of replacing the mass properties during reconstruction.
[MuJoCo XML reference](https://mujoco.readthedocs.io/en/latest/XMLreference.html#compiler-inertiafromgeom)

| Segment | Reconstructed mass (kg) | Compiled mass (kg) |
|---|---:|---:|
| Proximal knee | 0.449846436 | 0.449846436 |
| Shank | 3.590757963 | 3.590757963 |
| Foot and output assembly | 0.915158936 | 0.915158936 |
| Total | 4.955763334 | 4.955763334 |

An analytic translated cuboid verified volume, centroid and inertia, including
with reversed winding. An alternative surface-flux calculation reproduced the
volumes of all 260 source meshes, with maximum absolute difference
4.74 × 10⁻²⁰ m³ from the builder. Summing 650 part instances at the same assigned
densities reproduced each compiled segment mass within 1.61 × 10⁻¹² kg.
All visual mesh scales in the URDF are unity. The earlier CAD audit independently
checked centroidal aggregation and the compiled inertia tensors.

The mass arithmetic is internally consistent for the supplied geometry, density
rules and segment assignments. Physical fidelity remains an inventory and
measurement question. About 2.485 kg comes from the default aluminium rule,
0.842 kg from effective BA30 battery densities and 0.387 kg from the steel
load-cell rule. The optional added motor mass is zero. The inventory includes
two instances of the same BA30 rear-cover mesh, each estimated at 0.298 kg.
Verify which battery instances and parts the actual assembly contains before
interpreting those estimates. The small displayed-geometry omissions retain
their inertial contributions.

The local MyoAssist XML totals 5.377474 kg including its 0.849991 kg socket.
Removing that socket yields 4.527483 kg. Our bench model is 0.421711 kg lighter
than the socket-inclusive total and 0.428280 kg heavier than its socket-excluded
total. Component equivalence between the two models remains unverified, so
neither difference establishes an error against a physical measurement.
Use the compiled 4.955763 kg as a model-derived mass in Section III-B.

For physical verification, define the exact bench configuration and weigh the
assembled leg, then weigh the moving segment assemblies with their attachments.
Reconcile the motor, battery, load-cell, foot and mounting components with
`Part_Mass_Inventory.csv`. Review assigned densities and possible overlapping
assembly representations by component. A total weight establishes total mass
only. Check segment centres of mass and inertia before adjusting dynamics.

## Current-domain gain study

The knee gains `(1.5, 0.15, 0.06)` and ankle gains `(2.0, 0.0, 0.075)` match the
values declared by the five supplied hardware controller scripts. In the
reported assembled replay, simulation uses these gains with configuration C,
which uses referred actuator-side finite differences for derivative feedback.
Hardware differentiates joint error. The sweep holds configuration C fixed.
The units of Kp, Ki and Kd are A/deg, A/(deg·s) and A·s/deg, respectively.

The development trajectories were faster walking and ramp ascent. The
evaluation trajectories were slow walking, ramp descent and both stair
trajectories. The latter were excluded from gain selection in this experiment
but had previously been inspected in the project. All targets were the recorded
references. Initial poses, coupled bench model, drivetrain, 0.5701 Nm/A
conversion, provisional ±31° runtime ankle envelope, 5 ms controller interval
and 0.5 ms physics step were fixed. Evaluation was repeated at 0.25 ms physics
with the controller interval unchanged.

For each joint, the grid used 50%, 100% and 150% of baseline Kp and Kd and
integral gains of 0, 0.15 and 0.30. The other joint retained baseline gains.
The two conditional grid winners were also combined and evaluated on the
development trajectories. After removing repeated baseline evaluations, this
gave 54 candidates. The objective was the mean of four ratios, one for each
development trajectory and joint. Each ratio divides simulated target tracking
RMSE by the corresponding hardware-gain baseline simulation RMSE. Admissibility
required completion, zero simulated current clipping and zero belt fit
exceedances for both joints. Hardware angle agreement was recorded as a
secondary metric and was excluded from gain selection.

All 54 candidates met the development constraints. The hardware gains ranked
27th by the development tracking score. The combined candidate had the lowest
tested development score, 0.6637 relative to the baseline of 1.000. It was the
only evaluated candidate within 5% of that score. The selected proportional
and derivative gains were on the high edges of the grid. The search sampled
one joint at a time plus the combined winner and does not establish the global
optimum of the coupled system.

| Gain set | Knee Kp / Ki / Kd | Ankle Kp / Ki / Kd | Development score |
|---|---|---|---:|
| Hardware baseline | 1.50 / 0.15 / 0.060 | 2.00 / 0.00 / 0.0750 | 1.0000 |
| Development winner | 2.25 / 0.30 / 0.090 | 3.00 / 0.00 / 0.1125 | 0.6637 |

| Evaluation trajectory | Knee target RMSE, baseline → winner (°) | Ankle target RMSE, baseline → winner (°) | Knee peak requested current, baseline → winner (A) | Winner admissible |
|---|---|---|---|---|
| Slow walking | 1.620 → 1.079 | 0.578 → 0.394 | 18.20 → 25.76 | Yes |
| Ramp descent | 5.492 → 3.663 | 1.247 → 0.808 | 35.10 → 50.78 | Yes |
| Stairs descent | 3.140 → 2.052 | 1.249 → 0.807 | 24.47 → 32.12 | Yes |
| Stairs ascent | 3.026 → 2.022 | 0.908 → 0.642 | 44.56 → 70.23 | No |

The combined gains lowered target tracking error on every evaluation trajectory,
including the clipped stairs ascent run, while raising applied RMS current in
all eight joint–trajectory combinations. The mean of the applied RMS current
ratios was 1.279. The stairs ascent knee exceeded the simulation's 60 A clip
in its raw PID request. The logged applied peak was 60.00 A. It clipped for
one 5 ms controller interval, held across 10 physics steps. The simulation
completed the trial and the gain set failed the study's zero-clipping rule.
At 0.25 ms it still exceeded the clip, with a peak request
of 70.10 A and 20 clipped steps. The hardware baseline passed all four evaluation
trials at both step sizes. The 60 A threshold is the configured simulation
limit. These commanded and applied simulation currents do not establish
hardware current or a physical operating limit.

Improved reference tracking also changes agreement with the fixed-gain
hardware recordings. For example, ankle hardware-difference RMSE on ramp
descent increased from 0.691° to 0.949°. The recordings only establish hardware
behavior for the recorded controller, so alternative simulation gains do not
have a matching hardware trial. Keep the hardware gains for the paper's
sim-to-real baseline and describe the sweep as a simulation sensitivity study.

## Follow-up ranking across all six trajectories

We extended the same grid retrospectively to all six recorded references and
ranked candidates by the mean of the 12 joint/trajectory target RMSE ratios
to the hardware baseline. At 0.5 ms, 44 of 54 candidates passed all six.
The lowest admissible score was 0.835701, a 16.4% reduction, with knee gains
1.50/0.15/0.060 and ankle gains 3.00/0/0.1125. A repeat of these gains at
0.25 ms passed all six again. Peak requested current was 51.45 A in the
ranking and 51.52 A in the finer-step check, both at the ankle in fast walking.
Ankle RMSE against the recorded hardware worsened on five of the six
trajectories. This ranks the tested sets for simulated target tracking on
these recordings. It provides no independent evaluation of the selected set.
The original split study and the hardware-gain sim-to-real baseline still
serve their original purposes. See
`build/assembled_gain_ranking_20261107/Ranking.json` and its trial log.

## Text for the manuscript

**Section III-B, model construction.** The inertial properties in the exported
URDF were replaced using mesh volumes and assigned material densities. The
resulting part masses, centres of mass and inertia tensors were aggregated into
three rigid segments and encoded as explicit MJCF inertials. The compiled
fixed-base model had a total reconstructed mass of 4.956 kg. A separate
surface-flux volume calculation reproduced the segment mass totals within
numerical precision. The mass estimates depend on the assumed material
densities and the components represented by the CAD assembly.

**Section III-E, parameter provenance and calibration methodology.** The
hardware controller gains were used as the baseline in the assembled
simulation, which used configuration C for derivative feedback. We evaluated
54 gain sets with fixed model parameters and selected candidates by simulated
target tracking error on two development trajectories. Each admissible trial
had to complete without current clipping or leaving the belt model's fitted
range. The baseline and the lowest-error development candidate were then
compared on four trajectories excluded from gain selection.

**Section IV, characterization and results.** The selected candidate reduced
the normalized development tracking score by 33.6%. On stairs ascent, however,
it requested a peak simulated knee current of 70.23 A, exceeding the configured
60 A clip. The hardware gains completed all four evaluation trajectories
without clipping and were retained for the reported hardware–simulation
comparisons. The evaluation figure shows the tracking and current trade-off.

## Evidence and reproduction

The mass check is under `build/assembled_mass_check_20261107`. The sweep is under
`build/assembled_gain_sweep_20261107`, with the declared protocol, per-run metrics,
candidate table, selection record, and PNG and SVG figures. The grid figure
shows one Ki slice per joint with the opposite joint at baseline. The complete
grids in `build/assembled_gain_sweep_figures_20261107` show all Ki slices and
the additional combined candidate for both tuning score and peak requested
current. That folder also contains the stairs ascent angle and current traces
with a zoom of the 5 ms clipping event and the source simulation CSV.
The evaluation figure uses hatching to identify the rejected trial. Baseline hardware
difference RMSEs matched the archived results for all six trajectories. The
four assembled runtime unit tests passed.

```powershell
.venv\Scripts\python.exe -B tools\audit_assembled_mass.py --outdir build\mass_check_new
.venv\Scripts\python.exe -B experiments\sweep_assembled_gains.py --outdir build\gain_sweep_new
.venv\Scripts\python.exe -B experiments\plot_assembled_gain_sweep.py --sweep build\assembled_gain_sweep_20261107 --outdir build\gain_sweep_figures_new
.venv\Scripts\python.exe -B experiments\rank_assembled_gains.py --sweep build\assembled_gain_sweep_20261107 --outdir build\gain_ranking_new
```
