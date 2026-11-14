# AK80-9 V3 parameter sources and gait sensitivity

14 November 2026.

The [CubeMars AK80-9 V3.0 KV100 product page](https://www.cubemars.com/product/ak80-9-v3-0-robotic-actuator.html) gives the nominal motor constant of 0.095 N·m/A and 9:1 planetary reduction. The [V3 Drive Software User Manual V1.0.1](https://www.cubemars.com/data/cms/202610/ak-series-motor-module-v3-driver-software-user-manual-en.pdf), PDF p. 31, gives the AK80-9 KV100 output torque coefficient of 0.7044 N·m/A and defines the relationship with torque-producing current Iq. Its servo current-loop description on p. 23 points to that same parameter table. A [manufacturer-authored AK Series Module Product Manual V3.2.0, hosted by Robu](https://robu.in/wp-content/uploads/2026/04/ak-series-prodcut-datasheet.pdf) lists 0.5701 for AK80-9 KV100 in its force-control parameter table. These are different published values. The applicable driver revision and actual torque–current slope on the recorded test units remain to be verified. The manual provides no conversion that explains the difference between 0.7044 and the ideal product 9 × 0.095 = 0.855.

| Parameter | Paper reporting |
|---|---|
| Planetary reduction | 9:1, CubeMars product specification. |
| External belt reduction | 50/11 for the archived assembled gait model, from CAD. 4.61 is the Best et al. source build. Confirm the physical pulley tooth counts separately. |
| Rotor torque constant | 0.095 N·m/A as the nominal AK80-9 specification. Archived internal model used 0.1108 N·m/A from Best et al. with current rescaling. The 0.095 equivalence replay explicitly preserved output gain and effective current-dependent friction. |
| Effective gearbox-output torque coefficient | 0.5701 N·m/A in the archived simulations and torque logging. 0.7044 N·m/A is the newer manual reference and an offline sensitivity case. Both are referenced before the external belt. Apply the 9:1 ratio only when converting a rotor-side quantity. |
| Actuator inertia | Archived 0.00983 kg·m² from Best et al. represents rotor plus gearbox at the gearbox output. CubeMars lists rotor inertia of 1118.3238 g·cm², or 0.00011183238 kg·m². The rotor contribution reflected through 9:1 is 0.00905842 kg·m². The gearbox contribution remains unresolved. |
| Damping and friction | 0.0606 N·m·s/rad, 0.171 N·m and 0.0821 N·m/A are Best et al. model coefficients. CubeMars's 0.51 N·m backdrive torque is a different quantity. No directly transferable V3 coefficients were found. Archived effective current-dependent friction slope is 0.04694 N·m per commanded amp after rescaling. |
| Belt stiffness and hardening | 876 N·m/rad and 14913 N·m/rad², with fit range approximately ±0.055 rad. Best et al. fitted the external OSL belt on the ankle. The AK80-9 manufacturer documentation does not specify these values. |

[Best et al.](https://doi.org/10.1109/TMECH.2024.3508469) fitted actuator parameters on a Dephy ActPack 4.1 built around the AK80-9 motor. [Nesler et al.](https://graythomas.github.io/Preprints/NeslerThomasDivekarRouseGregg2022RAL.pdf) independently characterized an AK80-9 using a Dephy FASTER driver and estimated reflected inertia at 0.009211 kg·m². Its friction model and setup differ from the present CubeMars V3 setup. The two inertia estimates and the catalogue rotor contribution support the scale of the borrowed inertia as an engineering starting point. The damping, friction and belt values are published model assumptions carried over to preserve a reproducible baseline. Their accuracy on these units remains unknown.

For direct identification, record the installed board and firmware revisions and compare measured gearbox-output torque against actual q-axis current on a bench fixture. Use both directions and varied speeds and loads to distinguish speed-dependent drag, Coulomb friction and current-dependent losses. Use dynamic data for reflected inertia and separate torque-versus-deflection measurements for the external belt. Existing gait logs contain commanded current and estimated torque, so they cannot independently establish this calibration.

## Offline 0.7044 sensitivity

All six recorded gait CSVs were replayed with unchanged gains, mass, inertia, belt law, timing and effective friction per commanded amp. The 0.7044 case tests a 23.56% increase in the generated torque coefficient. The firmware manual does not specify a separate loss model, so this comparison isolates gain sensitivity and does not establish how to combine the manual coefficient with physical friction. Baseline RMS reproduced the archive within 1e-10 degrees. The 0.095 rotor case at 0.5701 also reproduced the archive within floating-point roundoff.

| Trial | Knee RMS, 0.5701 → 0.7044 | Ankle RMS, 0.5701 → 0.7044 |
|---|---:|---:|
| Slow walk | 0.762 → 0.576° | 0.237 → 0.243° |
| Fast walk | 2.272 → 1.809° | 0.662 → 0.691° |
| Ramp ascent | 1.351 → 1.139° | 0.505 → 0.707° |
| Ramp descent | 2.381 → 1.836° | 0.691 → 0.814° |
| Stairs up | 1.090 → 0.850° | 0.330 → 0.288° |
| Stairs down | 1.277 → 1.002° | 0.675 → 0.669° |

RMS is simulation versus recorded hardware at paired timestamps. All runs completed with zero clipping at the existing simulation limit and zero belt fit exceedances. The comparison uses the recorded angle trajectories on the fixed-base bench simulation. Evidence, replay script, input hashes and paired traces are in `build/ak80_gait_07044_20261114/`. No physical actuator test was performed.
