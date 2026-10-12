# Physical ankle control alignment

**SIMULATION ONLY — NOT A VALIDATED MODEL OF THE PHYSICAL ANKLE.** No quantity in this
document has been measured on OSL V2 hardware.

> ### Historical stub-result correction
>
> **The §10 metrics in the first version of this document came from
> `tests/stub_mujoco.py`, not from MuJoCo.** `StubModel.__init__(path)` stores the MJCF
> path and never parses it — it hardcodes its own ranges and inertias — so the run
> reported `models/osl_v2_bench.xml`, exposed `ankle_qpos`/`ankle_dof`, and produced
> correctly-labelled plots while no line of the CAD model was compiled. It is also a
> **different plant**: ankle armature 0.0 vs 0.008, damping 0.05 vs 0.25, frictionloss
> 0.05 vs 0.3.
>
> Fixed: the experiment now **requires** real MuJoCo and exits with an explanation
> otherwise; `--backend stub` is explicit, writes to `build/ankle_current_step_STUB/`,
> and marks every CSV, metrics file and plot `STUB — NOT MUJOCO`. The metrics file
> records the backend, MuJoCo version, model path + SHA, `nq/nv/nu/nbody/ngeom/nmesh`,
> timestep, step and update counts, and the ankle's compiled armature/damping/
> frictionloss with a `model_ankle_properties_match_mjcf` flag.
>
> **The former §10 numbers are stub figures and are retained only as a clearly labelled
> historical wiring check.** A later real-MuJoCo result is recorded in §9 and §10.
> Every actuator and belt parameter is either derived from Best et al. 2025 (a different
> build), reported second-hand from a script that is **not in this workspace**, or taken
> from the official CubeMars product specification recorded below. Nothing here
> authorises a hardware test.

This document reconciles three control pathways that are easy to confuse, records what
is confirmed versus assumed, and states exactly what would have to be measured before
any of it could be called validated.

## Current real-MuJoCo verification

The authoritative compiled model currently reports `nq=2`, `nv=2`, `nu=2`,
`nbody=4`, `ngeom=217`, and `nmesh=80`, from `models/osl_v2_bench.xml`
(SHA-256 prefix `c417e691ff90a6f4`). A text count finds 221 `<geom>` tags, but
four of those are `<default>` templates rather than instantiated geoms; MuJoCo
therefore correctly compiles 217 geoms. The earlier `221` figure compared that
template-inclusive text count with compiled metadata. The protected XML and its
compiled inputs were not changed to force either number.

The geometry sign derivation now uses the actual compiled vertices of mesh
`z_lowprofilevariflexfoot_ossur_vlpe5xx0` (the
`z_lowProfileVariFlexFoot_ossur_VLPE5XX0.stl` asset), not the foot geom origin.
MuJoCo's compiled vertex array already includes its mesh preprocessing/centering
offset, so the helper applies the authored geom pose and the compiled body
forward-kinematics exactly once and does not apply `mesh_pos`/`mesh_quat` a second
time. The transformed mesh spans about 242 mm along X and 56 mm along Z;
representative forefoot and heel vertices move oppositely when the compiled ankle is
probed at positive and negative angles. Positive MuJoCo q raises the forefoot and
lowers the heel, so it is dorsiflexion. With the reported (not hardware-verified)
convention that positive encoder angle means plantarflexion, the resulting
`mjcf_q_to_encoder_sign` is `-1`. This remains **provisional-by-premise** because
the encoder polarity is second-hand. It is independent of
`controller_output_to_motor_current_sign`, which remains a separate controller
mapping.

The ankle metrics use a 2% settling band, i.e. `0.1 deg` for this 5 deg step, and
settling is the first sample after the last excursion outside that band (therefore
the trace must remain in-band through the end). In the real run, the last out-of-band
sample was `t=1.5235 s`, at `4.89999346 deg`; the first in-band sample was
`t=1.5240 s`, at `4.90000927 deg`, so `settling_time_s=1.524`. The response never
exceeded the target by more than the 2% meaningful-overshoot threshold, so
`peak_time_s` is reported as **not applicable**, rather than as the final recording
timestamp.

---

## Official AK80-9 V3.0 product specification

The official CubeMars product page was retrieved on **2026-10-12**:

<https://www.cubemars.com/product/ak80-9-v3-0-robotic-actuator.html>

The page currently lists the following AK80-9 V3.0 specifications:

| Manufacturer-listed item | Value | Status in this project |
|---|---:|---|
| torque constant | 0.095 N.m/A | **Manufacturer-listed; shaft reference is not explicitly stated on the page** |
| internal reduction | 9:1 | **Manufacturer-listed for AK80-9 V3.0** |
| rated torque | 9 N.m | **Manufacturer-listed actuator rating** |
| peak torque | 22 N.m | **Manufacturer-listed actuator rating** |
| rated current | 12 A | **Manufacturer-listed; motor/driver/protocol scope is not further resolved here** |
| peak current | 28 A | **Manufacturer-listed; motor/driver/protocol scope is not further resolved here** |

The product comparison separately identifies the AK80-6 as a **6:1** actuator. This
supports the possibility that `0.095 × 6 ≈ 0.5701`, but it does not establish how the
existing script computed its value or prove that an AK80-6 parameter was used in an
AK80-9 setup.

These are manufacturer specifications for the actuator product, not measurements of
the OSL V2 rig. My current working understanding is that the listed `0.095 N.m/A`
refers to the motor rotor before the internal planetary gearbox. That interpretation
is **not independently verified hardware documentation**: the product page itself does
not state the shaft reference. Under that provisional interpretation, `0.095 × 9 =
0.855 N.m/A` is the ideal actuator-output equivalent when losses are neglected. The
simulation retains this working hypothesis alongside the existing script and paper
alternatives.

## 0. What was actually checked, and what could not be

Before anything else, because several premises could not be verified:

| Premise | Status |
|---|---|
| `experiments/run_step_response.py` and `tests/test_step_response.py` exist | **CONFIRMED** — both present, dated 2026-10-10, previously untracked |
| The hardware script `ankle_step_test_PID_control_me.py` | **NOT IN THIS WORKSPACE.** Searched the whole filesystem. Every number attributed to it below is second-hand and unverified. |
| The CubeMars AK80-9 V3.0 product page | **CONFIRMED as an official source** — retrieved 2026-10-12; the torque-constant shaft reference remains unstated |
| The bench model has an ankle joint with its own actuator | **CONFIRMED** from the MJCF |
| The CAD contains 50- and 11-tooth pulleys | **CONFIRMED** from the mesh assets |
| The bench model represents the ankle *drivetrain* | **FALSE — see §2.** The belt and pulleys are visual geoms only. |

Because the script is unavailable and the product page does not state the torque
constant's shaft reference, the torque conversion remains configurable. The product
ratings and 9:1 reduction are primary-source specifications; the gains, encoder
convention and script-reported current semantics remain separately identified
assumptions or second-hand values.

---

## 1. Three pathways that must not be confused

| | A. Knee bench (original) | B. Option C drivetrain | C. Physical ankle (this work) |
|---|---|---|---|
| Joint | knee | knee | **ankle** |
| Actuation | MuJoCo `position` actuator | belt torque via `qfrc_applied` | belt torque via `qfrc_applied` |
| Control domain | joint torque, **radians** | joint torque → current, radians | **current, degrees** |
| Gains | Kp 600 N·m/rad, Kd 17.253 N·m·s/rad | same | **Kp 0.75 A/deg, Kd 0.03 A/(deg/s)** |
| Loop rate | every physics step (2 kHz) | every physics step | **1 kHz, command held** |
| Entry point | `experiments/run_step_response.py` | `experiments/run_bench_ab19_drivetrain.py` | `experiments/run_ankle_current_step.py` |

**600 N·m/rad and 0.75 A/deg are not comparable numbers.** They have different units,
act in different domains, and close through different hardware. Neither experiment
replaces the other and no gain was changed to make any of this work.

---

## 2. The finding that constrains everything else

**The MJCF contains no transmission.** Searched for `<tendon>`, `<equality>`,
`gear=` and `<general>`: zero matches. The `gt3_5mm_325mm_belt`,
`p_b0001_InputPulley_5mm_11teeth` and `p_b0012_OutputPulley_5mm_50teeth` meshes are all
`class="visual"`.

So in the protected model the ankle is **directly actuated**: its position servo applies
torque straight to the joint, with no 9:1 planetary and no belt anywhere in the physics.
The belt you can see in the viewer is scenery.

Consequences, stated plainly:

- The original bench **does not** represent the physical ankle drivetrain, and no amount
  of gain tuning on the position servo would make it do so.
- Any ankle experiment that wants a drivetrain has to supply one. This work supplies it
  by composing the existing Option-C `DrivetrainLayer`, which is why the belt equations
  are **not** restated anywhere in the new code.
- The pulley meshes are still useful evidence: they are where the 50/11 tooth counts
  come from.

**What the CAD does confirm about the ankle belt.** The 50-tooth output pulley sits in
body `foot` (joint side) and the 11-tooth input pulley in body `shank` (motor side), so
the ankle belt **reduces** by 50/11 — the direction, not just the magnitude, is read off
the geometry rather than assumed.

---

## 3. Architecture

```
  ┌─────────────────────── PHYSICAL RIG (not in this workspace) ────────────────────┐
  │                                                                                  │
  │  external encoder ──deg──> Python PID @1 kHz ──amps──> driver CURRENT LOOP       │
  │        ^                   Kp 0.75 A/deg                      │                  │
  │        │                   Kd 0.03 A/(deg/s)                  v                  │
  │        │                   Ki 0                           AK80-9 motor           │
  │        │                   ENCODER_TO_MOTOR_SIGN = -1         │                  │
  │        │                                                       v                  │
  │        │                                            internal 9:1 planetary       │
  │        │                                                       │                  │
  │        │                                                       v                  │
  │        └────────────────── ankle joint <──── external belt ────┘                  │
  └──────────────────────────────────────────────────────────────────────────────────┘

  ┌─────────────────────────── SIMULATION (this work) ───────────────────────────────┐
  │                                                                                  │
  │  EncoderConvention ──deg──> CurrentPID @1 kHz ──amps──> ■ IDEAL CURRENT LOOP ■   │
  │  (deg/rad + sign)           same law, same units            (command = applied)  │
  │        ^                                                      │                  │
  │        │                                      ConversionAudit.layer_current_for  │
  │        │                                      (rescales into the layer's frame)  │
  │        │                                                      v                  │
  │        │                                   DrivetrainLayer — Best et al. eqs     │
  │        │                                   rotor k_t → n_a → actuator shaft      │
  │        │                                   J_a, B_a, Coulomb + current friction  │
  │        │                                                      │                  │
  │        │                                              COMPLIANT belt             │
  │        │                                   θ_s = θ_j − θ_a/n_t                   │
  │        │                                   τ_j = −sgn(θ_s)(p1|θ_s| + p2|θ_s|²)   │
  │        │                                                      │                  │
  │        │                          qfrc_applied[ankle_dof] <───┘                  │
  │        │                                   │                                      │
  │        └─────── MuJoCo ankle hinge <───────┘   (position servo DISCONNECTED)      │
  └──────────────────────────────────────────────────────────────────────────────────┘

  ■ = where the simulation substitutes an assumption for hardware.
```

**The two substitutions, both labelled in code:**

1. **Ideal current loop.** Applied current equals the clipped command within each
   control interval. No winding model, no first-order lag. Deliberately *not* added:
   no measured time constant exists for this driver, and inventing one would make the
   simulation look more credible without being more correct.
2. **The belt is the paper's model with the CAD's ratio** — a labelled hybrid
   (`ANKLE_PAPER_CAD_BELT`). `PAPER` itself is never mutated.

---

## 4. Parameter provenance

Frame abbreviations: **R** rotor, **AO** actuator output (after the 9:1), **J** joint
(after the belt).

| Parameter | Value | Units | Frame | Source | Status | Ratio applied? |
|---|---|---|---|---|---|---|
| `n_a` planetary | 9.0 | – | R→AO | PAPER; AK80-**9** name | confirmed-ish | once, inside `DrivetrainLayer` eq (1) |
| `n_t` belt (CAD) | 4.545455 | – | AO→J | **MJCF mesh names, 50/11** | **CONFIRMED** | once, in `θ_s = θ_j − θ_a/n_t` |
| `n_t` belt (paper) | 4.61 | – | AO→J | Best et al. | confirmed for *their* build | alternative, `--belt paper` |
| overall (CAD) | 40.909 : 1 | – | R→J | 9 × 50/11 | derived | — |
| overall (paper) | 41.490 : 1 | – | R→J | 9 × 4.61 | derived | — |
| `k_t` rotor | 0.1108 | N·m/A | R | PAPER fit | provisional | — |
| `k_t` manufacturer-listed | 0.095 | N·m/A | rotor-side **working interpretation; page does not state frame** | CubeMars AK80-9 V3.0 product page | **listing confirmed; rotor interpretation provisional** | — |
| `k_t_output` **script** | 0.5701 | N·m/A | AO (claimed) | hardware script, second-hand | **UNRESOLVED** | default |
| `k_t_output` **paper** | 0.9972 | N·m/A | AO | 0.1108 × 9 | derived | `--kt paper` |
| `k_t_output` **manual_rotor** | 0.8550 | N·m/A | AO | 0.095 × 9, ideal/no-loss derivation | **preferred working hypothesis, provisional** | `--kt manual_rotor` |
| `J_a` | 9.83e-3 | kg·m² | AO | PAPER | provisional | already includes 9:1 — **do not reflect again** |
| `B_a` | 6.06e-2 | N·m·s/rad | AO | PAPER | provisional | — |
| `f_c`, `f_g` | 0.171, 0.0821 | N·m, N·m/A | AO | PAPER | provisional | — |
| `p1`, `p2` | 876, 14913 | N·m/rad, N·m/rad² | J | PAPER Fig. 3 fit | provisional | — |
| ankle armature | 0.008 | kg·m² | J | **MJCF** | **CONFIRMED** | — |
| ankle damping | 0.25 | N·m·s/rad | J | **MJCF** | **CONFIRMED** | — |
| ankle frictionloss | 0.3 | N·m | J | **MJCF** | **CONFIRMED** | — |
| ankle ROM | [−30, +20] | deg | J | **MJCF** | **CONFIRMED** | — |
| ankle forcerange | ±168.2 | N·m | J | **MJCF** | **CONFIRMED** | — |
| foot mass | 0.9152 | kg | J | **MJCF/CAD** | **CONFIRMED** | — |
| Kp | 0.75 | A/deg | encoder | script, second-hand | reported | — |
| Ki | 0 | A/(deg·s) | encoder | script, second-hand | reported | — |
| Kd | 0.03 | A/(deg/s) | encoder | script, second-hand | reported | — |
| `ENCODER_TO_MOTOR_SIGN` | −1 | – | — | script, second-hand | reported | — |
| encoder sign | −1 | – | J↔encoder | **derived at runtime**, see §6 | provisional | — |
| current clip | 60 | A | drive | script, second-hand | **simulation only** | — |
| rated / peak torque | 9 / 22 | N·m | actuator | CubeMars AK80-9 V3.0 product page | **manufacturer-listed** | — |
| rated / peak current | 12 / 28 | A | scope not stated | CubeMars AK80-9 V3.0 product page | **manufacturer-listed; scope unresolved** | — |

---

## 5. Torque and current conversion

### The chain, with each ratio applied exactly once

```
I_q [A, against k_t_output]
   │  ConversionAudit.layer_current_for() — rescales by k_t_output / (k_t·n_a)
   v
I_q [A, rotor frame the layer expects]
   │  × k_t         (rotor constant)          ──> τ [N·m]  ROTOR
   │  × n_a = 9     (planetary, ONCE)         ──> τ [N·m]  ACTUATOR OUTPUT
   v
actuator shaft dynamics: J_a·θ̈_a = τ_m − τ_f − B_a·θ̇_a − τ_j/n_t
   │
   v
θ_s = θ_j − θ_a/n_t          (belt ratio enters HERE, ONCE, in the kinematics)
τ_j = −sgn(θ_s)(p1|θ_s| + p2|θ_s|²)        ──> τ [N·m]  JOINT
```

**The double-count guard.** `DrivetrainLayer` implements eq (1) with the *paper's* rotor
`k_t` and its own `n_a`. Handing it a current sized against a different output constant
would silently apply the wrong torque per amp. `layer_current_for()` rescales by
`k_t_output / (k_t·n_a)`, so the amps keep their meaning (same physical output torque)
while the layer keeps its own correct gear bookkeeping. Verified numerically to 6e-9 N·m
over a full run, and tested in `test_the_planetary_ratio_is_applied_exactly_once`.

### Four torque frames, separately named

`I_q × k_t_output` is **not established as motor-rotor torque** — the script-selected
constant is claimed to be an actuator-**output** constant, so the product is an estimate
at the output shaft. The CSV
therefore carries four distinct channels and never conflates them:

| Channel | Frame | Validity |
|---|---|---|
| `implied_rotor_torque_Nm` | ROTOR | DERIVED as output/`n_a`; only as trustworthy as `n_a` |
| `estimated_actuator_output_torque_Nm` | ACTUATOR OUTPUT | `I_q × k_t_output` — an **estimate** through an unresolved constant |
| `belt_torque_actuator_side_Nm` | ACTUATOR OUTPUT | from the layer's belt state |
| `belt_torque_joint_side_Nm` | JOINT | `= actuator side × n_t`, asserted once |

None is a measured torque.

### The torque-constant problem

| Candidate | Value at AO | Implied rotor (÷9) | Provenance |
|---|---|---|---|
| script `KT = 0.5701` | 0.5701 | 0.063344 | script-reported AO value; shaft unresolved |
| manufacturer-derived `0.095 × 9` | 0.8550 | 0.095000 | manufacturer-listed 0.095, treated as rotor-side; ideal/no-loss derivation |
| PAPER `k_t × n_a` | 0.9972 | 0.110800 | paper fit, not AK80-9 V3.0 primary data |

The manufacturer-derived value is not a measured output constant: it follows my
current understanding that the page's 0.095 N·m/A is rotor-side and assumes that the
9:1 reduction is lossless. The page does not confirm that shaft reference. It is the
preferred **working hypothesis**, not a manufacturer-confirmed output constant. The
sensitivity study therefore retains all three configurations and does not alter or
discard results based on this preference.

**One checkable coincidence — a LEAD, not proof.** `0.5701 / 0.095 = 6.0011`. Six is the **AK80-6**
reduction — not the AK80-9 the rig reportedly uses. If the script's `KT` was built by
applying 6:1 to a ~0.095 N·m/A rotor constant, every torque it logged is low by
9/6 = **1.5×**.

This remains a **hypothesis** from a four-significant-figure coincidence, and it says
nothing about how the source value was actually generated. The official AK80-9 page
confirms 9:1 for the named product and separately identifies AK80-6 as 6:1, but it does
not connect the existing script value to either ratio. Until the script's provenance and the 0.095 shaft reference are obtained, `--kt`
selects the parameterisation for a simulation run and the default merely reproduces
the script; the provisional working preference does not change that default.

### The informal 4.8–5 N·m observation

A noisy graph was read by eye during a lab session; ~4.8–5 N·m was multiplied by ~4.61 to
give ~22–23 N·m. **This is not ground truth and must not be used to fit anything.**

| Interpretation | Joint-side result |
|---|---|
| as logged × n_t (what produced "22–23") | 21.8–23.1 N·m |
| if the 6-vs-9 hypothesis holds (×1.5 first) | 32.7–34.6 N·m |

Two unresolved questions sit under it: *which shaft* the plotted quantity was defined at,
and *which* torque constant produced it. Until both are answered, applying 4.61 to it is
an operation on an unknown quantity. The ×1.5 column is not a correction — it is the size
of the open question.

---

## 6. Signs — two separate mappings, and one is derived from the other's absence

**These are different quantities and the first must never be inferred from the second.**

| | Name | What it is | Source |
|---|---|---|---|
| **1** | `mjcf_q_to_encoder_sign` | MuJoCo ankle coordinate → encoder degrees. A **coordinate-frame** fact. | **DERIVED from geometry**, below |
| **2** | `controller_output_to_motor_current_sign` | controller output → driver current command. A **controller-internal** fact. The script's `ENCODER_TO_MOTOR_SIGN`. | REPORTED, second-hand |

### Correction to the previous version of this section

It claimed "three independent lines agree on `s_enc = −1`". **Two of the three were
circular** — the loop-stability condition and "the physical test ran without running
away" both involve mapping 2. Only the ROM-asymmetry argument was independent, and it is
weak alone. The derivation below replaces all of it and touches mapping 2 nowhere.

### Mapping 1, derived from geometry

Inputs:

| | Input | Status |
|---|---|---|
| (a) | ankle hinge axis `(0, 1, 0)` | **CONFIRMED** from the MJCF (the knee's is `(0, −1, 0)` — not shared) |
| (b) | Vari-Flex foot mesh in the foot-body frame, whose origin **is** the ankle joint: x ∈ [−0.1665, +0.0756] m, z ∈ [−0.0915, −0.0350] m; extent 242 mm along X vs 56 mm along Z | **CONFIRMED** — parsed from the STL vertices, transformed by the geom's pos/quat. Not the mesh-frame origin. |
| (c) | anterior = −X | **SELF-ESTABLISHING from (b)**: the long axis is X, 167 mm one way against 76 mm the other; on a prosthetic foot the long side is the forefoot. No longer taken from the CAD notes. |
| (d) | plantarflexion = toe down, heel up | definitional |

For a hinge about **+Y**, a positive rotation gives `z' = −x·sin θ + z·cos θ`, so
`dz/dθ|₀ = −x`:

| Point | x [m] | dz/dθ [m/rad] | Positive θ |
|---|---|---|---|
| toe | −0.1665 | **+0.1665** | **raises** the toe |
| heel | +0.0756 | **−0.0756** | **lowers** the heel |

Toe up **with** heel down is **dorsiflexion**, and both ends agree — so the result does
not depend on which point is picked. Checked at finite angles too: at +20° the toe rises
61.6 mm and the heel drops 20.9 mm, so it is not a small-angle artefact.

```
positive MuJoCo ankle q  =  DORSIFLEXION
encoder calls PLANTARFLEXION positive  (REPORTED)
=> the conventions oppose  =>  mjcf_q_to_encoder_sign = -1
```

**Corroboration, also free of mapping 2:** under this mapping the authored ROM
[−30°, +20°] reads as 30° plantarflexion / 20° dorsiflexion. A prosthetic ankle allows
more plantarflexion than dorsiflexion, so the asymmetry agrees.

### The one residual uncertainty — a premise, not a step

Input (d) pairs with the **reported** claim that the encoder calls plantarflexion
positive. The hardware script is not in this workspace, so that is second-hand. The
geometry is solid; the mapping inherits exactly the confidence of that single premise.
**If the encoder actually counts dorsiflexion positive, mapping 1 flips to +1** and
nothing else in the derivation changes.

So it is **provisional-by-premise, not unresolved-by-geometry**. It stays an explicit
parameter (`--encoder-sign`, default 0 = derive at runtime), every run prints how it was
obtained, and the positive-feedback detector stays armed. To settle it: command a known
small plantarflexion on the rig and record the encoder's sign, or read the encoder's
mounting and counting direction off the assembly.

### Runtime derivation, and why it demands two points

`derive_mjcf_q_to_encoder_sign()` probes ±15° on the **compiled** model and tracks
representative forefoot and heel vertices from the compiled Vari-Flex mesh. It does not
use foot geom origins as toe/heel points: MuJoCo recenters mesh assets during
compilation. The helper uses actual compiled vertices, applies the authored geom pose
and body forward kinematics once, and respects the compiled mesh offset without applying
`mesh_pos`/`mesh_quat` twice. It **requires the toe and heel to move oppositely** — that
is what a hinge must do, and demanding both turns a single-point guess into a check. It
returns `None` with a stated reason (rather than a guess) when the geometry cannot be
read, which is the case on the stub. Anterior is decided from the geometry's own longer
axis, not from a convention.

### The loop condition is a consequence, not evidence

`check_sign_consistency()` takes both mappings as given and reports
`s_enc · s_mot = +1` ⇒ negative feedback. With mapping 1 = −1 and mapping 2 = −1 the
product is **+1**, so the loop is negative feedback. This is **downstream** of the two
mappings and is reported as such; using it to infer mapping 1 is the circularity removed
above.

### Torque-chain consistency, checked against both mappings

`I_q > 0 → τ_m > 0 → θ_a rises → θ_s = θ_j − θ_a/n_t < 0 → ` paper (4) gives `τ_j > 0 →
qfrc_applied > 0 → q rises` (dorsiflexion, by mapping 1) `→ encoder reading falls`
(mapping 1 again, sign −1). With mapping 2 = −1 a *positive* encoder error produces a
*negative* current, which lowers `q` toward plantarflexion and *raises* the reading
toward the reference. Consistent end to end.

### Joint limits in both frames

With encoder sign −1 the mapping is `enc = −deg(q)`, so the interval **ends swap**:

| Frame | Range | Source |
|---|---|---|
| CAD ROM, MuJoCo coords | [−30.0, +20.0]° | **CONFIRMED** from the MJCF |
| CAD ROM, **encoder frame** | **[−20.0, +30.0]°** | derived, sign applied |
| Physical controller, encoder frame | [−29.0, +29.0]° | **REPORTED**, second-hand |

**Mismatch: the reported physical range extends 9.0° beyond the CAD travel at the low
encoder end** (−29° would need +29° of MuJoCo coordinate, but the MJCF authors +20°). The
high end has 1° of margin. So a command the physical rig accepts can be outside this
model's authored travel; it would hit a joint stop here. The MJCF is protected and has
**not** been widened to hide that.

The 5° step is inside **both** ranges, which is why it stays the right test while the
mapping is provisional. `limit_report()` prints this table every run, and
`test_the_encoder_frame_limit_mapping_is_computed_not_assumed` pins it. **Do not sweep
wider until the mapping is resolved.**

### A reversed sign does **not** crash

This matters for interpretation: with `s_enc = +1` the run stays finite — the authored
ROM catches it — and all metrics still compute. The focused sign test confirms a large
standing error and displacement away from the reference; saturation is not required
when the 60 A simulation clip exceeds the wrong-sign steady command. The experiment
detects that signature and prints a `POSITIVE-FEEDBACK SIGNATURE` warning, because a
completed run with plausible-looking numbers is the failure most likely to be mistaken
for a result.

---

## 6b. The driver's current loop is now OPTIONAL, and off by default

Until now this document recorded "ideal current loop" as a flat assumption. It is now an
adjustable one, so its consequences can be measured instead of argued about.

```
dI_actual/dt = (I_target - I_actual) / tau_i

integrated exactly over one physics step:
    I_next = I_target + (I_previous - I_target) * exp(-dt / tau_i)
```

| | |
|---|---|
| where it sits | `PID -> sign -> clip -> [current loop] -> ConversionAudit rescale -> DrivetrainLayer` |
| what it operates on | **amps only**. It applies no planetary ratio and no belt ratio — those stay in `layer_current_for` (n_a, once) and the layer's kinematics (n_t, once) |
| timing | evolves at the **0.5 ms physics step** while the 1 kHz command is held, so the current keeps responding between controller updates |
| initial condition | `i0 = 0 A`, restored by `reset()`, recorded in the run metadata |
| default | `--current-loop-tau-ms 0` = exact passthrough |

**`tau_i = 0` is a special case, not a limit.** It assigns the target directly, never
evaluates `exp(-dt/0)`, and introduces no one-step delay.
`test_zero_tau_reproduces_the_pre_change_step_bit_for_bit` reconstructs the pre-change
`step()` body and compares 1200 samples across five channels: **0 differ**. The exact
exponential form is used rather than forward Euler because at tau_i = 1 ms and
dt = 0.5 ms, Euler's coefficient would be 0.5 against the true 0.6065 — a 21 % per-step
error — and it goes unstable for dt > 2·tau.

**Five distinct current channels** now appear in the CSV, so a target can never be
mistaken for a measurement: `pid_output_pre_sign_A`, `current_request_A` (signed,
pre-clip), `current_applied_A` (signed, clipped — the command), `current_actual_modelled_A`
(after the loop), `layer_rotor_current_A` (rescaled for the layer). The **modelled
actual** current is what drives the drivetrain — verified by a test that fails if a
lagged run produces identical joint motion to an ideal one.

**Nonzero tau_i is an illustrative model assumption. The real driver's time constant has
never been measured here, and the sweep in
`build/ankle_current_loop_sensitivity/` cannot estimate it** — it only measures how much
this model's answer depends on that unknown.

## 7. Current limits

| Figure | Value | What it is |
|---|---|---|
| `sim_clip_A` | 60 A | the script's code-level ceiling, reproduced so the simulated controller saturates where the physical one does |
| manufacturer-listed rated | 12 A | AK80-9 V3.0 product page; **motor-vs-driver/protocol scope unconfirmed** |
| manufacturer-listed peak | 28 A | same |

**None of these is a safety authorisation.** A software command ceiling is not a motor
operating limit. The script's limit was not increased, the script was not modified, and
nothing here was executed against hardware. Whether any current is safe to command
requires the lab supervisor and the manufacturer's documentation.

The metrics file reports `peak_applied_current_envelope` as a *label* — "within reported
rated", "above reported rated, within reported peak", or "ABOVE reported peak —
simulation only, not a permitted command".

---

## 8. Equations as implemented

**Controller** (`CurrentPID.update`), degree domain:

```
e      = ref_deg − meas_deg
I_raw  = Kp·e + Ki·∫e dt + Kd·de/dt          (derivative ON ERROR)
I_cmd  = s_mot · clip(I_raw, ±clip_A)        (sign applied AFTER the clip)
```

Five behaviours copied deliberately from the reported script:

1. **Derivative on error**, not measurement. With a step reference these differ by an
   impulse exactly where a step response is read. A measurement variant exists behind
   `--derivative-on measurement` and is not the default.
2. **First update suppresses the derivative** (`prev_error_deg` unset). This is the
   single most consequential detail in the file: unsuppressed, a 5° step at 1 kHz gives
   `0.03 × 5/0.001 = 150 A` — 2.5× the clip. It is the difference between saturating
   instantly and never saturating.
3. **Ki = 0**, so the law is PD. The integrator exists with clamping anti-windup so a
   future nonzero Ki is testable; at Ki = 0 the state stays exactly 0.0.
4. **Sign after saturation**, so a symmetric clip stays symmetric.
5. **Request and applied are reported in the same frame**, differing only by clipping.
   *(First version logged pre-sign against post-sign, so they differed on every step and
   saturation was invisible. Fixed; `raw_signed_A` now exists for exactly this.)*

**Scheduling.** 1 kHz controller, 0.5 ms physics ⇒ exactly 2 steps per update, command
held. The ratio is computed from the model's own timestep and the run **refuses to start**
if 1 kHz does not divide exactly, rather than rounding into a drifting schedule.

**Drivetrain.** Unchanged from `oslbench/drivetrain.py` / `drivetrain_sim.py`. Not
restated here, by design.

---

## 9. Validation results

The following are the latest Windows `.venv` verification results. The ankle experiment
used **REAL MuJoCo**, not `tests/stub_mujoco.py`; no physical hardware was used.

| Check | Result |
|---|---|
| `tests/test_ankle_current_step.py` | **34 passed, 0 failed** |
| Full suite `tests/run_tests.py` | **173 passed, 0 failed, 0 skipped** |
| Oracle `experiments/verify_against_oracle.py` | **PASS** — frozen trace and metrics reproduced |
| Real ankle experiment | **completed with REAL MuJoCo** |
| τ_j = n_t × τ_a over a full run | worst 2.5e-9 N·m |
| τ_m = I_applied × k_t_output over a full run | worst 6.1e-9 N·m |
| ankle servo force, all 8000 steps | **0 nonzero**, peak 0.0 N·m |
| `qfrc_applied` away from the ankle dof | 0.0 everywhere |
| authored ROM / forcerange after the run | unchanged |
| determinism (same run twice) | bit-identical |

**Protected baseline:** `oslbench/controller.py`, `models/osl_v2_bench.xml`,
`tests/oracle/`, `oslbench/drivetrain.py`, `oslbench/drivetrain_sim.py` — all untouched
by this work. Existing unrelated worktree changes are not part of this reconciliation.

The oracle was run successfully in the project virtual environment. It reproduced the
frozen baseline exactly within repository tolerances, including RMS error 4.361613°,
peak error 9.090662°, and peak torque 16.004121 N·m. The earlier statement that MuJoCo
was unavailable referred to the obsolete sandbox/stub verification and is superseded by
this run.

**Agreement with hardware: NOT ESTABLISHED.** No comparison has been made, because no
physical telemetry is available in this workspace.

---

## 10. Current real-MuJoCo ankle result

The latest run used `--kt script --belt cad`, retained the existing gains and current
clip, and used the geometry-derived encoder sign `−1`. It completed with REAL MuJoCo
using the compiled model identified in §0.

| Metric | Value |
|---|---|
| final angle | 4.94934° (target 5°) |
| steady-state error | 0.0506584° |
| rise time 10–90% | 0.105 s |
| overshoot | 0% |
| peak time | not applicable — no meaningful overshoot |
| settling (±2%) | 1.524 s |
| peak current request / applied | 3.75 A / 3.75 A (**no saturation**) |
| current envelope | within reported rated |
| max \|θ_s\| | 1.37061e-3 rad = 0.0249× the fitted edge |

The settling definition is the first sample after the last excursion outside the target
±2% band. For this 5° step the band is ±0.1° (4.9°–5.1°): the last out-of-band sample
is `t=1.5235 s`, `4.89999346°`, and the first in-band sample is `t=1.5240 s`,
`4.90000927°`. The trace never exceeds the target by more than the meaningful 2%
threshold, so peak time is not applicable rather than being labelled the final recording
timestamp.

### Historical stub wiring-check results

These older values came from `tests/stub_mujoco.py`, not from the compiled CAD model.
They are retained only for provenance and must not be compared with or quoted as the
current experiment result. The stub ankle had armature 0.0 versus 0.008, damping 0.05
versus 0.25, and frictionloss 0.05 versus 0.3.

| Metric | Historical stub value |
|---|---|
| final angle | 4.959° |
| steady-state error | 0.041° |
| rise time 10–90% | 0.099 s |
| overshoot | 0.027% |
| settling (±2%) | 0.160 s |
| peak current request / applied | 3.75 A / 3.75 A |
| max \|θ_s\| | 9.68e-4 rad |

---

## 11. Remaining unknowns

Ranked by how much they would change the answer:

1. **The torque constant and its shaft.** The official page now establishes 0.095 N·m/A
   and 9:1 for AK80-9 V3.0, but not the constant's shaft reference. The 0.5701 script
   value remains unresolved. **Also resolve whether the rig is AK80-9 or AK80-6** — the
   `0.5701 = 0.095 × 6` coincidence makes this a concrete, checkable question.
2. **Whether the physical ankle belt is the CAD's 50/11.** Confirmed in the CAD;
   unconfirmed on the actual rig.
3. **What the plotted torque in the lab graph actually was** — motor EM torque,
   actuator-output torque, or an internally computed estimate. Until this is answered the
   4.8–5 N·m observation cannot be placed in any frame.
4. **Safe current envelope.** Supervisor + documentation. Motor vs driver vs protocol
   range are three different limits.
5. **Encoder polarity premise.** The MuJoCo-to-encoder sign is now derived from the
   compiled foot geometry, but the statement that positive encoder angle means
   plantarflexion remains second-hand and therefore provisional-by-premise.
6. **Current-loop bandwidth.** The ideal-loop assumption is unquantified without it;
   the product-page current ratings do not establish driver response dynamics.
7. **Transmission efficiency.** Not modelled at all; the paper's friction terms are a
   different build's.
8. **Encoder resolution, latency, filtering.** None modelled.
9. **Whether the foot's CAD inertia matches the physical foot/load.**

---

## 12. Physical comparison protocol

To make a real comparison, the following telemetry is needed from the rig. Nothing below
has been received; **none of it should be fabricated**.

**Per sample, at the controller rate, with a common clock:**

| Channel | Units | Why |
|---|---|---|
| timestamp | s, t=0 at PID engagement | the simulation uses the same zero |
| reference angle | deg | confirms the commanded step |
| encoder angle | deg | the comparison variable |
| commanded current | A, pre-clip | separates controller from saturation |
| applied/measured current | A | **the one that would test the ideal-loop assumption** |
| logged torque | N·m + **which shaft** | resolves §5 |
| saturation flag | bool | |
| loop period | s, actual | tests the 1 kHz assumption |

**Metadata, once per run:** actuator part number and revision; driver firmware; the
manufacturer's torque constant *with its shaft*; pulley tooth counts as built; encoder
model, resolution, mounting and any filtering; the exact gains and limits in force.

**Comparison procedure:**

1. Align on PID engagement, not on power-up.
2. Compare *encoder* angle to *encoder* angle — never encoder against raw MuJoCo qpos
   without the sign convention.
3. Compare commanded current to commanded current. Compare measured current only against
   commanded, as a test of the ideal-loop assumption — the simulation has no measured
   current to offer.
4. Compare rise time, overshoot, settling and steady-state error using the *same*
   definitions (`calculate_metrics`, shared with the knee experiment).
5. Only after the shaft question is settled, compare torque — in one named frame.
6. Re-run the simulation across all three `--kt` choices and both `--belt` choices and
   report the **spread**, not the best-fitting one. Picking the parameterisation that
   matches best is curve-fitting to a single trace.

**A disagreement would not by itself invalidate the model.** With the torque constant
unresolved to 1.75× and the current loop assumed ideal, the honest first question is
which parameter is wrong — not whether the physics is.

---

## 13. Files

| File | Role |
|---|---|
| `oslbench/ankle_current_control.py` | **new** — encoder boundary, current PID, conversion audit, ankle wiring |
| `experiments/run_ankle_current_step.py` | **new** — the 5° step experiment |
| `tests/test_ankle_current_step.py` | **new** — 34 focused tests currently pass |
| `tests/run_tests.py` | **modified** — one line, collects the new suite |
| `docs/PHYSICAL_ANKLE_CONTROL_ALIGNMENT.md` | **new** — this document, reconciled with the latest verification |
| `build/ankle_current_step/` | **generated output** — latest real-MuJoCo CSV, PNG, and metrics |

The latest verification changed no gains, protected MJCF, original controller,
drivetrain implementation, tests, generated outputs, or oracle data. This
documentation reconciliation changed only this Markdown file. Nothing was committed.
