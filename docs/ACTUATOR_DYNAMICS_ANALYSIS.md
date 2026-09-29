# Actuator Dynamics — Best et al. (2025) mapped onto this repository

**Source paper.** T. Kevin Best, Gray C. Thomas, Senthur R. Ayyappan, Robert D. Gregg,
Elliott J. Rouse, *"A Compensated Open-Loop Impedance Controller Evaluated on the
Second-Generation Open-Source Leg Prosthesis,"* IEEE/ASME Transactions on Mechatronics,
vol. 30, no. 6, pp. 4732–4743, December 2025. DOI 10.1109/TMECH.2024.3508469.

**Status of this document.** Analysis only. **No code, model or data file was modified in
producing it.** Every number attributed to the paper below was read out of the PDF text
layer and the load-bearing ones were additionally verified against a rasterised page image.
Every number attributed to this repository was read out of the repository. Nothing has been
implemented and nothing from the paper has been reproduced.

---

## 1. Executive summary

The paper gives us three things we did not have: a measured parameter set for the exact
actuator our model describes, an explicit statement that the belt transmission is compliant
rather than a rigid ratio, and a validated account of what goes wrong when a controller
ignores that compliance. Mapped onto our bench model, the consequences are larger than the
armature pilot anticipated and they point in a different direction.

Three findings dominate.

**First, our three placeholder dynamic parameters are all too small, and by known factors.**
Referring the paper's measured actuator inertia through the belt ratio gives a joint-side
reflected inertia of about 0.209 kg·m², against the 0.010 kg·m² currently authored — a factor
of about 21. Joint-side viscous damping comes out near 1.29 N·m·s/rad against 0.30, a factor
of 4.3. Joint-side Coulomb friction at our operating torque comes out near 2.3 N·m against
0.40, a factor of 5.7, and most of that is a *current-dependent* term that MuJoCo's constant
`frictionloss` cannot express at all. These are arithmetic consequences of published measured
values, not new claims about hardware.

**Second, the "just set armature correctly" move is only valid in a limit the paper says does
not hold.** Lumping the actuator inertia into the joint requires the belt deflection to be
identically zero. The paper measured the belt's stiffness at 876 N·m/rad near zero load,
rising with deflection. With that spring between the reflected actuator inertia and the link
inertia, the drivetrain has a resonance in the 10–17 Hz band, while the closed-loop bandwidth
we currently run is 5.7–7.6 Hz. The separation is roughly a factor of two. A single lumped
inertia is not a safe approximation at that separation, so `armature` alone cannot represent
this drivetrain.

**Third, the belt stiffness is the same order as the stiffness our controller commands.** We
command Kp = 600 N·m/rad at the knee. A 876 N·m/rad series spring between the actuator and the
joint would render, at low load, roughly 356 N·m/rad — a 41 % shortfall. This is precisely the
failure mode the paper measured on its uncompensated baseline controller ("`C0` was unable to
effectively render higher stiffness values, likely because `C0` ignores the compliance of the
belt transmission"). Our simulation cannot exhibit it, because our position servo acts directly
on the joint. That is the single most consequential idealisation in the current model, and it
is not the one we had been probing.

A fourth point resolves the number the brief asked about. The paper's drivetrain is
9:1 planetary × 4.61:1 belt = **41.49** total. The **49.4** used in this repository is a MuJoCo
actuator *gain* copied from MyoAssist's OSL model, paired with a 58.4 for the ankle. The OSL v2
knee and ankle drivetrains are identical — the paper says so, and our own CAD export confirms
it by using the same 11-tooth input and 50-tooth output pulley parts at both joints — so a
number that differs between knee and ankle cannot be an OSL v2 drivetrain ratio. Details and
the full reconciliation are in §7. **The two numbers have not been merged and neither has been
overwritten.**

The recommended architecture (§9) is a separate actuator-dynamics layer that keeps the
CAD-derived multibody model untouched (Option C), reached in stages, with a read-only
rigid-equivalent experiment as the first runnable step (§10).

---

## 2. The OSL v2 actuator model (paper §III-A1)

### 2.1 What the paper models

The actuator is a Dephy ActPack 4.1, built on a T-Motor AK80-9: an exterior-rotor brushless
motor with an **integrated 9:1 planetary gear reduction**. The paper models it as a rigid
rotational inertia driven by a current-proportional torque source, opposed by viscous and
friction losses. It does not model electrical dynamics, motor thermal behaviour, torque
ripple, or the current controller itself — the current controller runs at 10 kHz on the
actuator, which is far above the mechanical bandwidth of interest, so `I_q` is treated as a
commanded input that is achieved.

### 2.2 The equations, as printed

Equation (1), the actuator output torque:

```
tau_a = tau_m - tau_f(thetadot_a, I_q) - B_a * thetadot_a - J_a * thetaddot_a
```

with the motor torque

```
tau_m = I_q * k_t * n_a
```

and equation (2), the friction:

```
tau_f(thetadot_a, I_q) = sgn(thetadot_a) * (f_c + f_g * |I_q|)
```

The paper's words for each term: `B_a` "represents viscous losses due to the actuator velocity",
`J_a` "captures the combined effects of **rotor and gearbox** inertial torques due to the
actuator's acceleration", and `f_c` and `f_g` "parameterize coulomb and gear friction,
respectively". The model is credited to Nesler et al. (refs [27], [44]).

### 2.3 State variables

| symbol | meaning | where it lives |
|---|---|---|
| `I_q` | q-axis motor current, the **input** to the whole drivetrain | rotor / motor windings |
| `theta_a`, `thetadot_a`, `thetaddot_a` | actuator **output** angle, velocity, acceleration | the post-planetary shaft, i.e. the belt **input pulley** |
| `tau_m` | motor torque **already referred to the actuator output** (it carries the factor `n_a`) | actuator output shaft |
| `tau_a` | net actuator output torque | actuator output shaft |
| `tau_f` | friction torque opposing `tau_m` | actuator output shaft |

### 2.4 Identified parameters — verified from the paper

All five read from p. 4735, confirmed against a rasterised image of that page.

| parameter | value | unit | what it is | reference frame |
|---|---|---|---|---|
| `k_t` | 110.8 × 10⁻³ | N·m/A | motor torque constant | **rotor** (it is multiplied by `n_a` to reach the output) |
| `n_a` | 9 | — | the ActPack's integrated planetary reduction | rotor → actuator output |
| `J_a` | 9.83 × 10⁻³ | kg·m² | rotor **and gearbox** inertia combined | **actuator output shaft** |
| `B_a` | 6.06 × 10⁻² | N·m·s/rad | viscous loss | **actuator output shaft** |
| `f_c` | 17.1 × 10⁻² | N·m | Coulomb friction, constant part | **actuator output shaft** |
| `f_g` | 82.1 × 10⁻³ | N·m/A | gear friction, proportional to `|I_q|` | N·m at the **actuator output**, per amp at the **rotor** |

### 2.5 Answering the brief's four questions about frames

**Is `J_a` rotor inertia or rotor + gearbox?** Rotor **and** gearbox — the paper states this
explicitly ("the combined effects of rotor and gearbox inertial torques").

**At what shaft is `J_a` expressed?** At the **actuator output**, after the 9:1. Three
independent reasons. It multiplies `thetaddot_a`, and `theta_a` is the actuator output angle
(equation (3) relates `theta_a` to the joint through the belt ratio *alone*, and the Fig. 4
caption says "the actuator angle `theta_a` is reduced through the transmission ratio of the
belt-drive `n_t`"). The identification measured "the torque at the actuator output". And the
magnitude is consistent: `J_a / n_a² = 9.83e-3 / 81 = 1.21 × 10⁻⁴ kg·m²`, which is the right
order for the rotor of an 80 mm outrunner. **`J_a` already contains the 9:1 reflection.** This
is the single most important frame fact in the paper for our purposes, and the one most easily
got wrong.

**At what shaft is `B_a` expressed?** Same — actuator output, since it multiplies `thetadot_a`.

**At what shaft is friction expressed?** Same — `tau_f` is subtracted from `tau_m` in (1), and
`tau_m` is an actuator-output quantity. Note `f_g` is dimensionally mixed: it converts rotor
current into actuator-output torque.

**What exactly is `n_a`?** The 9:1 planetary reduction integrated into the ActPack 4.1 /
AK80-9. It is *not* the belt, and it is *not* a total drivetrain ratio.

### 2.6 How the parameters were identified

Two benchtop experiments, with two actuators opposing one another through a contactless torque
sensor (FUTEK TRS605). First, steady-state pairs of voltage and current commanded for five
seconds each, on a grid `v ∈ [−40, 40] V`, `i ∈ [−17.5, 17.5] A`, measuring actuator-output
torque, current and position. Second, a non-steady-state test for inertia: one actuator ran
sinusoidal velocity profiles while the other commanded zero current. The model was then
regressed on the combined data.

### 2.7 Reported quality

Variance accounted for **99.7 %**; RMS residual **3.0 % of peak torque**; RMS output torque
residual "under 0.65 N·m". Dropping everything except the current-proportional term raises the
normalised RMS residual to **6.5 %** — the paper's justification that the inertia, viscous and
friction terms are worth carrying. All verified from the paper text.

---

## 3. The transmission model (paper §III-A2)

### 3.1 The belt is not a rigid ratio — this is the paper's central modelling claim

"Initial testing showed that the belt is slightly elastic and it thus introduces a passive
compliance to the drivetrain dynamics that is **similar to an SEA**." The transmission is a
single-stage Gates PowerGrip GT3 belt drive, 45 mm wide, with a nominal **4.61:1** reduction.
The paper notes the design recommends **3 mm pitch** belts and offers a **5 mm pitch** option
which is "stiffer but ha[s] greater output impedance". (This matters for us — see §7.4.)

### 3.2 The kinematic decomposition

Equation (3):

```
theta_j = theta_a / n_t + theta_s
```

`theta_s` is "the transmission's effective angular deflection" and `n_t` is "the transmission's
gear ratio". Because `theta_s` adds directly to a joint-side angle, **`theta_s` is referred to
the joint side**. Rearranged, `theta_a = n_t (theta_j − theta_s)`, which the paper uses in its
equation (11).

### 3.3 The torque–deflection law

Measured on a rotary dynamometer (Baldor BSM90N3150AF motor, JR3 45E15A4 load cell) with the
shank rigidly mounted and **the actuator position held at zero displacement**, while the
dynamometer drove the joint slowly from 0.0 to 0.167 rad. Ten repeats. Inertial contributions
of the mounting hardware were characterised and subtracted.

```
rho(theta_s) = p2 * theta_s^2 + p1 * theta_s      for theta_s in R+
p2 = 14913 N.m/rad^2     p1 = 876 N.m/rad         R^2 = 0.997
```

Equation (4) extends it to both directions: `tau_j = −tau_s = −sgn(theta_s)·rho(|theta_s|)`.
Equation (5) gives the local stiffness:

```
K_s(theta_s) = d rho / d theta_s = 2*p2*|theta_s| + p1
```

A useful identity that follows (and that we will want in code): inverting (4) and substituting
gives `K_s = sqrt(p1² + 4·p2·|tau_j|)`, i.e. the local stiffness can be obtained from torque
directly without solving for deflection first.

The paper quantifies what a constant-stiffness simplification costs: "ignoring the belt's
stiffening behavior can result in steady-state torque and impedance errors of up to **20 %**
during large torque conditions." It also notes "a minor backlash behavior around zero
deflection, but choose[s] to neglect it for model simplicity", and returns to that in its
limitations section.

### 3.4 Torque flow, end to end

```
I_q  --[ k_t * n_a ]-->  tau_m
tau_m - tau_f - B_a*thetadot_a - J_a*thetaddot_a  =  tau_a        (actuator output shaft)
tau_a --[ belt, ratio n_t ]-->  tau_j = n_t * tau_a               (paper eq. 7)
                        equivalently, through the spring:
theta_s = theta_j - theta_a/n_t   ->   tau_j = -sgn(theta_s) * rho(|theta_s|)
tau_j acts on the foot/link inertia J_f, together with any ground reaction torque tau_grf
```

The two expressions for `tau_j` are the same statement seen from either side of the spring:
`tau_j = n_t·tau_a` is the quasi-static force balance, and `rho(theta_s)` is what actually sets
the transmitted torque dynamically.

### 3.5 What is nonlinear, and what is not

Nonlinear: the `sgn(thetadot_a)` in the friction (a discontinuity at velocity reversal); the
`f_g|I_q|` term, which makes the friction depend on the *input*, not only on the state; the
quadratic `rho(theta_s)`, i.e. a deflection-dependent stiffness; and the `sgn(theta_s)`
odd-symmetry in (4). Also nonlinear and deliberately omitted: backlash.

Linear: `tau_m = I_q k_t n_a`; the viscous term `B_a·thetadot_a`; the inertial term
`J_a·thetaddot_a`; and the kinematic relation (3).

---

## 4. Experimental identification — methodology summary

| what | rig | excitation | measured | result |
|---|---|---|---|---|
| actuator parameters `k_t, J_a, B_a, f_c, f_g` | two opposed actuators through a FUTEK TRS605 contactless torque sensor | steady-state `(v, i)` grid, 5 s each, `v ∈ [−40,40] V`, `i ∈ [−17.5,17.5] A`; plus sinusoidal velocity profiles against a zero-current actuator | actuator-output torque, current, position | VAF **99.7 %**, RMS residual **3.0 %** of peak, < 0.65 N·m |
| belt parameters `p1, p2` | rotary dynamometer (Baldor BSM90N3150AF + JR3 45E15A4), shank rigidly mounted, **ankle** joint | slow constant-rate ramp 0.0 → 0.167 rad with actuator held at zero displacement, ×10 | actuator angle, joint angle, joint torque | quadratic fit, **R² = 0.997** |

Two methodological points that carry over to any attempt of ours. The actuator identification
needs a *second* actuator as a load and a reaction-torque transducer; there is no way around
some form of external torque measurement. The belt identification is much cheaper: with the
motor held, `theta_s` follows directly from the difference between the joint encoder and the
motor encoder, so only the torque measurement is external.

The paper characterised the **ankle** and explicitly did not repeat it for the knee: "We did
not individually test the knee joint on the dynamometer, as its identical mechanical
construction to the ankle joint would make knee-specific experiments redundant." Our bench is a
**knee** bench, so every parameter we take from this paper is being transferred across joints —
on the paper's own authority, but transferred nonetheless. That must be stated whenever the
numbers are quoted.

---

## 5. Experimental validation (paper §IV) — figures verified

Three experiment sets, all on the ankle, comparing the proposed controller `C1` against an
uncompensated baseline `C0`.

**Constant impedance.** Grid of `Kd ∈ {100,200,300,400,500}` N·m/rad × `Bd ∈ {1,3,5,7,9}`
N·m·s/rad, `theta_eq = 0`. Dynamometer drove `theta_j = a·sin(2πft)` at `f ∈ {1,2,3,4,5}` Hz,
20 cycles per frequency, at three amplitudes `a ∈ {0.035, 0.070, 0.105}` rad. Data filtered with
a 4th-order Butterworth lowpass at 15 Hz. Each trial regressed to
`tau_j = −K̂·theta_j − B̂·thetadot_j − Î·thetaddot_j` by least squares. Frequency response
obtained by fitting sinusoids and taking gain and phase.

Results: mean VAF of the second-order fit 89.1 % (`C0`) and 88.6 % (`C1`); frequency-response
error 55.7 ± 28.3 % (`C0`) versus 33.1 ± 40.2 % (`C1`), a significant 22.6 % reduction
(p = 0.026); RMS torque error lower for `C1` at every amplitude (p < 0.05), by 1.9 N·m on
average.

**Drivetrain-model accuracy on that set — the numbers the brief asked to verify:**
VAF **96.3 ± 2.7 %**; RMS model error **2.1 N·m**, which is **4.2 %** of the **49.5 N·m** peak
torque measured in the experiment. All three verified verbatim.

**Variable impedance.** Dynamometer replayed normative walking kinematics at 80 steps/min, 30
cycles, half with a 0.035 rad, 200 ms perturbation at 11 % or 33 % of the gait cycle;
impedance estimated by regressing a second-order model to the deviation trajectories over the
first 150 ms. Peak non-perturbed torque **98.4 ± 0.8 N·m**, ROM 0.417 ± 0.002 rad. Drivetrain
model VAF **97.7 ± 0.2 %**, RMS model error **6.0 ± 0.2 N·m** = 6.1 % of peak.

**Treadmill walking.** One participant with above-knee amputation, 1 m/s, phase-based walking
controller. Mean absolute torque error against the desired impedance law: **16.2 N·m** for `C0`,
**9.6 N·m** for `C1`.

**Reconciliation of "97 %" and "96.3 ± 2.7 %".** The abstract's "97 % mean explained variance
across a diverse array of experiments" is the aggregate across the three experiment sets; the
96.3 ± 2.7 % figure is specifically the constant-impedance set, and 97.7 ± 0.2 % is the
variable-impedance set. The discussion states the general form: "The model's RMS error was
6.1 % of the peak torque or lower in both experiments, with VAF values of 96 % or better."
**There is no discrepancy** — they are different scopes of the same evaluation. The 99.7 %
figure is separate again: it is the *actuator-only* identification of §III-A1, not the full
drivetrain.

**One notational observation, offered cautiously.** The baseline controller, equation (21), is
printed as `I_q = [Kd(theta_eq − theta_j) − Bd·thetadot_j] / [k_t (n_a n_t)²]`. As printed, with
joint-side angles in the numerator, that carries an apparent extra factor of `n_a·n_t`
relative to `I_q = tau_j / (k_t n_a n_t)`. Both text renderings of the PDF agree on the
squared form, so it is either a typesetting issue or the angles are intended as motor-side. It
does not affect anything we take from the paper — `C0` is the baseline being argued against,
not the model — and it is flagged here only so nobody transcribes it later without noticing.

### 5.1 What we would need to reproduce this

To reproduce the **actuator identification**: a second ActPack to act as an opposing load, a
contactless reaction-torque sensor of the TRS605 class, current-controlled drive with `I_q`
telemetry, and the ability to command steady-state voltage/current pairs. We have none of this.
Not reproducible.

To reproduce the **belt characterisation** (the higher-value target, and much cheaper): hold
the motor at zero displacement, drive the joint slowly through its range, and log motor
encoder, joint encoder and joint torque. `theta_s` is then `theta_j − theta_a/n_t` from the two
encoders alone. The OSL v2's standard sensor suite already includes motor encoders, joint
encoders and a 6-axis midshank load cell, so the only genuinely missing piece is a trusted
torque reference — either the load cell, if it can be calibrated for this, or an external
dynamometer. This is the one hardware measurement worth asking for.

To reproduce the **validation**: a dynamometer capable of position-controlling the joint
through prescribed sinusoids and gait kinematics while measuring reaction torque. Out of reach
for now.

**We have reproduced none of this.** Nothing in this document is a measurement.

---

## 6. Paper model versus our model — parameter and effect audit

Read out of the repository on 2026-09-26: `models/osl_v2_bench.xml` (357 lines),
`tools/build_mjcf.py`, `oslbench/model.py`, `oslbench/controller.py`, `oslbench/simulation.py`,
`docs/MYOASSIST_FINDINGS.md`, `README.md`, and the CAD mesh inventory in
`osl_v2_0_assembly/meshes/`.

What the bench XML actually authors at the knee:

```xml
<option timestep="0.0005" integrator="implicitfast" gravity="0 0 -9.81"/>
<joint name="knee" type="hinge" axis="0 -1 0" range="-0.0872664625997 2.09439510239"
       damping="0.3" armature="0.01" frictionloss="0.4"/>
<position name="knee_pos" joint="knee" kp="60.0"
          ctrlrange="-0.0872664625997 2.09439510239" forcerange="-142.2 142.2"/>
```

with `Kp = 600`, `Kd = 17.253` written into the *compiled* model at run time by
`PDController.write_to_model`, and sensors `knee_q`, `knee_qd`, `knee_tau` (an `actuatorfrc`).

### 6.1 The table

| effect in the paper's model | what our model has today | status |
|---|---|---|
| motor torque constant `k_t = 0.1108 N·m/A` | nothing. No current appears anywhere in the model or code. The only trace is the reconstructed `0.096 N·m/A × 30 A` behind `forcerange` | **MISSING** |
| actuator gear reduction `n_a = 9` | nothing explicit. Absorbed into the `forcerange` arithmetic via a different ratio | **MISSING** |
| belt reduction `n_t = 4.61` | nothing in the model. Present in CAD as an 11-tooth input and 50-tooth output pulley (= 4.5455) and in a comment in `build_mjcf.py:144` ("the 50/11 belt ratio"), used nowhere | **MISSING** (as a model quantity) |
| actuator inertia `J_a = 9.83e-3 kg·m²` at the actuator shaft | no actuator shaft exists. `nq = nv = 2` | **MISSING** |
| joint-side reflected inertia `n_t²·J_a ≈ 0.209 kg·m²` | `armature = 0.010`, documented as a placeholder | **PLACEHOLDER**, low by ≈21× |
| actuator viscous damping `B_a = 6.06e-2` → joint side `n_t²·B_a ≈ 1.29` | `damping = 0.30`, documented as a placeholder | **PLACEHOLDER**, low by ≈4.3× |
| Coulomb friction `f_c = 0.171` → joint side `n_t·f_c ≈ 0.79 N·m` | `frictionloss = 0.40`, documented as a placeholder | **PLACEHOLDER**, low by ≈2× against the constant part alone |
| current-dependent friction `f_g·|I_q|`, ≈1.5 N·m joint-side at our peak torque | nothing. MuJoCo `frictionloss` is a constant and cannot depend on the command | **MISSING**, and **not representable** in the current formulation |
| belt compliance `rho(theta_s)`, `K_s = 876…2600 N·m/rad` | nothing. The actuator applies torque directly to the joint | **MISSING** — the largest structural gap |
| backlash | nothing (the paper also neglects it) | **MISSING — not needed yet**, matching the paper |
| motor current dynamics / electrical time constant | nothing | **NOT NEEDED FOR CURRENT MODEL** — the paper omits it too (10 kHz current loop) |
| torque–speed envelope | nothing; a flat `forcerange` box | **MISSING**, and the paper does not model it either. Our peak demand is ≈11 % of authority, so it is not currently binding |
| torque limit | `forcerange ±142.2 N·m` knee, `±168.2` ankle | **APPROXIMATED** — arithmetic on `49.4 × 2.88`; see §7. Paper's design figure is ~160 N·m for 10 s, identical at both joints |
| controller rate | `dt = 0.5 ms` (2 kHz), control law evaluated every step | **APPROXIMATED**. Hardware: 300 Hz outer logic, 1 kHz impedance/position, 10 kHz current. Our loop is faster than any real loop, so no delay or quantisation is represented |
| actuator-side position `theta_a` | does not exist | **MISSING** |
| joint-side position `theta_j` | `knee_q` sensor, exact | **PRESENT** |
| transmission deflection `theta_s` | does not exist | **MISSING** |
| joint torque `tau_j` | `knee_tau` (`actuatorfrc`) | **PRESENT**, but it is the commanded servo force, not a torque arrived at through a drivetrain |
| link inertia `J_f` and gravity | CAD-derived, `I_body = 0.251998 kg·m²`, `mgd = 8.8529 N·m` | **PRESENT** and well-sourced |

### 6.2 One thing that is present but easy to over-credit

`armature`, `damping` and `frictionloss` all exist as attributes, and all three map onto named
effects in the paper's model. That is a coincidence of vocabulary, not evidence of
representation. `build_mjcf.py:279-282` says so plainly: "`damping`, `armature`,
`frictionloss` and `kp` do NOT [have sources] — they are plausible placeholders". The presence
of an attribute called `armature` does not mean reflected actuator inertia is modelled; it
means there is a slot where it could go.

### 6.3 A CAD-versus-paper discrepancy our own comments got backwards

`tools/build_mjcf.py:247-248` explains the ankle's larger `forcerange` by saying "the ankle
module carries the higher belt reduction because push-off is the more demanding task." The
paper states the opposite — the knee and ankle drivetrains are **identical** — and our own CAD
export agrees with the paper: `p_b0001_InputPulley_5mm_11teeth.stl`,
`p_b0012_OutputPulley_5mm_50teeth.stl` and `GT3_5mm_325mm_belt.stl` each appear exactly twice
in the URDF, once per joint. The asymmetric `142.2 / 168.2` pair therefore does not come from
an asymmetric belt. This comment should be corrected; the `forcerange` values themselves are a
separate decision (§7.5).

---

## 7. The 49.4 versus 9 × 4.61 discrepancy

### 7.1 The four numbers, and what each one is

| number | where it appears | what it physically is | verification status |
|---|---|---|---|
| **9** | paper §II | planetary reduction inside the Dephy ActPack 4.1 / T-Motor AK80-9 | **VERIFIED** in the paper |
| **4.61** | paper §II | single-stage Gates GT3 belt reduction, OSL v2 | **VERIFIED** in the paper |
| **41.49** | implied, `= 9 × 4.61` | total rotor → joint ratio for the OSL v2 as described in the paper | derived from two verified numbers |
| **50/11 = 4.5455** | our CAD export: `p_b0001_InputPulley_5mm_11teeth`, `p_b0012_OutputPulley_5mm_50teeth`, both present at both joints | the belt reduction of the **specific build our Onshape export describes** | **VERIFIED** from mesh names and URDF link counts |
| **40.909** | implied, `= 9 × 50/11` | total ratio implied by our CAD plus the paper's planetary | derived |
| **49.4** (knee) / **58.4** (ankle) | `gainprm` of the `motor` actuators in MyoAssist's `OpenSourceLeg_KA_L1` MJCF, per `docs/MYOASSIST_FINDINGS.md:110-111` | a **MuJoCo actuator gain** mapping a ±2.88 control signal to joint newton-metres | **VERIFIED as an MJCF gain**. **NOT verified as an OSL v2 drivetrain ratio** |
| **2.88** | the same MJCF `ctrlrange` | reconstructed here as `0.096 N·m/A × 30 A`, i.e. a motor-shaft torque at an assumed current limit | provenance is a deleted git comment; **weak** |
| **142.2 / 168.2 N·m** | our `forcerange` | `49.4 × 2.88` and `58.4 × 2.88` | arithmetic, **VERIFIED** as arithmetic |

### 7.2 Are 49.4 and 41.49 the same physical quantity?

Dimensionally, both are drivetrain ratios, so the question is fair. Three pieces of evidence
say the 49.4/58.4 pair does not describe the OSL v2 drivetrain.

They differ between knee and ankle. The OSL v2's knee and ankle drivetrains are identical —
the paper asserts it in three separate places, including as the reason it skipped
knee-specific dynamometer testing — and our CAD export independently confirms it by using the
same pulley and belt parts at both joints. A number that is 49.4 at the knee and 58.4 at the
ankle cannot be an OSL v2 ratio.

Neither divides by the planetary into anything recognisable: `49.4/9 = 5.49` and
`58.4/9 = 6.49`, against a belt ratio that is 4.61 in the paper and 4.5455 in our CAD.

The paired torque constant does not match either: `0.096` against the paper's measured
`0.1108 N·m/A`.

**Most likely explanation, explicitly labelled a hypothesis:** 49.4 and 58.4 look like
**OSL v1** transmission ratios, a generation in which the knee and ankle transmissions did
differ. This cannot be checked offline — no network is available in this environment — so it
stays a hypothesis. What is *not* a hypothesis is that these numbers are inconsistent with the
OSL v2 drivetrain as the paper describes it and as our own CAD export is built.

### 7.3 What has and has not been changed

**Nothing.** `forcerange` still reads ±142.2 / ±168.2, and the armature pilot's documented
ladder still records `N ≈ 49.4` as its stated assumption. One number has not been silently
swapped for the other. §10 proposes how to record the decision.

### 7.4 A second discrepancy the CAD exposes: belt pitch

The paper says the OSL v2 "design recommends 3 mm pitch belts, but includes an option to be
assembled with 5 mm pitch belts; these belts are stiffer but have greater output impedance."
Our CAD export is unambiguously the **5 mm** variant: 5 mm pulleys at both ends, and a
`GT3_5mm_325mm_belt`. The paper does not state which pitch was fitted for the belt
characterisation.

Two consequences. The ratio difference is explained naturally — 4.61 is very close to a 3 mm
pairing such as 60/13 = 4.615, while 5 mm pitch with the same centre distance forces different
tooth counts, giving our 50/11 = 4.5455. And **if the paper characterised the 3 mm belt, then
`p1 = 876 N·m/rad` is a lower bound for the hardware our CAD describes**, because the paper
tells us the 5 mm belt is stiffer. Using `p1 = 876` for our model is therefore conservative in
the direction of *more* compliance than our build would have. Both statements should travel
with the parameters.

### 7.5 Consequences that follow immediately

The armature pilot's ladder was built as `N² × J_rotor` with `N = 49.4`. Two structural
problems, now visible. `N` should have been the **belt ratio alone**, because `J_a` already
contains the 9:1 reflection — reflecting through 41.49 (or 49.4) and a rotor-side inertia is
only equivalent if the rotor inertia used is `J_a/n_a²`, which was not how the ladder was
built. And the rotor inertia was guessed, where the paper measures the quantity directly.
Using 49.4 instead of 41.49 over-reflects by `(49.4/41.49)² = 1.42`.

The `forcerange` values inherit the same provenance problem. This does not make them wrong as
*numbers* — the paper's own design figure is ~160 N·m peak for 10 s, so ±142.2 is of the right
order — but it does mean the existing required wording remains mandatory: *the model
represents the available joint torque authority derived from the motor/gear relationship, but
idealizes the torque production.* They are not experimentally verified hardware values, and
changing them would invalidate the frozen oracle, so this pass proposes no change to them.

---

## 8. Reinterpreting the armature pilot

This is the part of the analysis the brief flagged as most important, so the reasoning is set
out step by step and the conclusion is a refusal as much as a number.

### 8.1 The coordinate transformation, derived explicitly

Assume, **for this step only**, a rigid belt: `theta_s ≡ 0`, hence `theta_a = n_t·theta_j`.
Substituting into (1) and using `tau_j = n_t·tau_a`:

```
tau_j = n_t*tau_m  -  n_t*tau_f  -  n_t^2*B_a*thetadot_j  -  n_t^2*J_a*thetaddot_j
```

So under the rigid assumption the joint-side equivalents are:

| quantity | expression | with `n_t = 4.61` (paper) | with `n_t = 50/11` (our CAD) |
|---|---|---|---|
| reflected inertia | `n_t² · J_a` | **0.2089 kg·m²** | 0.2031 kg·m² |
| viscous damping | `n_t² · B_a` | **1.288 N·m·s/rad** | 1.252 N·m·s/rad |
| Coulomb friction, constant part | `n_t · f_c` | **0.788 N·m** | 0.777 N·m |
| current-dependent friction | `n_t · f_g · |I_q|` | command-dependent | command-dependent |
| torque constant at the joint | `k_t · n_a · n_t` | 4.597 N·m/A | 4.533 N·m/A |

Against the current model: `armature` 0.010 → **≈21× low**; `damping` 0.30 → **≈4.3× low**;
`frictionloss` 0.40 → **≈2× low** against the constant part alone.

The current-dependent part is not a footnote. To hold 16 N·m at the joint while moving —
our measured AB19 peak — the actuator must produce `16/4.61 = 3.47 N·m`, which by the paper's
own inverse relation needs `I_q = (3.47 + f_c)/(k_t·n_a − f_g) = 3.98 A`, giving a friction
torque of `f_c + f_g·|I_q| = 0.498 N·m` at the actuator, i.e. **2.29 N·m at the joint** — of
which only 0.79 N·m is constant. So at our own operating point the *variable* part of the
friction is roughly twice the *constant* part, and the total is **≈5.7× our placeholder**.
MuJoCo's `frictionloss` cannot express this, because it does not know the command.

### 8.2 Does the paper's model permit reducing all of this to a single `armature`?

**No — not at the bandwidth we operate.** The reduction in §8.1 required `theta_s ≡ 0`, and the
paper's principal finding is that it is not. The question is whether the belt is *stiff enough*
that treating it as rigid is harmless, and that is a quantitative question with a checkable
answer.

Put the reflected actuator inertia (0.209 kg·m²) and the link inertia (0.252 kg·m², CAD) on
either side of the belt spring `K_s`:

| boundary condition | expression | at `K_s = p1 = 876` (unloaded) | at `K_s = 1312` (our 16 N·m peak) |
|---|---|---|---|
| free–free two-mass mode | `sqrt(K_s·(1/J_1 + 1/J_2))` | 87.6 rad/s = **13.9 Hz** | 107 rad/s = **17.1 Hz** |
| actuator against a held joint | `sqrt(K_s/J_1)` | 64.8 rad/s = **10.3 Hz** | 79.2 rad/s = **12.6 Hz** |

Our closed-loop natural frequency is 47.85 rad/s = **7.62 Hz** as currently modelled, and
36.1 rad/s = **5.74 Hz** once the corrected inertia is used. **The belt mode sits only about
1.8× to 3× above the closed loop.** Lumping two inertias into one is defensible when that
separation is an order of magnitude; at a factor of two it is not. `armature` alone therefore
cannot represent this drivetrain, and setting it to 0.209 should be understood as a deliberate
approximation with a stated validity limit, not as "adding the actuator dynamics".

(`K_s` values above follow from `K_s = sqrt(p1² + 4·p2·|tau_j|)`: 876 N·m/rad at zero load,
1312 at 16 N·m, 1874 at 46 N·m, 2576 at the paper's 98.4 N·m walking peak. The deflections are
small — 0.84°, 1.92°, 3.27° respectively — which is exactly why belt compliance is easy to
overlook and, at these stiffnesses, wrong to overlook.)

### 8.3 The static consequence, which is larger than the dynamic one

A series spring of `K_s` between a commanded joint stiffness `Kp` and the joint renders, at
steady state, `Kp·K_s/(Kp + K_s)`. At our `Kp = 600` and the unloaded `K_s = 876`, that is
**356 N·m/rad — a 41 % shortfall.** The paper's footnote 1 endorses exactly this reading
("solving the harmonic sum of a series spring with the local spring constant"), and its
discussion reports the measured version: `C0` "was unable to effectively render higher
stiffness values, likely because `C0` ignores the compliance of the belt transmission; an
additional series compliance always reduces the effective stiffness."

There is a related structural point. Our bench controller feeds back `knee_q` and `knee_qd`,
i.e. **joint-side** states. The paper deliberately does not do this: "we choose to use only the
actuator's states for feedback control, as noncollocated feedback (i.e., using the joint states
in (8) directly) poses stability risks in a series-elastic system." Our control law is the
non-collocated configuration the paper avoids — invisible in our model, because our model has
no series elasticity for it to be non-collocated across.

### 8.4 What this does to the pilot's conclusions

The pilot's ladder spanned `armature` 0.010 → 0.488. The physically grounded rigid-equivalent
value, 0.209, **sits inside that ladder**, between the third and fourth rungs. So the pilot's
upper rungs turn out to have been probing the physically relevant region, and its baseline rung
— the value actually authored in the model — is the one that is far off. Read off the pilot's
measured table, moving from 0.010 to the 0.244 rung changed peak AB19 torque from 16.00 to
30.54 N·m and step overshoot from 5.0 % to 15.7 %. That is the scale of what correcting this
parameter alone would do.

Three things the pilot's conclusions survive unchanged: RMS tracking error remains a weak
indicator (the closed-form error law contains no inertia term); peak torque and transient
overshoot remain the informative observables; and none of it says anything about hardware.

Two things that must now be restated. The ladder's stated assumption — "N ≈ 49.4, `J_rotor`
between 3e-5 and 2e-4" — is superseded as a *construction*, though the resulting span happens
to bracket the right answer. And the pilot's framing, that armature is "the biggest modelling
error", is no longer supportable: on the evidence here the **missing belt compliance is
larger**, because it changes the *structure* of the plant rather than one coefficient, and
because it alone would cost 41 % of the rendered stiffness.

---

## 9. Proposed actuator-dynamics architecture

### 9.1 The options

**Option A — equivalent joint-side inertia, damping and friction.** Change three numbers:
`armature ≈ 0.209`, `damping ≈ 1.288`, `frictionloss ≈ 0.788`.
*Physical fidelity:* correct in the rigid-belt limit; no compliance, no current-dependent
friction. *Mathematical correctness:* exact only for `theta_s ≡ 0`, which §8.2 shows does not
hold at our bandwidth. *CAD compatibility:* perfect. *Future controller work:* unaffected —
the interface does not change. *Timestep:* no requirement. *Validation against bench data:* none
possible; the paper never reports a rigid-equivalent model. *Complexity:* trivial.
*Debuggability:* nothing new becomes visible.

**Option B — explicit actuator DOF and compliant belt inside MuJoCo.** Add a hinge for the
actuator output shaft, couple it to the joint through the belt ratio, and put the nonlinear
spring between them.
*Fidelity:* highest — `theta_a`, `theta_j`, `theta_s` become real states that MuJoCo's
integrator handles implicitly, including in the mass matrix. *Mathematical correctness:*
best. *But:* MuJoCo's joint `stiffness` is linear, so `rho(theta_s) = p2·theta_s² + p1·theta_s`
cannot be authored in the XML; it requires a per-step callback (`mjcb_passive` or
`qfrc_applied`), which moves part of the model out of the XML and breaks the repo's rule that
the XML *is* the model. *CAD compatibility:* requires inserting a body that has no CAD
counterpart. *Cost:* `nq` goes 2 → 3, which breaks the nine preconditions in
`oslbench/model.py`, the oracle, and every metrics path. *Timestep:* not a blocker —
`K_s ≈ 2600` against 0.209 kg·m² gives ≈112 rad/s, comfortable at 0.5 ms.

**Option C — a separate actuator-dynamics layer, CAD multibody intact.** Keep the MJCF as a
pure CAD multibody driven by a `motor` (torque) actuator. Implement equations (1)–(5) in a new
`oslbench/` module that integrates `theta_a` at the sim timestep, computes
`theta_s = theta_j − theta_a/n_t` from the MuJoCo joint state, and applies
`tau_j = −sgn(theta_s)·rho(|theta_s|)` to the joint.
*Fidelity:* the same equations as B. *Mathematical correctness:* the actuator inertia is
outside MuJoCo's mass matrix, so the coupling is explicit rather than implicit — a
co-simulation. Stability is governed by `dt < 2/omega ≈ 18 ms`; at 0.5 ms there is a 36×
margin, so this is safe here, and it would need re-checking only if `K_s` or `dt` changed
substantially. *CAD compatibility:* perfect — the MJCF stays exactly what it claims to be, and
`armature` can hold the true bearing inertia rather than a lumped fiction. *Future controller
work:* the best fit of the four. It creates precisely the interface the paper uses — current
or desired actuator torque in, joint torque out — which is what any later OSL controller work
needs, including the postponed PD-tracker and phase-driven reference work. *Validation:* we
could implement both `C0` and `C1` and check the qualitative result the paper measured.
*Complexity:* moderate and **additive** — a new module plus an optional hook, with the existing
position-servo path untouched and still the default. *Debuggability:* best of the four —
`theta_a`, `theta_s`, `tau_s`, `I_q` are explicit arrays that `oslbench/logging.py` can already
carry.

**Option D — MuJoCo `general` actuator with a user activation dynamic.** Rejected on
inspection: `dyntype="user"` provides first-order activation dynamics, which cannot express a
second-order actuator with an output-side nonlinear spring.

### 9.2 Recommendation

**Option C, reached in stages, with Option A as the first rung.**

The reasoning is not convenience — Option A is by far the most convenient and is explicitly
*not* the recommendation. It is that Option C is the only one that satisfies three constraints
simultaneously: it keeps the CAD-derived multibody model as the single source of geometric and
inertial truth, which is this project's actual contribution and must not be diluted by inserted
fictional bodies; it represents the belt compliance, which §8 shows is the dominant gap and
which Option A cannot represent at all; and it produces the current-in / torque-out interface
that the paper's controller and all of our postponed controller work both require. Option B is
more mathematically elegant and would be the right answer if MuJoCo could author a nonlinear
joint spring; since it cannot, B pays the full cost of restructuring the model while *still*
needing a per-step Python callback — the same dependency that is C's main drawback, but with
the oracle broken as well.

Option A remains worth running first, as a measurement rather than a commitment: it is
read-only, it takes minutes, and it tells us how much of the change is attributable to the
inertia and friction corrections alone before any compliance is introduced. That decomposition
is worth having before the harder work starts.

---

## 10. Minimal implementation plan

Staged so that each stage is independently runnable and nothing is broken on the way.

**Stage 1 — rigid-equivalent parameters, read-only.** A new
`experiments/run_paper_parameters.py`, structured exactly like `run_armature_pilot.py`: write
`dof_armature = 0.2089`, `dof_damping = 1.2879`, `dof_frictionloss = 0.7883` into the
*compiled* model in memory, re-run the AB19 task and the step task, restore, and report deltas
against the frozen oracle. No existing file is touched.

Predictions stated **before** running, so that this is a test rather than a fit:
`I_eff = 0.4609 kg·m²`; `omega_n = 36.08 rad/s = 5.74 Hz`; `zeta = 0.558`; step overshoot
≈ 12.1 %; peak AB19 torque ≈ 28.2 N·m (from the pilot's verified `1.06 · I_eff · 57.72`
relation); RMS tracking error ≈ 4.58° (from `e = (Kd + b)·rms(qdot_ref)/Kp` with the larger
`b`). If the measured values land near these, the model is behaving as understood; if not,
something in the chain is wrong and that is worth more than the experiment itself.

**Stage 2 — `oslbench/drivetrain.py`, pure numpy, no MuJoCo.** The paper's equations (1)–(5) as
functions, with every constant carrying its provenance in the docstring and the `theta_a` /
`theta_j` / `theta_s` frames stated at the top. Includes `rho`, `rho_inverse`, `K_s`, the
`K_s = sqrt(p1² + 4·p2·|tau|)` identity, the friction law, and the inverse-friction current
solve. Unit-testable without MuJoCo, hence runnable anywhere: check `rho(0.055) ≈ 93 N·m`
against the top of the paper's Fig. 3, check that `rho_inverse` inverts `rho`, check the
stiffness identity.

**Stage 3 — `docs/DRIVETRAIN_PARAMETERS.md`.** One page recording the decision: which ratio
this repo uses where, that `n_a = 9` and `n_t = 50/11` for our export with 4.61 as the paper's
figure for the 3 mm build, that 49.4 / 58.4 survive only as the provenance of the existing
`forcerange` until a decision is taken about them, and the 5 mm-versus-3 mm belt caveat on
`p1`. This is the artefact to put in front of the professor.

**Stage 4 — the compliant layer (Option C), behind a flag, default off.** `oslbench/actuator.py`
integrating `theta_a` and computing `tau_j` through `rho`, plus a `motor`-actuator variant of
the bench model **as a new file**, leaving `models/osl_v2_bench.xml` untouched. The existing
position-servo path stays the default and the oracle stays reproducible.

**Stage 5 — the comparison.** The same reference trajectory through (i) the current idealised
model, (ii) the rigid-equivalent model, (iii) the compliant model, reporting rendered stiffness,
peak torque, transient behaviour and where they diverge.

### Constraints that hold throughout

The validated AB19 experiment must remain reproducible and the frozen oracle — RMS 4.3616°,
peak error 9.0907°, peak torque 16.0041 N·m, 0 % saturation — must remain available as a
baseline. If a model change moves those numbers, **that is expected and will be reported as a
model change**, never absorbed silently. Separation of model / actuator dynamics / controller /
simulation / logging / metrics / plotting is preserved; no monolithic experiment script.

---

## 11. Validation plan

**What can be checked with no hardware at all.** That our implementation of `rho` reproduces the
paper's Fig. 3 at readable points (the curve reaches ≈93 N·m at ≈0.055 rad; the model gives
93.3). That `rho_inverse` and the `K_s` identity are self-consistent. That the rigid-limit
torque balance in §8.1 is reproduced by the code. That the predicted series-stiffness shortfall
appears when a stiffness command is passed through the compliant layer. All of these check our
*implementation*, not the paper, and none of them is a validation of the model against reality.

**The cheapest hardware measurement worth asking for.** Hold the motor at zero displacement,
drive the joint slowly across its range, and log motor encoder, joint encoder and joint torque.
That yields `theta_s` from the encoders and `tau_j` from the load cell, which is our own `rho`
fit for the 5 mm belt actually fitted to our hardware — the parameter the paper leaves
ambiguous for our build. One session, no dynamometer strictly required if the midshank load
cell can be trusted for this.

**The measurement that would close the actuator side.** A free-swing ring-down with the motor
open-circuit and then short-circuit separates mechanical from electrical damping and gives an
independent estimate of reflected inertia at the joint. Much cheaper than the paper's two-actuator
rig, and it tests the number that matters most to us.

**What we would still not be able to do.** Reproduce the constant-impedance or variable-impedance
validation, both of which need a rotary dynamometer.

**We have not reproduced any part of this paper.** Everything above is a plan.

---

## 12. Research direction

**What the paper already solved.** The OSL v2 drivetrain model and its identification; the
characterisation of the belt as a nonlinear series spring; a compensating controller combining
feedback linearisation with actuator-state feedback; and hardware validation across a
dynamometer grid, simulated walking, and treadmill walking with a participant. None of this is
available to us as a contribution. **The actuator model is not ours and must never be described
as novel.**

**What we would be reproducing.** Embedding their published parameter set into a simulator, and
implementing their equations. That is replication, and should be called replication.

**What is realistically ours.** Two things, neither of them large, both defensible. The
CAD-provenance multibody model — per-link inertia tensors derived from the Onshape V2 export —
is something neither this paper nor MyoAssist's `OpenSourceLeg_KA_L1` provides; the paper gives
drivetrain parameters and a total mass, and MyoAssist parameter-matches lumped assemblies to
published totals. And the question the earlier research-direction note landed on — *what does a
controller tuned against an actuator-idealised simulator get wrong* — is materially stronger
now than it was a week ago, because its weakest ingredient has been removed. It no longer
depends on guessed rotor inertias and an assumed gear ratio; it can be run against a
**measured, published, hardware-identified** parameter set for this exact device. That is a
real improvement in defensibility, and it is the honest reason to be more confident, not less.

**What must not be claimed.** That we have validated anything against hardware. That the
physical OSL V2 has any particular parameter value beyond what the paper measured. That the
drivetrain model, the belt model, the identification method or the controller are ours. That a
MuJoCo model of the OSL is new. And no novelty claim should be manufactured to fill the gap —
if the honest description is "we replicate a published drivetrain model inside an open
CAD-derived simulator and quantify what changes", that is what it should say.

**The required wording, still in force.** On torque: *the model represents the available joint
torque authority derived from the motor/gear relationship, but idealizes the torque
production.* The 49.4, 58.4, 30 A and 0.096 N·m/A figures are **not** experimentally verified
hardware values. On mass: ours is ≈4.9558 kg against the official `KA_L1` ≈4.5275 kg excluding
socket, with the foot at 0.9152 against 0.2910 kg unresolved — and **neither is simply
"better"**.

---

## Next steps, in order

1. **Run Stage 1 today** — `experiments/run_paper_parameters.py`, read-only, writing the three
   rigid-equivalent values into the compiled model in memory and restoring them. Check the
   measured results against the five predictions in §10. This is the smallest useful change
   that can be run and validated immediately, and it touches no existing file.

2. **Write `oslbench/drivetrain.py`** — the paper's equations as a tested, numpy-only module
   with provenance in the docstrings. No model change, no MuJoCo dependency, testable in any
   environment.

3. **Write `docs/DRIVETRAIN_PARAMETERS.md`** — record which ratio is used where, why 49.4 and
   58.4 are not OSL v2 drivetrain ratios, and the 5 mm-versus-3 mm belt caveat. Fix the
   backwards comment at `tools/build_mjcf.py:247-248`. This is the professor-facing artefact.

4. **Decide, explicitly and on the record, what happens to `armature`, `damping`,
   `frictionloss` and `forcerange` in `models/osl_v2_bench.xml`** — including whether the
   frozen oracle is re-frozen against the corrected plant or retained as a historical baseline.
   Do not edit the XML before that decision is made.

5. **Build the compliant layer (Option C) behind a flag** — `oslbench/actuator.py` plus a
   `motor`-actuator bench variant as a *new* model file, default off, oracle path untouched.
   Then run the three-way comparison of §10 Stage 5.

Also outstanding, unrelated to this analysis: `docs/VALIDATION.md` still states that the
real-MuJoCo oracle comparison "has not been executed", which is no longer true. One line.
