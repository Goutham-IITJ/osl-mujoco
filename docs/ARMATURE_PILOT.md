# Armature Pilot — a simulation sensitivity study

**What this is:** a one-parameter sensitivity pilot inside the MuJoCo bench model. It asks
how much the bench's reported numbers change when one uncertain model parameter — the knee's
reflected rotor inertia (`armature`) — is varied over a plausible range. **It is not a
hardware study, not a validation, and not a claim of novelty.**

| | |
|---|---|
| **Date** | 2026-09-22 |
| **Status** | **complete — all numbers below are measured MuJoCo output** from `.venv\Scripts\python.exe experiments\run_armature_pilot.py` |
| **Script** | `experiments/run_armature_pilot.py` (read-only; restores the model) |
| **Data** | `build/armature_pilot/armature_pilot.csv` |
| **Supersedes** | the numpy-surrogate estimates circulated before this run. Every table below is now real MuJoCo. |

---

## 0. Summary in one paragraph

The `armature` value in the bench model is a placeholder, not a measurement. We varied it
across a plausible range and held everything else fixed, to find out which of the bench's
reported quantities are sensitive to it. Three different answers came back. RMS tracking
error against the AB19 gait trajectory changed **modestly — 4.362° to 5.074°, a factor of
1.16** across a 48× change in the parameter. Peak knee torque during the same gait cycle
changed **substantially — 16.00 to 46.37 N·m, a factor of 2.90**. And step-response overshoot
changed **most of all — 5.0 % to 23.6 %, a factor of 4.72**. The practical reading is that
smooth-gait tracking accuracy is a weak indicator of whether this parameter is right, while
peak torque and transient overshoot are strong ones. That is a statement about which
observables to report from this simulation. It says nothing about the physical leg, because
no hardware was involved.

---

## 1. Provenance — MEASURED, DERIVED, ASSUMED

This separation is the most important part of the document. Read it before quoting any
number.

### MEASURED — MuJoCo outputs from the actual pilot run

RMS tracking error, peak tracking error and peak knee torque for each plant on the AB19 gait
task; overshoot percentage and time-to-peak for each plant on the step task; the selected
`Kp*` and the resulting RMS in the re-tuning experiment; and the RMS obtained when the
placeholder-tuned controller is deployed on each other plant.

The baseline row reproduces the frozen benchmark: **RMS 4.362° against the oracle's 4.3616°.**
That is the check that the script perturbed nothing it should not have.

### DERIVED — arithmetic on measured or authored values, no simulation involved

`I_eff = I_body + armature`; `ζ = (Kd + b) / (2√(Kp · I_eff))`; `ω_n = √(Kp / I_eff)`; the
`Kd` values produced by the ζ = 0.7 rule; and every ratio quoted anywhere in this document.

**ζ and ω_n in the tables below are calculated from these formulae, not extracted from the
simulated response.** §5 shows they are consistent with the measured overshoot to within
0.4 percentage points, which is corroboration, not independent measurement.

### ASSUMED — chosen values, not measurements

Every armature value tested is hypothetical. The rotor-inertia range J_rotor = 3 × 10⁻⁵ to
2 × 10⁻⁴ kg·m² is a plausible span for a BLDC rotor of this class; it is not from a
datasheet. The reduction N ≈ 49.4 is taken from project documentation, not from a
manufacturer specification. Joint damping `b = 0.3 N·m·s/rad` and `frictionloss = 0.4 N·m`
are themselves placeholders, held fixed at those placeholder values throughout. The target
ζ = 0.7 is an engineering convention, not a requirement. The step operating point of 30°
flexion and the 10° step size are arbitrary choices. `I_body = 0.251998 kg·m²` is CAD-derived,
which is well founded, but it is a property of the CAD model rather than a measurement of a
physical leg.

**No claim is made anywhere in this document that the physical OSL V2 has any of the tested
armature values.** The ladder was constructed to span a plausible range. It is not known to
contain the true value.

---

## 2. What was varied

MuJoCo's `armature` on a joint is added to the diagonal of the mass matrix. It models the
**rotor inertia of the motor seen from the joint side**: a geared rotor spins N times faster
than the joint, so its inertia appears at the joint multiplied by N². With N ≈ 49.4 (ASSUMED),
N² = 2440.

| J_rotor (kg·m²) | armature = N²·J_rotor | vs the authored value | `I_eff = I_body + armature` |
|---|---|---|---|
| — | **0.0100** (authored placeholder) | 1× | 0.2620 |
| 3 × 10⁻⁵ | 0.0732 | 7.3× | 0.3252 |
| 5 × 10⁻⁵ | 0.1220 | 12.2× | 0.3740 |
| 1 × 10⁻⁴ | 0.2440 | 24.4× | 0.4960 |
| 2 × 10⁻⁴ | 0.4881 | 48.8× | 0.7401 |

`I_eff` spans **0.2620 → 0.7401 kg·m², a factor of 2.83**. Armature is the parameter worth
probing first because it is an acknowledged placeholder and because, over this range, it is
comparable to or larger than the entire CAD-derived link inertia.

---

## 3. What was held fixed

Kp = 600 N·m/rad and Kd = 17.253 N·m·s/rad (except in §6, where re-tuning is the experiment);
`I_body = 0.251998`; joint damping 0.3; **frictionloss 0.4 — see the correction note below**;
the ±142.2 N·m forcerange; the 0.5 ms timestep and `implicitfast` integrator; the ankle's own
authored servo holding its keyframe angle; the AB19 reference (subject AB19, levelground, ccw,
normal, trial 01_01, resampled to 2410 steps); and the control law itself.

The armature was written into the **compiled `mjModel` in memory** and restored at the end of
the run. `models/osl_v2_bench.xml` was never opened for writing.

> **CORRECTION.** An earlier draft of this document stated that the step experiment was run
> frictionless. **That was wrong.** `frictionloss = 0.4 N·m` and `damping = 0.3 N·m·s/rad`
> remained active for every step run reported here. Nothing was idealised away. The Coulomb
> deadband is f_c/Kp = 0.038°, negligible against a 10° step, which is why the response is
> still close to second-order — but the friction was present.

---

## 4. Result — fixed gains, varying the reflected inertia

Kp = 600, Kd = 17.253 throughout. Columns marked MEASURED come from MuJoCo; ζ and ω_n are
DERIVED.

| armature | I_eff | ζ *(derived)* | ω_n *(derived)* | RMS ° | peak err ° | peak τ N·m | overshoot |
|---|---|---|---|---|---|---|---|
| 0.0100 | 0.2620 | 0.700 | 47.85 | **4.362** | 9.091 | **16.00** | **5.0 %** |
| 0.0732 | 0.3252 | 0.628 | 42.95 | 4.429 | 9.245 | 19.82 | 8.2 % |
| 0.1220 | 0.3740 | 0.586 | 40.05 | 4.488 | 9.382 | 22.82 | 10.5 % |
| 0.2440 | 0.4960 | 0.509 | 34.78 | 4.654 | 9.777 | 30.54 | 15.7 % |
| 0.4881 | 0.7401 | 0.416 | 28.47 | **5.074** | 10.622 | **46.37** | **23.6 %** |

Across a 48× change in armature (2.83× in `I_eff`):

| quantity | span | factor | source |
|---|---|---|---|
| RMS tracking error | 4.362 → 5.074° | **1.16×** | AB19 gait, MEASURED |
| peak tracking error | 9.091 → 10.622° | 1.17× | AB19 gait, MEASURED |
| **peak knee torque** | 16.00 → 46.37 N·m | **2.90×** | AB19 gait, MEASURED |
| **step overshoot** | 5.0 → 23.6 % | **4.72×** | step task, MEASURED |
| damping ratio ζ | 0.700 → 0.416 | 0.59× | DERIVED |
| bandwidth ω_n | 47.85 → 28.47 rad/s | 0.59× | DERIVED |

**The three headline sensitivities: modest for smooth-gait RMS, large for peak gait torque,
largest for transient overshoot.**

### Which task each number came from

The RMS, peak-error and peak-torque columns come from the **AB19 gait run** (2410 steps, one
cycle, dt = 0.5 ms). The overshoot column comes from a **separate step run** — 10° command
step at 30° flexion, 1.5 s, same gains, friction and damping active, different initial
condition and different input. The two experiments share nothing but the model and the
controller.

Worth noting for future design: in a step the *initial* torque is `Kp · Δ` ≈ 105 N·m
regardless of inertia, so the step is **not** where torque sensitivity appears. Torque
sensitivity belongs to the gait task; overshoot sensitivity belongs to the step task. They
are complementary observables, not redundant ones.

---

## 5. Why the numbers move — two consistency checks

Neither of these is a new measurement. Both check that the measured results are explained by
elementary mechanics rather than by something unexplained in the model.

### Peak gait torque is inertia-dominated

The AB19 reference has a peak angular acceleration of 57.72 rad/s². Comparing measured peak
torque against `I_eff × q̈_peak`:

| armature | I_eff | measured peak τ | I_eff × q̈_peak | ratio | % of ±142.2 N·m |
|---|---|---|---|---|---|
| 0.0100 | 0.2620 | 16.00 | 15.12 | 1.058 | 11.3 % |
| 0.0732 | 0.3252 | 19.82 | 18.77 | 1.056 | 13.9 % |
| 0.1220 | 0.3740 | 22.82 | 21.59 | 1.057 | 16.0 % |
| 0.2440 | 0.4960 | 30.54 | 28.63 | 1.067 | 21.5 % |
| 0.4881 | 0.7401 | 46.37 | 42.72 | 1.085 | 32.6 % |

The ratio is flat at ≈ 1.06, so the torque demand during this gait cycle is essentially
`I_eff · q̈_ref`, with the remaining ~6 % attributable to gravity, damping and friction. **The
torque scales almost linearly with the assumed reflected inertia.** No run saturated; the
worst case used a third of the available authority.

### Overshoot matches second-order theory

For a linear second-order system, overshoot = `exp(−πζ/√(1−ζ²))`. Using the DERIVED ζ:

| armature | ζ *(derived)* | predicted overshoot | **measured overshoot** | difference |
|---|---|---|---|---|
| 0.0100 | 0.700 | 4.60 % | 5.0 % | +0.4 pt |
| 0.0732 | 0.628 | 7.92 % | 8.2 % | +0.3 pt |
| 0.1220 | 0.586 | 10.31 % | 10.5 % | +0.2 pt |
| 0.2440 | 0.509 | 15.60 % | 15.7 % | +0.1 pt |
| 0.4881 | 0.416 | 23.76 % | 23.6 % | −0.2 pt |

Agreement within 0.4 percentage points across the whole range. **The knee model behaves as a
clean second-order system here**, which means the overshoot trend needs no explanation beyond
the ζ formula: `Kd` was chosen for one value of `I_eff`, and raising `I_eff` lowers ζ because
it sits under a square root in the denominator.

In plain terms: the controller is a spring (Kp) plus a damper (Kd). More reflected inertia
means the spring accelerates the system more slowly, but the system also carries more
momentum on arrival. The damper is unchanged and cannot absorb the extra momentum, so the
joint travels further past the target.

---

## 6. Result — re-tuned gains, and the transfer cost

For each plant, the repo's own selection rule was re-run: the smallest Kp on a log grid
(100 → 8000) meeting RMS ≤ 4.3616°, with `Kd = 2ζ√(Kp·I_eff) − b` at ζ = 0.7 evaluated on
**that** plant.

| armature | I_eff | **Kp\*** | **Kd\*** | Kp\* ratio |
|---|---|---|---|---|
| 0.0100 | 0.2620 | **610** | 17.40 | 1.00× |
| 0.0732 | 0.3252 | 746 | 21.51 | 1.22× |
| 0.1220 | 0.3740 | 877 | 25.05 | 1.44× |
| 0.2440 | 0.4960 | 1162 | 33.31 | 1.90× |
| 0.4881 | 0.7401 | **1736** | 49.88 | **2.85×** |

Now deploy the gains chosen on the placeholder plant (Kp = 610, Kd = 17.40) on each other
plant:

| armature | RMS, placeholder-tuned | RMS, plant-tuned | penalty |
|---|---|---|---|
| 0.0100 | 4.326 | 4.326 | 1.000× |
| 0.0732 | 4.392 | 4.359 | 1.008× |
| 0.1220 | 4.449 | 4.313 | 1.032× |
| 0.2440 | 4.612 | 4.318 | 1.068× |
| 0.4881 | 5.024 | 4.320 | **1.163×** |

So the selection rule picks a gain up to 2.85× different, yet on the RMS metric the
"mismatched" controller loses at most 16 %.

### Caveat that must accompany the 2.85× figure

The movement in `Kp*` is driven by the ζ = 0.7 rule, which couples `Kd` to `√(Kp · I_eff)`.
From the closed-form error law of this control law, `e = (Kd + b)·rms(q̇_ref)/Kp`, the smallest
Kp meeting a spec `e` is `Kp* = (Kd + b)·rms(q̇_ref)/e` — **independent of `I_eff` if `Kd` is
held fixed.** The optimum moves because the *tuning rule* reads `I_eff`, not because the RMS
metric itself demands a different stiffness. Quote the 2.85× only with this attached.

---

## 7. Interpretation

Stated as narrowly as the data supports:

> **Within this simulation, at fixed gains, raising the assumed reflected inertia produces a
> modest change in smooth-gait RMS tracking error (1.16×), a significant change in peak gait
> torque (2.90×), and a much larger change in transient step overshoot (4.72×).**

The practical consequence for how this bench is used: **RMS tracking error against a smooth
periodic reference is a weak indicator of whether the plant parameters are right.** It can
look acceptable while the modelled dynamics are substantially different. Peak torque and
transient overshoot are far more informative about this particular parameter, and both are
already available from runs we perform.

Two corollaries for how the bench is described:

1. The AB19 tracking benchmark validates the kinematics, the reference pipeline and the
   control law. On the evidence here it is **not** a sensitive test of the model's dynamic
   parameters, and should not be described as one.
2. If peak torque is going to be reported at all — and it is the quantity that maps to motor
   current and thermal load — then its strong dependence on an assumed parameter has to be
   reported alongside it.

**This is a sensitivity result about the simulation, not a discovery about actuators, and not
a claim of novelty.** Sensitivity analysis of a model parameter is routine practice; the value
here is that the specific numbers for this specific bench are now known rather than guessed.

---

## 8. What this pilot establishes / What it does not establish

### Establishes

- Within the bench model, with all other parameters fixed, a 48× change in `armature`
  (2.83× in `I_eff`) changes AB19 RMS tracking error by 1.16×, peak AB19 knee torque by
  2.90×, and 10° step overshoot by 4.72×.
- The measured overshoot across the range is explained by second-order theory using the
  derived ζ, to within 0.4 percentage points — so the trend needs no further explanation.
- The measured peak gait torque is ≈ 1.06 × `I_eff` × peak reference acceleration across the
  range, i.e. torque demand on this task is inertia-dominated.
- The repo's gain-selection rule returns a Kp differing by up to 2.85× depending on the
  assumed armature — subject to the §6 caveat that this is a property of the ζ = 0.7 rule.
- A controller tuned on the placeholder plant still meets the RMS spec to within 1.16× on
  every other plant tested.
- The pilot script reproduces the frozen benchmark on the baseline row (4.362 vs 4.3616),
  so it did not disturb the validated configuration.

### Does not establish

- **Anything about the physical OSL V2.** No hardware was used. All five plants are
  simulations.
- **That the physical leg has any of the tested armature values.** The ladder was built to
  span a plausible range from an assumed N and an assumed rotor-inertia range. It is not
  known to contain the true value, and no value in it is asserted to be correct.
- **That the physical leg is underdamped, or that any hardware would overshoot.** The ζ and
  overshoot figures describe simulated plants built from assumed parameters.
- That `armature = 0.01` is wrong. It is an acknowledged placeholder, but this pilot measured
  no ground truth against which to call it wrong.
- Anything about damping or frictionloss, which were held fixed. Since damping and armature
  both enter ζ, a one-at-a-time sweep cannot separate their effects; a leave-one-out design
  would be needed.
- Anything about other controllers, other control laws, the ankle, ground contact, walking,
  or a person. This is one PD loop, one joint, one gait trajectory, one step, fixed base.
- That the rest of the model is correct. Every result here is conditional on the CAD inertia,
  the control law and the reference pipeline being right.
- That any of these differences matter clinically or functionally. No such criterion was
  defined or applied.

---

## 9. Reproducing

```powershell
.venv\Scripts\python.exe experiments\run_armature_pilot.py
```

Writes `build/armature_pilot/armature_pilot.csv`. Touches no existing file; the armature is
written into the compiled `mjModel` in memory exactly as the gains are, and restored at the
end. `--skip-retune` runs the fixed-gain and step experiments only.
