# Next phase: gains, live sensing, viewer performance, and where the research goes

**Status: proposal. No code, no gains, no model and no existing file were changed to
produce this.** The only new file is this one. Every number labelled `[PREDICTED]` comes
from a read-only numpy reproduction of the bench plant described in §1.9; every number
labelled `[ORACLE]` is the frozen MuJoCo result in `tests/oracle/`.

---

# Part 0 — The professor-facing version

**The short answer on the gains: the professor is right, and the reason is more
interesting than "the numbers are too big."**

Kp = 600 N·m/rad is 10.5 N·m/deg. A human knee's quasi-stiffness peaks at roughly
3–6 N·m/deg in weight acceptance and is close to an order of magnitude lower than that
through swing. So the controller is currently rendering a joint impedance two to three
times stiffer than the stiffest moment of human stance, and holding it there for the whole
cycle, including swing. As an *impedance* it is not defensible.

But Kp = 600 was not chosen because anyone wanted a stiff leg. It was chosen because it was
the smallest gain that made the tracking error acceptable. And the tracking error it was
fixing is an artifact of the control law, not a property of the leg.

The law in use is `tau = Kp·(q_ref − q) − Kd·q̇`. That final term damps *absolute* joint
velocity toward zero. It has no idea the reference is moving. So when the knee is supposed
to be moving at 5 rad/s, this term is actively braking it with 86 N·m, and the only way the
loop can keep moving is to sustain a position error large enough that `Kp·error` cancels the
brake. That gives, exactly,

    steady-state error  e = (Kd + b)/Kp · q̇_ref

With the current pair that is 29.3 ms of pure time lag. Multiply by the reference's RMS
angular velocity (2.588 rad/s) and it predicts an RMS tracking error of **4.3385°**. The
measured MuJoCo result is **4.3616°**. The prediction is within 0.5 %. The entire headline
error of this experiment is one missing term in the control law.

Add the velocity reference — `tau = Kp·(q_ref − q) + Kd·(q̇_ref − q̇)`, which is the
textbook PD *tracking* law rather than the *regulator* MuJoCo's position actuator
implements — and the same accuracy is available at a tenth of the stiffness:

| control law | Kp | as impedance | RMS error |
| --- | --- | --- | --- |
| current, `−Kd·q̇` | 600 N·m/rad | 10.47 N·m/deg | 4.28° `[PREDICTED]` / 4.3616° `[ORACLE]` |
| velocity tracking | **60 N·m/rad** | **1.05 N·m/deg** | **4.54°** `[PREDICTED]` |
| velocity tracking | 200 N·m/rad | 3.49 N·m/deg | 1.74° `[PREDICTED]` |
| + model feedforward | **20 N·m/rad** | **0.35 N·m/deg** | **0.10°** `[PREDICTED]` |

Peak torque stays between 16 and 17 N·m in every row. Nothing about torque authority,
saturation or the model changes.

So the proposal is: **do not retune Kp. Fix the law, then Kp stops being a tuning knob and
becomes a physical quantity — the impedance you actually want the prosthesis to render —
which is a number your hardware team can hand you rather than one I have to sweep for.**

That reframing is what makes the rest of the plan possible. Once stiffness is decoupled
from tracking accuracy, the interesting questions become the physical ones: what the real
actuator's reflected inertia, friction and bandwidth actually are (the model currently uses
placeholders, and the armature placeholder is probably 7–25× too small — §1.6), and how
much that fidelity actually matters. That last question — **how much actuator realism does
a prosthesis simulation need before its control results transfer?** — is the research
direction I would defend, because it is a question rather than an artifact, it is the one
thing our physical bench uniquely enables, and it is named as future work by the authors of
the framework we would otherwise be duplicating.

---

# Part 1 — Kp and Kd from first principles

## 1.1 The current control equation

From `oslbench/controller.py:95-104`, verbatim:

```
command:    q_cmd = clip(q_ref, -0.0872664626, 2.0943951024)      rad   (= [-5, 120] deg)
law:        tau   = Kp·(q_cmd - q) - Kd·q̇                          N·m
actuator:   tau   = clip(tau, -142.2, +142.2)                     N·m
```

with `Kp = 600.0` N·m/rad and `Kd = 17.253` N·m·s/rad. `write_to_model` (`:116`) installs
this into MuJoCo's `position` actuator as `gainprm[0] = Kp`, `biasprm = [0, −Kp, −Kd]`,
which makes MuJoCo evaluate the same algebra internally.

**This is a regulator, not a tracker.** The derivative channel is `−Kd·q̇`, not
`+Kd·(q̇_ref − q̇)`. MuJoCo's `position` actuator with `kv` cannot express the second form,
because `ctrl` is a single scalar. That is a structural property of the actuator type, not
an oversight in our code — but it is the root cause of everything in §1.7.

## 1.2 What Kp physically is

Kp is a **virtual rotational stiffness**, units N·m/rad. It is the slope of the torque the
actuator produces against the angular displacement of the joint from its commanded
equilibrium. Physically it is the spring the prosthesis *feels like* at the knee.

Three equivalent readings, all useful:

- **As an impedance.** 600 N·m/rad = 10.472 N·m/deg. Human knee quasi-stiffness in stance
  is commonly reported around 3–6 N·m/deg for an adult of this mass, and swing-phase knee
  impedance in finite-state prosthesis controllers is typically well under 1 N·m/deg. So
  the current value is roughly 2–3× peak human stance stiffness and 10–100× swing. *(These
  comparison ranges are from recall and must be pinned to a citation — see §4.6.)*
- **As a loop gain.** Kp sets the closed-loop natural frequency, `ω_n = √(Kp/I_eff)`.
- **As a disturbance rejection budget.** A disturbance torque `τ_d` produces a standing
  error `τ_d/Kp`. At Kp = 600 a 1 N·m unmodelled torque costs 0.095°.

## 1.3 What Kd physically is

Kd is a **virtual rotational damping**, units N·m·s/rad. It dissipates energy proportional
to joint velocity. In the current law it damps *absolute* velocity, so it is a viscous brake
against the world; in the tracking law it would damp *error* velocity, which is what damping
is normally for.

Kd = 17.253 N·m·s/rad = 0.301 N·m·s/deg. For reference, the joint's own authored mechanical
damping is `b = 0.3` N·m·s/rad — the servo is adding **58× the joint's physical damping**.

## 1.4 The plant equation

`knee_prox` carries no free joint, so the bench is welded to the world. With the ankle held
at its keyframe by its own authored servo, the knee is a genuine single-DOF pendulum:

    I_eff · q̈  =  tau_act  −  m·g·d·sin(q)  −  b·q̇  −  f_c·sgn(q̇)

| symbol | value | units |
| --- | --- | --- |
| `I_eff` | 0.261998 | kg·m² (= 0.251998 CAD link inertia + 0.010 armature) |
| `m·g·d` | 8.8529 | N·m (gravity moment at full extension) |
| `b` | 0.30 | N·m·s/rad |
| `f_c` | 0.40 | N·m |
| `dt` | 0.0005 | s |

Two properties of this plant that matter and are easy to miss:

- The passive pendulum frequency is `√(mgd/I_eff)/2π = 0.9252 Hz` — which sits essentially
  on top of the gait fundamental (1/1.205 s = 0.830 Hz). The leg's own unforced dynamics
  are at the frequency of the task.
- Gravity contributes an *effective stiffness* of `d/dq[mgd·sin q] = mgd·cos q ≤ 8.853`
  N·m/rad, i.e. 0.155 N·m/deg. Any Kp above ~45 N·m/rad already dominates gravity fivefold.

## 1.5 The closed-loop equation

Linearise about an operating point `q₀` and write `e = q_ref − q`:

    I_eff · ë  +  (Kd + b) · ė  +  (Kp + mgd·cos q₀) · e  =  I_eff·q̈_ref + b·q̇_ref
                                                             + mgd·sin q₀ + f_c·sgn(q̇)
                                                             + Kd·q̇_ref        ← ONLY in the
                                                                                  tracking law

The bracketed `Kd·q̇_ref` term is present in the tracking law and **absent in ours**, which
is why our right-hand side carries a term proportional to reference velocity that nothing
cancels.

Neglecting gravity's small contribution to stiffness:

    ω_n = √(Kp / I_eff)              ζ = (Kd + b) / (2·√(Kp · I_eff))

At Kp = 600, Kd = 17.253: **ω_n = 47.85 rad/s = 7.616 Hz, ζ = 0.700.**

| Kp | Kd (ζ=0.7) | ω_n rad/s | f_n Hz | lag (Kd+b)/Kp | Kp as N·m/deg | friction deadband |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 30 | 3.625 | 10.70 | 1.703 | 130.83 ms | 0.524 | 0.764° |
| 60 | 5.251 | 15.13 | 2.409 | 92.51 ms | 1.047 | 0.382° |
| 100 | 6.866 | 19.54 | 3.109 | 71.66 ms | 1.745 | 0.229° |
| 150 | 8.477 | 23.93 | 3.808 | 58.51 ms | 2.618 | 0.153° |
| 200 | 9.834 | 27.63 | 4.397 | 50.67 ms | 3.491 | 0.115° |
| 300 | 12.112 | 33.84 | 5.386 | 41.37 ms | 5.236 | 0.076° |
| **600** | **17.253** | **47.85** | **7.616** | **29.26 ms** | **10.472** | **0.038°** |
| 1200 | 24.524 | 67.68 | 10.771 | 20.69 ms | 20.944 | 0.019° |
| 2000 | 31.747 | 87.37 | 13.905 | 16.02 ms | 34.907 | 0.011° |

## 1.6 Parameter provenance

This is the table to put in front of the professor. "Assumed" and "placeholder" are not the
same thing: assumed means a considered engineering choice, placeholder means a number
inserted so the file would compile.

| Parameter | Value | Provenance | Confidence |
| --- | --- | --- | --- |
| knee link inertia about the knee axis | 0.251998 kg·m² | **CAD-derived**, per-link tensors from the untouched Onshape V2 export, verified at runtime against `mj_mulM` | High |
| link masses, COMs, geometry, joint axes | — | **CAD-derived**, same source | High |
| knee ROM `[-5°, 120°]` | −0.0873 … 2.0944 rad | **Authored** from the OSL V2 spec | Medium-high — the KA_L1 model uses `[0°, 120°]`; the −5° admits hyperextension KA_L1 clips |
| `forcerange` ±142.2 N·m | — | **Derived** from the 49.4 gain × ±2.88 N·m motor-shaft ctrlrange relationship. Use the required wording: *the model represents the available joint torque authority derived from the motor/gear relationship, but idealizes the torque production.* Not experimentally verified hardware values | Medium |
| `armature` = 0.010 kg·m² | 0.010 | **PLACEHOLDER.** README says so explicitly. See below — probably 7–25× too small | **Low** |
| `damping` = 0.30 N·m·s/rad | 0.30 | **PLACEHOLDER** | **Low** |
| `frictionloss` = 0.40 N·m | 0.40 | **PLACEHOLDER** | **Low** |
| `m·g·d` = 8.8529 N·m | — | **Computed** from CAD masses and COMs | High |
| `dt` = 0.5 ms, `implicitfast` | — | **Authored** solver choice | High (`ω_n·dt = 0.044`, far from any stability limit) |
| ζ = 0.7 | 0.7 | **Assumed** — a generic servo convention (fast, ~4.6 % overshoot). Not a prosthesis requirement, not measured | Medium |
| **Kp = 600 N·m/rad** | 600 | **Selected by sweep.** Criterion recorded in `BENCH_TUNING_AND_DATASET.md` §D: *"the largest gain that is still arguable as a physically implementable impedance."* That is a judgement, not a derivation | **Low** |
| Kd = 17.253 N·m·s/rad | 17.253 | **Derived** from ζ = 0.7, Kp = 600, I_eff, b — arithmetically sound, but inherits Kp's and the placeholders' weakness | Medium (conditional on Kp) |
| ankle servo kp = 60, kv = 0 | — | **PLACEHOLDER**, authored in the MJCF, deliberately left alone as a boundary condition | Low, but irrelevant here |

**The armature placeholder deserves its own paragraph, because it is the biggest single
error in the plant.** `armature` in MuJoCo is the rotor inertia reflected through the
transmission, which scales as `N²·J_rotor`. The OSL V2 knee's torque authority implies a
total reduction of `N ≈ 49.4`, so `N² = 2440`. For a BLDC rotor inertia anywhere in the
normal range for this actuator class:

| `J_rotor` | reflected `N²·J` | vs the 0.010 placeholder | resulting `I_eff` | Kd(ζ=0.7, Kp=600) | ω_n |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 3e−5 kg·m² | 0.0732 | 7.3× | 0.3252 (+24 %) | 19.256 (+11.6 %) | 6.84 Hz |
| 5e−5 | 0.1220 | 12.2× | 0.3740 (+43 %) | 20.672 (+19.8 %) | 6.38 Hz |
| 1e−4 | 0.2440 | 24.4× | 0.4960 (+89 %) | 23.852 (+38.3 %) | 5.54 Hz |
| 2e−4 | 0.4881 | 48.8× | 0.7401 (+183 %) | 29.201 (+69.3 %) | 4.53 Hz |

In other words the reflected rotor inertia is plausibly **comparable to or larger than the
entire CAD link inertia**, and the model currently ignores it. This is the single
highest-value number to obtain, and it is directly measurable on our own bench.

## 1.7 Why Kd = 17.253 was selected, and whether that is justified

**The selection chain, honestly stated:**

1. Someone chose ζ = 0.7 as the target damping ratio. Generic servo practice.
2. `kd_for_damping_ratio` (`controller.py:56`) solves `ζ = (Kd + b)/(2√(Kp·I_eff))` for Kd,
   crediting the joint's own 0.3 N·m·s/rad. Arithmetically correct, and the credit matters
   at low Kp.
3. A four-point sweep (Kp ∈ {60, 200, 600, 2000}) over five synthetic references picked
   Kp = 600 on accuracy-vs-plausibility grounds.
4. Kd followed from step 2 once Kp was fixed.

**What is justified:** step 2 is sound and robust. The dominant term is `√(Kp·I_eff)`, and
I_eff is 96 % CAD-derived, so the placeholder armature perturbs Kd by well under 2 % *at the
current armature value*. The `−b` credit is small (1.7 % of Kd) so the damping placeholder
barely propagates either. Step 2 is not where the problem is.

**What is not justified:**

- **ζ = 0.7 is a convention, not a requirement.** No prosthesis specification asks for
  ζ = 0.7 at the knee. The value that matters physically is the *damping the device is
  meant to render*, which during swing is a deliberate design parameter of the prosthesis
  and during stance is dominated by the load the bench does not carry.
- **Kp = 600 is not derived from anything.** The stated criterion is explicitly a judgement
  call, and §1.8 shows the accuracy it was bought for is recoverable by other means.
- **The whole chain is conditioned on placeholders that are probably wrong** (§1.6). If
  `I_eff` is really 0.37–0.50 rather than 0.262, then ζ = 0.7 at Kp = 600 needs
  Kd ≈ 20.7–23.9, so the *current* Kd is under-damped by 20–38 % with respect to the real
  plant, and ω_n is 5.5–6.4 Hz rather than 7.6 Hz.
- **It ignores realizability.** A virtual spring-damper rendered by a sampled controller
  driving a geared actuator has a hard passivity limit (Colgate & Brown Z-width):
  `b_physical > Kp·T/2 + |Kd|`. At a 1 kHz control rate, Kp = 600 / Kd = 17.253 demands
  17.55 N·m·s/rad of *physical* joint damping; the model has 0.30 and the hardware will
  have less. That criterion is conservative (it is a passivity bound for human interaction,
  not a stability bound for a bench), and note it is violated at every gain in the table —
  which is itself the finding: **the binding hardware constraint is on Kd, not Kp**, and
  nobody has asked what Kd the real OSL can render.

## 1.8 The decisive result: the error is the missing velocity reference

For the current law, in steady tracking of a smooth reference, `ë ≈ 0` and the residual is
set by the uncancelled `Kd·q̇_ref` term:

    e  =  (Kd + b)/Kp · q̇_ref        →  a pure time lag of (Kd + b)/Kp seconds

Evaluate it against the AB19 reference (`rms(q̇_ref) = 2.588318`, `peak = 5.107727` rad/s):

| quantity | closed-form prediction | `[ORACLE]` measured in MuJoCo | agreement |
| --- | --- | --- | --- |
| lag | 29.255 ms | 29.500 ms | 0.8 % |
| RMS error | 4.3385° | 4.3616° | **0.53 %** |
| peak error | 8.5615° | 9.0907° | 5.8 % (remainder is inertial) |

**The headline result of this experiment is explained to within half a percent by one
missing term.** It is not a statement about the leg, the gains, the CAD model or the human
data. It is the signature of using a position *regulator* to follow a *trajectory*.

Consequences, from the read-only numpy plant (§1.9), sweeping Kp with Kd = 2·0.7·√(Kp·I_eff) − b:

| Kp | N·m/deg | Kd | current `−Kd·q̇` | `+Kd(q̇_ref−q̇)` | `+` velocity tracking `+` model feedforward |
| ---: | ---: | ---: | --- | --- | --- |
| 20 | 0.349 | 2.905 | RMS 18.194°, τ 7.7 | RMS 9.118°, τ 11.8 | **RMS 0.103°, τ 16.1** |
| 40 | 0.698 | 4.232 | RMS 14.684°, τ 9.6 | RMS 5.958°, τ 14.9 | RMS 0.099°, τ 16.1 |
| **60** | **1.047** | **5.251** | RMS 12.630°, τ 10.7 | **RMS 4.539°, τ 16.0** | RMS 0.096°, τ 16.1 |
| 100 | 1.745 | 6.866 | RMS 10.187°, τ 12.1 | RMS 3.104°, τ 16.7 | RMS 0.091°, τ 16.1 |
| 150 | 2.618 | 8.477 | RMS 8.462°, τ 13.1 | RMS 2.228°, τ 17.1 | RMS 0.087°, τ 16.1 |
| 200 | 3.491 | 9.834 | RMS 7.378°, τ 13.8 | RMS 1.738°, τ 17.2 | RMS 0.084°, τ 16.1 |
| 300 | 5.236 | 12.112 | RMS 6.053°, τ 14.5 | RMS 1.206°, τ 17.3 | RMS 0.082°, τ 16.1 |
| **600** | **10.472** | **17.253** | **RMS 4.277°, τ 15.4** | RMS 0.623°, τ 17.2 | RMS 0.078°, τ 16.1 |
| 1200 | 20.944 | 24.524 | RMS 3.004°, τ 15.8 | RMS 0.314°, τ 17.0 | RMS 0.076°, τ 16.1 |

Read the bolded cells: **velocity tracking at Kp = 60 matches the current result at
Kp = 600.** Ten times less stiffness, same accuracy, and the peak torque is unchanged at
16–17 N·m in every single row — which is itself worth reporting, because it says none of
this is about torque authority.

Add gravity/inertia/friction feedforward — all of which are *known quantities in
simulation*, taken from the reference, not from the measured state, so no noise is injected
— and tracking becomes essentially exact at any stiffness in the physiological range.

The feedforward torque is exactly the plant's required torque:

    tau_ff = I_eff·q̈_ref + b·q̇_ref + mgd·sin(q_ref) + f_c·sgn(q̇_ref)

peak 16.097 N·m, and open-loop it alone tracks the reference to 0.075° RMS `[PREDICTED]`.

## 1.9 Validity of these predictions

Everything marked `[PREDICTED]` is from a numpy reproduction of the equation in §1.4,
integrated with semi-implicit Euler and Coulomb friction as a stiction test, driven by the
**same** reference the real experiment uses — the repo's own `oslbench.reference` module
loading `build/AB19_knee_gait_reference.csv` and resampling it onto the 0.5 ms grid. It
touches no file and imports no MuJoCo.

Its fidelity, at the validated operating point:

| | numpy surrogate | `[ORACLE]` MuJoCo | error |
| --- | ---: | ---: | ---: |
| RMS error | 4.2791° | 4.3616° | −1.89 % |
| peak error | 8.9243° | 9.0907° | −1.83 % |
| peak torque | 15.39 N·m | 16.004 N·m | −3.8 % |
| measured lag | 29.0 ms | 29.5 ms | −1.7 % |

So the surrogate is consistently ~2 % optimistic — almost certainly because it models
gravity as `mgd·sin q` about a COM assumed to lie on the link axis, and uses a different
integrator. **That is fine for comparing control laws, which is all it is used for, and not
fine for reporting an absolute number.** Every predicted value above should be re-measured
in MuJoCo before it is quoted anywhere.

## 1.10 What to ask the professor and the hardware team

Ordered by how much each answer changes the model. The first three are the ones I would
actually chase.

1. **Rotor inertia and total reduction ratio of the OSL V2 knee actuator.** `J_rotor`
   (kg·m², motor datasheet) and `N` (belt ratio × internal gearbox). These give `armature =
   N²·J_rotor` directly. Current placeholder 0.010 is probably 7–25× low, and it moves
   `ω_n` by up to −40 %. *If the datasheet is unavailable, this is measurable on our own
   bench: a free-swing ring-down of the shank with the motor de-energised and open-circuit
   vs. short-circuit gives both the inertia and the electrical damping.*
2. **What control rate does the real OSL run at, and what Kp/Kd does its own controller
   use?** The `opensourceleg` library's impedance controller has stiffness and damping
   parameters in defined units. Whatever those are, *those* are the physically appropriate
   gains, and the simulation should be asked to match them rather than to out-perform them.
   This single answer turns Kp from a tuning knob into a measured constant.
3. **Joint friction and viscous damping, measured.** A slow constant-velocity sweep at
   several speeds gives `f_c` (intercept) and `b` (slope) by least squares. Replaces two
   placeholders in one afternoon on the bench.
4. **Closed-loop torque bandwidth and end-to-end latency.** A chirp on torque command with
   the output locked gives the −3 dB point; a step gives the transport delay. This is the
   number that decides whether *any* of our simulated bandwidth claims are meaningful.
5. **Is impedance or trajectory tracking the intended control paradigm for this project?**
   This is the question for the professor rather than the hardware team, and it determines
   the whole gain-selection criterion (§1.11). Our bench experiment implicitly assumed
   trajectory tracking; prosthetics practice is overwhelmingly impedance.
6. **Torque–speed envelope and thermal limits.** Lower priority: peak demand is 11 % of
   authority, so this is second-order for *this* experiment.

## 1.11 Physically grounded gain-selection methods, ranked

**Recommended: (A) then (B).** Not a bandwidth target, and not another sweep.

**(A) Impedance specification — Kp and Kd are prescribed, not tuned.** Set Kp and Kd to the
stiffness and damping the prosthesis is *intended to render*, taken from the hardware
controller or from the human quasi-stiffness literature, possibly phase-dependent
(high in stance, low in swing — which is what every deployed powered knee does). Then
tracking accuracy is achieved by the *feedforward* path, not by the gains. This is the only
method under which the simulation's gains are comparable to the hardware's, which is the
whole point of a twin.

**(B) Realizability bounds — establish the admissible box first.** Lower bounds on Kp from
physics: friction deadband `f_c/Kp < 0.25°` → Kp > 91.7; a 1 N·m disturbance costing < 1° →
Kp > 57.3; dominating gravity stiffness fivefold → Kp > 44.3. Upper bounds on Kd from the
sampled-system passivity limit and from measured physical damping. This gives a defensible
*range* — roughly **Kp ∈ [60, 150] N·m/rad, i.e. 1.0–2.6 N·m/deg** — inside which (A)
chooses.

**(C) Target bandwidth.** Defensible only once the actuator's own bandwidth is measured: a
loop bandwidth above the actuator bandwidth is fiction. Cannot be used yet, and this is
exactly what makes the current 7.6 Hz claim unsupported.

**(D) Match the hardware controller's gains.** Strictly the most rigorous — but only
meaningful once the *plant* matches too, otherwise identical gains on different plants give
different behaviour. Becomes the right answer after Phase 3.

**(E) Another sweep.** Explicitly rejected. A sweep optimises a tracking metric that §1.8
shows is not a property of the leg.

## 1.12 What the professor probably means by "far off"

Most likely, in order:

1. **"That is not an impedance a leg can have."** 10.5 N·m/deg held through swing. Almost
   certainly the primary objection, and it is correct.
2. **"You have a damping coefficient 58× the joint's own."** Kd = 17.25 vs b = 0.30.
3. **"Where did these come from?"** They came from a sweep whose criterion was a judgement
   call, over a plant with three placeholder parameters (§1.6).
4. **"A real actuator cannot do that."** No bandwidth, latency, backlash or compliance in
   the model, so the simulation will happily accept gains the hardware would reject —
   possibly violently.
5. **Possibly: "you are solving the wrong problem."** Which §1.8 confirms outright.

**Nothing in this section requires changing Kp yet.** The recommended order is: fix the law
(a controller change with no physics assumption), re-run the oracle to confirm the *current*
pair still reproduces the frozen result, then re-derive gains under method (A)/(B) once
§1.10 items 1–3 are answered.

---

# Part 2 — Architecture for live human data driving the bench

## 2.1 The layering

Six layers, each with one job, each independently testable, each replaceable. The arrows
carry data only; nothing downstream can reach back.

```
 [1] ACQUISITION          thigh IMU, own thread, bounded queue
       |                  ImuSample(t_host, t_dev, seq, gyro[3], acc[3], quat?)
       v
 [2] ORIENTATION          complementary / Madgwick -> sagittal thigh pitch
       |                  ThighState(t, theta_thigh, thetadot_thigh, valid)
       v
 [3] GAIT PHASE           phase variable from the thigh phase portrait
       |                  PhaseState(t, phi in [0,100), phi_dot, cadence, confidence)
       v
 [4] REFERENCE MODEL      q_ref = f(phi, speed, task)   <-- today: the AB19 spline
       |                  Reference(q_ref, qdot_ref, qddot_ref)          <-- NOTE
       v
 [5] CONTROLLER           oslbench.controller  -- UNCHANGED in structure
       |                  tau
       v
 [6] SIMULATION           oslbench.simulation.step()  -- UNCHANGED
       |
       +--> [7] VISUALISATION (viewer + dashboard)  -- pure consumers
```

Proposed file layout, all new, nothing existing moved:

```
oslbench/sensing/source.py      [1]  ImuSource protocol + Csv/Synthetic/Serial/Udp impls
oslbench/sensing/orientation.py [2]  complementary + Madgwick, sagittal projection
oslbench/gait/phase.py          [3]  phase variable, adaptive oscillator alternative
oslbench/gait/reference_model.py[4]  q_ref(phi, speed, task), and its derivatives
experiments/run_live_stream.py       thin entry point
```

## 2.2 What the controller should actually receive

**`q_ref`, `q̇_ref`, and optionally `q̈_ref` — in joint space, in radians, on the physics
clock. Nothing else.** Not the IMU, not the thigh angle, not the phase.

Part 1 makes this non-negotiable: the controller *needs* `q̇_ref` to be a tracker at all,
and `q̈_ref` if feedforward is used. So layer [4] must emit derivatives, not just a
position. That is a requirement discovered by Part 1 and it should be designed in from the
start rather than retrofitted.

**Why the controller must not consume raw IMU** — five reasons, strongest first:

1. **Frame and quantity mismatch.** The IMU measures *thigh* orientation in the world. The
   controller commands a *knee* angle. Anything that converts between them is, by
   definition, a gait model — so it belongs in layers [2]–[4] where it can be tested,
   swapped and validated, not buried in the control law.
2. **Noise into the derivative channel.** With `Kd = 17.25`, 0.05 rad/s of gyro noise is
   0.86 N·m of torque chatter. Feeding an unfiltered sensor signal anywhere near the
   derivative path is how simulations acquire behaviour that destroys hardware.
3. **Dropouts and latency become torque transients.** A missed BLE packet is a step change
   in `q_ref`, and a step into a stiff PD is an impulse. The phase layer can coast on
   `φ̇` through a dropout; a direct pass-through cannot.
4. **It destroys the validation chain.** The controller and stepping loop are currently
   proven bit-identical to the frozen oracle. If the controller's inputs change type, that
   proof no longer applies to the thing being run. Keeping the controller's signature
   `(q_ref, q̇_ref)` means the oracle still covers it.
5. **It makes the result uninterpretable.** If live tracking is poor, you need to know
   whether the sensor, the estimator, the reference model or the controller is at fault.
   A pipeline with a single fused block cannot answer that. Layered, each stage has its own
   ground truth.

*The one strong reason to break this rule, for completeness:* if the research question
itself is end-to-end learned control from raw sensing (an RL policy consuming IMU directly).
That is a different project, and it is the one MyoAssist already supports.

## 2.3 Prerecorded replay vs live sensing — the actual distinction

It is not "file vs device". It is **who owns the clock, and whether the phase is known or
estimated.**

| | prerecorded replay (today) | live sensor-driven |
| --- | --- | --- |
| phase source | computed analytically, `φ = 100·(t mod T)/T` | **estimated** from a noisy signal |
| clock | the simulation's own `data.time`; deterministic | the **sensor's** arrival times; the sim must follow |
| period | fixed at 1.2050 s forever | varies stride to stride; may stop, stumble, change speed |
| repeatability | bit-identical across runs | never repeats |
| failure modes | none | dropout, bias drift, mis-detected heel strike, phase slip |
| what is being tested | the controller and the plant | **the estimator**, mostly |
| latency | zero by construction | real, must be budgeted and measured |

The engineering consequence: **the current pipeline gets the reference by integer-indexing
an array at step `k`.** That is a clock assumption baked into the data structure. The single
enabling refactor is to replace `ref_rad[k]` with `reference_model(φ)`, where `φ` is
supplied from outside. With `φ` supplied by the analytic clock, the result must remain
bit-identical to the oracle — which is exactly how you prove the refactor changed nothing.

## 2.4 Minimum viable version — four steps, in this order

**MVP-0 — phase-driven reference, no hardware.** Change only *how* `q_ref` is looked up:
from `array[k]` to `f(φ)` with `φ` from the existing analytic clock. Acceptance test:
`experiments/verify_against_oracle.py` still passes bit-identically. Zero new physics, zero
new dependencies, one afternoon. **Everything else depends on this and nothing else does.**

**MVP-1 — synthetic IMU over a real socket.** Generate a thigh-angle signal (from AB19's own
thigh kinematics, or a parameterised sinusoid at gait cadence), corrupt it realistically
(gyro white noise + random-walk bias, accelerometer noise, 100 Hz sampling, occasional
dropped packets, injected transport delay), and stream it over UDP to the simulator from a
separate process. **This is the highest-value step in the whole of Part 2**, because it is
the only configuration where the *true* phase is known exactly, so the phase estimator's
error can be measured rather than guessed. It also exercises every real-time concern —
threading, queueing, clock domains, interpolation, dropout handling — with no hardware risk
and full repeatability.

**MVP-2 — recorded real IMU, replayed in real time.** A short walking recording from a real
IMU (or the Camargo dataset's own IMU channels, if present in the deposit — check). Real
sensor characteristics, still repeatable, still has approximate ground truth from the
dataset's heel-strike labels.

**MVP-3 — live IMU on a person.** Only after MVP-1 and MVP-2 are quantified.

## 2.5 Signals, rates, and synchronisation

**Minimum signal set:** sagittal-plane thigh angular velocity (`gyro_y`) and sagittal thigh
angle. One IMU, thigh-mounted. That is genuinely enough for a phase variable. Everything
else is validation instrumentation: a shank IMU (cross-check), a foot switch or insole
(ground-truth heel strike, for measuring estimator error only — never an input).

**Rates.** IMU 100 Hz minimum, 200 Hz preferred. Orientation filter at the sensor rate.
Phase estimator at the sensor rate. Reference *evaluation* at the physics rate, 2 kHz.
Physics `dt` stays 0.5 ms and is never negotiable.

**The 100 Hz → 2 kHz gap is the central synchronisation problem.** A zero-order hold
produces a 10 ms staircase on `q_ref`, and each 10 ms step is a discontinuity into a stiff
PD — with `q̇_ref` obtained by differencing, it is an impulse train. The correct treatment
falls straight out of Part 1: **extrapolate the phase, not the angle.**

    phi(t) = phi_k + phi_dot_k · (t - t_k)      evaluated every physics step
    q_ref  = f(phi(t)),   qdot_ref = f'(phi(t)) · phi_dot_k

`f` is already a cubic spline with an analytic derivative (`oslbench.reference.NaturalCubic`
has `.deriv`), so `q̇_ref` is exact and free, with no differentiation noise anywhere. Clamp
the extrapolation after ~50 ms without a sample and freeze, so a disconnected sensor coasts
to a stop instead of running away.

**Three clocks, and they must be reconciled explicitly:** the sensor device clock (drifts,
may not exist), the host wall clock (the only shared reference), and MuJoCo's `data.time`
(advances only when you step). Timestamp on arrival at the host, keep `t_dev` for
jitter diagnosis only, and let the simulation advance in wall-clock-paced blocks (see Part 3
§3.6 — it currently does not pace at all).

**Latency budget, to be measured not assumed.** Sensor sampling + filter group delay +
transport + queue + estimator lag + the controller's own `(Kd+b)/Kp` lag. Wired USB/serial
or UDP over Ethernet is 1–5 ms. **BLE is typically 30–100 ms and is probably disqualifying**
for any real-time claim — that alone may decide the hardware choice. Note that the
controller's 29 ms lag (§1.8) *adds* to sensor latency, which is another reason to fix Part 1
before Part 2 lands.

---

# Part 3 — Diagnosing the live-viewer lag

## 3.1 What the code actually does

From `oslbench/viewer.py:334-470`:

```
FRAME_HZ = 60.0
spf = max(1, int(round(speed * (1.0/FRAME_HZ) / dt)))      # 0.5x -> 17 steps/frame

while viewer.is_running():
    dash.pump()                       # on the physics thread
    for _ in range(spf): sim.step()   # 17 x 0.5 ms = 8.5 ms of sim time
    viewer.sync()
    dash.update(...)                  # json.dumps of ~1280 floats, physics thread
```

**There is no wall-clock pacing anywhere in this loop.** No sleep, no frame deadline, no
catch-up logic. The achieved playback rate is therefore

    achieved = spf · dt / (wall time per iteration)

and to hit the requested 0.5× each iteration must complete in 17.0 ms. If an iteration takes
40 ms, the demo runs at 0.21× and looks like slow motion — while the *plots stay perfectly
correct*, because they are indexed by `k`, i.e. by gait phase, not by wall time. **That is
exactly the reported symptom.**

**The instrumentation already exists.** Line ~449 computes `rt = (data.time − sim0) /
(perf_counter() − wall0)` and prints it into the dashboard status line as
`"0.5x requested, N.NNx achieved"`. **Read that number first** — it converts the whole
question from "does it feel laggy" to "the loop is achieving 0.18×", before running any
benchmark at all.

## 3.2 The leading hypothesis, stated so it can be refuted

Not a machine problem and not a MuJoCo problem: **the scene is an undecimated CAD assembly.**

| measurement | value |
| --- | --- |
| geoms in `models/osl_v2_bench.xml` | 221 (218 visual, 1 collision) |
| distinct mesh assets | 80 |
| unique triangles loaded | ~1,310,000 |
| **triangles rasterised per frame** | **~3,552,000** |
| at 60 Hz | ~213 M triangles/s, roughly doubled by the shadow pass |
| physics | 2 DOF, 1 collision geom, 3 bodies |

The top contributors are fasteners: a single McMaster socket-head screw mesh at ~64,500
triangles, instanced 8 times (515,776 triangles of screw), 352,260 of a second screw,
246,972 of a third, 186,576 of a spacer instanced 13 times — plus a fully modelled
Raspberry Pi 4 including its USB ports and SDRAM package. 213–426 M triangles/s is 10–50×
over the budget of a typical laptop integrated GPU.

Meanwhile the physics is a 2-DOF pendulum that should step in single-digit microseconds:
17 steps ≈ 0.1–0.3 ms against a 17 ms frame budget. **If this hypothesis is right, physics is
using under 2 % of the loop and rendering is using nearly all of it.**

Two secondary suspects, both real and both cheap to measure:

- `WebDashboard.update()` runs `json.dumps` over four decimated arrays (`MAX_PLOT_PTS = 320`
  → ~1280 floats) plus scalars, **on the physics thread, every frame**. The docstring's
  claim that the dashboard "cannot slow the physics" is true of the *browser's* polling —
  the page reads a pre-encoded body — but not of the *encoding*, which is synchronous.
- `dash.pump()` and `viewer.sync()` are also on the physics thread, and the passive viewer's
  render thread contends for the GIL regardless.

## 3.3 The ten-second experiment to run before any benchmark

Every visual geom is `class="visual"` → `group="2"`. **The MuJoCo viewer can toggle geom
groups live with the number keys.** Start the demo, press the key for group 2, and watch the
achieved-rate readout in the dashboard status line.

- Rate jumps from ~0.2× toward ~0.5× → **rendering is the bottleneck, confirmed, no code
  changed.**
- Rate does not move → rendering is not the bottleneck; proceed to the full benchmark.

This single test discriminates the leading hypothesis from everything else and costs
nothing.

## 3.4 The four-configuration benchmark

New file `experiments/benchmark_realtime.py` (proposed, not written). A fixed workload —
one full AB19 cycle, `n = 2410` steps — run in each configuration, with per-section
`time.perf_counter()` accumulators around the four blocks already identified.

| | configuration | what it isolates |
| --- | --- | --- |
| **A** | headless, physics only, no viewer, no dashboard | pure `mj_step` throughput — the floor |
| **B** | viewer, no dashboard (`--dashboard none`) | cost of `launch_passive` + `sync()` |
| **C** | viewer + browser dashboard (the demo as shipped) | cost of `pump` + `update` + JSON |
| **D** | C, but render decoupled: `sync()` every *m*-th iteration, m ∈ {1, 2, 4, 8}; and separately with visual geoms hidden | whether the cost scales with **render count** or with **step count** |

Add **B′**: viewer open but visual geoms hidden (`viewer.opt.geomgroup[2] = 0`) — the
programmatic version of §3.3, which separates "rendering at all" from "rendering *this
scene*".

**Metrics per configuration:** wall time; simulated time; real-time factor; steps/second;
and the four accumulators — `t_physics`, `t_sync`, `t_dash_update`, `t_dash_pump` — as
absolute ms and as a percentage of wall time. Report mean **and** 95th percentile per
iteration, because a stutter is a tail problem and a mean will hide it. Also log the
achieved `rt` trace over time to see whether it degrades as the plot arrays grow.

**Machine context to record alongside, once:** CPU model, GPU (integrated vs discrete),
whether the MuJoCo window is on a high-DPI or scaled display, monitor refresh rate, whether
vsync is in force, Windows power plan, and whether anything else is contending for the GPU.
A 4K scaled display quadruples fragment work and is a common and invisible cause of exactly
this symptom.

## 3.5 Decision table — result to root cause

| observation | conclusion | minimum fix |
| --- | --- | --- |
| A ≫ 2410 steps in 1.205 s, and B ≈ A | physics fine, rendering fine → look at the dashboard | see next rows |
| A fast, **B ≪ A**, and **B′ ≈ A** | **rendering this scene** — the leading hypothesis | decimate/hide visual meshes (§3.7) |
| A fast, B ≪ A, **B′ ≈ B** | rendering *at all* is expensive → GPU, driver, display scaling, vsync | lower window resolution; disable shadows/reflection |
| B fine, **C ≪ B**, `t_dash_update` large | **JSON encoding on the physics thread** | move `update` off-thread, or publish at 10 Hz not 60 Hz |
| B fine, C ≪ B, `t_dash_update` small | GIL contention with the browser/HTTP thread | serve on a separate process |
| D: cost ∝ number of `sync()` calls | **render-bound**, confirmed independently | decouple render rate from step rate |
| D: cost ∝ number of steps | physics-bound (unlikely at 2 DOF) | reduce `spf`, or accept |
| all configurations slow, high 95th-percentile tail, low means | **Python/OS scheduling**, not any one block | raise process priority; `timeBeginPeriod`; check power plan |
| achieved rate degrades as `k` grows | array slicing / plot growth | pre-allocate the decimated buffers |

## 3.6 The fix that is needed regardless of the outcome

**The loop has no wall-clock pacing, and it should.** Even on a machine fast enough to keep
up, `--speed 0.5` currently means "take 17 steps per iteration and go as fast as you can",
not "run at half real time". Add a frame deadline:

```
next_frame = perf_counter()
while ...:
    ...                                   # pump, step x spf, sync, update
    next_frame += (1.0/FRAME_HZ)
    slack = next_frame - perf_counter()
    if slack > 0: time.sleep(slack)       # ahead of schedule: wait
    else:         next_frame = perf_counter()   # behind: reset, do not spiral
```

This makes `--speed` mean what it says, makes the demo reproducible across machines, and —
importantly for Part 2 — is the same mechanism a live sensor stream will need to keep the
simulation locked to wall time rather than to whatever throughput the machine happens to
have.

## 3.7 If rendering is confirmed, the ranked fixes

1. **Hide fasteners and electronics from the visual group.** The screws, dowel pins,
   washers, bearings and the Raspberry Pi contribute the majority of the triangles and
   nothing to the demonstration. Assign them a separate geom group and default it off. No
   mesh files touched, no physics touched — a generator change in `tools/build_mjcf.py`.
2. **Decimate the remaining meshes.** CAD STL exports are typically 50–500× denser than a
   real-time renderer needs. Quadric decimation to a ~50k-triangle budget for the whole
   scene is standard and visually near-lossless at demo distances.
3. **Decouple render rate from step rate** (config D): `sync()` at 30 Hz while stepping
   continuously.
4. **Disable shadows and floor reflectance** in the viewer options — one flag each, roughly
   halves fragment work.
5. **Reduce window size**, or check display scaling.

The first two are the real fix and neither changes a single physics parameter — which
matters, because the model is protected by a sha256 assertion in the tests and any change to
it must be deliberate and re-validated.

---

# Part 4 — Research direction

## 4.0 A warning about the literature check, which must be read first

**The literature search could not be executed.** This environment has no network egress:
`WebSearch` is unavailable to the model, `web_fetch` is restricted to a one-host allowlist,
and direct HTTP from the sandbox is blocked by the proxy. A delegated agent hit the same
wall on four independent routes.

Everything below about prior art is therefore **recall, not search**. Where I write
"apparently open" it means *"I have no recollection of this being done"*, which is a
materially weaker claim than "a search found nothing" and **is not evidence of novelty**.
Section §4.6 lists the exact searches to run, ordered by how much damage a wrong answer
does. **Nothing in this section should be said to the professor as a novelty claim until
§4.6 items 1–4 are done.**

## 4.1 Critical assessment of the proposed umbrella framing

> *"A calibrated, sensor-driven digital twin of the OSL V2 for real-time human gait
> interaction."*

I would not use this, for four reasons.

**"Digital twin" is doing too much work and invites the wrong question.** The term properly
denotes a model calibrated against, and synchronised with, a *specific physical instance*.
Right now the model is calibrated against nothing — three of its dynamic parameters are
placeholders and its actuator is ideal. Using the term before Phase 3 promises something the
artifact does not deliver, and in robotics venues it has been diluted to the point of being
a mild red flag. More practically: an OSL *does* already exist in MuJoCo, shipped inside
MyoAssist's `assist_sim` as `OpenSourceLeg_KA_L1` and `OpenSourceLeg_A_L1`, with model
resources contributed by Rouse's own group. **"I built the OSL in MuJoCo" is not available
as a contribution.** What is ours is *how* — per-link CAD inertia tensors from the Onshape
V2 export rather than a body-level parameter match — and that is a modelling-provenance
claim, not an existence claim.

**It bundles two independent contributions, so neither gets evaluated sharply.**
"Calibrated" (system identification) and "sensor-driven" (real-time interfacing) are
separate pieces of work with separate evidence. A framing that requires both to land makes
the project twice as likely to fail and half as easy to review.

**"Real-time human gait interaction" over-promises on a fixed base.** The bench is welded to
the world. There is no ground contact, no body weight, no socket load, no human coupling —
so there is no *interaction* in any sense a reviewer would accept. What we have is
*sensor-driven reference generation*, which is a real and useful thing and should be named
accurately. Also note "human-in-the-loop" is an established term of art (Zhang et al.,
*Science* 2017) meaning online optimisation on real hardware with a real person; using it
for "an IMU streams into my simulator" will be misread, and correcting a reviewer's
expectation is expensive. Prefer **"sensor-in-the-loop"** or **"live-sensor-driven"**.

**It names an artifact, not a question.** "A twin of X" can always be answered with "so
what?". A question cannot.

**The framing I would defend instead:**

> **How much actuator fidelity does a prosthetic-knee simulation need before its control
> results transfer to hardware? A bench-identified answer for the Open-Source Leg V2.**

This is a question; it has a falsifiable answer; it *uses* the calibration work rather than
claiming it as the result; it is grounded in the one asset nobody else in this comparison
has, which is our own physical bench; and — this is the strongest part — it is named as
future work **by the authors of the framework we would otherwise be duplicating**. The
MyoAssist 1.0 preprint states that its simplified rigid-body, direct-torque representation
is "a practical starting point … while allowing more detailed human-device interface models,
**actuator dynamics**, and device-specific controllers to be incorporated as the framework
matures." A gap the incumbent authors name in print is the most defensible gap available.

Part 1 makes this framing sharper than it would otherwise be: we now have a *quantitative*
demonstration that a control result from this simulation (RMS 4.36°) was 90 % an artifact of
an idealisation in the controller. The same question applied to the *plant* idealisations is
exactly the proposed contribution.

## 4.2 The five candidate directions, assessed

**(A) Sensor-driven real-time OSL digital twin.** *Verdict: excellent engineering, weak as a
standalone contribution.* The novelty rests on plumbing. The adjacent literature is
uncomfortably close — real-time musculoskeletal pipelines (`rtosim`, Pizzolato et al.; EMG-
driven real-time models, Durandau/Sartori) are mature, and **Sartori and Durandau are
co-authors on MyoAssist 1.0**, so this is territory that team can enter at will. *Keep it as
infrastructure (Phase 2/4), not as the claim.*

**(B) Bench-calibrated OSL twin via system identification.** *Verdict: solid, necessary,
insufficient alone.* The method is deliberately textbook and must be presented as such —
constant-velocity friction sweeps, ring-down for inertia, chirp for bandwidth, least squares
on the rigid-body-plus-friction regressor. The contribution is the **artifact and its
validation**: an identified parameter set for the OSL V2 in MJCF-ready form with reported
fit error, replacing placeholders that are currently 7–25× off (§1.6). It has an obvious
home, because `assist_sim` ships a direct-torque OSL with no transmission dynamics. *This is
Phase 3 and it is the enabler, not the paper.*

**(C) A + B combined.** *Verdict: this is the umbrella framing, and §4.1 applies.* Too broad.

**(D) Adaptive gait-reference generation by speed/terrain/condition.** *Verdict: do not
claim this.* Speed- and incline-parameterised lower-limb kinematics is well covered (Embry,
Villarreal, Macaluso & Gregg, *IEEE TNSRE* 2018; Reznick et al., *Scientific Data* 2021 is a
dataset built precisely for it; Best, Welker, Rouse & Gregg, *Science Robotics* 2023 does
continuous speed and incline on hardware with amputee subjects). Re-implement it as a
component if needed; never present it as a finding.

**(E) Sim-to-real validation quantifying how each actuator idealisation changes
performance.** *Verdict: the strongest of the five, and my recommendation.* The structural
template exists in legged robotics — Tan et al., *RSS* 2018 ablate actuator modelling and
latency; Hwangbo et al., *Science Robotics* 2019 learn the actuator instead — but I have no
recollection of it being done for a prosthesis with an identified rather than randomised
parameter set. Two reasons to believe the gap is real rather than a hole in my memory:
MyoAssist's authors name it themselves (§4.1), and the composed MuJoCo prosthesis
environment that makes it cheap is only months old.

## 4.3 Direction E, specified

**Research question.** For a powered knee prosthesis in simulation, which actuator and
transmission idealisations — reflected rotor inertia, Coulomb and viscous friction, torque
bandwidth, transport latency, torque–speed limits, transmission compliance — materially
change the control performance a simulation predicts, and by how much? Equivalently: what is
the minimum-fidelity actuator model whose control results still transfer?

**Why it is not repeating MyoAssist.** MyoAssist composes human + device and optimises
controllers *given* a device model. This asks what the device model has to contain. It sits
beside the framework and its output — an identified, ablation-justified actuator model —
would be a contribution *to* it, not a duplicate of it.

**Required inputs.** (1) An identified parameter set from our bench — Phase 3. (2) The
validated tracking pipeline — done. (3) One fixed controller, held constant across all
ablations. (4) The physical bench for the ground-truth arm.

**What is simulation-only.** Every ablation, the full factorial, and the sensitivity
analysis. Cheap and parallel.

**What needs the physical bench.** The identification itself, and the one measurement that
gives the whole thing teeth: running *the same controller* on the real bench and measuring
where each simulated variant lands relative to it. Without that arm, this is a sensitivity
study; with it, it is a sim-to-real result.

**Evaluation metrics.** Tracking RMS/peak error, peak and RMS torque, phase lag, torque
saturation fraction, and — the honest one — the *rank correlation* between simulated and
measured performance across a set of controllers. Fidelity matters if and only if it changes
which controller you would choose.

**Difficulty.** Medium. The simulation work is a well-understood loop; the risk is entirely
in bench access, instrumentation quality, and the identification converging.

**What counts as a meaningful contribution.** A defensible statement of the form: *"for
trajectory-tracking control of the OSL V2 knee, reflected inertia and transport latency
change predicted performance by X % and Y %, while torque–speed limits and transmission
compliance change it by under Z % — so a simulation intended for controller development must
model the first two and may omit the last two."* That is useful to everyone building on
MyoAssist, and it is falsifiable.

**Literature that must be checked first.** §4.6 items 1, 2, 4 and 5.

## 4.4 A second direction worth keeping, if a fallback is wanted

**Three-way divergence measurement.** Run one controller in three places — the ideal
simulated device, the identified simulated device, and the physical fixed-base OSL — and
report where and by how much the three diverge across the gait cycle. Narrower than (E),
lower risk, and it *requires* the physical bench, which is the scarce resource and therefore
the defensible one. It is also a natural first paper-shaped output of Phases 3 and 5.

## 4.5 What must not be claimed as novel

1. **"A MuJoCo model / digital twin of the Open-Source Leg."** Two already ship in
   MyoAssist's `assist_sim`, with model resources contributed by Rouse's group.
2. **Gait phase estimation from a single thigh IMU, or phase-variable prosthesis control.**
   A decade of work from Gregg, Rouse and Young, including a *Science Robotics* paper. Any
   estimator we build is a component we re-implement, and we should say so *before* anyone
   asks.
3. **Speed/terrain-adaptive reference generation** (§4.2 D), and **PD tracking of a recorded
   human knee trajectory on a bench** — the latter is a validation step, and Part 1 confirms
   it: the residual error is a property of the control law, not a finding about the leg.

## 4.6 The searches to run the moment network is available

Ordered by how much a wrong answer costs.

1. `MyoChallenge 2024 prosthesis locomotion Open-Source Leg` — if the OSL was a public RL
   benchmark track, a whole competition field has already exercised an OSL-in-MuJoCo and
   §4.1's "already done" hardens considerably.
2. Resolve the MyoAssist 1.0 preprint (bioRxiv `10.64898/2026.08.25.746839`) and the ICORR
   2025 paper (`10.1109/ICORR66766.2025.11063089`); extract the **Limitations / Future Work**
   section verbatim and in full. **This is the single highest-value artifact for the whole
   project.** Also check for any follow-up preprint adding actuator dynamics — that would
   close direction (E) outright.
3. `sim-to-real actuator fidelity ablation prosthesis exoskeleton` and `actuator dynamics
   fidelity musculoskeletal simulation assistive device` — the genuine novelty test for (E).
4. `Open-Source Leg v2 design paper Rouse 2024 2025` — find the correct hardware citation
   for the V2 geometry, or establish that none exists. Also `Azocar 2020 Nature Biomedical
   Engineering open-source bionic leg` and `Azocar BioRob 2018` for the benchtop
   characterisation methodology.
5. `system identification powered knee prosthesis actuator` and `Elery Rezazadeh Gregg
   powered knee-ankle prosthesis IEEE T-RO 2020` — the template for Phase 3.
6. Human knee quasi-stiffness values with a citation (`Shamaei Dollar knee quasi-stiffness
   stance walking`) — needed to make §1.2 and §1.11 quotable rather than recalled.
7. The `opensourceleg` library's impedance controller default gains and their units — the
   fastest route to §1.10 item 2, and it may be answerable from the documentation alone.

## 4.7 One correction to carry into any discussion of the dataset

Our own audit flagged four defects in the AB19 `human_knee_moment` / `human_knee_power`
columns: |moment| and |power| correlated at 0.9996, peak magnitude ~15× a plausible walking
value, peak on the very first sample, and monotone decay like a filter start-up transient.
Read together, those four point at **our extraction, not the dataset** — a ×1000 unit error
(Camargo et al. document these as mass-normalised, N·m/kg), one column derived from the
other, and a window-edge transient.

**Do not tell the professor that the Camargo dataset has defective kinetics.** The defensible
sentence is: *"our extraction of the kinetic columns is unverified and self-inconsistent, so
the experiment is angle-driven only"* — which is already true of the experiment, and is the
version that survives a professor who has used that dataset.

---

# Part 5 — Phased plan

## Phase 1 — Physically justified gains (no hardware required)

**Objective.** Replace "Kp = 600 because a sweep liked it" with "Kp is the impedance the
device is meant to render, and accuracy comes from the feedforward path."

**Modules.** `oslbench/controller.py` (add the velocity-reference and feedforward terms and
their MuJoCo installation), `oslbench/simulation.py` (pass `q̇_ref` through — it already has
it), `oslbench/model.py` (expose the plant constants the feedforward needs),
`tests/test_oslbench.py` (new tests for the new law), `experiments/verify_against_oracle.py`
(unchanged, used as the regression gate).

*Implementation note:* MuJoCo's `position` actuator cannot express `+Kd·q̇_ref` because
`ctrl` is scalar. Two clean options — (a) add a second `velocity` actuator on the same joint
with `kv = Kd` and `ctrl = q̇_ref`, so the pair sums to exactly `Kp(q_ref−q) + Kd(q̇_ref−q̇)`
while keeping everything inside MuJoCo; or (b) switch to a `motor` actuator and compute the
torque in Python, which also carries the feedforward naturally. **(a) is the smaller change;
(b) is the more flexible.** This is a physics-adjacent decision and should be reported before
it is made, per the standing process rule.

**Experiment.** (i) Re-run `verify_against_oracle.py` with the *current* gains and the
*current* law — must still pass bit-identically, proving the refactor changed nothing.
(ii) Re-run the §1.8 table in real MuJoCo, replacing every `[PREDICTED]` with a measurement.
(iii) Sensitivity sweep over `armature ∈ [0.01, 0.25]` to quantify how much the conclusions
depend on the placeholder.

**Measurable result.** A table showing that velocity tracking at Kp ≈ 60 N·m/rad
(1.05 N·m/deg) matches or beats the current Kp = 600 result, with unchanged peak torque and
zero saturation — measured, not predicted.

**Hardware dependency.** None. This is the phase to do first *because* it needs nothing.

**Show the professor.** The §1.8 comparison with real MuJoCo numbers, and the §1.6 provenance
table. The headline sentence: *"the 4.36° error was one missing term; correcting it lets the
joint stiffness drop by a factor of ten into the physiological range at no cost in accuracy."*

## Phase 2 — Real-time data interface (no hardware required)

**Objective.** Make the simulation consume an externally supplied gait phase in real time,
without changing a single number.

**Modules.** `oslbench/gait/reference_model.py` (new — `q_ref(φ)`, `q̇_ref(φ, φ̇)` from the
existing spline), `oslbench/sensing/source.py` (new — the source protocol plus CSV replay
and synthetic streaming), `oslbench/viewer.py` (add the wall-clock pacing of §3.6),
`experiments/run_live_stream.py` (new, thin).

**Experiment.** MVP-0 then MVP-1 (§2.4). Stream synthetic thigh IMU at 100 Hz over UDP from a
separate process, with injected noise, bias, dropouts and delay; measure phase-estimation
error against the known truth; measure end-to-end latency.

**Measurable result.** (i) `verify_against_oracle.py` still passes bit-identically after
MVP-0 — the proof that the interface changed no physics. (ii) A latency budget with measured
numbers for each stage. (iii) Phase-estimation RMS error in % gait cycle against ground
truth, as a function of injected sensor noise and dropout rate.

**Hardware dependency.** None for MVP-0/MVP-1. MVP-2 needs one recording; MVP-3 needs an IMU.

**Show the professor.** The latency budget and the phase-error-vs-noise curve. These are the
numbers that decide whether a live demo is honest, and they exist before any hardware is
bought.

## Phase 3 — Calibrated actuator model (needs the bench)

**Objective.** Replace `armature = 0.010`, `damping = 0.30`, `frictionloss = 0.40` with
measured values, and add whatever actuator dynamics the measurements show to matter.

**Modules.** `tools/build_mjcf.py` (the generator, so the model stays generated and never
hand-edited), `models/osl_v2_bench.xml` (regenerated — note its sha256 is asserted in the
tests and must be deliberately updated), `oslbench/model.py` (new expectations),
`experiments/identify_actuator.py` (new).

**Experiment.** On the bench, with the output free or locked as appropriate: free-swing
ring-down for inertia (motor open- vs short-circuit separates mechanical from electrical
damping); constant-velocity sweeps at several speeds for `f_c` and `b` by least squares;
step and chirp on torque command for bandwidth and transport delay; a torque–speed sweep if
time allows. Report fit error for every parameter — an identification without a residual is
not a measurement.

**Measurable result.** A parameter table with values, units, method, fit error and
confidence intervals; and the delta it makes to `ω_n`, `ζ` and the Phase 1 tracking result.

**Hardware dependency.** Total. This phase *is* the bench.

**Show the professor.** The before/after parameter table, and the sentence that follows from
§1.6: *"the armature placeholder was off by a factor of N, which moved the closed-loop
natural frequency from 7.6 Hz to F Hz."* That is the moment the model stops being a drawing
and becomes a measurement.

## Phase 4 — IMU-driven gait phase and reference (needs one IMU)

**Objective.** Close the loop from a real thigh IMU on a real person to the simulated knee.

**Modules.** `oslbench/sensing/orientation.py`, `oslbench/gait/phase.py` (both new),
`oslbench/gait/reference_model.py` (extended with a speed/cadence parameter).

**Experiment.** MVP-2 then MVP-3 (§2.4). Walk at several cadences; measure phase error against
a foot-switch ground truth; measure how tracking degrades relative to the Phase 1 offline
result; measure recovery from deliberate dropouts and stops.

**Measurable result.** Phase RMS error in % gait cycle, live; end-to-end latency, measured;
and the degradation in knee tracking attributable to each of estimation error, sensor noise
and latency, separated.

**Hardware dependency.** One thigh IMU, wired or low-latency wireless (§2.5 — BLE is probably
disqualifying). Optionally a foot switch, for validation only.

**Show the professor.** The live demo, with the honest caption: the phase is *estimated*, the
error is *measured*, and the bench is still a bench — there is no load, no ground and no
interaction.

## Phase 5 — Integration and the actual research result (needs the bench)

**Objective.** Run direction (E): the actuator-fidelity ablation, with the physical bench as
ground truth.

**Modules.** `experiments/ablate_fidelity.py` (new), plus a controller held fixed across all
variants.

**Experiment.** A factorial over the idealisations — reflected inertia (placeholder vs
identified), friction (none / viscous / Coulomb+viscous), torque bandwidth (ideal vs
identified first-order), transport latency (0 vs measured), torque–speed limit (flat vs
enveloped) — evaluated on a common controller set, and compared against the same controllers
run on the physical bench.

**Measurable result.** A sensitivity ranking of the idealisations, and the rank correlation
between simulated and measured controller performance under each fidelity level. The
publishable sentence is the one in §4.3.

**Hardware dependency.** Total, for the ground-truth arm.

**Show the professor.** A single figure: predicted-vs-measured performance for each fidelity
level, with the fidelity threshold at which the ranking stops changing marked on it.

## Ordering and dependencies

```
Phase 1 (gains)  ──►  Phase 2 (interface)  ──►  Phase 4 (live IMU)
      │                      │                        │
      └──────────────────────┴────────► Phase 5 (ablation, the result)
                    ▲
Phase 3 (identification, needs bench) ─┘
```

Phase 1 is the gate: it is cheap, needs nothing, fixes the thing the professor actually
objected to, and makes every later number interpretable. Phase 3 can run in parallel with
Phase 2 the moment bench time is available. Phase 5 needs both 1 and 3, and is the only
phase that produces a research claim.

**Before Phase 5 is described to anyone as novel, §4.6 items 1–4 must be done.**

---

## Appendix — one stale line to fix later

`docs/VALIDATION.md` §"What still has to be run" states that the oracle comparison has not
been executed against real MuJoCo. That is now out of date — it has been run and it passed.
That section needs a one-line correction, deliberately left unmade here because this task was
read-only.
