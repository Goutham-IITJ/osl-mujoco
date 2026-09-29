# OSL V2 Drivetrain Parameters — provenance, frames, and confidence

**Scope.** This file is the authority on *where every number in `oslbench/drivetrain.py`
came from* and *which shaft it lives on*. It is not a summary of the model; for that,
read the module docstring. It is not a set of measurements of our hardware either — we
have no OSL V2 actuator on a dynamometer, and nothing below is a measurement we made.

**Primary source for every PAPER-DERIVED row.**

> T. K. Best, G. C. Thomas, S. R. Ayyappan, R. D. Gregg, E. J. Rouse, "A Compensated
> Open-Loop Impedance Controller Evaluated on the Second-Generation Open-Source Leg
> Prosthesis," *IEEE/ASME Transactions on Mechatronics*, vol. 30, no. 6, pp. 4732–4743,
> Dec. 2025. DOI 10.1109/TMECH.2024.3508469. Section III-A, equations (1)–(5), Fig. 3.

The actuator model form (1)–(2) is credited by Best et al. to Nesler et al. (their refs
[27], [44]); the *values* are the ones Best et al. regressed for the OSL V2.

## The four labels, used strictly

| Label | Means |
|---|---|
| **MEASURED (paper)** | Best et al. measured it on OSL V2 hardware and report the fitted value. Not measured by us. |
| **PAPER-DERIVED** | Stated by the paper as a design fact (e.g. a catalogue gear ratio), not a regression output. |
| **CAD-DERIVED** | Read off our own Onshape export of the OSL V2. Ours, and independent of the paper. |
| **DERIVED (ours)** | Arithmetic we performed on the rows above. Traceable, but it inherits their uncertainty. |
| **ASSUMED** | Nobody measured it and no source states it. |

Nothing in this document is labelled "experimentally verified by us," because nothing is.

## Parameter table

| Parameter | Symbol | Value | Units | Physical meaning | Source | Coordinate / frame | Confidence / status |
|---|---|---|---|---|---|---|---|
| Planetary reduction | `n_a` | 9 | – | Integrated planetary gearbox inside the Dephy ActPack 4.1 (T-motor AK80-9 based) | PAPER-DERIVED — Sec. II, "integrated 9:1 planetary gear reduction" | rotor → actuator output | High. A catalogue integer, not a fit. |
| Belt reduction (paper) | `n_t` | 4.61 | – | Single-stage Gates Powergrip GT3 belt drive, 45 mm width | PAPER-DERIVED — Sec. II, "belt transmission with an 4.61:1 reduction" | actuator output → joint | High for *their* build. See the open decision below. |
| Belt reduction (our CAD) | — | 50/11 = 4.5455 | – | Pulley tooth count in our own Onshape export | CAD-DERIVED — our export | actuator output → joint | High for *our* CAD. Differs from the paper by 1.4 %. Not substituted. |
| Torque constant | `k_t` | 110.8 × 10⁻³ | N·m/A | Rotor torque per q-axis amp | MEASURED (paper) — Sec. III-A1 regression | **ROTOR** (electrical/rotor side) | High. VAF 99.7 % for the actuator fit. |
| Actuator inertia | `J_a` | 9.83 × 10⁻³ | kg·m² | "the combined effects of rotor and gearbox inertial torques" — the n_a² rotor reflection is **already inside it** | MEASURED (paper) — Sec. III-A1, sinusoidal-velocity experiment | **ACTUATOR OUTPUT** | High as published. **Not** joint inertia; **not** rotor inertia. |
| Actuator damping | `B_a` | 6.06 × 10⁻² | N·m·s/rad | Viscous loss vs. actuator velocity | MEASURED (paper) — Sec. III-A1 | **ACTUATOR OUTPUT** | High as published. |
| Coulomb friction | `f_c` | 17.1 × 10⁻² | N·m | Velocity-sign-dependent, current-independent friction | MEASURED (paper) — Sec. III-A1 | **ACTUATOR OUTPUT** | High as published. |
| Gear friction coefficient | `f_g` | 82.1 × 10⁻³ | N·m/A | Friction that grows with \|I_q\| — load-dependent gear losses | MEASURED (paper) — Sec. III-A1 | **mixed**: rotor amps → actuator-output N·m | High as published. Do not rescale by n_a. |
| Belt linear term | `p1` | 876 | N·m/rad | Belt stiffness at zero deflection; `K_s(0) = p1` | MEASURED (paper) — Fig. 3 quadratic fit, R² = 0.997, ten trials | **JOINT** | Fitted on the **ankle** only, on an unstated belt pitch. See caveats. |
| Belt quadratic term | `p2` | 14913 | N·m/rad² | Stiffening rate: `dK_s/d\|θ_s\| = 2·p2` | MEASURED (paper) — Fig. 3 quadratic fit | **JOINT** | Same caveats as `p1`. |

### Derived quantities (arithmetic on the table above)

| Quantity | Expression | Value | Units | Frame | Note |
|---|---|---|---|---|---|
| Total nominal ratio (paper belt) | `n_a·n_t` | 41.49 | – | rotor → joint | **The OSL V2 total mechanical ratio.** Not 49.4. |
| Total nominal ratio (our CAD belt) | `9 · 50/11` | 40.909 | – | rotor → joint | 1.4 % below the paper's. |
| Actuator-output torque constant | `k_t·n_a` | 0.9972 | N·m/A | actuator output | What one amp buys at the belt input. |
| Joint torque constant | `k_t·n_a·n_t` | 4.5971 | N·m/A | joint | **Rigid, quasi-static limit only.** |
| Implied rotor inertia | `J_a/n_a²` | 1.2136 × 10⁻⁴ | kg·m² | rotor | DERIVED (ours). The paper never reports a rotor inertia separately. |
| Belt stiffness at zero load | `K_s(0) = p1` | 876 | N·m/rad | joint | Exact, by construction of (5). |
| Belt stiffness at the bench peak | `√(p1² + 4·p2·\|τ_j\|)` | 1312.2705 | N·m/rad | joint | At τ_j = 16.004120587 N·m, our bench's measured peak. 1.498× the zero-load value. |
| Deflection at the bench peak | `ρ⁻¹(16.004120587)` | −0.01462719 | rad (−0.8381°) | joint | Sign is negative by (4); see the module docstring. |

### Rigid-limit reflections — *comparison values, not the paper's model*

These are what you get if you collapse the whole compliant drivetrain into one rigid
joint-side approximation. The paper never forms these products. They exist so the gap
between the rigid approximation and the paper's model is quantifiable.

| Quantity | Expression | Paper belt (4.61) | Our CAD belt (50/11) | Units |
|---|---|---|---|---|
| Joint-side inertia | `n_t²·J_a` | 0.208908 | 0.203099 | kg·m² |
| Joint-side damping | `n_t²·B_a` | 1.287877 | 1.252066 | N·m·s/rad |
| Joint-side Coulomb friction | `n_t·f_c` | 0.788310 | 0.777273 | N·m |

`n_t²` for inertia and damping, `n_t` for friction: an impedance picks up the ratio
twice (the torque scales by `n_t`, the velocity it multiplies scales by `1/n_t`), a
torque only once.

## THE OPEN DECISION: which belt ratio should an integrated model use?

Both values are recorded and **neither has been substituted for the other**:

- **paper belt ratio = 4.61** (Best et al., Sec. II) — the ratio every one of their
  fitted parameters was identified against.
- **our CAD belt ratio = 50/11 = 4.5455** — the ratio our own Onshape export actually
  has.

They differ by 1.4 %, and squared (for inertia and damping) by 2.9 %. The choice is
**deliberately left open** for the future integrated model, for a reason worth stating:
`J_a`, `B_a`, `f_c`, `f_g`, `p1` and `p2` were all fitted *with 4.61 in the loop*, so
using our 4.5455 with their parameters mixes two builds. Using 4.61 with our CAD
geometry mixes them the other way. Neither is obviously right, the difference is small
enough that no present conclusion turns on it, and pretending the question is settled
would be worse than carrying it. `oslbench/drivetrain.py` defaults to the paper's 4.61
and exposes `CAD_BELT_RATIO` plus `with_belt_ratio()` so that any swap is explicit at
the call site.

## A number that is NOT a ratio: 49.4 / 58.4

`49.4` and `58.4` appear in the MyoAssist `OpenSourceLeg_KA_L1` model as gear/gain
constants. They are **not** OSL V2 mechanical reductions and never were. The physical
OSL V2 total ratio is **41.49** (paper) or **40.909** (our CAD). Do not label either
of them 49.4.

Related, and required wording for anything said about our own model's torque limits:
the model represents the available joint torque authority derived from the motor/gear
relationship, but idealizes the torque production. The figures 49.4, 58.4, 30 A and
0.096 N·m/A are **not** experimentally verified hardware values.

## Caveats attached to the paper's own numbers

These are the paper's limitations, not ours, and they travel with the parameters:

1. **The belt was characterised on the ankle, not the knee.** The paper: "while our
   experiments focus on the ankle joint, the same characterization and control approach
   can be applied to the knee joint, as they are mechanically identical." So `p1` and
   `p2` reach our *knee* bench on the paper's assertion of identical construction.
   Reasonable — but it is a transfer across joints, not a knee measurement.

2. **The belt pitch behind `p1`/`p2` is not stated.** The design "recommends 3 mm pitch
   belts, but includes an option to be assembled with 5 mm pitch belts; these belts are
   stiffer but have greater output impedance." The paper never says which pitch the
   characterised unit had. Our CAD export is unambiguously the 5 mm variant, so if the
   fit was done on 3 mm, then **`p1 = 876` is a lower bound for our hardware.**
   Unresolved, and it cannot be resolved from the paper.

3. **The fit is exercised to ~0.055 rad.** Fig. 3's abscissa runs to about 0.055 rad
   (~93 N·m), though the text describes driving the joint to 0.167 rad. Beyond ~0.055
   rad, evaluating `ρ` is extrapolating their regression.

4. **Backlash is real and deliberately excluded.** Verbatim: "there appears to be a
   minor backlash behavior around zero deflection, but choose to neglect it for model
   simplicity." No backlash model is implemented, because inventing one would not be
   the paper's model.

5. **Equation (21) has an apparent typesetting error.** It is printed as
   `I_q = [K_d(θ_eq − θ_j) − B_d·θ̇_j] / [k_t(n_a n_t)²]`, which carries an extra factor
   of `n_a·n_t` relative to the dimensionally consistent `k_t·n_a·n_t`. Verified in both
   the flowed and layout text extractions. **Not load-bearing here:** (21) is the
   *baseline* controller C0, which is not implemented in this module.

6. **`f_g` has mixed units by construction.** It maps a rotor-side current in amps to
   an actuator-output torque in N·m. It is not a per-rotor-torque coefficient and must
   not be scaled by `n_a`.

## Six things worth being explicit about

### 1. Why the existing `armature` placeholder is not the full actuator model

`models/osl_v2_bench.xml` authors `armature = 0.01`, `damping = 0.3`,
`frictionloss = 0.4` at the knee. Those three numbers are PLACEHOLDERS — they were not
derived from the actuator. More importantly, even with perfect values they are the
wrong *shape*: MuJoCo's `armature` is a constant added to the joint's diagonal inertia,
`damping` a constant linear coefficient, and `frictionloss` a constant Coulomb
magnitude. The paper's actuator has an inertia and a damping **on the other side of a
spring**, plus a friction term **proportional to current**. The vocabulary coincidence
(three parameters with three matching names) makes the bench look closer to the paper
than it is; that coincidence is the single easiest thing to over-credit here.

### 2. Why `J_a` cannot simply be multiplied by the total 41.49 ratio

Two separate errors would be committed at once.

First, `n_a` is **already inside `J_a`**. The paper's own words: `J_a` "captures the
combined effects of rotor and gearbox inertial torques." It was identified by a
measurement at the actuator output, with the gearbox attached. Multiplying by `n_a²`
reflects the rotor across the planetary a second time — a factor of 81 of
double-counting.

Second, the correct reflection to the joint side is `n_t²`, not `(n_a n_t)²`, giving
`4.61² × 9.83e-3 = 0.208908` kg·m². This is visible directly in the paper's own (7),
`τ_j = n_t(τ_a^des − B_a θ̇_a − J_a θ̈_a)`: the factor in front of the inertial and
viscous terms is `n_t`, and only `n_t`.

And even 0.208908 is only the **rigid-limit** equivalent. It is a useful comparison
number. It is not the paper's model, because it presumes the belt is stiff.

### 3. Why belt compliance requires an additional state

A rigid drivetrain has **one** mechanical degree of freedom: fix `θ_j` and
`θ_a = n_t·θ_j` follows. A compliant drivetrain has **two**, because the belt can
stretch — `θ_a` and `θ_j` evolve independently, and their mismatch *is* the deflection
`θ_s = θ_j − θ_a/n_t`. There is no way to recover `θ_s` from `θ_j` alone; it is genuine
extra state carrying the energy stored in the belt.

This is exactly why MuJoCo integration is not a parameter edit. Our bench is `nq = 2`
(knee + ankle hinges). Representing the paper's drivetrain on the knee explicitly needs
one more coordinate for the actuator shaft, plus a nonlinear spring coupling it to the
joint. And MuJoCo's joint `stiffness` is **linear**, so `ρ(θ_s) = p2 θ_s² + p1 θ_s`
cannot be authored in XML at all — it has to be applied as an external torque, or
approximated by its local linearisation `K_s`.

### 4. Why current-dependent friction cannot be one constant `frictionloss`

The paper's friction is `τ_f = sgn(θ̇_a)(f_c + f_g|I_q|)`. MuJoCo's `frictionloss` is a
single constant. It can represent `f_c`, and it cannot represent `f_g|I_q|` at all.

This is not a rounding-error omission. At our bench's measured peak knee torque of
16.004 N·m, working backwards through the rigid chain gives `τ_a = 3.472` N·m and hence
`I_q = (τ_a + f_c)/(k_t n_a − f_g) = 3.98` A. Reflected to the joint:

| Friction component | Value at τ_j = 16.00 N·m | Representable as MuJoCo `frictionloss`? |
|---|---|---|
| constant, `n_t·f_c` | 0.789 N·m | yes |
| current-dependent, `n_t·f_g·\|I_q\|` | 1.507 N·m | **no** |
| total | 2.295 N·m | no |

**The part MuJoCo cannot express is the larger part** — roughly twice the part it can.
A single constant chosen to match at one operating point will be wrong everywhere else,
and wrong in a load-dependent way.

### 5. What the standalone drivetrain model represents

- The actuator's torque production from q-axis current, through the planetary, to the
  actuator output shaft: paper (1), (2).
- Velocity-dependent Coulomb friction **and** current-dependent gear friction, with the
  paper's sign convention.
- Actuator-output viscous damping and actuator-output inertia, on the correct side of
  the transmission.
- The belt's nonlinear, stiffening torque–deflection law over the full domain
  `θ_s ∈ ℝ`: paper (4), and its local stiffness (5).
- The kinematic three-shaft relationship `θ_j = θ_a/n_t + θ_s` and its inverse: (3),
  (11), (12).
- The two-sided torque identity — `τ_j = n_t τ_a` from the actuator side and
  `τ_j = −sgn(θ_s)ρ(|θ_s|)` from the belt side — as the constraint that closes the
  system.
- The friction-compensating current (6), included purely because it is the exact
  algebraic inverse of (1)–(2) and therefore the sharpest test of that algebra.

### 6. What it still does NOT represent

- **Electrical dynamics.** No winding inductance/resistance, no back-EMF, no voltage
  limit, no current-loop dynamics. The paper omits these too, and says why: the current
  loop runs at 10 kHz, far above the mechanical bandwidth.
- **Thermal behaviour.** No winding temperature, so no thermal derating of `k_t` or of
  continuous torque.
- **Torque ripple, cogging, and commutation effects.**
- **Backlash.** Observed by the paper, deliberately excluded (see caveat 4).
- **Belt inertia and belt damping.** The belt is a massless, purely elastic spring here;
  no hysteresis loss, no creep, no tension-dependence, and no dependence on the
  tensioner setting.
- **Load dependence of the transmission ratio.** `n_t` is a constant; tooth-engagement
  and pulley-eccentricity effects are absent.
- **Any controller.** The compensator (8), (16), (17), (20) is *not* implemented — only
  the plant, plus (6).
- **Joint or foot inertia and any environment.** The paper's `J_f` and `τ_grf` (their
  Fig. 4) are outside this module; on our bench those roles are played by the
  CAD-derived MuJoCo bodies.
- **Hard stops, joint range limits, and the actuator's torque/current saturation.**
- **Anything about our own hardware.** We have not measured an OSL V2 actuator. Every
  number here is Best et al.'s, used on their authority.

## Files

| File | Role |
|---|---|
| `oslbench/drivetrain.py` | the equations, MuJoCo-free and numpy-free |
| `tests/test_drivetrain.py` | the unit tests, runnable in a bare interpreter |
| `experiments/check_drivetrain_model.py` | numerical sanity checks + the three-way comparison |
| `docs/DRIVETRAIN_INTEGRATION_OPTIONS.md` | the four MuJoCo-integration options, scored; the belt-mode frequency table; the recommendation |
| `docs/ACTUATOR_DYNAMICS_ANALYSIS.md` | the repository-side mapping and provenance record this builds on |
