# Why Case B diverged — diagnosis of the Option-C drivetrain instability

**Status: this was a DIAGNOSIS-ONLY document, and it has since been acted on.** As
written, nothing here was fixed; the collocation change it recommends in section 11 has
since been made in `oslbench/drivetrain_sim.py`. `Kp = 600` and `Kd = 17.253` are
unchanged and have **not** been retuned. Sections 1 and the appendix describe the code
**as it stood when the instability was diagnosed** — that is deliberate, because the
diagnosis is only legible against the code that produced the failure. Where a section
describes something that has since changed, it says so inline (see section 4).

What this work added: `experiments/diagnose_drivetrain_instability.py`, this document, and
the CSV/Markdown output under `build/drivetrain_instability/`. Nothing else was written.
`oslbench/controller.py` and `models/osl_v2_bench.xml` are tracked in git and
`git diff --ignore-cr-at-eol` is empty for both — those two are *attested* unchanged.
`oslbench/drivetrain.py`, `oslbench/drivetrain_sim.py`,
`experiments/run_bench_ab19_drivetrain.py` and `tests/test_drivetrain.py` are still
untracked (`??`), so git cannot attest to them; their unchanged state rests on mtimes,
all of which predate this work (2026-09-27 and 2026-10-03). `tests/oracle/*` shows a
whole-file diff that is **CR/LF only** — zero content change — and the AB19 reference CSV
is clean. One further file, `tests/stub_mujoco.py`, carries a +9-line working-tree change
(it gained a `qfrc_applied` array so the stub reproduces MuJoCo's raw-dof-force path);
that change is dated 2026-10-03_17:17, i.e. it belongs to the Option-C integration, not to
this diagnosis, and `stub_mujoco` is a test double, not a production file.

**This is a simulation result. It is not a hardware claim.** Nothing below says the
physical Open-Source Leg is unstable, would be unstable, or has ever been run with this
controller. Every statement is about the behaviour of *our* model of *their* drivetrain
under *our* bench controller.

The failure being diagnosed, as reported from the real-MuJoCo run:

| | Case A (position servo, drivetrain OFF) | Case B (drivetrain ON) |
|---|---|---|
| RMS tracking error | 4.3616 deg | — |
| peak joint torque | 16.0041 N·m | — |
| outcome | completes the gait cycle | **diverges at t = 0.6560 s** |
| engine complaint | none | NaN / Inf / huge QACC at DOF 0, then the belt-energy term overflows |

That blow-up time is not only a verbal report. `MUJOCO_LOG.TXT` in the repository root
holds MuJoCo's own line from that run, and it is the only real-engine artefact this
document rests on:

```
WARNING Sun Oct  4 07:55:30 2026: Nan, Inf or huge value in QACC at DOF 0.
The simulation is unstable. Time = 0.6560.
```

---

## 1. The cause

**The derivative term of the knee controller is non-collocated with the actuator it
drives, and across a compliant belt that term injects energy instead of removing it.**

Concretely: `PDCurrentSource` computed, **at the time of the diagnosis** (the code at
that line now reads the actuator side — see section 4):

```
tau_req = Kp*(clip(theta_ref) - theta_j) - Kd*theta_j_dot   # JOINT position AND JOINT velocity
I_q     = tau_req / k_t_joint
```

(the `clip` is `PDController.command`, `oslbench/controller.py:95-98`, which clips the
reference into `ctrlrange`; it is irrelevant to the stability argument but the law is not
quite the textbook one and the difference should not be hidden.)

and that current is applied to the **actuator shaft**, on the far side of the belt. The
joint never receives the motor torque directly; it only ever feels `tau_j`, the belt
torque. So `-Kd*theta_j_dot` is a force applied at one body, proportional to the velocity
of a *different* body, with a spring between them.

In the belt mode the two inertias swing in **antiphase against the spring**. In that mode
`theta_j_dot` and `theta_a_dot/n_t` have opposite signs, so a torque proportional to
`-theta_j_dot` applied on the actuator points *along* the actuator's own motion. It does
positive work on the mode every cycle. The mode grows; the belt hardens as it grows,
which raises the mode frequency but does not stop the growth; `|theta_s|` leaves the
range Best et al. actually fitted; the torque the belt law then reports is extrapolation;
and eventually MuJoCo's integrator is handed an acceleration it cannot represent and
raises `mjWARN_BADQACC` at DOF 0 — the knee.

**The divergence is therefore downstream of a feedback-architecture defect, not the
defect itself.** The NaN is the last symptom, not the cause. Section 7 sets out the
energy/power test that would make the instability measurable long before any number
overflows, and the real-MuJoCo run in section 7 shows it did.

This is not a surprise in hindsight, and that is worth saying plainly:
`docs/ACTUATOR_DYNAMICS_ANALYSIS.md:558-563` already recorded that Best et al. feed back
the **actuator's** states, and that using joint states instead is the non-collocated
configuration the paper avoids — adding that this was "invisible in our model, because
our model has no series elasticity for it to be non-collocated across."
`docs/DRIVETRAIN_INTEGRATION_OPTIONS.md` scored *"actuator state available to the
controller"* as a selection criterion for both Option B (line 120) and Option C (line
140). To be accurate about our own record: it was **not** among the four reasons actually
given for choosing C (lines 162-180 — protected files untouched, `nq = 2` keeps the oracle
alive, B's advantage evaporates, reversibility). So the capability was recognised and
scored; it was simply never used. The belt is now that series elasticity, and the
controller was never rewired to the actuator side. The instability is the bill for that.

### Why `Kd` and not `Kp`

`Kp` acts on `theta_j`, a *position*, and a position-proportional torque across a spring
is a stiffness, not a power source — it can detune the mode but it cannot pump it.
`Kd` acts on a *velocity*, and velocity feedback is exactly the term that sets whether
energy flows in or out. The analytic model in section 2 and the controlled comparison in
section 4 both isolate `Kd`.

---

## 2. Analytic prediction — the engine-independent evidence

This is a linear 2-mass model of the same architecture, in `closed_loop_A()` in the
diagnostic script. It involves **no simulation engine at all**, so it is valid evidence
regardless of which engine produced the tables further down, and it was computed and
written down *before* the run tables were read.

State `x = [theta_j, theta_ar, theta_j_dot, theta_ar_dot]` with `theta_ar = theta_a/n_t`,
linearised about `theta_s = 0`:

```
joint       I_j  * theta_j_ddot  = K_s*(theta_ar - theta_j) - b_j*theta_j_dot
actuator    J_ar * theta_ar_ddot = tau_req - B_ar*theta_ar_dot - K_s*(theta_ar - theta_j)
```

with `I_j = 0.261998` kg·m² (incl. armature 0.01), `b_j = 0.3` N·m·s/rad,
`J_ar = n_t²·J_a = 0.208908143` kg·m², `B_ar = n_t²·B_a = 1.287877260` N·m·s/rad. The
motor torque enters the **actuator row only** — that is Option C's defining property and
the reason non-collocation is possible at all.

Largest real part of the closed-loop eigenvalues at `Kp = 600`:

| `K_s` (N·m/rad) | belt mode | `Kd`=0 | `Kd`=1 | `Kd`=5 | `Kd`=17.253 | **`Kd` boundary** |
|---|---|---|---|---|---|---|
| 876 (zero deflection) | 13.82 Hz | −1.5808 | −0.2146 | +6.3096 | **+18.1718** | **1.1170** |
| 1312.27 (the 16.0 N·m bench load) | 16.91 Hz | −1.6376 | −0.5849 | +4.9205 | **+17.7623** | **1.4105** |
| 2516.43 (the 0.055 rad fit edge) | 23.42 Hz | −1.6662 | −0.7747 | +4.0234 | **+17.1842** | **1.6394** |

Same table with the derivative term read from the **actuator** instead:

| `K_s` | `Kd`=0 | `Kd`=1 | `Kd`=5 | `Kd`=17.253 | `Kd` boundary |
|---|---|---|---|---|---|
| 876 | −1.5808 | −2.5502 | −6.6484 | −9.8917 | none in [0, 200] |
| 1312.27 | −1.6376 | −2.6581 | −6.8701 | −19.8400 | none in [0, 200] |
| 2516.43 | −1.6662 | −2.7114 | −6.9522 | −21.8532 | none in [0, 200] |

Three things fall out of this:

**`Kd = 17.253` is roughly 12× the stability boundary of the architecture it is being
used in.** At the bench's own operating stiffness the boundary is `Kd < 1.4105`;
production is 12.2× that. At zero deflection it is 15.4×.

**The same `Kd` is unconditionally stabilising when read from the actuator.** Not merely
tolerable — monotonically better, with no boundary found anywhere in `[0, 200]`. The sign
of the effect flips with which side of the belt the velocity is measured on. That is the
signature of a collocation problem and of nothing else.

**`Kp` is close to innocent at its current value.** At `Kd = 17.253` and
`K_s = 1312.27`: `Kp = 0` gives +14.6114, `Kp = 60` gives +14.8753, `Kp = 600` gives
+17.7623. Removing `Kp` entirely leaves 82% of the growth rate. (At `Kp = 6000` even the
collocated variant goes unstable at +39.1564, so `Kp` is not *arbitrarily* innocent —
just innocent here.)

A caveat that belongs with the numbers, not in a footnote: this linearisation drops
Coulomb friction (`f_c`, `f_g`, and the knee dof's `frictionloss = 0.4`), gravity, the
belt's hardening, the knee/ankle inertial coupling and MuJoCo's `implicitfast` velocity
term. It locates a boundary; it does not replace a run. One thing it does **not** drop is
the ±142.2 N·m `forcerange` clamp, because the Option-C path does not have one either —
`PDCurrentSource` calls `torque_unclamped`, and `qfrc_applied` bypasses `forcerange`
altogether (appendix items 2 and 6). The linear model and the run agree in having no
torque limit; ±142.2 N·m enters this document only as a *reference line* for judging
whether a number is physically sane.

---

## 3. `Kd` stability table

Swept at `Kp = 600`, variant J (production arithmetic, verified bit-identical to
`PDCurrentSource` in section 0 of the script: 8625 triples, worst difference
0.000e+00 A). Runs abort on the first of `|tau_j| > 1422` N·m (10× knee authority),
`|theta_s| > 0.2808` rad, `|theta_j_dot| > 50` rad/s, `|I_q| > 400` A, a non-finite
value, or a new MuJoCo `BADQACC/BADQPOS/BADQVEL/BADCTRL` warning. One honest limit on
that wording: the guard is checked *after* `sim.step()` returns, so a first offending
sample is **recorded and then aborted on**, not prevented. The other case — an
`OverflowError` raised *inside* `belt_potential_energy` mid-step, which is the failure the
header table reports — is now caught around `sim.step` and recorded as `UNSTABLE` with the
exception in the reason column, rather than killing the run. What the guards buy is that
no *sequence* of growing values is allowed to run on to overflow, which is what the brief
asked for.

Three verdicts, not two: `MARGINAL` means the run reached the end of the gait cycle but
exceeded ±142.2 N·m of knee authority or left the paper's fitted belt range on the way.
In the stub rehearsal of this sweep a binary stable/unstable classifier called `Kd = 5`
"stable" while it peaked at 1220.89 N·m. That rehearsal number was `engine=STUB` and not
a measurement, but it is why the third verdict was built — and the real run vindicated it:
`Kd = 5` completed the cycle at **774.22 N·m**, 5.4× the knee's authority, and a binary
classifier would have reported it as a success.

→ **`build/drivetrain_instability/diag_A_kd_sweep.csv`** carries all the requested
quantities per `Kd` — the eleven asked for (`t_unstable`, max `|theta_j|`, `|theta_a|`,
`|theta_s|`, `|theta_j_dot|`, `|theta_a_dot|`, `|tau_j|`, `|I_q|`, max controller output,
max stored belt energy — ten, plus the stable/unstable verdict itself) and two added here:
the fit-range exit time and the MuJoCo warning time.

### MEASURED — real MuJoCo, `engine=MUJOCO`

| `Kd` | verdict | t unstable | max \|θ_s\| (rad) | ×fit edge | max \|τ_j\| (N·m) | max \|I_q\| (A) |
|---|---|---|---|---|---|---|
| 0.000 | STABLE | — | 0.02995 | 0.54 | 39.61 | 11.18 |
| 0.250 | STABLE | — | 0.02907 | 0.53 | 38.07 | 11.06 |
| 0.500 | STABLE | — | 0.02825 | 0.51 | 36.64 | 11.01 |
| 1.000 | STABLE | — | 0.02693 | 0.49 | 34.41 | 10.88 |
| 1.250 | STABLE | — | 0.02630 | 0.48 | 33.36 | 10.84 |
| 1.500 | STABLE | — | 0.02573 | 0.47 | 32.41 | 10.80 |
| 2.000 | STABLE | — | 0.02822 | 0.51 | 36.60 | 10.68 |
| 5.000 | **MARGINAL** | — | 0.20037 | 3.64 | **774.22** | 27.89 |
| 10.000 | **UNSTABLE** | 0.5700 s | 0.28221 | 5.13 | 1434.95 | 58.69 |
| 17.253 | **UNSTABLE** | **0.3485 s** | 0.28529 | 5.19 | 1463.69 | 96.33 |

Empirical bracket: fully STABLE through `Kd = 2`, first non-STABLE at `Kd = 5`, so the
boundary lies in **(2, 5)**. Production `Kd = 17.253` is 3.5× the smallest non-stable
value tested. The `MARGINAL` band earned its keep on the first real run: `Kd = 5`
completed the cycle while peaking at **774.22 N·m**, 5.4× the knee's authority, and left
the fitted belt range at t = 0.7045 s. A binary classifier would have called that a
success and put the boundary at 10.



The test window is one AB19 gait cycle, 1.205 s. A `Kd` whose growth rate is slow cannot
reveal itself in that window, so it gets scored stable for want of time:

| `Kd` | predicted σ (1/s) | e-folding | growth over 1.205 s |
|---|---|---|---|
| 1.000 | −0.585 | decaying | 0.49× |
| 1.250 | −0.228 | decaying | 0.76× |
| 1.500 | **+0.127** | 7.87 s | **1.17×** |
| 2.000 | **+0.834** | 1.20 s | **2.73×** |
| 5.000 | +4.920 | 0.203 s | 376× |
| 10.000 | +10.890 | 0.092 s | 5.0e+05 |
| 17.253 | +17.762 | 0.056 s | 2.0e+09 |

So a swept bracket that reads `(2, 5)` and an analytic boundary of 1.41 are **not in
conflict** — `Kd = 1.5` and `Kd = 2` are predicted to be unstable but to grow only 1.2×
and 2.7× before the reference data runs out. The analytic boundary is the sharper number;
the sweep's bracket is an upper bound on it. This is the one place where the two methods
could have looked contradictory, so it is priced explicitly rather than left for a reader
to trip over.

---

## 4. Joint velocity vs actuator velocity

Two variants inside the diagnostic script, **both at `Kp = 600`, `Kd = 17.253`**, same
model, same reference, same initial condition, same timestep, same CAD geometry, same
belt parameters. The only difference is one expression:

```
variant J (production)   qdot = theta_j_dot                 # read from MuJoCo's qvel
variant A (diagnostic)   qdot = theta_a_dot / n_t           # read from the layer's own state
```

Variant A divides by `n_t` deliberately, so that `Kd` keeps its units and its nominal
damping value and the *only* thing that changes is which side of the belt the velocity
is measured on. Without that division the comparison would confound collocation with a
9:1 gain change.

→ **`build/drivetrain_instability/diag_C_variants.csv`**, plus full per-step traces in
`diag_trace_variantJ.csv` and `diag_trace_variantA.csv`.

### MEASURED — real MuJoCo, and this is the decisive result

| | variant J (joint velocity) | variant A (actuator velocity / n_t) |
|---|---|---|
| verdict | **UNSTABLE** | **STABLE** |
| time of instability | 0.3485 s | — (completed all 1.205 s) |
| max \|θ_s\| | 0.28529 rad (5.19× the fit edge) | **0.01515 rad (0.28× — inside it)** |
| max \|τ_j\| | 1463.69 N·m (10.3× authority) | **16.70 N·m (11.7% of authority)** |
| RMS error over its window | 2.682 deg | 4.951 deg |

Two things in that table deserve to be read carefully rather than skimmed.

First, variant J's RMS error of 2.682 deg is **lower** than variant A's 4.951 deg, and
that is not evidence in J's favour. J's window is only the 698 steps before it diverged,
and the divergence had barely begun to show in the angle when the torque was already
through 1463 N·m. A tracking metric computed over a truncated window of a diverging run
is not comparable to one computed over a complete cycle. This is exactly the trap the
`MARGINAL` verdict exists to catch.

Second, variant A's peak `|τ_j| = 16.70 N·m` sits within 4% of Case A's validated
ideal-actuator peak of 16.0041 N·m, and its RMS of 4.951 deg within 14% of Case A's
4.3616 deg. The collocated law does not merely avoid diverging — it lands close to the
rigid benchmark, which is what a correctly wired compliant transmission should do at this
load level.

**Also pre-registered and confirmed:** section P predicted, before any of these runs were
read, that variant A would survive the same `Kd` that killed variant J. It did. Had it
not, the explanation in section 1 would have been wrong and this document would say so.

**Variant A is a diagnostic probe, not a proposed fix.** It is not wired into
`controller.py` or `drivetrain_sim.py` and nothing in this document recommends that it
be, yet. See section 11.

> **Update, after this diagnosis was signed off:** variant A's arithmetic has since been
> adopted in production — `PDCurrentSource` now reads `theta_a_dot / n_t`, with `Kp` and
> `Kd` unchanged at 600 and 17.253. Section 0 of the diagnostic now reports production as
> **variant A** (exact match over 43 125 states; variant J differs by up to 50.69 A), so
> this section's "diagnostic probe" framing describes the state of the repository at the
> time of the diagnosis, not now. The change and its A/B result are recorded separately;
> the gains have still not been retuned.

---

## 5. Timestep

The same case at `h = 0.5 ms` and `h = 0.25 ms`, with the timestep set on the **in-memory
`mjModel`** for the duration of one run. `models/osl_v2_bench.xml` is never written — the
file still says `timestep="5e-4"`.

The full `Kd` sweep is run at **both** step sizes, deliberately: comparing a sweep at one
step size against a different grid at the other would produce an incomparable bracket and
an unearned conclusion.

→ **`build/drivetrain_instability/diag_D_timestep.csv`**.

### MEASURED — real MuJoCo

| | h = 0.5 ms | h = 0.25 ms |
|---|---|---|
| verdict at `Kd = 17.253` | UNSTABLE | UNSTABLE |
| time of instability | 0.3485 s | **0.3488 s** |
| steps run | 698 / 2410 | 1396 / 4820 |
| max \|θ_s\| | 0.28529 rad | 0.28273 rad |
| `Kd` bracket over the same 10-point sweep | **(2, 5)** | **(2, 5)** |

The blow-up time moved by 0.3 ms — 0.09% — on a halving of the step, and the bracket did
not move at all. A step-size-limited explicit-integration instability would have done the
opposite: halving `h` would have pushed the threshold up or removed the divergence. This
is as clean a negative result as the test could give, and it is what rules numerics out.

The question this answers: if halving `h` leaves the `Kd` bracket where it was while
moving the blow-up time, the mechanism is not a step-size-limited explicit-integration
instability. That is also what the independent arithmetic says. Two belt-mode ranges are
in circulation in this repository and they must not be conflated:

| source | range | inertia used | fit range |
|---|---|---|---|
| section 2 of this document | **13.82 – 23.42 Hz** | `I_j = 0.261998` (**with** the 0.01 armature) | inside the 0.055 rad fit edge |
| `docs/DRIVETRAIN_INTEGRATION_OPTIONS.md:37-43` | 13.94 – 26.69 Hz | `I_body = 0.251998` (**without** armature) | top row is at τ_j = 160 N·m, **1.7× past** the fit edge |

Section 2's range is the one to quote: it uses the inertia the model actually has and it
stays inside the region Best et al. fitted. Either way the conclusion about *numerics* is
the same — at `h = 0.5 ms` an explicit spring is stable up to roughly `2/omega`, and these
modes sit 24–46× inside that bound, so the belt was never numerically stiff at this
timestep.

---

## 6. Belt-mode frequency

Measured by detrending `theta_s(t)` over the final 35% of the steps that ran (the
approach to divergence, where the growing mode dominates the slow tracking signal) and
counting hysteretic zero crossings. Predicted from the same 2-mass model as section 2:

```
f = sqrt(K_s * (1/J_ar + 1/I_j)) / (2*pi)
```

`K_s = p1 + 2*p2*|theta_s|` hardens with amplitude, so a single predicted number would be
dishonest. The script reports the prediction at both the **mean** and the **peak** `K_s`
over the window, and the measured value is expected to sit in that band, not on a point.
The percentage difference is reported against the mean-`K_s` prediction and is **not**
forced toward it.

One guard matters here: a run that is *not* ringing still has a nonzero detrended
residual, and a "frequency" read off tracking residual is noise. The script computes
`osc_fraction = rms(detrended)/rms(raw − mean)` and refuses to call it a mode below 0.35,
printing the raw fraction and the run's own verdict next to the verdict so the threshold
can be overruled by a reader. The threshold is a judgement call and is labelled as one.

→ **`build/drivetrain_instability/diag_F_belt_mode.csv`**.

### MEASURED — real MuJoCo

| | variant J (diverging) | variant A (stable) |
|---|---|---|
| coherent oscillation in θ_s | **yes** — oscillatory RMS 1.02 of total | **no** — 0.24, below the 0.35 gate |
| measured frequency | **25.424 Hz** over 7 half cycles | not quoted (not ringing) |
| predicted at mean `K_s` = 3701.29 | 28.401 Hz (**−10.48%** vs measured) | — |
| predicted at peak `K_s` = 9385.05 | 45.225 Hz | — |
| envelope growth σ | **+16.347 /s** | −0.977 /s |

The measured 25.424 Hz sits below the predicted band's lower edge by 10.5%, and that
direction is the expected one: the prediction uses the mean `K_s` over a window in which
the amplitude — and therefore the stiffness — was still growing, so the mode spent part of
the window softer than the window average. The number is reported as measured, not nudged
toward the prediction.

Two cross-checks make this more than a single number. The measured growth rate **+16.347
/s** is within 8% of the linear model's **+17.762 /s** predicted for `Kd = 17.253` at the
bench's own `K_s` — two independent estimates of the same instability, one from an
eigenvalue and one from counting peaks in a MuJoCo trace. And variant A, correctly, yields
**no mode at all**: its oscillatory fraction of 0.24 falls below the gate, its envelope
decays at −0.977 /s, and the gate's job is precisely to stop a "frequency" being read off
a stable run's tracking residual. The 8.516 Hz the estimator produced for variant A is
printed with that caveat attached and must not be quoted as belt physics.

---

## 7. Energy and power — the mechanism, not just the verdict

This is the part that distinguishes "it is unstable" from "here is where the energy comes
from". Per step the script logs `tau_j`, `theta_j_dot` and their product; the actuator
torque, actuator velocity and actuator-side power; `theta_s`; and the stored belt energy
`U(theta_s) = p2|theta_s|³/3 + p1 theta_s²/2`. Over the analysis window it integrates:

- **`W_kd_on_actuator`** — the work actually done by the derivative term on the actuator,
  `integral of (-Kd * qdot_used) * theta_a_dot/n_t dt`. This is the suspect.
- **`W_kd_if_collocated`** — the counterfactual `integral of (-Kd * theta_j_dot) * theta_j_dot dt`,
  which is `-Kd * integral(theta_j_dot²)` and therefore **≤ 0 by algebra, for any gain,
  always**. A damping term cannot do positive work on the body whose velocity it reads.
- `W_motor`, `W_friction`, `W_Ba`, `W_belt_to_joint`, the change in stored belt energy,
  and the **fraction of the window in which `theta_j_dot` and `theta_a_dot/n_t` have
  opposite signs** (the antiphase fraction — the direct test of the mode shape the
  explanation depends on).

The decisive comparison is the first two columns of the same row. `W_kd_if_collocated` is
negative by construction. If `W_kd_on_actuator` is **positive** on the same window, the
derivative term is a net energy source, and the sign flip is caused by *where the
velocity was measured*, not by how large `Kd` is. A gain cannot change the sign of
`-Kd·v²`; a collocation error can.

Work integrals are reported over each variant's own window **and**, whenever the two
windows overlap by more than four samples, over a matched window — so that window length
does none of the arguing. Mean powers are given alongside the integrals for the same
reason.

→ **`build/drivetrain_instability/diag_E_energy.csv`**.

### MEASURED — real MuJoCo. This is the mechanism, in numbers.

| window | `W_kd_on_actuator` (actual) | `W_kd_if_collocated` (counterfactual) | antiphase fraction | Δ stored belt energy |
|---|---|---|---|---|
| variant J, own window [0.2265, 0.3485] s | **+229.12 J** (+1870 W mean) | −199.43 J | **0.878** | **+148.74 J** (peak U 151.07 J) |
| variant A, own window [0.7830, 1.2045] s | **−82.79 J** (−196 W mean) | −86.97 J | 0.154 | +0.022 J |
| variant A, matched to J's window | **−1.697 J** (−13.9 W mean) | −1.860 J | 0.212 | +0.005 J |

**The sign flips.** On the same 0.1225 s window, with the same gain, the same belt and the
same reference, the derivative term delivers **+229.12 J into** the actuator under
joint-velocity feedback and **−1.697 J out of** it under actuator-velocity feedback. The
counterfactual column is negative in every row, as algebra requires. A gain cannot change
the sign of `-Kd·v²`; only reading `v` on the wrong side of the belt can. That is the
diagnosis reduced to one comparison.

The supporting numbers agree. The **antiphase fraction is 0.878** for variant J — the two
inertias spend 88% of the window moving in opposite directions, which is the mode shape
the explanation requires, measured rather than assumed — against 0.212 for variant A on
the identical window. And the energy has somewhere to go: stored belt energy rises by
**+148.74 J** in variant J against **+0.005 J** in variant A, so the +229 J injected is
visibly accumulating in the spring rather than being dissipated. Variant J's motor did
+255.44 J of work in 0.12 s while friction and `B_a` removed only 44.7 J.

The matched-window row is what makes this argument airtight: variant A's own window is
3.4× longer, so comparing own-window integrals would have let window length do the
arguing. On the matched window the comparison is +229.12 J against −1.697 J.

This section is written as a test, not as a result: it states what the numbers would have
to look like for the collocation explanation to hold. The real-MuJoCo run has now filled
it in, above, and the predicted sign flip appeared.

---

## 8. Did `theta_s` stay inside the paper's fitted range?

**This matters independently of the instability.** Best et al.'s Fig. 3 abscissa stops at
roughly `|theta_s| = 0.055` rad (≈93.3 N·m). Past that, `rho(theta_s)` is extrapolation
of their regression, not their data — so any torque the model reports beyond that point
is a statement about our extrapolation, not about their belt.

The script flags the first time each run leaves that range (`theta_s_fit_exit_s`) and
reports the peak as a multiple of the fit edge. Leaving the range is a **flag, not a
fatal** — the run continues to the abort guard — but it is recorded, because a torque
figure quoted from outside the fitted range should never be quoted without it.

→ **`build/drivetrain_instability/diag_G_theta_s_range.csv`**.

### MEASURED — real MuJoCo

| `Kd` | max \|θ_s\| (rad) | × fit edge | inside the fitted range? |
|---|---|---|---|
| 0.000 – 2.000 | 0.02573 – 0.02995 | 0.47 – 0.54 | **yes, all of them** |
| 5.000 | 0.20037 | 3.64 | no — left at t = 0.7045 s |
| 10.000 | 0.28221 | 5.13 | no — left at t = 0.3615 s |
| 17.253 | 0.28529 | 5.19 | no — left at t = **0.2260 s** |

This answers the question the brief asked in the sharp direction. Every stable run sits at
roughly **half** the fit edge, so the experiment is not operating by extrapolation as a
matter of course. And at production `Kd` the deflection leaves the fitted range at
t = 0.2260 s while the run does not diverge until **0.3485 s** — the growth is already
well underway inside the region Best et al. fitted, and only its later magnitudes are
extrapolation. So the instability is **not an artefact of leaving the fit**; but the
1463 N·m peak is quotable only as "what our extrapolation of their regression returns",
never as belt physics. Both halves of that sentence are needed, and now both are measured.

---

## 9. Measured, assumed, derived — and which engine produced it

Read this table together with one fact: **every row marked MEASURED is measured only once
`experiments/diagnose_drivetrain_instability.py` has been run under real MuJoCo.** Each
output CSV carries an `# engine=STUB|MUJOCO` provenance line precisely so this cannot be
fudged. **The run of 2026-10-04 was `engine=MUJOCO`** (headline: `engine : REAL MuJoCo`),
so the MEASURED rows below are now measured, and every "MEASURED — real MuJoCo" table in
sections 3 through 8 comes from it. Two things are worth keeping in view anyway: under the
stub those same rows would come from `tests/stub_mujoco.py`'s hand-written integrator, and
the stub has no `warning` array at all, so `mujoco_warning_s` degrades to "never"
structurally rather than reporting an absence of warnings. The DERIVED rows use no engine
and were valid before any run.

| Quantity | Status |
|---|---|
| `theta_j`, `theta_j_dot`, stability outcome, blow-up time, MuJoCo warning time | **MEASURED** from the engine's own `qpos`/`qvel`/`warning` — real only under `engine=MUJOCO` |
| `theta_a`, `theta_a_dot`, `theta_s`, `tau_j`, `I_q`, `U` | **COMPUTED BY OUR LAYER** from the paper's equations; measured in the sense that they are what the integration produced, derived in the sense that no engine owns them |
| peak torques, peak currents, work integrals, antiphase fraction | **MEASURED** from those traces — same engine caveat |
| measured belt-mode frequency, growth rate σ | **MEASURED**, by detrend + zero-crossing + half-cycle peak fitting — estimator-dependent, and the estimator is in the script |
| the t = 0.6560 s blow-up in the header table | **MEASURED, real MuJoCo** — the one real-engine datum here, from `MUJOCO_LOG.TXT` |
| eigenvalues, `Kd` boundaries, predicted σ, predicted belt-mode frequency | **DERIVED** from the linear 2-mass model; engine-independent, so valid without any run; linearisation assumptions listed in section 2 |
| `J_ar = n_t²·J_a`, `B_ar = n_t²·B_a`, `k_t_joint = n_t·k_t·n_a = 4.597092` | **DERIVED** from the paper's parameters |
| `I_j = 0.261998`, `b_j = 0.3`, `forcerange = ±142.2` | **HARDCODED LITERALS** in the diagnostic (`I_J`, `B_J`, `KNEE_AUTHORITY_NM`), so every section 2/3 number rests on the literal, not on the compiled model. They were *originally* read off the compiled model — `bench.i_eff` recomputes `I_j` via `mj_mulM` and agrees to all six digits — but only test F actually calls it. Treat them as transcribed, cross-checked once. |
| `p1 = 876`, `p2`, `J_a`, `B_a`, `f_c`, `f_g`, `k_t`, `n_a`, `n_t` | **ASSUMED** — taken from Best et al. 2025 for a different (ankle) build. Not measured on our hardware. |
| the 0.055 rad fit edge | **ASSUMED** — read off the paper's figure axis |
| `OSC_GATE = 0.35`, the 10× authority abort threshold, the 35% analysis window | **ASSUMED** — judgement calls, chosen before the runs, labelled in the script |
| anything about physical hardware | **NOT ESTABLISHED.** No hardware was involved at any point. |

A further assumption worth stating because it is load-bearing for the whole phase: our
belt is the 5 mm (stiffer) build, while `p1`/`p2` come from the paper's build whose pitch
is never stated, so `p1 = 876` may be a **lower bound** for us. A stiffer belt would move
the `Kd` boundary up — section 2 measures it rising from 1.1170 to 1.6394 as `K_s` goes
from 876 to 2516 N·m/rad — but would not change its existence. Even at the highest `K_s`
tried, the boundary is still 10.5× below production `Kd`, and the actuator-velocity
column has no boundary at any `K_s`.

---

## 10. Implementation error, or a real consequence of the compliant model?

**Both, and the distinction matters, so here it is split.**

It is **an implementation error** in the narrow sense that `PDCurrentSource` was written
to feed joint states into an actuator-side current, while `docs/ACTUATOR_DYNAMICS_ANALYSIS.md`
had already recorded that the paper deliberately does the opposite, and
`docs/DRIVETRAIN_INTEGRATION_OPTIONS.md` had scored actuator-state availability as a
criterion of the architecture we picked (without listing it among the four reasons for
picking it — see section 1). The wiring does not match what we already knew the paper
does. That is a defect in our code, and it is fixable without touching `Kp`, `Kd`, the
belt law, the paper's parameters, the XML or the timestep.

It is **a genuine consequence of the compliant actuator model** in the sense that nothing
has been shown to be numerically wrong, and all four legs of that now hold: the belt mode
is 24–46× inside the explicit-spring stability bound at this timestep; the operator split
was previously verified symplectic with zero secular drift; the sign chain through eqs
(3), (4) and (7) is covered by `test_power_balance_across_the_massless_belt`; and halving
the timestep moved the blow-up time by 0.09% and the `Kd` bracket not at all (section 5,
measured). A rigid transmission has no belt mode to destabilise, so the same
controller was stable for as long as the model had no series elasticity. The instability
appeared exactly when the physics that makes it possible was added.

**That second half is itself a result worth reporting**, and it is the actuator-fidelity
finding this phase was looking for: an ideal-torque-source benchmark of this joint is not
merely *less accurate* once a compliant belt is in the model — with joint-velocity
feedback it is *unstable*, and by roughly 12× in `Kd`. (A belt *model*, fitted by Best et
al.; this says nothing about a physical belt.) That belongs in the write-up. It should not
be retuned away quietly.

What it is **not**: it is not a sign error in the belt law, not an artefact of the stub,
and not a statement about hardware. It is also **not** a numerical-integration or timestep
failure: that was argued from the stability bound and has since been confirmed by the
halved-step run, which moved the blow-up time by 0.3 ms and the bracket not at all.

---

## 11. The eight physics questions, answered

**1. Is the instability controller architecture, actuator model, belt model, numerical
integration, an implementation/sign error, or a combination?**
Controller architecture, implemented as written. Specifically the collocation of the
derivative term, not the gain's value. The belt model and the actuator model are doing
what the paper's equations say; the integration is well inside its stability bound, and
the measured `Kd` bracket is **(2, 5) at both `h = 0.5 ms` and `h = 0.25 ms`**; the sign
chain is test-covered. It is a combination only in the weak sense that the architecture
defect is harmless without the belt and the belt is harmless without the defect.

**2. Does `Kd` actually cause it?**
Yes, and both the model and the run agree. Analytically: `Kp = 600, Kd = 0` is stable
(max Re = −1.6376) and `Kp = 0, Kd = 17.253` is unstable (+14.6114), so dropping `Kp` to
zero retains 82% of the growth rate while dropping `Kd` to zero removes all of it.
Measured, those three cases ran as: `Kp = 600, Kd = 0` **STABLE** (RMS 2.066 deg,
max \|θ_s\| 0.02995 rad); `Kp = 0, Kd = 17.253` **UNSTABLE at 0.9270 s** (RMS 27.170 deg
— it diverges with no position feedback at all); `Kp = 600, Kd = 17.253` **UNSTABLE at
0.3485 s**. The `Kd` boundary at `Kp = 600` is 1.4105 and production is 12.2× it.

**3. Does joint-velocity feedback destabilise the belt mode while actuator-velocity
feedback damps it?**
Yes, on three independent lines of evidence. Analytically: joint velocity has a boundary
at `Kd ≈ 1.41`, actuator velocity has none anywhere in `[0, 200]` and gets monotonically
more stable. In the run (section 4): variant J diverges at 0.3485 s, variant A completes
the cycle at the same gains with peak \|τ_j\| of 16.70 N·m. And mechanistically (section
7): on a matched window the derivative term delivers **+229.12 J into** the actuator under
joint-velocity feedback and **−1.697 J out of** it under actuator-velocity feedback, with
the antiphase fraction measured at 0.878 — the mode shape the explanation needs, observed
rather than assumed.

**4. Is `Kd = 17.253` meaningful once the flexible transmission is present?**
No. `Kd = 17.253` came from `kd_for_damping_ratio(600, 0.261998, 0.3, zeta=0.7)`, which
returns 17.2530523841 and was committed rounded — a **single-mass** critical-damping
calculation on `I_j` alone, for a joint driven directly.
That derivation has no belt in it and no second inertia, so it cannot know about a mode it
does not model. The number is correct for the plant it was derived for and inapplicable to
the plant it is now used in. (Separately, and already recorded: the control law is a
regulator, `-Kd*theta_j_dot`, not a tracker — there is no `theta_ref_dot` term, which is
the bulk of the 4.36 deg Case A error. That is a different problem and is not touched
here.)

**5. Is the instability visible before any numerical overflow?**
Yes, and the script measures the gap rather than asserting it. It watches MuJoCo's own
`mjWARN_BADQACC/BADQPOS/BADQVEL/BADCTRL` counters directly (resolved by name, so no index
is hard-coded) and records `mujoco_warning_s` alongside `t_unstable_s`. The abort guards
fire on physical thresholds — 10× knee authority, the belt-deflection equivalent of that
torque, 50 rad/s, 400 A — all of which are crossed while every number is still finite.
Measured: at production `Kd` the diagnostic aborted on `|tau_j| = 1463.7 N·m` at
**t = 0.3485 s** with every state still finite, whereas the unguarded experiment's own
NaN came at **t = 0.6560 s**. So the instability was detectable **0.3075 s — 615 steps —
before** the arithmetic broke, and the deflection had already left the fitted range at
0.2260 s, earlier still. The NaN is the last symptom by a wide margin.

**6. Is `theta_s` staying inside the 0.055 rad fitted belt range, or does the experiment
immediately extrapolate outside the paper's fitted region?**
It stays inside, and comfortably: every stable run peaks at **0.47–0.54× the fit edge**,
so the experiment does not operate by extrapolation as a matter of course. Diverging runs
start inside it and leave during the runaway — at production `Kd`, at t = 0.2260 s, which
is 0.12 s *before* the divergence. So the onset is inside the fitted region (the
instability is not an extrapolation artefact), but the large torques reported afterwards
are outside it and must not be quoted as belt physics.
`diag_G_theta_s_range.csv` has the exit time and the multiple of the fit edge per run.
Note that before this work there was **no guard anywhere** that flagged leaving that
range; now at least the diagnostic has one.

**7. Is the instability reproducible when the timestep is halved?**
Yes. Measured: UNSTABLE at both `h = 0.5 ms` (t = 0.3485 s) and `h = 0.25 ms`
(t = 0.3488 s), with the `Kd` bracket at **(2, 5) on both grids** — the same 10-point
sweep run at each step size, so the brackets are comparable rather than coincidental. The
blow-up time moved by 0.09%. That is the predicted result, from the belt mode sitting
24–46× inside the explicit-spring bound, and it is what rules out a step-size-limited
integration instability.

**8. What is the smallest defensible controller-architecture change, if any, that would
fix it?**
Read the derivative term from the actuator instead of the joint — replace
`theta_j_dot` with `theta_a_dot/n_t` in the one expression in `PDCurrentSource`, leaving
`Kp = 600` and `Kd = 17.253` untouched. It is one line, it changes no gain, no parameter,
no equation, no XML and no timestep, it is what Best et al. do, and it is the capability
`docs/DRIVETRAIN_INTEGRATION_OPTIONS.md` scored for Option C and then never used. The
analytic model says it is unconditionally stabilising in `Kd` at this `Kp`, and the
measured run (section 4) says it completes the cycle at peak `|τ_j| = 16.70 N·m`.

**This was a recommendation when this document was first written, and has since been
implemented** — see the note in section 4. The two conditions set on it at the time were
both met before any production file was edited: (a) the real-MuJoCo tables in sections 3–8
agree with the analytic prediction, including the pre-registered claim that variant A
would survive the gain that killed variant J; and (b) the caveat stands unchanged — `Kd =
17.253` is still a single-mass number with no belt in its derivation, so even after
collocation it is *defensible* rather than *right*. Re-deriving it for the 2-mass plant
remains a separate, open task.

What should **not** be done: adding artificial damping, clamping `theta_s`, adding
saturation, changing the belt equation or the paper's parameters, permanently changing the
timestep, or retuning `Kp`/`Kd` to make the divergence go away. Every one of those hides
the finding in section 10 instead of fixing the defect in section 1.

---

## 12. Provenance of the tables

The Linux sandbox this diagnostic was written in has **no `mujoco` module** (and no
network to install one). `tests/stub_mujoco.py`'s `install()` returns **real MuJoCo
whenever it is importable** and falls back to the stub otherwise, so the same script
produces real measurements inside `.venv` on Windows and wiring checks in the sandbox.

Every generated file and every table stamps `engine=STUB` or `engine=MUJOCO` in its
provenance header, and the script prints the engine in its banner and repeats the warning
at the end. **A table stamped `engine=STUB` is a wiring check and not a measurement.**

The run this document reports was made on **2026-10-04 under `engine=REAL MuJoCo`**, in
`.venv` on Windows, and it is the source of every "MEASURED — real MuJoCo" table in
sections 3 through 8. Its own integrity checks passed in the same run: 27 runs audited
(A 10 + B 3 + C 2 + D 12) with **0 steps** of nonzero knee position-actuator force, so the
servo was off throughout and no number above is a double count; and section 0 confirmed
that one of the two compared variants is production arithmetic bit for bit. The
engine-independent material in section 2 was valid before any of this and does not depend
on it.

Command, for reproduction:

```
.venv\Scripts\python.exe experiments\diagnose_drivetrain_instability.py
```

It writes `build/drivetrain_instability/RESULTS.md` plus eleven CSVs and overwrites
whatever was there, so the stub outputs currently in that directory are replaced in place.
It performs 27 runs of the gait cycle — ten for the `Kd` sweep, three for the `Kp`/`Kd`
isolation, two for the velocity variants, and twelve for the timestep comparison, eleven
of which are at the halved step and so twice the length.

Then the unchanged-baseline checks:

```
.venv\Scripts\python.exe tests\run_tests.py
.venv\Scripts\python.exe experiments\verify_against_oracle.py
```

The drivetrain-OFF path must be bit-identical, and it should be: the diagnostic loads its
own fresh model per run, disconnects the servo only inside its own runs, never writes the
XML, and the oracle CSVs in the working tree differ from `HEAD` by **line endings only**
(`git diff --ignore-cr-at-eol -- tests/oracle/` is empty — zero content change).

---

## 13. Files

| File | Role |
|---|---|
| `experiments/diagnose_drivetrain_instability.py` | the whole diagnostic; sections 0, P, A–H |
| `build/drivetrain_instability/RESULTS.md` | machine-generated tables, engine-stamped |
| `build/drivetrain_instability/diag_P_analytic.csv` | eigenvalues and `Kd` boundaries (no engine) |
| `build/drivetrain_instability/diag_A_kd_sweep.csv` | the `Kd` sweep, eleven quantities per run |
| `build/drivetrain_instability/diag_A_growth_budget.csv` | predicted σ vs what one gait cycle can show |
| `build/drivetrain_instability/diag_B_kp_kd.csv` | the three `Kp`/`Kd` isolation cases |
| `build/drivetrain_instability/diag_C_variants.csv` | joint vs actuator velocity |
| `build/drivetrain_instability/diag_D_timestep.csv` | the halved step: 11 runs at 0.25 ms plus one 0.5 ms control. The 0.5 ms arm of the comparison is `diag_A_kd_sweep.csv` — same grid, same guards, so the two are directly comparable |
| `build/drivetrain_instability/diag_E_energy.csv` | work integrals, mean powers, antiphase fraction |
| `build/drivetrain_instability/diag_F_belt_mode.csv` | measured vs predicted frequency, with the mode gate |
| `build/drivetrain_instability/diag_G_theta_s_range.csv` | fit-range excursions |
| `build/drivetrain_instability/diag_trace_variantJ.csv` | per-step trace, 24 channels |
| `build/drivetrain_instability/diag_trace_variantA.csv` | per-step trace, 24 channels |

Nothing in this diagnosis has been committed.

---

## Appendix — the Option-C control path, read off the code

This is the inspection that section 1 rests on. Every answer is from the source **as it
stood when the instability was diagnosed**, i.e. before the collocation fix; item 2 in
particular describes the law that diverged, not the law in the repository now. No
production file was modified to produce this inspection.

| # | Question | Answer |
|---|---|---|
| 1 | Where do `Kp`/`Kd` enter the Option-C path? | **Only** through `PDCurrentSource` → current. The same `PDController` is also passed as `knee=` to the simulation, but `disconnect_knee_servo()` zeroes its `gainprm`/`biasprm` in the compiled `mjModel`, so the MuJoCo-side gains are dead. `self.knee` survives purely for reporting the controller's *request*. |
| 2 | What does `PDCurrentSource` compute? | `tau_req = pd.torque_unclamped(q_ref, theta_j, theta_j_dot)`, then `I_q = tau_req / k_t_joint` with `k_t_joint = n_t·k_t·n_a = 4.597092` N·m/A. **Unclamped** — no `forcerange`, no current limit, because `drivetrain.py` documents no drive-current limit. |
| 3 | Does the derivative term use joint velocity, actuator velocity, or velocity error? | **Joint velocity**, `data.qvel[knee_dof]`, read pre-step. Not actuator velocity. Not a velocity *error* — there is no `theta_ref_dot`, so the term is `−Kd·theta_j_dot`, a regulator. |
| 4 | What quantity becomes `I_q`? | The requested **joint** torque in N·m, divided by `k_t_joint`. |
| 5 | Where is `I_q` sent into the drivetrain? | `layer.advance(theta_j, i_q, dt)` → `D.motor_torque(i_q, p)` (paper eq 1) and `D.friction_torque(theta_a_dot, i_q, p)` (eq 2). |
| 6 | Where is `tau_j` injected into MuJoCo? | `data.qfrc_applied[bench.knee_dof] = rec.tau_j_mean`, written **before** `super().step()` calls `mj_step`. A raw generalized force: it bypasses `gainprm`/`biasprm`/`forcerange` and is **not** reported in `actuator_force`. |
| 7 | Is the old position servo definitely disconnected? | Yes, and three ways over. Zeros are written into the compiled `mjModel`; `run_case_b` refuses to start if `sim.servo_connected`; and `servo_violations` is counted every step with a nonzero exit. Section H aggregates the count over **all 27 runs** and prints the breakdown (A 10 + B 3 + C 2 + D 12) so the word "all" can be checked rather than taken on trust. It must read 0 for any other number in this document to mean anything. |
| 8 | Integration order within one 0.5 ms step | read `theta_j`, `theta_j_dot` pre-step → `current_source` evaluates the PD law on those joint signals → `layer.advance` integrates the shaft semi-implicitly, belt force explicit at the old configuration → write `qfrc_applied` → write `data.ctrl` (multiplied by zeroed gains, so 0) → `mj_step` under `implicitfast`. |
| 9 | Which state does the drivetrain layer own? | `theta_a` and `theta_a_dot`, as plain Python floats. MuJoCo has **no dof** for them. |
| 10 | Which state does MuJoCo own? | `theta_j = qpos[knee_qpos]` and `theta_j_dot = qvel[knee_dof]`, exclusively, plus the ankle dof. The layer only ever *reads* them. |

Items 3, 6, 9 and 10 together are the defect: the controller reads state MuJoCo owns
(item 10), and applies the resulting torque to state the layer owns (item 9), with the
belt between them (item 6). Items 1, 2 and 7 establish that there is exactly one torque
path and no hidden second actuator, so the behaviour cannot be a double-count.
