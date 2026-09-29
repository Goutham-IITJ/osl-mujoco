# Integrating the drivetrain model into MuJoCo — four options, one recommendation

**Status: ANALYSIS ONLY. Nothing in this document has been implemented.** No MJCF, no
model-building code, no controller and no oracle file was modified while writing it. The
standalone model in `oslbench/drivetrain.py` remains completely separate from the bench.

The question this file answers is narrow: *given the paper's drivetrain model, which is now
encoded and tested, what is the right way to put it inside MuJoCo next?* Four options are
scored against eight criteria, and exactly one is recommended.

## What an integration has to carry

From Best et al. (Sec. III-A, eqs (1)–(5)), five things must survive the move into MuJoCo, or
the integration is not an integration:

1. **`I_q` as the input.** The plant's input is a q-axis current, not a position setpoint.
2. **Actuator-output inertia and damping on the far side of the belt** — `J_a`, `B_a` act on
   `θ_a`, not on `θ_j`.
3. **Current-dependent friction** `f_g·|I_q|`, which at our bench's peak is **1.91×** the
   constant Coulomb part (1.507 vs 0.789 N·m referred to the joint).
4. **A second mechanical coordinate**, because `θ_s = θ_j − θ_a/n_t` stores energy and cannot
   be recovered from `θ_j`.
5. **A nonlinear, stiffening spring law** `ρ(θ_s) = p2 θ_s² + p1 θ_s`. MuJoCo's joint
   `stiffness` is **linear**, so this cannot be authored in XML by any option.

Point 5 is decisive for the comparison below: *every* option that represents the belt honestly
needs a per-step torque callback. That removes the main argument in favour of the
"do-it-in-XML" option.

## The new quantitative input: how fast is the belt mode?

This had to be known before the options could be scored, because it decides whether an extra
DOF costs us a smaller timestep. Two-mass estimate with the paper's actuator reflected to the
joint (`n_t²·J_a` = 0.208908 kg·m²) against our CAD knee-distal inertia (`I_body` = 0.251998
kg·m², CAD-DERIVED), reduced inertia `I_red` = 0.114219 kg·m²:

| `τ_j` [N·m] | `K_s` [N·m/rad] | belt mode [rad/s] | [Hz] | period [ms] | explicit-spring `h_max ≈ 2/ω` | steps/period at `h` = 0.5 ms |
|---|---|---|---|---|---|---|
| 0 | 876.00 | 87.58 | 13.94 | 71.8 | 22.8 ms | 143 |
| 16.004 (our bench peak) | 1312.27 | 107.19 | 17.06 | 58.6 | 18.7 ms | 117 |
| 50 | 1936.49 | 130.21 | 20.72 | 48.3 | 15.4 ms | 96 |
| 93.29 (Fig. 3 edge) | 2516.43 | 148.43 | 23.62 | 42.3 | 13.5 ms | 85 |
| 160 (paper 10 s peak) | 3211.18 | 167.67 | 26.69 | 37.5 | 11.9 ms | 75 |

**The belt mode is not numerically stiff.** At the bench's existing `timestep = 0.0005` we are
24–46× inside the explicit-spring stability bound, and we resolve the fastest belt oscillation
with ~75 steps. *No option below requires a smaller timestep.* This kills what would otherwise
have been the strongest objection to representing the belt explicitly.

It also says something less comfortable. With `B_a` as the only dissipation acting on that
mode, the model's belt-mode damping ratio is `n_t²B_a / (2√(p1·I_red))` ≈ **0.064**. That is a
property of the equations *as the paper wrote them* — the paper reports no belt damping or
hysteresis term at all, so any real belt dissipation is simply **absent from the model**. It is
not a claim about how our hardware behaves. The practical consequence is only this: a lightly
damped 14–27 Hz mode will be clearly visible in any integrated run, so the integration cannot
be validated by eyeballing joint angles alone.

### And how much does it matter at our operating point?

| Gain | Closed-loop `ω_n` (with `I_eff` = 0.261998) | Separation from the 13.94 Hz belt mode | Rendered stiffness in series with `K_s(0)` = 876 | Shortfall |
|---|---|---|---|---|
| `kp` = 60 — authored in the MJCF text, **never actually run** | 15.13 rad/s = 2.41 Hz | 5.8× | 56.15 N·m/rad | 6.4 % |
| `kp` = 600 — **the live gain of every run, including the frozen oracle** | 47.85 rad/s = 7.62 Hz | 1.8× | 356.10 N·m/rad | **40.7 %** |

**Read the second row, not the first.** `models/osl_v2_bench.xml:335` does author
`kp="60.0"`, but `oslbench/controller.py:45` sets `KP = 600.0` as the live default and
`PDController.write_to_model` writes it into the compiled `mjModel` on **every** run before the
first step. So 60 is a number in a file that no experiment has ever integrated at, and 600 is
the gain behind every result we hold — the AB19 benchmark, the frozen oracle metrics, the gain
sweep, all of it. (Verify rather than trust: `grep -n '^KP' oslbench/controller.py` and
`grep -n 'knee_pos' models/osl_v2_bench.xml`.)

This is the sharpest finding of the stage, and the correct tense for it is the present. The
series compliance is eating **40.7 %** of the commanded stiffness **now**, and the belt mode
sits less than a factor of two above the closed loop **now**. This is the paper's own measured
C0 failure — "an additional series compliance always reduces the effective stiffness" — and our
bench cannot show it, because the bench has no belt.

The consequence is therefore not a warning about a future change. It is a statement about the
benchmark as it stands: **the validated result was produced at the gain where belt compliance
matters most, by a model that omits the belt entirely.** That does not make the frozen oracle
wrong — it is a correct simulation of the model it simulates, and it remains the reproducibility
authority for the software. It does mean the 40.7 % is a known, quantified gap between that
model and the paper's device, and closing it is what the drivetrain work is for. No gains are
being changed here; the number that changed is the description of which gain is live.

## Option A — equivalent joint-side parameters

Write `armature = n_t²J_a = 0.208908`, `damping = n_t²B_a = 1.287877`,
`frictionloss = n_t·f_c = 0.788310` into the knee joint and stop.

| Criterion | Assessment |
|---|---|
| Physical fidelity | **Low.** Requires `θ_s ≡ 0`, which the table above shows is a 6 %–41 % assumption depending on gain. Drops the belt mode, drops `f_g|I_q|` entirely (the larger friction term), and keeps a position servo, so there is still no current input. |
| Numerical stability | **Best of the four.** No new state, no new mode, nothing to destabilise. |
| Timestep | Unchanged; could even be coarsened. |
| Code complexity | **Lowest** — three attribute values. |
| nq = 2 compatibility | Perfect in shape, but it **edits `models/osl_v2_bench.xml`**, so the oracle metrics no longer describe the model in the repo. |
| Future controller work | **Poor.** No `I_q` input means the paper's compensator (8)/(16)/(17)/(20) can never be implemented or compared against C0/C1/C2. |
| Validation against hardware | **Poor.** Nothing new becomes observable. |

Verdict: already fully quantified as column B of `experiments/check_drivetrain_model.py` §4.
**Its value is as a measurement, not as a destination** — it tells us the rigid approximation
costs 20.89× in inertia, 4.29× in damping and 1.97× in Coulomb friction relative to the current
placeholders. Adopting it would make the bench a *better rigid model*, not a model of the
paper's device.

## Option B — explicit actuator and transmission states in MuJoCo

Add a geometry-free body carrying a hinge `θ_a` with `armature = J_a`, `damping = B_a`,
`frictionloss = f_c`, and couple it to the knee through the belt.

| Criterion | Assessment |
|---|---|
| Physical fidelity | **Highest.** Two genuine DOF; `θ_s` holds real energy; `τ_j` comes from (4) exactly; `I_q` is the input. |
| Numerical stability | **Good, and now quantified** — `h_max` ≈ 12 ms vs our 0.5 ms. One integrator advances both coordinates, so there is no operator-splitting error at all. This is B's real advantage over C. |
| Timestep | Unchanged. |
| Code complexity | **Medium-high.** And the XML cannot do the job alone: `stiffness` is linear, so `ρ` still needs a per-step applied torque, and `f_g|I_q|` needs one too. B therefore buys a callback *and* a model change, where C buys only the callback. |
| nq = 2 compatibility | **Breaks it.** nq becomes 3 (4 if the ankle follows). `tests/oracle/*` describes an nq = 2 model; the nine preconditions in `oslbench/model.py` assume it; the AB19 regression stops being comparable. |
| Future controller work | **Best.** `I_q` is the natural input and actuator-state feedback (which the paper deliberately prefers to noncollocated joint-state feedback) becomes available directly. |
| Validation against hardware | **Best** — `θ_a` and `θ_j` are separately observable, as they are on the real device. |

Verdict: the right *destination*, the wrong *next step*. It requires touching
`tools/build_mjcf.py` and `models/osl_v2_bench.xml` — both currently protected — and it
invalidates the only regression baseline we have, before we have anything to replace it with.

## Option C — CAD MJCF unchanged, external actuator-dynamics state layer

Keep `models/osl_v2_bench.xml` exactly as it is. Own `(θ_a, θ̇_a)` in a new Python layer, step
it alongside MuJoCo, and inject the belt torque into the knee through `qfrc_applied` (or a
torque actuator) each step. `I_q` is the layer's input.

| Criterion | Assessment |
|---|---|
| Physical fidelity | **High — the same equations as B.** The only difference is *who owns `θ_a`* and *which integrator advances it*, not what is represented. All five requirements above are met. |
| Numerical stability | **Good, with one quantified caveat.** The coupling is operator-split: our layer sees the joint state from the previous step. At the fastest belt frequency `ω·h` = 167.67 × 5e-4 = **0.084 rad** (4.8°) of coupling phase lag per step, which behaves like an artificial damping of order `ω·h/4` ≈ 0.02 — about a third of the model's own 0.064. That is an order-of-magnitude estimate, **not a proven bound**, and it is the one honest risk in this option. Mitigation is cheap: sub-step the layer 10× and the figure drops to ~0.002. Test is cheap too: set `B_a` = 0 and measure belt-potential energy drift. |
| Timestep | Unchanged. Optional sub-stepping affects the layer only, never MuJoCo. |
| Code complexity | **Low-medium, and purely additive.** One new module plus a step hook. No XML, no `build_mjcf.py`, no `model.py`. |
| nq = 2 compatibility | **Perfect, and this is the decisive point.** nq stays 2, so with the layer disabled the bench must reproduce `tests/oracle/bench_track_ab19_metrics.csv` *exactly* — the oracle survives as a live regression test and becomes the first acceptance criterion of the integration itself. |
| Future controller work | **Good.** `I_q` becomes the layer's input; the paper's compensator sits on top of the layer with no model change. Actuator-state feedback is available because we own `θ_a`. |
| Validation against hardware | **Good.** `θ_a`, `θ_s`, `I_q` and `τ_f` are all separately inspectable, being explicit variables in our own code rather than MuJoCo internals. |

## Option D — dual-model staging

Freeze `osl_v2_bench.xml`; have `build_mjcf.py` emit a *second* model with the actuator DOF
(i.e. B in a separate file); implement C as well; cross-validate the two against each other,
with C's closed-form static solutions as the arbiter.

| Criterion | Assessment |
|---|---|
| Physical fidelity | Highest eventually — it ends at B without ever losing the nq = 2 baseline. |
| Numerical stability | Best available, because agreement between a split scheme and a monolithic one *measures* the splitting error rather than assuming it away. |
| Timestep | Unchanged. |
| Code complexity | **Highest.** Two plants, two parameter paths, and a generator change to a protected file. |
| nq = 2 compatibility | Preserved by construction. |
| Future controller work | Best, but no sooner than C delivers it. |
| Validation | Strongest of all four. |

Verdict: the correct long-run shape of this work, and a poor immediate step. Doing D first means
building two integrations before either has been checked against a single number.

## RECOMMENDATION FOR THE NEXT STAGE: Option C

One option, as asked: **C — keep the CAD MJCF unchanged and add an external actuator-dynamics
state layer.**

Four reasons, in order of weight.

1. **It is the only option that leaves every protected file untouched** while still carrying the
   full paper model, including the second coordinate and the current-dependent friction.
2. **It keeps nq = 2, so the oracle stays alive as a test.** "Layer off ⇒ reproduces
   `rms_err_deg = 4.361613372292228` exactly" is a real, falsifiable acceptance criterion that
   B and D cannot offer. An integration that can prove it changed nothing when it should change
   nothing is worth more than an integration with a nicer topology.
3. **B's chief advantage largely evaporates on contact with MuJoCo's linear `stiffness`.**
   Since `ρ` and `f_g|I_q|` need a per-step callback in either case, B's extra cost buys only
   the removal of the splitting error — and the splitting error is now bounded well below the
   physics we are trying to see.
4. **It is reversible.** If the splitting error turns out to matter, C's own numbers become the
   reference for D, and nothing has to be unwound.

What would change this recommendation: if the zero-`B_a` energy-drift test shows drift above a
few percent per second even with sub-stepping, then the splitting error is not controllable at
our timestep and D (with B as the production plant) becomes correct instead.

## The smallest next integration experiment

Deliberately not gait, not a controller, not a sweep. One knee, one step in current, two
assertions:

1. **Null test.** Run the existing AB19 tracking case with the layer instantiated but disabled.
   *Pass:* every metric in `tests/oracle/bench_track_ab19_metrics.csv` reproduces to the last
   digit — `rms_err_deg = 4.361613372292228`, `peak_tau_Nm = 16.004120587370423`. This proves
   the layer is additive before it is asked to be correct.
2. **Static test.** Hold the knee at a fixed angle, command a constant `I_q`, run to steady
   state with the layer enabled. *Pass:* the settled `θ_s` matches
   `deflection_for_joint_torque(τ_j)` from `oslbench/drivetrain.py` to < 1e-6 rad, and `τ_j`
   matches `n_t·τ_a` to < 1e-9 N·m. This checks the coupling sign and the frame conventions —
   the two things most likely to be wrong — against a closed form that is already unit-tested.
3. **Drift check (diagnostic, not pass/fail).** Set `B_a` = `f_c` = `f_g` = 0, displace `θ_s`,
   release, integrate 2 s, and report the drift in `U = p2|θ_s|³/3 + p1θ_s²/2 + kinetic`. This
   *measures* the splitting error instead of estimating it, and decides whether sub-stepping is
   needed.

Nothing in this experiment needs a gait file, a reference trajectory, a gain change, or a
plotting pipeline. If all three come out clean, the integration is sound and gait can follow.
