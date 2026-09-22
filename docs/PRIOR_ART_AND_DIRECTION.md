# Part 4 — Prior art and research direction

**Read §0 first. It changes how you should use everything below.**

---

# §0 — The literature search did not run. Here is the proof.

You asked me to use current web/literature search and not rely on recall. I tried four
independent routes this session and all four failed:

| route | result |
| --- | --- |
| `WebSearch`, query "Open-Source Leg V2 actuator system identification prosthesis" | `400 ValidationException: tool type 'web_search_20250305' is not supported for this model` |
| `web_fetch` → `api.crossref.org` (a metadata API, not a web page) | `cowork-egress-blocked` — allowlist contains exactly one host, `agentrouter.org` |
| delegated agent on Sonnet, instructed to run one search | `503 当前分组 core 下对于模型 claude-sonnet-5 无可用渠道` |
| delegated agent on Haiku, same | `503 ... claude-haiku-4-5 无可用渠道` |

I did not fall back to `curl` or a Python HTTP client, because this environment forbids
routing around a blocked fetch, and because a scraped result I cannot verify would be worse
than an honest gap.

**So there is no literature table in this document, and §1 is not one.** What follows
instead is three things that are genuinely useful without network access:

1. **A recall-based prior-art map with explicit confidence tags** (§1). Every row is
   labelled `[HIGH]`, `[MED]` or `[LOW]` recall confidence, and every row carries the exact
   query that confirms or kills it. Treat it as *a set of hypotheses about the literature*,
   not as the literature.
2. **A search protocol you can execute yourself in about 40 minutes tonight** (§1.3), with
   a decision rule attached to each query so you know what each outcome means.
3. **The part that does not need the internet at all** — a critical assessment of whether
   your question is *well-posed and answerable*, independent of whether anyone has asked it
   (§2–§9). This turns out to be where the real problems are, and it is what I would
   actually spend the conversation with your professor on.

**One consequence you should internalise before tomorrow:** the honest thing to say to your
professor is *"I have a candidate question and a structured prior-art plan; I have not yet
run the search."* That is a perfectly normal state for a project at this stage, and it is
much stronger than presenting a novelty claim you cannot defend when he asks "did you check
X?"

---

# §1 — Prior-art map (RECALL, NOT SEARCH — every row needs verification)

## 1.1 Work I am reasonably confident exists

| # | Work | Yr | Platform | Sim env | HW? | Actuator identified? | Effects modelled | Multi-fidelity compared? | Ctrl perf compared? | Sim validated vs HW? | Contributes | Conf |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 1 | Azocar, Elery, Gabert, Rouse et al., *Design and clinical implementation of an open-source bionic leg*, **Nature Biomed Eng** | 2020 | **OSL v1** | none | yes | benchtop characterisation | torque, impedance, backdrivability | no | no | n/a | the OSL platform itself + benchtop characterisation methodology | HIGH |
| 2 | Elery, Rezazadeh, Nesler, Gregg, *Design and validation of a powered knee-ankle prosthesis with high-torque, low-impedance actuators*, **IEEE T-RO** | 2020 | powered knee-ankle | none | yes | yes — impedance, friction, backdrivability | reflected inertia, friction, transmission | no | partial | n/a | **the closest methodological template for your Phase 3** | HIGH |
| 3 | Tan, Zhang, Coumans et al., *Sim-to-Real: Learning Agile Locomotion for Quadruped Robots*, **RSS** | 2018 | Minitaur | Bullet | yes | yes — motor model fit from data | actuator model, **latency**, inertia | **YES — this is an ablation** | yes | yes | **the structural template for your whole idea, in quadrupeds** | HIGH |
| 4 | Hwangbo, Lee, ... Hutter, *Learning agile and dynamic motor skills for legged robots*, **Science Robotics** | 2019 | ANYmal | RaiSim | yes | yes — *learned* actuator net | full SEA dynamics, learned | partially (net vs ideal vs analytic) | yes | yes | shows a learned actuator model beats analytic; **occupies the "high-fidelity end" of your axis** | HIGH |
| 5 | Camargo, Ramanathan, Flanagan, Young, **J Biomech** — the lower-limb dataset | 2021 | human | n/a | n/a | n/a | n/a | n/a | n/a | n/a | your AB19 reference | HIGH |
| 6 | Best, Welker, Rouse, Gregg, *Data-driven variable impedance control ... adaptive speed and incline*, **Science Robotics** | 2023 | OSL-class knee-ankle | none | yes, amputee subjects | no | n/a | no | yes | n/a | **kills direction D outright** (speed/incline adaptive reference) | HIGH |
| 7 | Embry, Villarreal, Macaluso, Gregg, **IEEE TNSRE** — continuously varying speed/incline kinematics model | 2018 | human | n/a | n/a | n/a | n/a | n/a | n/a | n/a | also kills direction D | HIGH |
| 8 | Colgate & Brown, *Factors affecting the Z-width of a haptic display*, **ICRA** | 1994 | haptics | n/a | yes | n/a | sampled-system passivity | n/a | n/a | n/a | the `b > Kp·T/2 + |Kd|` bound in Part 1 §1.7 | HIGH |
| 9 | Caggiano, Wang, Durandau, Sartori, Kumar, *MyoSuite*, **L4DC** | 2022 | musculoskeletal | MuJoCo | no | no | none — torque actuators | no | no | no | the substrate MyoAssist sits on | HIGH |
| 10 | MyoAssist 1.0 preprint (bioRxiv `10.64898/2026.08.25.746839`) + ICORR `10.1109/ICORR66766.2025.11063089` | 2025-26 | myoLeg + OSL devices | MuJoCo | no | **no — direct torque, no transmission dynamics** | none | no | no | no | **the incumbent. Names actuator dynamics as future work in print** | HIGH |
| 11 | Shamaei, Sawicki, Dollar — knee quasi-stiffness in stance | ~2013 | human | n/a | yes | n/a | n/a | n/a | n/a | n/a | the citation that makes your "10.5 N·m/deg is too stiff" argument quotable | MED-HIGH |
| 12 | Pizzolato et al., *rtosim* / real-time OpenSim IK-ID | 2017 | human | OpenSim | yes | n/a | n/a | n/a | n/a | n/a | **weakens direction A** — real-time biomech pipelines are mature | MED-HIGH |
| 13 | Reznick et al., **Scientific Data** — continuously varying locomotion dataset | 2021 | human | n/a | n/a | n/a | n/a | n/a | n/a | n/a | also weakens D | MED |

## 1.2 The four rows that decide your project, all LOW-to-MED confidence

These are the ones to check first, because each one either kills or clears the direction.

| # | Claim I *suspect* is true | Why it matters | Conf |
|---|---|---|---|
| 14 | **MyoChallenge 2024 (NeurIPS competition track) included a prosthesis locomotion task using a transfemoral leg in MuJoCo.** | If true, a whole competition field has already run OSL-class prostheses in MuJoCo, and "we modelled the OSL in MuJoCo" is not just unoriginal but crowded. It does **not** kill the actuator-fidelity question — competition entries use the provided ideal model, which is precisely the idealisation you'd be interrogating — but it changes how you must position it. | **LOW-MED** |
| 15 | **No published actuator-fidelity ablation exists for a powered prosthesis specifically.** | This is the load-bearing novelty claim of the whole direction, and it is the claim I am *least* able to support. Absence of recall here is very weak evidence. | **LOW** |
| 16 | **There is an established literature in biomechanics on "how much muscle-model fidelity do you need"** (torque-driven vs muscle-driven vs EMG-driven comparisons in OpenSim). | Double-edged. **Good:** it proves the *question type* is publishable and gives you a rhetorical template. **Bad:** a reviewer may read your work as that known move transplanted to a new device. You must be ready to say why the device changes the answer. | MED |
| 17 | **No OSL V2-specific actuator parameter set has been published in a simulation-ready form.** | If true, that is your engineering contribution (§8.2). If false — e.g. the `opensourceleg` library or the V2 design paper publishes reflected inertia and friction — then Phase 3 shrinks from "identification study" to "look up the datasheet", which is *good for you* and bad for the claim. | MED |

## 1.3 The search protocol — run this yourself, ~40 min

Use Google Scholar, IEEE Xplore, and bioRxiv. For each: the query, and **what the outcome
means.**

**Tier 1 — can kill the direction (do these four first):**

1. `MyoChallenge 2024 prosthesis locomotion track` and `MyoChallenge myoLeg transfemoral prosthesis`
   → *If a prosthesis track exists:* read what device model it used. If it is a torque-source
   idealisation, **that is an argument FOR your work, not against it** — say "an entire
   competition optimised controllers against an actuator model that has never been validated
   against hardware." Reframe, don't retreat.
2. `actuator model fidelity sim-to-real transfer prosthesis` / `... exoskeleton` /
   `... assistive device`
   → *If you find a direct hit:* the direction is dead in its general form; narrow to
   "…for the OSL V2 specifically, with identified rather than randomised parameters."
   *If you find only legged-robot hits (Tan, Hwangbo):* **this is the good outcome** — you
   have a template to cite and a domain gap to fill.
3. Resolve the **MyoAssist 1.0 preprint** and read **Limitations / Future Work verbatim,
   in full.** → This is the single highest-value artifact for your project. Also search for
   any follow-up preprint from that group adding actuator dynamics. A follow-up would close
   the gap outright and you need to know tonight, not in six months.
4. `Open-Source Leg v2 design paper Rouse` + browse the `opensourceleg` Python library docs
   and the OSL hardware repo for actuator specs
   → Looking for: rotor inertia, gear ratio, friction, bandwidth, **and the default
   impedance-controller gains with units.** Finding these is *pure upside*: it answers Part 1
   §1.10 items 1–3 for free.

**Tier 2 — shapes the contribution:**

5. `system identification powered knee prosthesis actuator` and
   `Elery Rezazadeh Gregg powered knee-ankle prosthesis T-RO 2020`
6. `reflected inertia effect controller performance geared actuator`
7. `muscle model fidelity comparison OpenSim torque-driven` (the row-16 precedent)
8. `Shamaei Dollar knee quasi-stiffness walking` (for Part 1)

**Search hygiene:** for hits 2 and 3, also read the *citing* papers (Scholar's "Cited by"),
because a 2018 template paper's 2025 citers are where a prosthesis version would live.
Check the last two years of **ICORR, BioRob, ICRA, IROS, IEEE TNSRE, IEEE T-MRB, Wearable
Technologies, Journal of NeuroEngineering and Rehabilitation**.

---

# §2 — The critique that does not depend on the literature

This is the part I can do properly, and it is more important than §1 — because **your
question has two structural problems that would sink it even if it is 100 % unoccupied.**

## 2.1 Problem 1: "minimum fidelity" is not a property of the device

> *"What is the minimum actuator-model fidelity required for controller-development results
> to transfer?"*

Minimum fidelity is not a property of the OSL. It is a property of the tuple

    (device, controller class, task, performance metric, tolerance for "transfers")

Change any element and the answer changes. Model a controller that never reverses direction
and Coulomb friction stops mattering. Operate at 11 % of torque authority — as your bench
does — and the torque–speed envelope is *guaranteed* irrelevant, not discovered to be.
Choose a 5 % error tolerance instead of 1 % and half your effects drop out.

So if you run this with one PD controller on one AB19 trajectory on a fixed-base bench, the
honest title of your result is *"a sensitivity analysis of one controller on one task"* — and
a reviewer will say so. **This is the single biggest threat to the direction, and it is
fixable, but only by design choices made now** (see §3).

## 2.2 Problem 2: the answer may be guessable a priori

Write down what any experienced actuator engineer would predict before you run anything:

| effect | a priori prediction | confidence it's right |
| --- | --- | --- |
| reflected rotor inertia | **matters a lot** — it's 7–49× the placeholder and plausibly exceeds link inertia | high |
| Coulomb friction | matters near zero-crossings and in fine positioning | high |
| viscous friction | matters mildly, mostly absorbed into Kd | high |
| transport latency | matters for high-gain loops, scales with Kp | high |
| torque bandwidth | matters once commanded bandwidth approaches it | high |
| torque–speed limit | **irrelevant here** — you are at 11 % authority | very high |
| transmission compliance | matters above the series resonance | medium |

If your output is a table that matches this column, you have produced a confirmation, not a
finding. **"We already knew that" is the most common fatal referee comment for
sensitivity-analysis papers**, and it is a real risk here.

## 2.3 Problem 3: the fixed base removes the gap everyone cares about

The dominant sim-to-real gap in prosthetics is at the **socket/human interface** and at
**ground contact** — not in the actuator. A reviewer can fairly ask why you studied the
smallest of the three gaps.

There is a good answer, but you have to give it deliberately: *precisely because* the bench
removes the human and the ground, it is the only configuration where the actuator gap can be
**isolated and measured without confounds.** Frame the fixed base as experimental control,
not as a limitation you're apologising for. But you must say it first, before the reviewer
does.

## 2.4 Problem 4: partial models can be worse than no model

This is the most interesting problem and it is the one that rescues the direction.

Adding an *imperfectly identified* effect does not reliably improve prediction. A friction
model with a 30 % error in `f_c` can predict worse than assuming frictionless, because it
introduces a *biased* error where you previously had an *unbiased* one. Fidelity and
accuracy are not the same axis.

That means the monotonic ladder in your diagram — ideal → +inertia → +friction → +latency →
+compliance — encodes an assumption that **may be false, and whose falsity would be a
genuine finding.**

## 2.5 What survives

The version of your question that survives all four problems is *not* "which effects
matter." It is:

> **Does the actuator idealisation change the DESIGN DECISION, not just the predicted
> number?**

That is: if you tune a controller in the idealised simulation and tune the same controller
on the real bench, **do you arrive at different gains?** If ideal-sim says the best gain is
Kp = 600 and the bench says Kp = 150, you have not reported an error magnitude — you have
shown the simulation produces the *wrong design*, which is the only thing a controller
developer actually cares about.

**And you already have a proof of concept for exactly this phenomenon.**

Part 1 showed that an idealisation *in the controller* (the missing `q̇_ref`) caused the
optimisation to select a stiffness **ten times too high** — Kp = 600 instead of Kp ≈ 60 —
while the predicted tracking error looked perfectly reasonable at 4.36°. The error metric
did not reveal the problem; the *selected design* did. Verified to 0.53 % against the frozen
MuJoCo oracle.

That is your thesis in miniature, already demonstrated, already numerically tight. The
proposed project is: **does the same thing happen for plant idealisations, and by how much?**

Be careful and honest about one distinction: Part 1 is a *controller-side* idealisation, not
a *plant-side* one. It is suggestive, not proof. But it is an extremely good slide.

---

# §3 — Overlap map, candidates A–F

## A. Live IMU → gait phase → prosthetic knee simulation

- **Research question:** arguably none. "Can I stream a sensor into a simulator?" is an
  engineering question with a known answer.
- **Prior work:** real-time musculoskeletal pipelines are mature (row 12). Phase-variable
  estimation from a single thigh IMU is a decade of work from Gregg, Rouse and Young.
  **Critically: Sartori and Durandau are co-authors on both MyoSuite and MyoAssist** — this
  is their home territory and they can enter it at will.
- **Genuinely missing:** nothing identifiable.
- **MyoAssist overlap:** high in capability, low in current implementation.
- **OSL actuator-paper overlap:** none.
- **Build:** sensor thread, orientation filter, phase estimator, reference model.
- **Physical:** one IMU.
- **Simulation:** none new.
- **Contribution:** a demo.
- **Difficulty:** medium-low. Well-suited to an undergraduate.
- **Biggest risk:** you finish it and it is a nice demo with no claim attached.
- **Publishable:** **no**, not alone.
- **Verdict: keep as infrastructure, never as the claim.**

## B. OSL actuator system identification

- **Research question:** "What are the OSL V2 knee's dynamic parameters?" — a measurement,
  not a question.
- **Prior work:** the methods are textbook and row 2 is a direct template on a very similar
  device. Row 1 did benchtop characterisation on the v1.
- **Genuinely missing:** plausibly the V2-specific numbers in simulation-ready form
  (row 17, MED confidence — **check the `opensourceleg` library first, it may already be
  published**).
- **MyoAssist overlap:** complementary — they have no transmission dynamics at all.
- **OSL paper overlap:** **high.** This is what rows 1 and 2 do.
- **Build:** identification scripts, a regenerated MJCF.
- **Physical:** ring-down, constant-velocity sweeps, chirp. **Essential.**
- **Simulation:** validation only.
- **Contribution:** a parameter set with fit errors. Useful, citable, not a paper.
- **Difficulty:** medium. Risk is bench access and instrumentation, not intellect.
- **Biggest risk:** the numbers turn out to be published already, or the bench isn't
  instrumented well enough to get them.
- **Publishable:** as a component or dataset, not as a standalone paper.
- **Verdict: necessary enabler. Not the contribution.**

## C. Bench-calibrated OSL simulation

- **Research question:** "Does a calibrated model predict the bench better than an
  uncalibrated one?" — the answer is yes, by construction. **This is the weakest candidate
  because it cannot fail.**
- **Prior work:** model calibration against hardware is standard practice everywhere.
- **Genuinely missing:** nothing conceptual.
- **Overlap:** B + a validation step.
- **Contribution:** "we made a better OSL model" — which you correctly identified as not
  enough.
- **Publishable:** no.
- **Verdict: this is B with a graph. Fold it into B.**

## D. Sim-to-real validation of OSL controllers

- **Research question:** "Do controllers developed in simulation work on the OSL?"
- **Prior work:** sim-to-real validation is routine; the *finding* would have to be a
  specific, quantified divergence.
- **Genuinely missing:** an OSL-specific quantified divergence, plausibly.
- **MyoAssist overlap:** they train in sim and do not, to my recollection, validate on
  hardware — so this is adjacent, not overlapping.
- **Build:** a controller set + a hardware test rig.
- **Physical:** substantial — this is mostly a hardware project.
- **Contribution:** "controller X degrades by Y % on hardware." Narrow, but real and honest.
- **Difficulty:** high, dominated by hardware access.
- **Biggest risk:** bench availability; and that a single divergence number is a short
  paper.
- **Publishable:** marginal alone; **strong as the ground-truth arm of E.**
- **Verdict: not standalone, but it is the component that gives E its teeth.**

## E. Systematic actuator-fidelity / sim-to-real study

- **Research question:** as you wrote it — **and §2.1–2.2 say it is under-specified and
  possibly guessable.**
- **Prior work:** rows 3 and 4 are the template in quadrupeds; row 16 is the template in
  biomechanics. Row 15 (no prosthesis version) is **LOW confidence** and is the claim the
  whole thing rests on.
- **Genuinely missing:** plausibly the prosthesis instance with *identified* (not
  domain-randomised) parameters and a hardware ground truth. Unverified.
- **MyoAssist overlap:** **low, and this is the strongest structural argument you have** —
  their device model is a direct-torque idealisation and they name actuator dynamics as
  future work in print.
- **OSL paper overlap:** low. Rows 1–2 characterise actuators; they do not ask what
  simulation fidelity requires.
- **Build:** identification (B) + a fidelity ladder + multiple controllers + a hardware arm.
- **Physical:** identification *and* the ground-truth runs. Substantial.
- **Simulation:** the full matrix (§7).
- **Contribution:** potentially real — but only in the sharpened form (§2.5).
- **Difficulty:** **high for an undergraduate.** This is the honest assessment. It is a
  master's-thesis-shaped scope.
- **Biggest risk:** the a-priori-guessable answer (§2.2).
- **Publishable:** yes, conditionally — see §9.
- **Verdict: right direction, wrong scope, and the wrong metric as currently framed.**

## F. Narrowly scoped C + D + E

- **This is the recommendation**, but narrowed considerably harder than you are currently
  imagining — see §3 recommendation below and §7.
- **Verdict: yes, with the scope of §6–§7 and the question of §4.**

---

# §4 — Recommended research question

One question, precise, falsifiable:

> **When a trajectory-tracking controller for the Open-Source Leg V2 knee is tuned in
> simulation, which actuator-model idealisations change the *selected gains* — not merely
> the predicted error — relative to tuning the same controller on the physical bench?**

And the secondary question that makes it more than a sensitivity analysis:

> **Is the relationship monotonic — does each added actuator effect move the simulated
> optimum closer to the hardware optimum — or can a partially identified effect move it
> further away?**

Note what changed from your version, and why each change matters:

| your framing | revised | why |
| --- | --- | --- |
| "minimum fidelity required" | "which idealisations change the selected gains" | §2.1 — "minimum" is not well-defined; "changes the design" is |
| "results transfer" | "tuned gains differ" | makes the outcome a measurable number, not a judgement |
| error magnitude as the metric | **the location of the optimum** as the metric | §2.2 — error magnitudes are guessable; optimum shifts are not |
| implicitly monotone ladder | monotonicity as a *hypothesis to test* | §2.4 — non-monotonicity would be the most interesting possible result |

---

# §5 — Why this question matters

Concretely: **it tells a controller developer whether they are allowed to tune in
simulation.**

Everyone building prosthesis controllers in MuJoCo — including everyone using MyoAssist, and
including any MyoChallenge entrant — implicitly assumes that a controller tuned against an
idealised torque source is a good starting point for hardware. Nobody has measured the price
of that assumption for this device class. If the answer is "the idealised simulation selects
gains 10× too stiff," that assumption is actively dangerous: a 10× overstiff gain on real
hardware is not a mild inaccuracy, it is a safety problem on a device attached to a person.

Part 1 already showed one idealisation doing exactly that, with a factor of ten, in a case
where the error metric looked fine.

And it is decision-shaped for the framework's users: *model these two effects, you may omit
those three.* That is directly actionable by anyone extending MyoAssist.

---

# §6 — Minimum experimental setup

**Essential bench measurements** — without these there is no project:

1. **Free-swing ring-down of the shank**, motor de-energised, open-circuit and short-circuit.
   Gives effective inertia *and* separates mechanical from electrical damping.
   *This single measurement produces the reflected-inertia number that Part 1 identified as
   the top unknown (7–49× off).*
2. **Constant-velocity sweeps** at 6–10 speeds, both directions. Least squares on
   `τ = f_c·sgn(q̇) + b·q̇` gives Coulomb and viscous friction with a residual.
3. **Step and chirp on torque command**, output locked. Gives transport delay and the −3 dB
   torque bandwidth.
4. **The ground-truth runs:** the same controller set, same reference, executed on the
   physical bench with logged angle, velocity and torque. **This is what separates your work
   from a simulation study, and it is the part most at risk from bench access.**

**Desirable but not essential:** torque–speed envelope (predictably irrelevant at 11 %
authority — measure it once to *justify* excluding it); transmission compliance via locked-
output frequency response; thermal.

**Instrumentation you must confirm you have before promising any of this:** joint-side
encoder resolution and sampling rate, a torque measurement (or a trustworthy current→torque
constant), and a logging rate of at least 1 kHz. **Ask about this first.** If the bench
cannot log at 1 kHz with a joint-side torque signal, the identification quality caps out and
the whole plan needs rescoping — better to learn that tomorrow than in March.

---

# §7 — Simulation matrix

**Do not run a full factorial.** With six effects that is 64 cells, most uninformative, and
the marginal cells are the ones you can interpret. Run a **cumulative ladder plus leave-one-
out**, which is 2n + 1 cells instead of 2ⁿ.

**Ladder (cumulative):**

| level | model | what it represents |
| --- | --- | --- |
| L0 | ideal torque source, CAD inertia only, `armature = 0` | the MyoAssist-class idealisation |
| L1 | L0 + **identified reflected inertia** | the effect Part 1 flagged as 7–49× wrong |
| L2 | L1 + **identified viscous damping** | |
| L3 | L2 + **identified Coulomb friction** | the first non-smooth effect |
| L4 | L3 + **identified transport delay** | |
| L5 | L4 + **identified first-order torque bandwidth** | |
| L6 | L5 + torque–speed envelope | expected null; include to *prove* it is null |

**Leave-one-out from L5:** five additional cells, each removing one effect, to catch
interactions the ladder hides.

**Plus the row that tests §2.4:** each of L1–L5 re-run with the identified parameter
perturbed by ±1 standard error from the identification fit. If a level's prediction is not
robust to its own identification uncertainty, then "adding that effect" is not actually an
improvement, and that is a finding.

**Controllers — you need more than one, and this is non-negotiable.** Ranking and
optimum-location are meaningless with a single controller. Minimum viable set of four,
which you largely have or can build cheaply:

1. PD regulator, the current law (`−Kd·q̇`) — the existing validated baseline.
2. PD tracker (`+Kd(q̇_ref − q̇)`) — from Part 1.
3. PD tracker + model feedforward — from Part 1.
4. Impedance control with phase-dependent gains — the prosthetics-standard comparison.

**Per cell, per controller, measure:** tracking RMS and peak error; torque RMS and peak;
phase lag; saturation fraction; **and — the primary outcome — the gain pair (Kp*, Kd*) that
minimises a fixed cost function.** The headline result is the distance between (Kp*, Kd*) in
each simulated cell and (Kp*, Kd*) measured on the bench.

Secondary outcome: **Spearman rank correlation** of the four controllers' performance
ordering, simulated cell vs bench. The fidelity level at which the ranking stops changing is
the answer to "minimum fidelity" — and notice that this definition is measurable, unlike the
one in your original framing.

---

# §8 — Keep these three things separate

## 8.1 Existing knowledge — do not claim any of this

- A MuJoCo model or "digital twin" of the Open-Source Leg. Two ship inside MyoAssist's
  `assist_sim` with model resources from Rouse's own group.
- Benchtop characterisation of OSL-class actuators (rows 1, 2).
- Actuator-fidelity ablation as a *method* — rows 3 and 4 did it for quadrupeds, and the
  muscle-model-fidelity literature did it for biomechanics. **You are transplanting a known
  move to a new device.** Say so yourself, in the introduction, and cite them. Claiming the
  method is novel is the fastest way to lose a referee.
- Gait-phase estimation from a thigh IMU; phase-variable prosthesis control.
- Speed/terrain-adaptive reference generation (rows 6, 7, 13).
- PD tracking of a recorded human knee trajectory on a bench — that is a validation step,
  and Part 1 shows the residual there is a property of the control law, not of the leg.

## 8.2 Engineering contribution — useful, defensible, not novel

State these plainly as engineering, and they will be received well:

- The CAD-derived OSL V2 MJCF with per-link inertia tensors from the Onshape export. The
  *provenance* is the distinguishing feature, not the existence.
- An identified actuator parameter set for the OSL V2 in simulation-ready form, with fit
  errors. **Genuinely useful to others, and citable.**
- The live-sensor interface and the real-time visualisation.
- The refactored, oracle-validated pipeline.

## 8.3 Potential research contribution — one thing only

> An experimental determination of **which actuator idealisations change the controller
> design selected in simulation** for a powered knee prosthesis, with identified rather than
> randomised parameters, validated against the physical device.

Conditional on §1.3 Tier 1 coming back clean.

## 8.4 Tempting claims to avoid

| tempting | why it fails |
| --- | --- |
| "the first digital twin of the OSL" | two MuJoCo OSLs already exist |
| "novel actuator fidelity framework" | rows 3, 4, 16 |
| "real-time human-in-the-loop prosthesis simulation" | term of art (Zhang 2017) means online optimisation with a real person; and a fixed base cannot interact |
| "we improved sim-to-real transfer" | you would be *measuring* the gap, not improving it — a much more defensible and honest verb |
| "no prior work exists" | you haven't searched yet, and absence of recall is not absence |
| "our model is more accurate than MyoAssist's" | different purposes; this reads as territorial and invites a hostile review |

---

# §9 — Confidence

**MEDIUM.** Not high, not low, and the split is worth understanding precisely.

**What I am confident about (high):**

- The *question type* is publishable — rows 3, 4 and 16 prove the format is accepted.
- The *gap in MyoAssist specifically* is real, because its authors name actuator dynamics as
  future work in print. That is the most defensible single sentence you have.
- The *engineering* contributions (§8.2) are solid regardless of how the literature lands.
- **Part 1 is a genuine, verified, in-hand demonstration that an idealisation can shift a
  design decision by 10× while the error metric looks fine.** This de-risks the direction
  more than anything else you have, and it is already done.

**What I am not confident about (low), and why it holds the overall rating down:**

- **Whether row 15 is true.** The claim "no prosthesis actuator-fidelity ablation exists" is
  the load-bearing novelty claim, and it rests entirely on my failure to recall one, with no
  search performed. This is the weakest link in the entire argument.
- **Whether the answer will be interesting** (§2.2). There is a real chance the result is
  "inertia and latency matter, as everyone expected." The optimum-shift framing in §4
  mitigates this substantially but does not eliminate it.
- **Whether the scope fits an undergraduate timeline.** Identification + a 13-cell matrix +
  four controllers + a hardware ground-truth arm is, honestly, master's-thesis-shaped. §6's
  measurements 1–3 plus the L0/L1/L3 ladder on two controllers would be a credible reduced
  version, and I would open with that rather than the full matrix.

**What moves this to HIGH:** §1.3 Tier 1 queries 2 and 3 come back clean, *and* a pilot on
the existing simulation shows the tuned optimum moving materially between `armature = 0.01`
and `armature = 0.25`. **That pilot needs no hardware and could be done in a day** — it is
the cheapest possible test of whether the whole direction has a pulse, and I would propose it
to your professor as the immediate next step.

**What moves this to LOW:** a direct hit on query 2 (an existing prosthesis or exoskeleton
actuator-fidelity ablation), or a MyoAssist follow-up preprint that adds actuator dynamics,
or learning that bench access is limited enough that the hardware ground-truth arm cannot
happen. **Without the hardware arm this is a simulation sensitivity study, and it drops from
"potential research contribution" to "good engineering."**

---

## What to actually say tomorrow

Four sentences, in this order:

1. *"The 4.36° tracking error was 99.5 % explained by a missing velocity-reference term, and
   fixing it drops the required stiffness from 600 to 60 N·m/rad — so the idealisation in
   our controller was selecting a design that was ten times too stiff while the error metric
   looked fine."*
2. *"That suggests a research question: which idealisations in the actuator model change the
   controller we would select, not just the number we predict?"*
3. *"I have not run the literature search yet — I have a structured protocol and four
   queries that would kill or clear it, and I will have that answer this week."*
4. *"Before any hardware, I can test whether the direction has a pulse in one day: sweep the
   armature parameter across its plausible range and see whether the tuned optimum actually
   moves."*

Then ask him the three questions from Part 1 §1.10 — rotor inertia and gear ratio, the real
controller's gains and units, and whether the bench can log joint-side torque at 1 kHz. Those
three answers determine everything downstream, and they cost him thirty seconds each.
