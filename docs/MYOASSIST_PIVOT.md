# Should we pivot the human side of the walking sim to official MyoAssist?

Investigation dated 2026-09-01. **No code has been changed for this document.** The
handmade Phase 2 scaffold is untouched and still generates.

---

## 0. Read this first: what is verified and what is not

I could not read a single file from `myo_sim`, `assist_sim`, `myoassist` or
`myoassist.terrains`. This session has **no network egress at all** — `github.com`,
`raw.githubusercontent.com`, `api.github.com` and `myoassist.neumove.org` are all
refused by the gateway (`cowork-egress-blocked`), web search is not provisioned, and
`curl` from the Linux sandbox gets a 403 from the proxy. So every claim below is
sourced from one of three places, and labelled:

- **[1.0]** — the preprint you just uploaded: Robbins, Son, Tan, Wang, van Kanten,
  Sartori, Durandau, Kumar, Caggiano, Song, *"MyoAssist 1.0: An Open-Source Framework
  for Neuromechanical Simulation of Physical Human-Device Interaction"*, bioRxiv
  `10.64898/2026.08.25.746839`, posted 2026-08-26.
- **[0.1]** — Tan et al., *"MyoAssist 0.1: MyoSuite for Dexterity and Agility in
  Bionic Humans"*, ICORR 2025, DOI `10.1109/ICORR66766.2025.11063089`. **This is the
  paper that documents `myoOSL` numerically** and is the more useful of the two for us.
- **[LOCAL]** — measured in this repo.
- **UNVERIFIED** — file-level facts I could not retrieve. Not guessed at.

The papers are written by the repo authors, so for *architecture* they are as
authoritative as the code. What they cannot give is filenames, YAML keys, env IDs and
version pins. **To unblock those, clone the four repos into this folder (or any
connected folder) and I will read them offline** — that is the cheapest next action
and it costs you one `git clone` per repo.

---

## 1. The canonical transfemoral OSL combination

**`myolegs` (80 muscles) is the right MSK model, and your reasoning was correct.**
The chain of evidence, all verbatim:

- "The original myoLeg contains 28 DOFs and 80 muscle-tendon units. It was converted
  from the OpenSim full body model via MyoConverter" **[0.1 §IV-A]**
- "The myoLeg was modified to represent a transfemoral amputation at a 50% level."
  **[0.1]**
- "The OSL is integrated with myoLeg ... with **54 muscles (40 in the intact leg and
  14 in the hip joint of the amputated leg)**" **[0.1]**
- "Building on the **80-muscle myoleg model used in MyoAssist 0.1**, the current
  release adds a 26-muscle lower-limb model and a 416-muscle full-body model"
  **[1.0 §III-A]**

80 → amputate → 54 (40 + 14). That is the published myoOSL.

| model | muscles | what it is | verdict for us |
|---|---|---|---|
| `myolegs` | 80 | full-body skeleton with articulated, fully muscled legs; from Rajagopal 2016 via MyoConverter | **use this** — the only one that reproduces published myoOSL |
| `myolegs26` | 26 | reduced lower-limb model, new in 1.0 | later, for speed. Cannot reproduce myoOSL |
| `myofullbody` | 416 | full-body musculature, from *MuscleMimic* (Li et al. 2026) | no — cost with no benefit for a leg prosthesis |

One correction to how we have been describing it: **`myolegs` is not a
"bilateral lower-limb model"**. [0.1] calls it "a full-body model featuring an
articulated leg" — a full-body skeleton whose *musculature* is confined to the legs.
Minor, but it matters when we reason about the trunk and where the pelvis gets its mass.

## 2. What I could and could not inspect

Everything in this section is **UNVERIFIED** as file content: `KA_L1config.yaml`,
`KA_L1model.xml`, `A_L1*`, the `models/` listing, the composition source, the terrain
module, the examples, and every `pyproject.toml`.

Two things I *can* corroborate about the naming. [1.0] Fig. 2 lists exactly two OSL
entries among the 15 devices:

- **OSL Ankle** — powered prosthetic ankle — 2.2 kg — 29 N·m cont. / 145.25 N·m inst.
- **OSL Knee+Ankle** — powered prosthetic knee & ankle — 5.2 kg — same torques

So `A` = Ankle (transtibial) and `KA` = Knee+Ankle (transfemoral) is near-certain.
**`L1` I cannot decode** — neither paper explains it; plausibly a level or version
index, but that is a guess and I will not dress it up as a finding.

Your description of `KA_L1` — that it creates the transfemoral setup and attaches the
OSL to the residual femur — is **corroborated in substance, unverified in filename**.
Nothing in either paper contradicts any part of it.

## 3. What the composed environment actually contains

**The composition mechanism is MuJoCo's `mjSpec` API** — not dm_control, not XML
string surgery, not a pre-baked monolith **[1.0 §III-D]**:

> "This composition is implemented through MuJoCo's `mjSpec` API, allowing MyoAssist
> to construct compiled MyoAssist simulation environments from modular components
> rather than requiring a separate hand-built model for each use case."

And the device interface — **this is the single most important sentence in this
document for us**:

> "Device integration is specified through a MuJoCo geometry XML together with a YAML
> configuration file. The YAML file defines the device attachment sites on the
> musculoskeletal model, any added joints or actuators, optional overrides to baseline
> model properties, and initial poses. The selected musculoskeletal, device, and
> task-scenario components are then merged into a single compiled MuJoCo model, which
> can also be exported to XML for inspection or reuse."

Amputation is done **at compose time**, not shipped as a pre-amputated human:

> "In prosthesis environments, the relevant biological segments are removed or
> modified, residual-limb geometry is introduced, and the prosthetic device is
> attached in place of the removed anatomy."

The repo division of labour **[1.0 §II-B]** — note the composition code lives in
`myoassist`, *not* `assist_sim`, which is why `assist_sim` alone would never have
answered this:

> "`assist_sim`, which contains assistive-device models; `myoassist`, which contains
> the reinforcement-learning and controller-optimization frameworks, configuration
> files, examples, and utility code; and `myoassist.terrains`, a modular procedural
> terrain generator."

The chain, as published **[0.1]**: 23 internal DOFs, of which 19 are actuated by
54 muscles plus 2 OSL actuators, and **"the remaining 4 DOFs between the residual limb
and the OSL represent socket movement and are passively constrained"** (citing
LaPrè 2018).

## 4. Does their model already solve what we built by hand? Yes — all of it

| what we hand-built | what MyoAssist has | ours vs theirs |
|---|---|---|
| pelvis as a welded 10 kg box | full MSK pelvis + trunk, muscle-driven | theirs, decisively |
| residual thigh: 3.5 kg capsule, 0.20 m | femur cut at 50%, **inertial properties readjusted** for the residual limb, knee/ankle muscles on that side removed | theirs |
| socket: 4 passive spring-damped DOFs | 4 DOFs, "passively constrained" (LaPrè 2018) | **same abstraction, same count** |
| OSL attachment: rigid bolt | device attached via YAML-declared attachment sites | theirs (declarative) |
| contralateral leg: one lumped 12 kg capsule + box foot | fully muscled intact leg, 40 muscles | theirs, decisively |
| contact | GRF sensing, foot-contact reward terms | theirs |
| terrain | flat floor only | flat, rough, hills, stairs, obstacles; procedural; 3 m × 100 m with difficulty ramping **[0.1]** | theirs |
| walking task | none — we had no driver | target-velocity maps; profiles uniform / sinusoidal / step-changing / randomized | theirs |
| sensors | IMUs, load cell, joint sensors | 365-dim obs incl. **3-axis socket force**, 2 GRF, 100-point terrain height map **[0.1 Table II]** | comparable; ours has richer device-side IMU |
| prosthetic actuators | 2 position servos | 2 torque actuators + **4-state impedance controller** baseline (early/late stance, early/late swing) **[1.0]** | theirs |

**Conclusion: on the human side our scaffold is strictly inferior and should not be
developed further.** You called this correctly. The one thing that survives with
credit is the socket abstraction — we independently chose 4 passive DOFs, which is
exactly the official DOF budget.

## 5. Their OSL device vs our CAD-derived OSL V2

| | official MyoAssist OSL (KA) | our CAD-derived OSL V2 | who wins |
|---|---|---|---|
| geometry fidelity | detailed device meshes in renders; **mesh format/decimation UNVERIFIED** | Onshape V2 export, per-link meshes, visual-origin trap resolved | likely **ours** |
| mass | 5.377 kg, parameter-matched to hardware **[0.1]**; 5.2 kg in **[1.0]** Fig. 2 | **4.9558 kg**, summed from CAD densities (two motors absent) **[LOCAL]** | different *kinds* of number — see note below |
| COM | not published | per-link, derived from meshes + densities | **ours** |
| inertia | "readjusted"/matched at body level | full tensors per link from CAD | **ours** |
| knee ROM | 0–120° **[0.1]** | CAD-derived limits | tie |
| ankle ROM | −30…+30° **[0.1]** | CAD-derived, +20° dorsi limit in our sweep | theirs is the hardware spec |
| torque limits | knee 142.2 / ankle 168.2 N·m peak **[0.1]**; 29 cont. / 145.25 inst. **[1.0]** | knee `forcerange` ±142.2 N·m (we already cite [0.1]) | theirs (spec source) |
| socket model | 4 passive DOFs, published | 4 passive DOFs, unfitted stiffness/damping | theirs (grounded in LaPrè) |
| actuator model | direct joint torque, **no** motor/transmission dynamics | position servo, `kp=60` placeholder | neither — open work for both |
| contact geometry | not published | blade mesh at sole + 4 bounding boxes, 5.9 mm strict sweep margin | **ours** |
| sensors | 3-axis socket force, GRF | 6-axis load cell, 2 IMUs, touch, per-joint | **ours** |
| human model | 54-muscle amputated myoLeg | 4 placeholder primitives | **theirs, by a mile** |
| walking env | terrains + tasks + controllers | none | **theirs** |

Two traps to avoid when quoting numbers:

1. **5.377 kg is not the same kind of number as our 4.9558 kg.** Theirs is
   *parameter-matched to the published hardware mass*; ours is *derived from CAD
   geometry and densities* with two motors absent from the export. The gap is a
   finding, not an error. Also note [1.0] now publishes **5.2 kg** for the same
   device, so `build_mjcf.py`'s comment citing 5.377 kg should gain a second line.
2. **The two torque figures are different quantities.** 142.2/168.2 N·m peak per joint
   **[0.1]** vs 29 N·m continuous / 145.25 N·m instantaneous **[1.0]**. Do not average
   or reconcile them.

The device fidelity caveat in [1.0] is effectively an invitation to do exactly what we
want: "The simplified rigid-body, direct-torque representation provides a practical
starting point ... while allowing more detailed human-device interface models,
actuator dynamics, and device-specific controllers to be incorporated as the framework
matures." The acknowledgments also thank "Elizabeth Wilson and Elliott Rouse
(Open-Source Leg)" for "sharing model resources, specifications, and clarifications" —
so their OSL device is vendor-blessed, which is a reason to trust their *specs* even
while preferring our *geometry*.

## 6. Recommended architecture: **A now, B as the goal** — sequenced, not either/or

You suspected B is the long-term goal. Agreed, and the `mjSpec` + (XML, YAML) device
contract is what makes B realistic rather than a fork. But B should not be attempted
first, because the one thing that could force rework in our MJCF — the attachment-site
naming convention in the YAML schema — is precisely what I could not read.

- **Step A (do now).** Stand up the official environment unmodified: `myolegs` +
  transfemoral OSL + flat terrain. Our CAD model stays exactly where it is, as the
  high-fidelity parallel reference. Deliverable: a screenshot of a two-legged
  transfemoral human standing on terrain, and a printed model summary.
- **Step B (after reading the YAML schema).** Emit a *device-only* MJCF from our
  CAD pipeline and register it as a new device against the same human/socket/terrain/
  task framework. Then the comparison A-vs-B becomes a measurement — same human, same
  task, two devices — which is a far stronger result for a paper than either alone.

The decisive advantage of this ordering: **A costs nothing and cannot break anything**,
and it produces the artifact your professor asked for (a realistic walking model with
a hip and an intact limb) in the shortest path. B then becomes an incremental upgrade
with a built-in control condition.

## 7. Environment plan — and a real risk to your existing venv

**[LOCAL]** Current `.venv`: Python 3.10 (cp310 wheels), `mujoco 3.12.0`,
`numpy 2.2.6`, `glfw 2.10.2`, and **no** gymnasium / torch / stable-baselines3 /
myosuite. Nothing MyoAssist-related is installed anywhere on disk.

The `mujoco >=3.4,<3.12` pin you quoted is **UNVERIFIED** — I could not read their
`pyproject.toml`. But if it is real, **3.12.0 is excluded by exactly one minor
version**, so a naive `pip install` into the existing venv would *downgrade MuJoCo
underneath our working CAD model*. That is the concrete reason to keep environments
separate, and your instinct to do so was right.

Plan (no commands run yet):

1. Clone the four repos into a **sibling** folder, not inside `osl-mujoco`, so our git
   history stays clean: `myo_sim`, `assist_sim`, `myoassist`, `myoassist.terrains`.
   Then point me at them and I will read the real pins and write the exact install
   commands instead of guessing.
2. Create `.venv-myoassist` (separate interpreter, separate `site-packages`). Do not
   activate it in the same shell as the OSL venv.
3. Install `myoassist` per its own README and let it resolve its own MuJoCo — do not
   pin MuJoCo by hand.
4. Verify: `python -c "import mujoco; print(mujoco.__version__)"` in each venv, and
   confirm the OSL venv still reports 3.12.0 afterwards.
5. Note [1.0] archives "A fixed snapshot of the software corresponding to MyoAssist
   1.0 ... as version 1.0.0" on Zenodo — a single download that may bundle everything
   and pin a consistent set. Prefer it if the repos disagree.

Windows support is **UNVERIFIED** — neither paper mentions Windows, Linux or WSL.
Budget for the possibility that training needs WSL even if rendering does not.

## 8. Minimal proof of concept

Load `myolegs` + transfemoral OSL + flat terrain, and prove, with a render and a
printed summary rather than assertions: the human appears; both legs exist; the right
leg is transfemoral; the OSL is attached to the residual femur; OSL knee and ankle
joints are present; the model compiles; terrain and contact exist. Then export the
composed model to XML (the paper says this is supported) — **that exported XML is the
most valuable artifact of the whole exercise**, because it shows us their attachment
convention in concrete form and de-risks step B completely.

## 9. How walking is actually driven — we do not need to invent it

Supplied for free **[1.0]**: a **reflex-based neuromuscular locomotion controller
based on Song and Geyer**, 3-D and sagittal variants, generating muscle excitation
from time-delayed sensory feedback (muscle force, joint state, ground contact);
**CMA-ES** for controller-parameter optimization with a ranked 3-stage cost;
**Stable-Baselines3 PPO** with vectorized envs, compositional human/device actors
sharing a critic, and left-right symmetry options; a **4-state impedance controller**
for the powered prostheses; reference data plus imitation reward terms; target-velocity
maps; and standardized evaluation output (videos, reward curves, gait kinematics and
kinetics, muscle activations, sensor traces, device torque profiles).

Note the action space changed between releases: 0.1 exposed a discrete 5-way
controller-mode selector alongside 54 muscle actions, whereas 1.0 treats device
actuation as separable action components. Any older tutorial you find describes the
0.1 contract.

**So: point 10 of your brief is satisfied.** No sinusoidal hip trajectory, no
arbitrary point mass, and no prescribed pelvis motion is needed — locomotion is
muscle-driven by a published controller. Our welded-pelvis-plus-prescribed-trajectory
plan is now unnecessary, which is the single biggest saving from this pivot.

What remains ours to build: **IMU → gait-phase estimation → motor control** on the
device side (their prosthesis baseline is an impedance FSM keyed on load and joint
state, not an IMU gait-phase estimator), actuator and transmission dynamics, and
validation against real hardware.

## 10. Phase 2 audit — what dies, what survives **[LOCAL]**

Nothing has been deleted. Full triage against `HEAD` (single commit `e19a051`):

**Fully obsolete if we pivot** (both untracked, so removal is clean):
`models/osl_v2_walk.xml`, `models/osl_v2_walk.pred.json`.

**`tools/build_mjcf.py`** — +417 / −72 lines total, of which roughly **270 are
scaffold-only**: the anthropometric constants at lines 300–366 (`BODY_MASS`,
`PELVIS_*`, `HIP_HALF_WIDTH`, `RES_THIGH_*`, `SOCKET_*`, `HIP`, `CONTRA_*`),
`_emit_walk` (1218–1325), `_walk_controls` (1326–1366), `_walk_predictions`
(1560–1600), and the small `scene == "walk"` hooks in `build()` (1369–1435),
`predictions()` (1509–1510) and `main()` (1640).

**Survives, and is actively useful for option B** — this is the good news:

- **`_emit_device` (1167–1217) is shared by all three scenes.** Deleting the scaffold
  does not touch the device path at all. This refactor decoupled device emission from
  scene assembly, which is *exactly* what emitting a device-only MJCF for MyoAssist
  requires. It was not wasted work; it was a prerequisite.
- `_box_diag` / `_cyl_diag` / `_prim_inertial` (1145–1166) — generic primitive-inertia
  helpers, reusable if we ever need residual-limb geometry on the YAML side.
- **The Phase 1 `thigh` → `knee_prox` rename is now vindicated.** Under MyoAssist the
  thigh/femur belongs to the *human*; a device body called "thigh" would have been an
  active liability in a composed model.

**`tools/validate_mjcf.py`** — +40 / −9. Only the capsule bounding-box block (~12
lines) is scaffold-specific. The other two edits become *more* valuable under
MyoAssist, not less: measuring joint anchors **relative to the device root** rather
than to world is precisely what is needed to confirm our CAD geometry survived being
nested inside someone else's model, and splitting **device mass from total mass** is
what lets us range-check 4.96 kg of device inside a 75 kg human.

**Untouched by a pivot:** `models/osl_v2_bench.*`, `models/osl_v2_ground.*` (their
diffs are the rename and nothing else), and all the CAD/collision/flat-foot machinery.

**README** needs revision at lines 13, 26–46, 52, 61–62, 261–276, 332, 348 and
404–411, all of which describe the scaffold as the way forward.

---

## Summary

- **A.** Repo is healthy: CAD device twin validated in three scenes, all passing;
  one commit, everything else uncommitted; the walk scaffold is the only part in doubt.
- **B.** Reuse from MyoAssist: `myolegs` (80-muscle) amputated human, the transfemoral
  socket, `myoassist.terrains`, the composition pipeline, the reflex + CMA-ES + PPO
  controller stack, the 4-state prosthesis impedance baseline, and the task/reward
  definitions.
- **C.** Retain from ours: CAD meshes, per-link mass/COM/inertia, joint anchors and
  axes, the collision sweep and flat-foot derivation, the sensor suite, and the
  validation harness.
- **D.** Architecture: **A now** (official env unmodified, ours as parallel reference),
  **B next** (our device inside their framework via the documented XML+YAML seam).
- **E.** Separate `.venv-myoassist`; never pip into the existing `.venv`; read their
  real pins before installing.
- **F.** PoC: `myolegs` + transfemoral OSL + flat terrain, rendered, then **export the
  composed model to XML**.
- **G.** Ours to build: IMU → gait phase → motor control, actuator/transmission
  dynamics, hardware validation.

**Nothing is approved and nothing is modified.** The one thing blocking a firm
commitment to B is the device YAML schema, which needs the repos on disk.
