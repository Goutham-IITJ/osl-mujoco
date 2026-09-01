# MyoAssist transfemoral OSL: verified findings

Read from the actual repos in `myo_folder/` on 2026-09-01. Everything here is
file-level verified — the earlier `MYOASSIST_PIVOT.md` marked much of this
UNVERIFIED because the sandbox had no network. Where the two documents disagree,
**this one wins**.

## The composition seam is even cleaner than the paper implied

`assist_sim` is a thin, well-documented layer whose entire public API is:

```python
from assist_sim import load_combined
model, data = load_combined("myolegs", "OpenSourceLeg_KA_L1", export_xml="out.xml")
```

or from the CLI:

```bash
python -m assist_sim combine myolegs OpenSourceLeg_KA_L1 -o combined.xml
python -m assist_sim list
python -m assist_sim validate myolegs OpenSourceLeg_KA_L1
```

Its README states the contract plainly: *"`assist_sim` takes a baseline MSK model
and a device that YAML describes. It applies the prosthetic surgery (body removals,
tendon edits, mesh swaps). Then it attaches the device. It returns a compiled
`MjModel` ... `assist_sim` keeps the baseline MSK model on disk unchanged."*

Device keys are discovered by globbing `assist_sim/models/*/[NAME]config.yaml`, so
the real key is **`OpenSourceLeg_KA_L1`** (directory + config stem). Every device
works with all four MSK keys (`myolegs22`, `myolegs26`, `myolegs`, `myofullbody`) —
the README carries a compatibility matrix with ✓ in all four columns for both OSL
rows, and each pair has a frozen smoke signature in CI.

**Option B is not a fork — it is a supported extension point.** Adding our CAD
device means dropping a directory into `assist_sim/models/OSLV2_CAD/` with a
`L1config.yaml` and a model XML, and the registry picks it up automatically.

## The amputation is a 348-line declarative surgery spec

`KA_L1config.yaml` is far more thorough than anything we would have written by
hand. It is worth reading in full, but the structure is:

| section | what it does for `myolegs` |
|---|---|
| `body_removals` | `tibia_r` (cascades to talus/calcn/toes) **and** `patella_r` — the comment notes patella is a *sibling* of tibia so the cascade misses it, and if left it becomes "3 unconstrained DOFs floating inside the socket" |
| `mesh_replacements` | `femur_r` mesh → `osl_femur_trans_r` (residual-limb geometry) |
| `geom_removals` | `femur2_col_r`, a collider that spanned 0.174 m *past* the cut plane with `contype=1` |
| `body_overrides` | `femur_r` mass **6.06705 kg = 72.23 % of the intact 8.4 kg**, with re-derived `diaginertia`, `ipos`, `iquat`. Comment: without this "the prosthetic side outweighs the intact leg" |
| `actuator_removals` | 15 muscles (`bfsh`, `edl`, `ehl`, `fdl`, `fhl`, `gaslat`, `gasmed`, `perbrev`, `perlong`, `soleus`, `tibant`, `tibpost`, `vasint`, `vaslat`, `vasmed`) |
| `tendon_removals` | the matching 15 tendons |
| `tendon_modifications` | **the impressive part** — 8 hip-spanning muscles are surgically *re-anchored* rather than deleted, moving every wrap point at or distal to the cut plane `y = -0.283012`, including wrap-cylinder sidesites |
| `actuator_overrides` | re-derived `lengthrange` for each re-anchored muscle, because the compiler keeps authored length ranges and `recfem_r` would otherwise "describe a path it no longer has" |
| `attachments` | `device_body: osl_generic_socket` → `parent_body: femur_r` |
| `actuators` | the two OSL motors |
| `keyframe_overrides` | `stand`, `walk_left`, `walk_right`, `squat`, `lunge` |

The femur cut plane is **`y = -0.283012`** and the config retains **69.98 %** of
segment length on the 80-muscle lineage. Note the axis convention: this is a
**y-down** model, not our z-up device frame — relevant when we emit our device.

## The headline finding: their socket is RIGID, ours is not

`KA_L1model.xml` contains exactly **two joints** — `osl_knee_angle_r` and
`osl_ankle_angle_r`. There are no socket DOFs, and `attachments` adds none. So the
current `assist_sim` transfemoral OSL bolts the socket **rigidly** to the residual
femur. This matches the 1.0 paper's caveat about "rigid human-device attachments".

The published 4-DOF socket is in the **legacy** model,
`myo_sim/models/legacy/osl/assets/myolegs_osl_chain.xml`, which has:

```xml
<joint name="socket_piston"     type="slide" range="-0.035 0.035"      stiffness="20000" damping="10000"/>
<joint name="socket_rotation_1" axis="0 0 1" range="-0.122173 0.122173" stiffness="350"   damping="175"/>
<joint name="socket_rotation_2" axis="1 0 0" range="-0.122173 0.122173" stiffness="350"   damping="175"/>
<joint name="socket_rotation_3" axis="0 1 0" range="-0.122173 0.122173" stiffness="350"   damping="175"/>
```

One prismatic plus three rotational, spring-damped, ±7° on the rotations. **This is
structurally the same socket we built by hand** (piston + 3 rotations, passive,
spring-damped) — we chose ±10°/±15° and ±0.02 m where they use ±7° and ±0.035 m,
and their stiffness values are grounded in LaPrè 2018 while ours were invented.

So the socket is the one place where our Phase 2 work is *ahead of* the current
official device, and where their own numbers can replace our guesses. That is a
concrete, creditable contribution: port the legacy 4-DOF socket into a modern
`assist_sim` device config.

## Device comparison, from the actual XML

Their device is 7 bodies but only **3 moving segments** (socket + femoral pylon +
proximal adapter are rigidly linked; knee assembly + tibial pylon + ankle assembly
are rigidly linked; foot assembly moves with the ankle) — the same segment count
as ours.

| | official `OpenSourceLeg_KA_L1` | our CAD-derived OSL V2 |
|---|---|---|
| meshes | 5 device STLs, per *assembly* (`osl_knee_v2.stl` 6.0 MB, `osl_ankle_v2.stl` 5.7 MB) | 260 STLs, per *link*, from the Onshape export |
| bodies / moving segments | 7 / 3 | 3 / 3 |
| socket mass | 0.849991 kg | not modelled (we used a 0.6 kg placeholder) |
| knee assembly | 2.245500 kg | shank 3.590758 kg |
| ankle assembly | 1.739000 kg | — |
| foot | 0.291000 kg | 0.915159 kg |
| pylons + adapter | 0.251983 kg | knee_prox 0.449846 kg |
| **total** | **5.377474 kg** (exactly the published 5.377) | **4.955763 kg** |
| total minus socket | 4.527483 kg | 4.955763 kg → **ours is 0.428 kg heavier** |
| knee ROM | `0 2.0944` rad = 0–120° | CAD-derived |
| ankle ROM | `-0.5236 0.5236` rad = ±30° | CAD-derived, +20° dorsi in our sweep |
| knee motor | `motor`, `gainprm 49.4`, `ctrlrange ±2.88`, `gear 1` → **142.3 N·m** peak | position servo, `forcerange ±142.2 N·m`, `kp=60` placeholder |
| ankle motor | `gainprm 58.4`, `ctrlrange ±2.88` → **168.2 N·m** peak | — |
| foot contact | 3 capsules, `class="coll"`, `contype=1 conaffinity=0`, group 5 | blade mesh at the sole + 4 bounding boxes, 5.9 mm strict sweep margin |
| socket DOFs | **0 (rigid)** | 4 passive, spring-damped |
| load sensing | 2 sites: `r_socket_load_force`, `r_osl_load_force` | 6-axis load cell + 2 IMUs |
| inertia provenance | parameter-matched to hardware totals | per-link, from CAD geometry × density |

Their masses reproduce the published 5.377 kg **exactly** because the model *is*
the published parameter set. Ours is an independent derivation from geometry, and
the 0.43 kg gap (excluding sockets) is the interesting number — it is a real
cross-check, and it is the kind of thing worth a slide.

Note their mesh names are all `_v2` — so their geometry is also OSL V2, just lumped
per assembly rather than per link. Our fidelity advantage is *segmentation and
inertia provenance*, not raw mesh resolution.

## Verified install constraints

```
myo_sim      requires-python >=3.10   mujoco>=3.4              version 0.2.2
assist_sim   requires-python >=3.10   mujoco>=3.4,<3.12        version 1.0.0
                                      PyYAML>=6.0.2, numpy>=1.24, myo-sim>=0.2.1
myoassist    (RL/CO framework)        mujoco>=3.4,<3.5   <-- much tighter
                                      gymnasium==0.29.1, numpy==2.2.6,
                                      dm-control==1.0.36, myosuite==2.8.4,
                                      stable-baselines3, cma==4.0.0
```

Your `mujoco>=3.4,<3.12` recollection was right, and the ceiling is documented as
*measured*: "3.4 through 3.11 are verified green; the `mujoco-range` job ... tests
both ends on every PR". So **our existing 3.12.0 is excluded by exactly one minor
version**, exactly as suspected.

Two things that would have cost an afternoon each:

1. **`myoassist` needs `uv`, not pip.** From its `requirements.txt`: *"Install with
   `uv pip install -e .`, not plain pip. MyoSuite 2.8.4's metadata pins
   mujoco==3.1.2 and dm-control==1.0.16, but it runs on mujoco 3.4 ... The
   `[tool.uv] override-dependencies` in pyproject.toml relaxes those two pins so uv
   resolves the whole stack in one command; plain pip cannot."*
2. **The PoC does not need `myoassist` at all.** `myo_sim` + `assist_sim` is enough
   to compose the human + device and export the XML — no MyoSuite, no gymnasium, no
   torch, no `uv`. Only bring in `myoassist` when we want RL or the CMA-ES
   controller optimisation.

Flat ground is free: composed models include `myo_sim/models/scene/myosuite_scene.xml`,
which carries `<geom name="floor" type="plane" condim="3" contype="1"/>`. The
`myoassist.terrains` package is only needed for slopes, stairs and rough terrain.

## Revised recommendation

Unchanged in shape — **A now, B next** — but B is cheaper than I estimated, and B
has a specific first deliverable that is genuinely ours:

- **A (today):** `myo_sim` + `assist_sim` in a separate venv, run the PoC, show the
  render and the export. Nothing in `osl-mujoco` is touched.
- **B1:** port the legacy 4-DOF spring-damped socket into a modern device config,
  since the current official device is rigid and the legacy numbers are published.
- **B2:** emit our CAD device as `assist_sim/models/OSLV2_CAD/L1config.yaml` +
  model XML, mind the **y-down** frame convention, and compare the two devices under
  the same human and the same task.

The measurement in B2 — same human, same terrain, same controller, two devices, one
parameter-matched and one CAD-derived — is a result neither group has published.
