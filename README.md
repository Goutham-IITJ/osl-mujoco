# OSL V2 in MuJoCo

A physics model of the Open-Source Leg V2 (Neurobionics / University of Michigan),
built directly from the official Onshape CAD export and driven from plain Python.
No ROS, no Gazebo, no placeholder geometry.

The point of this repository is to have a trustworthy digital twin *before* the
real hardware is touched, so that motor control, IMU processing and gait-phase
estimation can be developed and broken safely.

## What you get

Two scenes are generated into `models/`. `osl_v2_bench.xml` welds the leg to the
world, which is the configuration for testing controllers and watching the joints
move without worrying about balance. `osl_v2_ground.xml` gives it a free joint,
puts a floor underneath, and carries collision geometry — the Variflex blade at
the sole, plus one bounding box per module above the ankle — for stance and
loading. Both scenes ship a `flat` keyframe that levels the sole — which is *not*
ankle = 0, see the flat-foot finding below — and in the ground scene it also drops
the leg onto the floor. Whether the ground scene stands or topples has not been
established — a single leg with no socket, hip or balance controller is not
obviously stable, though a static check suggests the COM does project inside the
sole footprint with margin. `check_model.py` prints the base height before and
after a second of settling, which answers it.

Both scenes carry the same three-body chain — thigh, shank, foot — with a knee
hinge and an ankle hinge, position actuators on both, and a sensor suite that
mirrors what the real leg reports: joint encoders, actuator torque, a thigh IMU,
a shank IMU, and the six-axis load cell.

```
python scripts/check_model.py                      # compile, CAD cross-check, contact
python scripts/view_osl.py                         # interactive viewer, bench
python scripts/view_osl.py --scene ground          # viewer with the floor
python scripts/view_osl.py --scene ground --flat   # standing, sole level
python scripts/view_osl.py --knee 60 --ankle -15   # start in a pose
python scripts/demo_sweep.py                       # sinusoidal knee/ankle sweep
python scripts/demo_sweep.py --headless --csv build/sweep.csv
```

In the viewer, `Tab` opens the control panel where the knee and ankle sliders
live and `space` pauses. The Group toggles hide geom groups: the visual meshes
are group 2, and group 3 is the collision geometry, which exists only in the
ground scene — the blade mesh at the sole, plus the four bounding boxes that stop
the shank and thigh reaching the floor. The bench scene has no collision
geometry at all.

## Model summary

Masses and centres of mass, the latter in **world** coordinates at zero joint
angles (the MJCF stores them body-relative, so the XML numbers will not match
these directly; `check_model.py` prints the world values):

| | mass | COM, world (m) | parts kept | parts culled |
|---|---|---|---|---|
| thigh | 0.4498 kg | +0.0002, −0.0074, +0.1997 | 28 | 60 |
| shank | 3.5908 kg | +0.0047, +0.0028, +0.0320 | 142 | 347 |
| foot | 0.9152 kg | −0.0066, −0.0037, −0.1608 | 47 | 26 |
| **total** | **4.9558 kg** | | **217** | **433** |

80 distinct meshes, 1,310,462 triangles. The knee hinge sits at
`(0.00065, 0, 0.19311)` and the ankle at `(0, 0, −0.13873)`, 331.84 mm apart,
both taken from the CAD rather than chosen. Knee range −5° to +120° (positive is
flexion), ankle −30° to +20° (positive is dorsiflexion). Those ranges are from
this export's own hard stops and they differ slightly from the 0–120° and
±30° quoted for "the OSL v2" in Tan et al. 2025; the difference is kept, because
the CAD is the better authority on the specific leg being modelled. Worth
re-measuring on the hardware.

Coordinates are the export's own, so `z = 0` is a point mid-pylon and not a
landmark. The bench-scene bounding box is
`[−0.1665, −0.0437, −0.2302] .. [0.0846, 0.0752, 0.2602]`: 490 mm of build
height from the sole at z = −0.2302 to the top of the proximal pyramid at
z = +0.2602, and 119 mm across. The ankle axis is 91.5 mm above the sole. In the
ground scene everything is lifted so the floor is z = 0, which puts the ankle
axis 93.5 mm above the floor at zero joint angles — 91.5 mm of build height plus
2 mm of spawn clearance, nothing else. The blade really is the lowest thing in
the model (`model_min_z` and the sole's own lowest z are the same −0.2302), and
every bounding box bounds meshes that are already inside that minimum, so there
is no fudge term. The `flat` keyframe sits 3.8 mm lower, at a base height of
0.2284 m, because rotating the sole level tucks the toe up.

Anterior is **−X**, posterior **+X**, and the sagittal plane is XZ. This comes
from the Variflex blade, whose long forefoot runs from the mounting bolts at
x ≈ +0.035 out to x = −0.1665.

## Why the URDF could not simply be loaded

why there is a build step.

The Onshape export (`osl_v2_0_assembly/urdf/`, 682 links, 681 joints, 14,584
lines) has no materials assigned, so every `<inertial>` block is nonsense — the
whole leg weighs 14.75 grams in the file, and 52 of the 657 inertia tensors have
a non-positive eigenvalue (8 of those also violate the triangle inequality that
any real rigid body must satisfy). MuJoCo's URDF reader eigen-decomposes each
inertia tensor while *parsing*, which means `compiler inertiafromgeom="true"`
never gets a chance to help: the load fails before the option is applied.
Sanitising the URDF in place was rejected for the same reason it was tempting —
682 hand-audited blocks is not a reviewable diff.

So `tools/build_mjcf.py` reads the export and emits native MJCF instead. Mass
comes from the exact mesh volume (divergence theorem over each closed triangle
mesh) multiplied by a density assigned per part family, and inertia is computed
about each segment's own COM and then rotated to principal axes. The original
URDF is never modified; everything generated lands in `models/` and `build/`.

Four further things had to be got right, and each was initially got wrong.

**Mesh poses need `<visual><origin>` composed on top of the link pose.** 207 of
the 682 links carry a non-identity visual origin, the largest a 206 mm
translation on the Variflex foot, and ignoring them produces an assembly where
the knee and ankle modules interpenetrate. Folding the origin in reproduces the
export's own declared centre-of-mass positions to a 0.000 mm median and 0.002 mm
at the 90th percentile, against 10.150 mm at the 90th percentile and 158.5 mm at
worst without it; the assembled leg becomes 251 × 119 × 490 mm rather than
211 × 327 × 532 mm, and 327 mm of medio-lateral width on a device under 120 mm
wide is the giveaway. `build/verify/visorigin_AB.png` shows both, and
`tools/verify_visorigin.py` regenerates it.

**Rigid clusters straddling a joint go to the segment opposite the housing.**
The 618 `fixed` joints contract by union-find into clusters, 38 of which carry
geometry, and each is assigned to thigh, shank or foot. Nine of the 38 sit
concentric with a hinge axis, where a plain "above or below the joint" split
would be a coin flip. The resolution is mechanical: in a belt-driven actuator the
output pulley rotates *relative to* its housing, and both housings are
shank-fixed, so a cluster concentric with the knee axis belongs to the thigh and
one concentric with the ankle axis belongs to the foot. That is a 20 mm radius
test in the sagittal plane, applied before the z-split.

**The Össur foot mate is fine.** An earlier pass concluded the mate was corrupt
and reconstructed the foot from anatomical constants; that was an artefact of the
missing visual origin. With the frames right, the foot's length axis is 89.73°
from the ankle axis and the sole is within 2.18° of level — a real build tilt,
deliberately preserved. `_check_foot()` re-verifies this on every build and warns
if it ever stops holding. **The model contains no invented geometry.**

**The floor needed more than a box under the sole, and the sole is not level at
ankle = 0.** The first ground scene put a single tilted box on the sole, fitted to
the blade's bounding box — so its lower face sat 4.6 mm *below* the blade mesh —
and gave nothing above the ankle any collision geometry at all. Commanding the
ankle therefore tipped the leg and the shank sank straight through the floor.
Both halves are fixed. The sole is now a `type="mesh"` geom on the blade itself,
which MuJoCo collides by its convex hull, so the toe spring and the heel roll-off
survive exactly and only the 14 mm arch is bridged — deliberately, since under
load the real blade flattens toward that chord. It shares the visual geom's pose
exactly, because the previous failure mode was the leg standing on something the
viewer does not draw. Above the ankle each module gets one axis-aligned bounding
box: `thigh_shell` 78 × 79 × 106 mm, `knee_module` 128 × 119 × 204 mm,
`mid_pylon` 66 × 102 × 69 mm and `ankle_module` 137 × 118 × 192 mm, respectively
21 %, 22 %, 11 % and 20 % full by the volume of the parts they contain. Those
fill fractions are printed so nobody mistakes a box for a shape claim.
Membership comes from the CAD's own named housings — the knee housings span
z = 0.047..0.239 and the ankle housings −0.185..−0.028, disjoint with a 75 mm gap
— rather than from a hand-picked threshold, and the eleven parts that fall
between them are exactly the ones you would name by hand: the pylon, the load
cell, the pyramid and six M5 bolts. Four boxes are enough, and that is checked by
code rather than asserted in a comment: `OslModel.sweep_clearance` walks a 26 × 26
grid of knee −5..120° against ankle −30..+20° and compares the lowest point of
every visual mesh against the lowest point that can actually collide. The answer
comes out twice. Over all 216 visual meshes the worst case is 5.9 mm of clearance;
over the 46 that no box contains — the foot's non-blade hardware, since the foot's
only collision geom is the blade mesh and a mesh bounds nothing — it is 23.6 mm,
at knee = −5°, ankle = −2°, where an M8 socket head hanging off the blade mount is
the binding part. The two differ because a part inside a box on its own body can
never rise above that box, so its clearance only measures how close a rotating box
corner passes to the part that defined it; the second figure is the one with
information in it. Either way no mesh can reach the floor before a collision geom
does. Both numbers are written into the sidecar and `check_model.py:sweep_test`
re-derives both from MuJoCo's own forward kinematics, finding the collidable geoms
by `contype`, the blade by mesh identity and the boxed parts by testing every
vertex in each box's frame, rather than reading any of it from the prediction — so
the CAD sweep and the MuJoCo sweep are two independent measurements of the same
pair of quantities. Both are affordable only because of one shortcut: since every
rotation here is about world Y, the lowest point of a rigid group is the minimum of
a linear functional on the sagittal plane and is therefore always attained at a
vertex of the group's convex hull projected to (x, z), which turns four screw
meshes of 193 k vertices each into a few dozen candidate points for all 676 poses
at once. That is an argument, not a sample, so `sweep_test` redoes each worst pose
with every vertex and fails if the reduced answer differs.
Capsules and z-slabs were both measured and rejected — PCA capsules wanted
r = 81 mm and len = 174 mm on a part whose bounding box is 128 × 119 × 204 mm and
still left 1.0 % of vertices 14.8 mm outside, and 25 mm z-slabs cost 23 geoms to
raise fill from 22 % to 30 %, because the looseness is in x and y where the
batteries hang off, not in z.

And the sole is 2.40° toe-down at ankle = 0, which is the other half of "the leg
falls over when I move the ankle". The dominant edge of the blade's lower convex
hull in the sagittal plane runs 149.5 mm at +2.3971°, from (−0.08936, −0.09146)
to (+0.06001, −0.08521) in the foot frame; the next-longest hull edge is only
34.7 mm, so the contact face is unambiguous by a factor of 4.3. Put the ankle at
+2.3971°, well inside the +20° limit, and the contact becomes a genuine
161 × 64 mm patch: 173 blade vertices within 2 mm of the lowest point, flat to
0.49 mm rms, with a residual tilt of +0.29° sagittal and under 0.01° frontal.
That the frontal residual is negligible is what makes this fixable at all, since
one ankle hinge could never have corrected a frontal tilt. The pose is the `flat`
keyframe in both scenes; `view_osl.py --flat` loads it. `build_mjcf.py` derives the
standing height by rotating CAD meshes and `validate_mjcf.py` re-derives it by
rotating the *emitted XML's* collision points about the *emitted* ankle anchor, and
both land on the same +2.00 mm above the floor, which is the spawn clearance and
nothing more. The angle and the edge are still one derivation, though, and would
agree with each other for free — so `check_model.py:flat_test` loads the keyframe
and pushes both endpoints of that edge through MuJoCo's forward kinematics,
requiring them to come out level there too, along with the base height, the set of
collidable geom names and the 2.00 mm clearance itself.

## Simplifications, stated plainly

**Small parts are dropped.** Anything below 0.1 cm³ loses its geometry and has
its mass lumped into the parent segment: 433 of 650 parts, but only 52 g, or
1.05 % of the mass, because they are fasteners, washers and bearing races.
`build/mass_audit.csv` lists every part with its volume, assigned density, the
rule that assigned it, and whether it was kept, sorted by mass. Regenerate it
with `python tools/validate_mjcf.py --audit`.

**The densities are assignments by part family, not measurements.** Aluminium at
2700 covers 245 parts and is the default; steel at 7850 covers 240 fasteners,
bearings and the load cell; electronics at 1500 covers 133 parts of the
Raspberry Pi and its connectors; NdFeB magnets 7500; the Dephy BA30 packs and
the FR-4 PCB 1900; the Variflex blade 1550 as carbon fibre; belts and printed
covers 1250. `tools/cad_findings.py` prints the full usage table.

**The total is 0.42 kg light, and the missing motors account for it.** The model
comes out at 4.9558 kg against a published 5.377 kg for a real OSL V2 build
(Tan et al., *MyoAssist 0.1*, ICORR 2025, which cites Best et al. 2024 and
Azocar et al. 2018; Elliott Rouse, who runs the OSL project, is a co-author).
The reason is that **the Onshape export contains no motors.** All 260 distinct
meshes were searched: the only motor-related part is `p_b0004_motorCoupling.stl`
at 8.8 g, present twice — the couplings that bolt *to* the motors. There is no
rotor, no stator, no gearbox and no Dephy actuator body anywhere in the export.
The 0.4212 kg shortfall is 0.2106 kg per motor, which is the right order for a
brushless motor of this class.

That is a hypothesis, and the way to settle it is to put a motor on a scale.
`MOTOR_MASS` in `build_mjcf.py` will lump a measured per-motor mass at each
coupling centroid, and **it defaults to zero on purpose**: inferring the motor
mass from (published − ours) and then citing the resulting agreement as
validation would be circular. The 4.9558 kg figure has to stand on its own, so
the shipped model stays purely CAD-derived.

An earlier version of this file claimed the opposite — that the model was too
*heavy*, against a 3.0–4.5 kg range. That range was recollection with no source
behind it and it was wrong. `MASS_RANGE` is now 4.70–5.20 kg, the window the
CAD-only total should land in given two absent motors, and both scenes sit
inside it.

**Everything inside the actuators is collapsed.** Belts, pulleys, gearboxes and
bearings contribute mass and geometry but no relative motion; the knee and ankle
are single hinges with lumped `armature`, `damping` and `frictionloss` standing
in for the transmission. Those numbers (armature 0.010 and 0.008 kg·m², damping
0.30 and 0.25 N·m·s, friction 0.40 and 0.30 N·m) are placeholders to be
identified from bench data, not derived quantities. Peak torque is *not* a
placeholder: the actuator `forcerange` is ±142.2 N·m at the knee and
±168.2 N·m at the ankle, the published figures from the same paper as the mass.
The position-servo gain `kp = 60` is a placeholder and a soft one — it will not
hold a limb under body weight and is expected to change as soon as the model is
loaded.

**Contact is faithful at the sole and coarse everywhere else.** The sole is the
blade's own mesh, collided by its convex hull, which is honest geometry. Above it
are four bounding boxes at roughly 20 % fill, whose only job is to keep the leg
out of the floor; any result that depends on *where* the shank or the knee touches
something is not yet meaningful. There is no self-collision model either —
MuJoCo's `filterparent` removes thigh↔shank and shank↔foot automatically, and
thigh↔foot was shown unreachable across the joint ranges rather than modelled.

**Contact timing is set, contact depth is measured rather than predicted.** Both
scenes now write `solref="0.002 1"` and `solimp="0.95 0.99 0.001 0.5 2"` instead
of inheriting the compiler defaults. The argument is about time, since that is
what `solref[0]` specifies: the default 0.02 s time constant is 40× the 0.5 ms
timestep and slower than the 10 ms scale on which heel strike, foot flat and toe
off resolve, so it would smear exactly the transitions an impedance controller
switches on. 0.002 s puts the contact an order of magnitude faster than the
control events, and is deliberately 4× the timestep rather than sitting on
MuJoCo's `timeconst >= 2 * timestep` stability floor. Resting penetration scales
as the square of that constant, so the change is worth about 100× — but the
absolute depth is **not** predicted anywhere in this repo, because the
coefficient depends on how the `solimp` impedance enters the solver's
regularisation and `check_model.py` can read `contact.dist` and simply measure
it. What is asserted is a bound: 0.5 mm, chosen against the 0.49 mm rms flatness
of the contact patch, past which the sole sinking starts to matter next to the
shape of the sole itself. One trap worth knowing, and the reason the floor plane
and the leg's collision geoms share a single `contact` default class: MuJoCo does
not take the stiffer of two touching geoms, it averages `solref` and `solimp`
between them, so a floor left at the default would have quietly given back most
of the benefit. `validate_mjcf.py` resolves the default-class chain of every
collidable geom and fails if they ever disagree.

## Layout

`tools/oslcad.py` is the geometry layer — STL reading, exact mesh mass
properties, the URDF graph, rigid-cluster contraction, density rules.
`tools/build_mjcf.py` turns that into the two MJCF scenes and is where every
measured constant lives. Alongside each scene it writes
`models/osl_v2_<scene>.pred.json`: the masses, world centres of mass, hinge
anchors and mesh bounding box it expects MuJoCo to report, so that
`check_model.py` can hold MuJoCo to the CAD instead of merely asking whether it
compiled.
`tools/validate_mjcf.py` re-implements MuJoCo's compile checks in pure Python so
the model can be verified without MuJoCo installed; both scenes currently pass
clean, 1767 and 1815 checks with no warnings. It also independently re-derives the
`flat` keyframe from the emitted XML, because a keyframe whose whole purpose is
"stand the leg level on the floor" is exactly the kind of claim that can be
silently false — the sole geometry it replaced sat 4.6 mm below the mesh it was
meant to hug and nothing noticed.
`tools/cad_findings.py` re-derives every geometric claim with its evidence and is
what `docs/CAD_FINDINGS.md` contains.
`tools/softrender.py` is a dependency-free rasteriser; `tools/verify_visorigin.py`
and `tools/render_poses.py` use it to regenerate the two evidence images.
`render_poses.py` reads the *generated* MJCF and steps it through seven
knee/ankle poses, which catches the class of error a compile check cannot: a
segment on the wrong body, a flipped axis, a misplaced hinge.

`scripts/check_model.py`, `scripts/view_osl.py` and `scripts/demo_sweep.py` are
the three things you actually run, sharing `scripts/mjcommon.py`. `check_model.py`
is the only one that is a test: it compiles both scenes, compares masses, centres
of mass, hinge anchors and mesh bounding boxes against the sidecar, reads the
resting contact depth off `contact.dist`, watches a second of settling, and
re-derives the collision sweep and the flat keyframe from MuJoCo's own kinematics.
It exits non-zero on any disagreement, so it is safe to put in front of anything
else. The other files in `scripts/`, plus `build/osl_v2_*.urdf`,
`osl_v2_first_sim.xml` and `run_osl_v2_first_sim.py`, are dead ends kept only for
the record: the URDF-sanitiser attempts and the early rods-and-capsules smoke test.
Neither is the simulation.

To rebuild from the CAD:

```
python tools/build_mjcf.py --report      # regenerate models/
python tools/validate_mjcf.py --audit    # pre-flight + mass audit, no MuJoCo
python tools/cad_findings.py             # re-derive the constants + evidence
python tools/verify_visorigin.py         # regenerate the frame-convention proof
python tools/render_poses.py             # visual articulation check
python scripts/check_model.py            # real MuJoCo compile + CAD cross-check
```

Only the last step needs MuJoCo, and it is the one step that has never been run
here — the build environment has no MuJoCo and no network, so "would compile" is
a prediction from a re-implementation of the checks, not a result.

That gap matters more than it sounds, because every check above shares one
codebase and therefore one set of assumptions. The specific thing it cannot see:
MuJoCo recentres mesh vertices onto the mesh centroid when it compiles, and is
supposed to fold the same translation into the geom frame so the mesh still
lands where it was authored. If it did not, every visual geom would be displaced
by its own centroid offset — tens of millimetres here — and nothing in the pure
Python tooling would notice, because it shares the convention it is trying to
verify. `check_model.py` closes that by taking MuJoCo's *own* stored vertices
(`mesh_vert`, post-recentring), placing them with MuJoCo's *own* resolved geom
frames (`geom_xpos`/`geom_xmat`, post-compensation), and comparing the resulting
bounding box against the CAD prediction in the sidecar. Agreement to 0.1 mm
means the whole chain — visual origins, cluster assignment, body offsets, mesh
recentring — is consistent. Disagreement says which of them is not. The same is
true of the two kinematic tests added with the contact work: the sweep and the
flat keyframe are both written to be re-derived from MuJoCo's forward kinematics
rather than from the sidecar, but neither has been executed against real MuJoCo
from here. They have been exercised against a stub with the correct rotation
conventions, including their failure paths, which establishes that they would fire
— not that they pass.

## Not done yet

Gait-phase estimation, IMU filtering, the actual motor control algorithms,
sim-to-real transfer. The sensor suite and the CSV log from `demo_sweep.py` exist
to feed exactly those, but none of them are started. Transmission parameters and
densities want identification against the real leg. Contact above the ankle is
bounding boxes, so knee-on-ground and shank contact will resolve at about the
right height but not the right shape. The contact timing is set and the tests that
would measure its consequences are written, but nothing in `check_model.py` has
been run against real MuJoCo from here, so the resting penetration and the
settling behaviour are still unmeasured — that single command is the cheapest
outstanding thing anyone can do to this repo. Most of all, there is no hip, no
thigh segment and no socket, so the ground scene is a leg standing on its own
rather than something that can walk — that is the next block of work, and it is
being built to line up with the `myoOSL` environment in MyoAssist (Tan et al. 2025)
rather than diverging from it.
