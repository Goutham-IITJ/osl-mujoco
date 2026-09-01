#!/usr/bin/env python3
"""
build_mjcf.py -- generate native MJCF for the OSL V2 straight from the
*untouched* Onshape export.

Why native MJCF instead of loading the URDF?  MuJoCo's URDF reader
eigen-decomposes every <inertial> block during parsing, before compiler
options apply, so `inertiafromgeom="true"` cannot rescue a bad tensor.  And
the export's inertials are not merely degenerate -- Onshape wrote it with no
materials assigned, so the total mass over all 682 links is 0.0148 kg, and 52
of the 657 inertial blocks have a non-positive eigenvalue (8 of those also
violate the triangle inequality) for a leg that really weighs 5.377 kg
(PUBLISHED_MASS below).  Every inertial in the export is therefore discarded
and mass is recovered from  mesh volume x assigned density  (see DENSITY_RULES
in oslcad.py), which lands at 4.9558 kg -- 0.42 kg light, and the export
contains no motors at all, which is very likely the whole of the difference.
See MOTOR_MASS.  build/mass_audit.csv lists every part.

Pipeline
--------
1. parse + forward-kinematically resolve the export            (oslcad.OslCad)
   -- including <visual><origin>, which 207 of the 682 links need
2. drop the 32 mesh-less Onshape phantom links
   (`parallel_*`, `planar_*`, `cylindrical_*`, `*_loop_closure`)
3. contract every `fixed` joint into rigid clusters (union-find)
4. assign each cluster to knee_prox / shank / foot, resolving the ones
   concentric with a hinge axis by the belt-drive rule below
5. re-verify the Ossur Variflex foot mate (_check_foot); nothing is repaired,
   the export's mate is correct
6. cull parts below CULL_VOLUME, lumping their mass into their segment
7. emit exact, guaranteed-valid inertials and two scenes

Nothing here edits the source URDF.

Usage:  python tools/build_mjcf.py [--report]
"""

from __future__ import annotations

import argparse
import collections
import json
import math
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from oslcad import OslCad, density_for, mat_to_quat, rpy_to_mat  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
URDF = os.path.join(ROOT, "osl_v2_0_assembly", "urdf", "osl_v2_0_assembly.urdf")
MESHES = os.path.join(ROOT, "osl_v2_0_assembly", "meshes")
MODELS = os.path.join(ROOT, "models")
MESHDIR_REL = "../osl_v2_0_assembly/meshes"

# ---------------------------------------------------------------------------
# Geometry constants measured from the CAD.  Every one of them is re-derived,
# with its evidence, by  python tools/cad_findings.py  -- whose output is what
# docs/CAD_FINDINGS.md contains.  If a number here and a number there disagree,
# the script is right.
# ---------------------------------------------------------------------------

# Knee and ankle hinge locations, both about world Y.  Established from
# concentric bearing / pulley clusters and cross-validated against the
# housing plate normals (which agree to 0.00 deg with each other and are
# 90.00 deg from the pylon).
KNEE_POS = np.array([0.00065, 0.0, 0.19311])
ANKLE_POS = np.array([0.00000, 0.0, -0.13873])

# +Z proximal, -Z distal, sagittal plane = world XZ.
#
# ANTERIOR IS -X.  Determined from the foot, which is the only unambiguous
# anatomical landmark in the assembly: the Variflex blade spans x = -0.1665 to
# +0.0756 while its mounting bolts sit at x ~ +0.035, so the long forefoot runs
# toward -X and the short heel toward +X.  (An earlier guess of +X, based on the
# Dephy battery bracket sitting at x ~ -0.052, was wrong -- batteries are not a
# reliable anterior/posterior cue.)  Both hinge-axis signs below happen to be
# unaffected by the correction.
ANTERIOR = np.array([-1.0, 0.0, 0.0])

# A rigid cluster whose centroid is this close to a hinge axis (measured in
# the sagittal plane) is that joint's bearing / output-pulley / gear-stop
# group.  In a belt-driven actuator the output pulley rotates *relative to
# the housing*, so it is rigidly part of the OTHER segment: the knee housing
# is shank-fixed, so knee-concentric clusters belong to the knee_prox; the ankle
# housing is shank-fixed, so ankle-concentric clusters belong to the foot.
AXIS_CLUSTER_RADIUS = 0.020

# Parts smaller than this are dropped from the model and their mass lumped
# into the segment (Goutham's choice: "cull tiny parts entirely").  0.10 cm^3
# keeps 217/650 links and 99.54 % of the total volume.
CULL_VOLUME = 0.10e-6  # m^3

# Mass of the physical OSL V2 knee-ankle build, from a published source.
#
# 5.377 kg is stated in Tan et al., "MyoAssist 0.1: MyoSuite for Dexterity and
# Agility in Bionic Humans", ICORR 2025, section IV-A, describing the mass they
# matched their myoOSL prosthesis model to.  They cite Best et al. 2024
# (IEEE/ASME T-Mech) and Azocar et al. 2018 (BioRob), and Elliott Rouse -- who
# runs the OSL project -- is a co-author, so this is as close to authoritative
# as a number gets without weighing the hardware.
#
# This REPLACES an earlier 3.0-4.5 kg range that was my own recollection with
# no source behind it.  That range made the model look 0.46 kg too heavy and
# pointed the finger at the density assignments.  Against the published figure
# the model is instead 0.42 kg too LIGHT, which is a different problem with a
# different and much more satisfying cause -- see MOTOR_MASS below.
PUBLISHED_MASS = 5.377  # kg

# Range the CAD-derived total should fall in.  It is deliberately BELOW
# PUBLISHED_MASS, because the export is missing both motors (MOTOR_MASS): two
# OSL-class brushless motors plausibly run 0.15-0.30 kg each, so a correct
# CAD-only total should land roughly 0.3-0.6 kg light.  The window is tight
# enough that a real density blunder still trips it.  validate_mjcf.py imports
# this so there is one number in the repo rather than three that disagree.
MASS_RANGE = (4.70, 5.20)  # kg, CAD-derived total, motors excluded

# THE MOTORS ARE NOT IN THE ONSHAPE EXPORT.
#
# All 260 distinct mesh files were searched for anything motor-like.  The only
# hit is p_b0004_motorCoupling.stl at 8.8 g, present twice -- the couplings
# that bolt TO the motors.  There is no rotor, no stator, no gearbox and no
# Dephy actuator body anywhere in the export.  The heaviest parts are housings,
# batteries, the load cell and the foot; nothing is a motor.
#
# That accounts for the gap: PUBLISHED_MASS - 4.9558 = 0.4212 kg, which is
# 0.2106 kg per motor if the two are identical -- the right order for a
# brushless motor of this class.
#
# DEFAULT IS ZERO, ON PURPOSE.  Two reasons.  First, this repo's claim is that
# it contains no invented geometry or mass, and a motor inferred from a
# residual is exactly that.  Second and more important, deriving the motor mass
# from (published total - our total) and then citing the agreement as
# validation would be circular.  The 4.9558 kg figure has to stand on its own.
#
# To include the motors, set this to a MEASURED per-motor mass -- weigh one
# when the hardware arrives, or take it from the Dephy actuator datasheet.  The
# mass is added as a point mass at each p_b0004_motorCoupling centroid, both of
# which are shank-fixed (consistent with the belt-drive rule: the housings are
# shank-fixed, so the motors are too).  Rotor inertia, once known, is also what
# JOINTS[*]["armature"] should be derived from, via the 50/11 belt ratio.
MOTOR_MASS = 0.0  # kg per motor; 0 = CAD only
MOTOR_LINKS = ("p_b0004_motorcoupling", "p_b0004_motorcoupling_1")

# The Variflex foot and its adapter.  NOTE: an earlier pass concluded that
# revolute_30 (adapter -> foot) was corrupt and reconstructed the foot from
# named anatomical constants.  That was wrong: the diagnosis had been made
# with the mesh placed by link pose alone, ignoring <visual><origin>.  Once
# the visual origin is folded in (see visual_origins() below) the export's own
# mate is correct -- the sole is within 2.18 deg of level, the foot's length
# axis is 89.73 deg from the ankle axis, and the M8 bolts sit centred inside
# the adapter.  Nothing about the foot is reconstructed; the residual 2.18 deg
# is the build's real alignment tilt and is deliberately preserved.
FOOT_LINK = "z_lowprofilevariflexfoot_ossur_vlpe5xx0"
ADAPTER_LINK = "p_b2015_proxpyramidvariflex"

# --- collision geometry ----------------------------------------------------
#
# WHAT WAS HERE BEFORE, AND WHY IT WAS NOT ENOUGH.  The ground scene used to
# carry exactly one collision geom: a 6 mm-thick box hugging the sole, tilted to
# follow the foot's alignment.  Two things were wrong with it.
#
#   1. The box was flat and the sole is not.  Fitted to the blade's bounding
#      box, its lower face sat 4.6 mm BELOW the blade mesh, so the leg stood on
#      a surface that does not exist, and every roll-over property of a carbon
#      blade -- toe spring, heel radius, the fact that the contact point
#      migrates forward through stance -- was thrown away.
#   2. Nothing above the ankle could collide at all.  Change the ankle angle
#      under control and the shank or the knee module simply passes through the
#      floor.  That is the "leg goes underground" symptom exactly.
#
# WHAT IS HERE NOW.  The sole is a `type="mesh"` geom on the blade's own mesh,
# at the same pose as its visual geom.  MuJoCo collides meshes by their convex
# hull, and for this part the hull is nearly the part: the toe spring and the
# heel roll-off are both convex, so they survive exactly.  The one thing the
# hull does not reproduce is the arch -- the Variflex LP is two overlapping
# blades, an upper forefoot blade and a lower J-shaped heel blade, and the hull
# bridges the 14 mm gap between their contact points.  That is deliberate and
# arguably closer to the truth than the raw surface: under body weight the real
# blade flattens toward the chord, and a rigid model cannot flatten, so the
# chord is the better stand-in for the loaded shape.  It is still an
# approximation and it is the first thing to revisit if roll-over shape starts
# to matter.  Zero invented numbers either way -- the geometry is the CAD's.
#
# Above the ankle the job is different.  The shank and knee_prox do not need a
# faithful contact shape; they need to not be able to reach the floor.  So each
# gets axis-aligned BOUNDING BOXES, one per real module, and the code says
# bounding rather than shape on purpose: measured fill fractions are 22.2 %
# (knee module), 11.2 % (mid/pylon), 19.8 % (ankle module) and 20.7 % (knee_prox).
# Those boxes enclose a lot of air.  Two alternatives were measured and dropped:
#
#   * capsules, fitted by PCA to each module -- rejected with numbers.  The knee
#     module wanted r = 81 mm and len = 174 mm on a part whose bounding box is
#     128 x 119 x 204 mm, with 1.0 % of vertices still 14.8 mm outside.  These
#     modules are blocky, not elongated; a capsule is the wrong primitive and
#     the fit says so.
#   * slicing each module into 25 mm z-slabs and boxing each slab -- 23 geoms
#     instead of 4 and fill only rises 22 % -> 30 %, because the looseness is in
#     x/y (batteries hang off one side), not in z.  Not worth 19 extra geoms.
#
# The boxes are validated for sufficiency rather than assumed, and by code
# rather than by this comment: OslModel.sweep_clearance sweeps the whole joint
# envelope (knee -5..120 deg x ankle -30..+20 deg, 26 x 26 poses) and reports the
# lowest point of any visual mesh against the lowest collision geom.  Over all
# 216 visual meshes the worst case is 5.9 mm clear, and over the 46 that no box
# contains -- the foot's non-blade hardware, since the foot's only collision geom
# is the blade mesh -- it is 23.6 mm clear, worst case at knee = -5 deg,
# ankle = -2 deg, where an M8 socket head hanging off the blade mount is the
# binding part.  The two figures differ because a part inside a box on its own
# body can never get below that box, so its clearance only measures how close a
# rotating box corner passes to it; the unboxed figure is the one with
# information in it.  Either way no mesh can reach the floor before a collision
# geom does.  Both numbers go into the sidecar and scripts/check_model.py
# re-derives them from MuJoCo's own forward kinematics, so neither the CAD-side
# sweep nor the MuJoCo-side one is taken on trust.
#
# Module membership is taken from the CAD's own named housings rather than from
# a z threshold someone picked: a shank part belongs to the knee module or the
# ankle module according to which housing's z-span its own z-span overlaps more,
# and to the mid/pylon group if it overlaps neither.  The two housings do not
# overlap (knee 0.0471..0.2389, ankle -0.1845..-0.0283, a 75.4 mm gap), so the
# rule is unambiguous, and the 11 parts that land in the middle are exactly the
# ones you would name by hand: the pylon, the load cell, the pyramid and their
# M5 bolts.
MODULE_SEEDS = {
    "knee_module": ("p_b1018_housingkneeright", "p_b1018_housingkneeleft"),
    "ankle_module": ("p_b2013_housingankleright", "p_b2013_housingankleleft"),
}

# Spawn clearance for the ground scene: the whole leg is lifted so the lowest
# collidable point starts this far above the floor.  Small enough not to matter,
# large enough that the first contact is resolved by the solver rather than by
# an initial penetration.
SPAWN_CLEARANCE = 0.002  # m

# --- joint / actuator parameters ------------------------------------------
# Positive = flexion, matching the OSL convention.  Knee flexion swings the
# shank posteriorly, so the knee axis is -Y; ankle dorsiflexion lifts the toe,
# so the ankle axis is +Y.
#
# PEAK TORQUES are published, not guessed: knee 142.2 N.m and ankle 168.2 N.m,
# from Tan et al. 2025 (see PUBLISHED_MASS), who matched their myoOSL actuators
# to them.  These replace an invented +-100 N.m that was in this file purely so
# the actuators had *some* limit.  The ankle being stronger than the knee looks
# wrong at first glance and is not -- the ankle module carries the higher belt
# reduction because push-off is the more demanding task.
#
# RANGES OF MOTION disagree with that same paper and OURS ARE KEPT.  The paper
# lists knee 0-120 deg and ankle -30 to +30 deg; the CAD gives knee -5 to +120
# and ankle -30 to +20.  Both differences are real and ours is better sourced:
# the -5 deg of knee hyperextension is the physical hard stop in this assembly,
# and the +20 deg dorsiflexion limit is where this particular Variflex adapter
# stack runs out of travel.  A paper describing "the OSL v2" in general is a
# weaker authority on our specific export than the export is.  Revisit only if
# the hardware measures otherwise.
JOINTS = {
    "knee": dict(
        axis=(0.0, -1.0, 0.0),
        range=(math.radians(-5.0), math.radians(120.0)),
        damping=0.30,
        armature=0.010,
        frictionloss=0.40,
        kp=60.0,
        forcerange=(-142.2, 142.2),  # published peak, Tan et al. 2025
    ),
    "ankle": dict(
        axis=(0.0, 1.0, 0.0),
        range=(math.radians(-30.0), math.radians(20.0)),
        damping=0.25,
        armature=0.008,
        frictionloss=0.30,
        kp=60.0,
        forcerange=(-168.2, 168.2),  # published peak, Tan et al. 2025
    ),
}
# NOTE on what is and is not identified above.  `forcerange` and `range` now
# have sources (published peak torque, CAD hard stops).  `damping`, `armature`,
# `frictionloss` and `kp` do NOT -- they are plausible placeholders, collected
# here so they can be fitted against bench data in one place.  `kp=60` in
# particular is far too soft to hold a stance limb and is expected to change as
# soon as the model carries load; see the walk scene.

SEGMENTS = ("knee_prox", "shank", "foot")

# ===========================================================================
# PHASE 2 -- HUMAN SCAFFOLD FOR THE `walk` SCENE.
#
# READ THIS BEFORE TRUSTING ANY NUMBER BELOW.  Everything above is measured
# from the CAD and is the point of this repo.  Everything in THIS block is the
# opposite: the pelvis, the residual (amputated) thigh, the socket and the
# intact contralateral leg are NOT in the Onshape export, because they are
# anatomy, not the device.  They exist only so the CAD-derived leg has a hip to
# hang from, a socket interface to load, and a second leg and a floor to walk
# against -- the "realistic, walkable" model the professor asked for.
#
# So these are PRIMITIVES (capsules / boxes) with PLACEHOLDER masses and
# lengths, and they are flagged as such at runtime (see OslModel._emit_walk,
# which appends a note to every build).  This is deliberately NOT the "generic
# capsule model" the ground rules forbid: that rule protects the DEVICE from
# being replaced by a rod, and the device here is still the exact CAD mesh
# tree.  The capsules are only the human it bolts onto, and the intent is to
# replace them -- or reconcile them -- with the myoOSL musculoskeletal model
# (MyoAssist / myo_sim) once the two are cross-checked.  Until then, treat
# every constant here as a round number chosen to be plausible, not as data.
#
# Anthropometry is 50th-percentile male (body mass ~75 kg), segment fractions
# after Winter, "Biomechanics and Motor Control of Human Movement" (2009), with
# the transfemoral residual limb taken at roughly a 50 % level.  None of it is
# fitted to this patient or this build.
# ===========================================================================
BODY_MASS = 75.0            # kg, reference 50th-pct male (Winter 2009)

# -- suspended pelvis (the "gantry"): welded to the world for now; a prescribed
#    trajectory will drive it later, so no balance controller is needed yet.
PELVIS_MASS = 10.0          # kg, pelvis + lumped lower-HAT the gantry carries
PELVIS_HALF = (0.070, 0.110, 0.060)   # m, pelvis box half-extents (x, y, z)
HIP_HALF_WIDTH = 0.090      # m, each hip offset laterally from pelvis midline
# device (residual) leg on +Y, intact leg on -Y.

# -- residual thigh (amputated limb the socket grips)
RES_THIGH_MASS = 3.5        # kg, transfemoral residual limb, ~50 % level
RES_THIGH_LEN = 0.20        # m, residual femur length (~1/2 of a ~0.40 m femur)
RES_THIGH_RADIUS = 0.065    # m

# -- socket + pyramid adapter: the compliant interface between limb and device.
#    FOUR PASSIVE DOFs, each a spring-damper toward 0 (no actuator): axial
#    piston (limb sinks into the socket under load) plus flexion, ab/adduction
#    and internal/external rotation of the socket on the residuum.  The
#    stiffnesses and dampings are UNFITTED -- placeholders in one place so bench
#    data can calibrate them later, exactly like JOINTS[*]['kp'].
SOCKET_MASS = 0.6           # kg, socket + liner + pyramid hardware
SOCKET_LEN = 0.06           # m, socket depth below the residual limb
SOCKET_RADIUS = 0.060       # m
SOCKET_DOF = (
    # (name, type, axis, range, stiffness, damping)  -- axis in socket frame,
    # z = limb long axis, x = anterior/posterior, y = medio/lateral.
    ("socket_piston", "slide", (0.0, 0.0, 1.0), (-0.020, 0.020), 80000.0, 800.0),
    ("socket_flex",   "hinge", (0.0, 1.0, 0.0),
     (math.radians(-10.0), math.radians(10.0)), 40.0, 2.0),
    ("socket_abad",   "hinge", (1.0, 0.0, 0.0),
     (math.radians(-10.0), math.radians(10.0)), 40.0, 2.0),
    ("socket_rot",    "hinge", (0.0, 0.0, 1.0),
     (math.radians(-15.0), math.radians(15.0)), 25.0, 1.5),
)

# -- hip joints (pelvis -> thigh).  Single sagittal flexion/extension hinge for
#    now, the DOF a walking trajectory drives; ab/ad and rotation can be added.
#    Positive = flexion (thigh swings anterior, i.e. toward -X): a +Y hinge on a
#    down-pointing (-Z) thigh moves the distal end toward -X, so axis = +Y.
HIP = dict(
    axis=(0.0, 1.0, 0.0),
    range=(math.radians(-20.0), math.radians(120.0)),
    damping=1.0, armature=0.0, frictionloss=0.0, kp=150.0,
    forcerange=(-200.0, 200.0),   # placeholder hip torque limit
)

# -- contralateral (intact) leg: ONE lumped rigid body (thigh+shank+foot),
#    hip-hinged so it can swing, with a box foot that contacts the floor.  Mass
#    is the whole-leg fraction 0.161 * BODY_MASS.
CONTRA_MASS = round(0.161 * BODY_MASS, 3)   # kg (~12.08)
CONTRA_RADIUS = 0.060       # m, lumped-leg capsule radius
CONTRA_FOOT_HALF = (0.110, 0.045, 0.030)    # m, foot box half-extents (x,y,z)
CONTRA_FOOT_XOFF = -0.030   # m, foot centred slightly anterior (-X)


# ---------------------------------------------------------------------------
# Contact stiffness.
#
# The scenes used to leave solref and solimp at MuJoCo's compiler defaults.  The
# argument for changing that is about TIME, not about penetration depth, because
# time is the thing solref actually specifies.
#
# solref[0] is the contact's time constant: how long the constraint takes to
# absorb a violation.  The default is 0.02 s.  A gait cycle is about 1 s and the
# events an impedance controller switches on -- heel strike, foot flat, toe off --
# resolve at the 10 ms scale, so a 20 ms contact smears exactly the transitions
# the state machine is supposed to detect.  It is also 40x the 0.5 ms timestep,
# which means the integrator is resolving a contact it did not need resolved that
# finely.  0.002 s puts the contact an order of magnitude faster than the control
# events instead of slower, which is the whole point.
#
# Static penetration falls out of the same change.  Whatever the exact
# coefficient in MuJoCo's soft-constraint algebra -- and it depends on how the
# solimp impedance enters the regularisation, which is not worth reconstructing
# from the docs when it can simply be measured -- the resting penetration scales
# as timeconst^2.  So 0.02 -> 0.002 is a 100x reduction, and the ABSOLUTE value is
# deliberately NOT predicted here.  There is no MuJoCo in the build sandbox, an
# analytic guess would be an unverified claim of exactly the kind this repo tries
# not to make, and scripts/check_model.py can read contact.dist and get the real
# answer in one line.  PENETRATION_LIMIT below is a bound on the answer, not a
# prediction of it.
#
# Why not stiffer?  MuJoCo needs timeconst >= 2 * timestep, or the constraint is
# stiffer than the integrator can follow and the contact rings or blows up.  With
# timestep = 0.0005 that floor is 0.001, so 0.002 is deliberately 4x the timestep
# and not sitting on the limit.
#
# solimp is written out explicitly rather than inherited.  The defaults
# (0.9 0.95 0.001 0.5 2) ramp impedance from 0.9 to 0.95 over 1 mm of depth;
# 0.95..0.99 sits closer to the rigid limit once the contact is engaged, which is
# what a foot on a floor wants.  Writing them also makes them visible in the XML,
# and every other physically meaningful constant in this model is visible.
#
# IMPORTANT, and the reason the floor shares a default class with the collision
# geoms: MuJoCo does NOT take the stiffer of two contacting geoms.  At equal geom
# priority it takes a solmix-weighted AVERAGE of solref and solimp.  Setting these
# on the leg and leaving the floor plane at the default would average 0.002 with
# 0.02 and hand back most of the benefit, silently.  Both sides are set from one
# place so they cannot drift apart, and validate_mjcf.py resolves the class chain
# of every collidable geom and fails if they ever disagree.
TIMESTEP = 0.0005
SOLREF = (0.002, 1.0)                      # (timeconst s, dampratio)
SOLIMP = (0.95, 0.99, 0.001, 0.5, 2.0)     # (d0, dwidth, width, midpoint, power)
FRICTION = (0.9, 0.9, 0.005)               # (sliding, torsional, rolling)
CONDIM = 3
# The bound check_model.py holds MuJoCo to once the sole is resting on the floor.
# Chosen against the geometry rather than against a formula: the contact patch is
# flat to 0.49 mm rms, so a penetration of half a millimetre is the point at which
# the sole sinking starts to matter next to the shape of the sole itself.  It is a
# guard against "the leg sank into the ground again", not a value to match.
PENETRATION_LIMIT = 0.0005                 # m
# Poses in the collision-sufficiency sweep, knee x ankle.  26 x 26 puts a sample
# every 5 deg on the knee and every 2 deg on the ankle, which is finer than the
# geometry varies: the margin below is a smooth function of both angles, so the
# grid only has to be fine enough not to step over the minimum, not to resolve
# it precisely.  See OslModel.sweep_clearance.
SWEEP_GRID = (26, 26)
# Slack when asking whether a part is inside a module box, which decides which of
# the sweep's two margins it counts toward.  Not a tightness claim: the boxes ARE
# the bounding boxes of their member parts, so the extremal parts sit exactly on
# the faces and the test is really "did anything move".  0.1 mm because
# check_model.py must reach the same verdict from MuJoCo's mesh vertices, which
# are float32 and so carry about 1e-5 m of rounding -- ten times smaller than this
# and ten times larger than an exact comparison would tolerate.  Getting the
# verdict wrong cannot hide a collision hole either way: the strict margin covers
# every visual mesh regardless of how it is classified.
BOX_CONTAIN_TOL = 1e-4                     # m

# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------


def visual_origins(urdf_path: str) -> dict:
    """
    <visual><origin> per link.

    This is load-bearing, not defensive.  207 of the 682 links carry a
    non-identity visual origin, the largest a 206 mm translation on the
    Variflex foot, and ignoring them makes the knee and ankle modules
    interpenetrate.  Two independent checks say the visual origin must be
    composed onto the link pose:

      * folding it in reproduces the part COM the export declares in
        <inertial><origin> to 0.002 mm at the 90th percentile, versus 10.15 mm
        at the 90th percentile (158.5 mm worst case) if the mesh is taken
        as-is;
      * the assembled leg is then 251 x 119 x 490 mm, a believable knee-ankle
        build, instead of 211 x 327 x 532 mm -- 327 mm of medio-lateral width
        on a device that is under 120 mm wide is the giveaway.

    Regenerate the side-by-side image with  python tools/verify_visorigin.py
    (writes build/verify/visorigin_AB.png) and the full numeric report with
    python tools/cad_findings.py.

    (The <inertial> *masses* are worthless, but its origin is geometric and
    therefore still trustworthy -- which is what makes it a usable cross-check.)
    """
    out = {}
    for el in ET.parse(urdf_path).getroot().findall("link"):
        o = el.find("visual/origin")
        if o is None:
            out[el.get("name")] = (np.zeros(3), np.eye(3))
        else:
            out[el.get("name")] = (
                np.array([float(v) for v in o.get("xyz", "0 0 0").split()]),
                rpy_to_mat(*[float(v) for v in o.get("rpy", "0 0 0").split()]),
            )
    return out


def mesh_ident(fname: str, used: dict) -> str:
    """Stable, unique, MJCF-safe asset name for a mesh file."""
    if fname in used:
        return used[fname]
    stem = os.path.splitext(os.path.basename(fname))[0]
    name = "".join(c if c.isalnum() else "_" for c in stem).strip("_").lower()
    name = name or "mesh"
    base, k = name, 1
    while name in used.values():
        k += 1
        name = f"{base}_{k}"
    used[fname] = name
    return name


def principal(I: np.ndarray) -> tuple:
    """
    Symmetric inertia -> (quat, diaginertia) with MuJoCo's two requirements
    enforced: strictly positive eigenvalues AND the triangle inequality
    (A + B >= C).  Returns (quat, diag, was_repaired).
    """
    w, V = np.linalg.eigh(0.5 * (I + I.T))  # ascending, orthonormal columns
    if np.linalg.det(V) < 0:
        V = V.copy()
        V[:, 0] = -V[:, 0]
    w = np.asarray(w, float)
    repaired = False

    floor = max(1e-9, 1e-6 * float(w.max()))
    if w.min() < floor:
        w = np.maximum(w, floor)
        repaired = True
    # w is ascending, so the triangle inequality reduces to w0 + w1 >= w2
    if w[0] + w[1] < w[2]:
        w[0] = w[2] - w[1] + 1e-9
        repaired = True
    return mat_to_quat(V), w, repaired


# Significant digits for every number emitted into the MJCF.
#
# This was 6, and 6 is not enough.  scripts/check_model.py holds MuJoCo to
# models/*.pred.json at 1e-6 kg and 1e-4 m, but the sidecar records full
# float64 while the XML was being rounded, so the XML itself was the tighter
# constraint: the shank's 3.590757962651618 kg became "3.59076", a 2.037 mg
# error, and the three segment masses summed to a 1.666 mg error on the total.
# Both surfaced as FAILs that looked like modelling bugs and were pure
# formatting.  The fix belongs here rather than in the tolerance -- loosening
# TOL_MASS would have hidden a real density error of the same size.  12 digits
# is comfortably inside float64's ~15-17 and costs a few kB of XML.
SIGDIG = 12


def fmt(v) -> str:
    return " ".join(f"{float(x):.{SIGDIG}g}" for x in np.asarray(v).ravel())


def _chain(pts: np.ndarray, sign: float) -> np.ndarray:
    """One monotone chain over lexicographically sorted 2-D points."""
    hull: list = []
    for p in pts:
        while len(hull) >= 2:
            (x1, z1), (x2, z2) = hull[-2], hull[-1]
            cross = (x2 - x1) * (p[1] - z1) - (z2 - z1) * (p[0] - x1)
            if sign * cross <= 0:
                hull.pop()
            else:
                break
        hull.append((float(p[0]), float(p[1])))
    return np.array(hull)


def sagittal_hull(pts: np.ndarray, lower_only: bool = False) -> np.ndarray:
    """
    Convex hull of a point cloud projected to the sagittal (x, z) plane.

    Both joints rotate about world Y, so a rotation acts on (x, z) alone and
    leaves y untouched.  The lowest point of a rigid group after any such
    rotation is the minimum of a linear functional, which is attained at a hull
    vertex -- so a group of 40k triangles can be reduced to a few dozen points
    ONCE and swept over the whole joint envelope for free.

    `lower_only` returns just the lower chain, which is what decides the face a
    floor plane can touch.  It is NOT enough for the sweep: the foot's total
    rotation is knee minus ankle, which reaches -150 deg at full flexion, and
    once the rotation passes 90 deg the lowest point migrates to the upper
    chain.  Sweeps therefore take the full hull.
    """
    p = np.unique(np.round(np.asarray(pts, float)[:, [0, 2]], 9), axis=0)
    p = p[np.lexsort((p[:, 1], p[:, 0]))]
    if len(p) < 3:
        return p
    low = _chain(p, +1.0)
    if lower_only:
        return low
    return np.unique(np.vstack([low, _chain(p[::-1], +1.0)]), axis=0)


def rot_y(a: float) -> np.ndarray:
    """2-D rotation acting on (x, z) for a right-handed turn about +Y."""
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, s], [-s, c]])


# ---------------------------------------------------------------------------
# model assembly
# ---------------------------------------------------------------------------


class OslModel:
    def __init__(self) -> None:
        self.cad = OslCad(URDF, MESHES)
        self.vorigin = visual_origins(URDF)
        self.notes: list = []
        self.seg_of_link, self.cluster_report = self._segment()
        self.kept, self.culled = self._cull()
        self.inertia = {s: self._segment_inertia(s) for s in SEGMENTS}
        self._lift: dict = {}
        self._check_foot()
        self.modules = self._modules()
        self.module_bounds = self._module_bounds()
        self.sole_edge, self.flat_foot_ankle = self._sole_contact_edge()

    # -- geometry ----------------------------------------------------------

    def geom_pose(self, link: str) -> tuple:
        """
        World pose of the link's *visual mesh*: link pose composed with the
        <visual><origin>.  Everything downstream -- centroids, inertias,
        emitted geoms, collision boxes -- uses this, never the bare link pose.
        """
        p, R = self.cad.pose[link]
        vp, vR = self.vorigin[link]
        return p + R @ vp, R @ vR

    def world_tris(self, link: str) -> np.ndarray:
        v = self.cad.triangles(self.cad.links[link].mesh_file).reshape(-1, 3)
        p, R = self.geom_pose(link)
        return v @ R.T + p

    def _check_foot(self) -> None:
        """
        Confirm the export's own foot mate is sane rather than assuming it.
        The previous pass reconstructed the foot from invented constants after
        diagnosing this mate as corrupt; that diagnosis was an artefact of
        ignoring <visual><origin>, so it is re-tested here on every build.
        """
        _, R = self.geom_pose(FOOT_LINK)
        flex = np.array([0.0, 1.0, 0.0])
        length_axis, sole_normal = R[:, 2], R[:, 1]
        a_len = math.degrees(math.acos(min(1.0, abs(float(length_axis @ flex)))))
        a_sole = math.degrees(math.acos(min(1.0, abs(float(sole_normal @ [0, 0, 1])))))
        w = self.world_tris(FOOT_LINK)
        lo, hi = w.min(0), w.max(0)
        self.foot_sole_z = float(lo[2])
        # One pass over every kept mesh, giving both the bbox that check_model.py
        # holds MuJoCo to and the lowest point the ground scene must clear --
        # which need not be the sole, since bolt heads and washers sit below the
        # blade.
        blo, bhi = np.full(3, np.inf), np.full(3, -np.inf)
        for seg in SEGMENTS:
            for n in self.kept.get(seg, []):
                t = self.world_tris(n)
                blo, bhi = np.minimum(blo, t.min(0)), np.maximum(bhi, t.max(0))
        self.mesh_bbox = (blo, bhi)
        self.model_min_z = float(blo[2])
        if a_len < 85.0 or a_sole > 8.0:
            self.notes.append(
                f"FOOT MATE LOOKS WRONG: length axis {a_len:.2f} deg from the "
                f"ankle axis (want ~90), sole {a_sole:.2f} deg from level "
                f"(want <8) -- investigate before trusting this model"
            )
        else:
            self.notes.append(
                f"foot mate taken from the export as-is and cross-checked: "
                f"length axis {a_len:.2f} deg from the ankle axis, sole "
                f"{a_sole:.2f} deg from level (real alignment tilt, preserved), "
                f"blade x=[{lo[0]:+.4f},{hi[0]:+.4f}] with the toe toward -X, "
                f"sole at z={lo[2]:+.4f} -> ankle {ANKLE_POS[2] - lo[2]:.4f} m "
                f"above the ground"
            )

    # -- segmentation ------------------------------------------------------

    def _segment(self) -> tuple:
        cad = self.cad
        seg_of_link, report = {}, []
        for rep, mem in cad.rigid_clusters().items():
            mesh = [m for m in mem if cad.links[m].mesh_file]
            if not mesh:
                continue  # Onshape phantom-only cluster: no geometry, no mass
            vol, c = 0.0, np.zeros(3)
            for m in mesh:
                mp = cad.mass_props(cad.links[m].mesh_file)
                p, R = self.geom_pose(m)
                vol += mp.volume
                c += mp.volume * (p + R @ mp.centroid)
            c = c / vol if vol > 0 else c
            dk = math.hypot(c[0] - KNEE_POS[0], c[2] - KNEE_POS[2])
            da = math.hypot(c[0] - ANKLE_POS[0], c[2] - ANKLE_POS[2])
            if dk < AXIS_CLUSTER_RADIUS:
                seg, why = "knee_prox", "concentric with the knee axis (rotating output)"
            elif da < AXIS_CLUSTER_RADIUS:
                seg, why = "foot", "concentric with the ankle axis (rotating output)"
            elif c[2] > KNEE_POS[2]:
                seg, why = "knee_prox", "above the knee axis"
            elif c[2] < ANKLE_POS[2]:
                seg, why = "foot", "below the ankle axis"
            else:
                seg, why = "shank", "between the axes"
            for m in mesh:
                seg_of_link[m] = seg
            report.append((seg, vol, c[2], len(mesh), max(
                mesh, key=lambda m: cad.mass_props(cad.links[m].mesh_file).volume), why))
        report.sort(key=lambda r: -r[2])
        return seg_of_link, report

    # -- culling -----------------------------------------------------------

    def _cull(self) -> tuple:
        kept = collections.defaultdict(list)
        culled = collections.defaultdict(list)
        for link, seg in self.seg_of_link.items():
            mp = self.cad.mass_props(self.cad.links[link].mesh_file)
            (kept if mp.volume >= CULL_VOLUME else culled)[seg].append(link)
        for s in SEGMENTS:
            kept[s].sort()
            culled[s].sort()
        return dict(kept), dict(culled)

    # -- mass properties ---------------------------------------------------

    def _segment_inertia(self, seg: str) -> dict:
        """Exact mass / COM / inertia of a segment, in world axes."""
        cad = self.cad
        M, first, I = 0.0, np.zeros(3), np.zeros((3, 3))
        parts = self.kept.get(seg, []) + self.culled.get(seg, [])
        for n in parts:
            mp = cad.mass_props(cad.links[n].mesh_file)
            rho, _ = density_for(n)
            m = rho * mp.volume
            if m <= 0.0:
                continue
            p, R = self.geom_pose(n)
            c = p + R @ mp.centroid
            Ic = rho * (R @ mp.inertia @ R.T)  # about c, world-aligned
            M += m
            first += m * c
            I += Ic + m * (float(c @ c) * np.eye(3) - np.outer(c, c))
        M, first, I = self._add_motors(seg, M, first, I)
        com = first / M
        Icom = I - M * (float(com @ com) * np.eye(3) - np.outer(com, com))
        return dict(mass=M, com=com, inertia=0.5 * (Icom + Icom.T), nparts=len(parts))

    def _add_motors(self, seg: str, M: float, first: np.ndarray,
                    I: np.ndarray) -> tuple:
        """
        Optionally lump MOTOR_MASS at each motor-coupling centroid.

        No-op unless someone sets MOTOR_MASS to a measured value; see the long
        comment there for why the default is 0.  Added as point masses, which
        understates the real rotor+stator inertia -- a 40 mm-diameter motor
        carries roughly m*r^2/2 ~ 4e-5 kg.m^2 about its own spin axis at
        0.2 kg, four orders below the segment's own tensor, so the point-mass
        approximation costs nothing at the segment level.  It does NOT cover
        reflected rotor inertia at the joint, which is what `armature` is for
        and which matters far more (x ratio^2 = x20.7).
        """
        if MOTOR_MASS <= 0.0:
            return M, first, I
        for n in MOTOR_LINKS:
            if self.seg_of_link.get(n) != seg:
                continue
            mp = self.cad.mass_props(self.cad.links[n].mesh_file)
            p, R = self.geom_pose(n)
            c = p + R @ mp.centroid
            M += MOTOR_MASS
            first += MOTOR_MASS * c
            I += MOTOR_MASS * (float(c @ c) * np.eye(3) - np.outer(c, c))
            self.notes.append(
                f"{seg}: +{MOTOR_MASS:.3f} kg lumped motor at {np.round(c, 4)} "
                f"(coupling {n}) -- MOTOR_MASS is set, so this model is NOT "
                f"purely CAD-derived"
            )
        return M, first, I

    # -- MJCF emission -----------------------------------------------------

    def _asset_block(self) -> tuple:
        used: dict = {}
        lines, tri = [], 0
        for seg in SEGMENTS:
            for n in self.kept.get(seg, []):
                f = self.cad.links[n].mesh_file
                if f in used:
                    continue
                name = mesh_ident(f, used)
                tri += self.cad.mass_props(f).ntri
                lines.append(f'<mesh name="{name}" file="{f}"/>')
        return used, sorted(lines), tri

    def _geoms(self, seg: str, origin: np.ndarray, used: dict) -> list:
        out = []
        for n in self.kept.get(seg, []):
            p, R = self.geom_pose(n)
            rgba = self.cad.links[n].rgba
            out.append(
                f'<geom class="visual" mesh="{used[self.cad.links[n].mesh_file]}"'
                f' pos="{fmt(p - origin)}" quat="{fmt(mat_to_quat(R))}"'
                f' rgba="{fmt(rgba[:3])} 1"/>'
            )
        return out

    def _inertial(self, seg: str, origin: np.ndarray) -> str:
        d = self.inertia[seg]
        q, diag, repaired = principal(d["inertia"])
        if repaired:
            self.notes.append(f"{seg}: inertia regularised to satisfy MuJoCo's checks")
        return (
            f'<inertial pos="{fmt(d["com"] - origin)}" quat="{fmt(q)}"'
            f' mass="{fmt(d["mass"])}" diaginertia="{fmt(diag)}"/>'
        )

    # -- collision geometry ------------------------------------------------

    def _modules(self) -> dict:
        """
        segment -> [(label, [link, ...]), ...], the groups that each get one
        bounding box.  Membership comes from the CAD's own named housings; see
        MODULE_SEEDS for the rule and why it is not a hand-picked z threshold.
        """
        def zspan(names) -> tuple:
            lo, hi = np.inf, -np.inf
            for n in names:
                t = self.world_tris(n)[:, 2]
                lo, hi = min(lo, float(t.min())), max(hi, float(t.max()))
            return lo, hi

        spans = {k: zspan(v) for k, v in MODULE_SEEDS.items()}
        gap = spans["knee_module"][0] - spans["ankle_module"][1]
        if gap <= 0.0:
            self.notes.append(
                f"MODULE SEEDS OVERLAP by {-gap * 1000:.1f} mm -- the "
                f"knee/ankle housing z-spans are supposed to be disjoint, so "
                f"the shank module split is no longer unambiguous"
            )

        def overlap(a, b) -> float:
            return max(0.0, min(a[1], b[1]) - max(a[0], b[0]))

        groups: dict = {k: [] for k in MODULE_SEEDS}
        groups["mid_pylon"] = []
        for n in sorted(self.kept.get("shank", [])):
            s = zspan([n])
            best, ov = None, 0.0
            for k, sp in spans.items():
                o = overlap(s, sp)
                if o > ov:
                    best, ov = k, o
            groups[best if best else "mid_pylon"].append(n)

        return {
            "knee_prox": [("knee_prox_shell", sorted(self.kept.get("knee_prox", [])))],
            "shank": [(k, groups[k]) for k in
                      ("knee_module", "mid_pylon", "ankle_module") if groups[k]],
            "foot": [],  # the blade mesh is the foot's collision geometry
        }

    def _module_bounds(self) -> dict:
        """
        segment -> [(label, lo, hi), ...].  Computed once, in __init__, so the
        fill-fraction notes are recorded once rather than once per scene.
        """
        out: dict = {}
        for seg in SEGMENTS:
            rows = []
            for label, names in self.modules.get(seg, []):
                lo, hi = np.full(3, np.inf), np.full(3, -np.inf)
                vol = 0.0
                for n in names:
                    t = self.world_tris(n)
                    lo, hi = np.minimum(lo, t.min(0)), np.maximum(hi, t.max(0))
                    vol += self.cad.mass_props(self.cad.links[n].mesh_file).volume
                box = float(np.prod(hi - lo))
                self.notes.append(
                    f"{seg}: bounding box {label} {np.round(hi - lo, 4)} m over "
                    f"{len(names)} parts, {100 * vol / box:.1f} % full -- "
                    f"protective volume, not a shape claim"
                )
                rows.append((label, lo, hi))
            out[seg] = rows
        return out

    def _module_boxes(self, seg: str, origin: np.ndarray) -> list:
        """
        One axis-aligned bounding box per module.  World-aligned, which is also
        body-aligned at qpos = 0, so no quat is needed and none is emitted --
        the validator asserts body frames are unrotated for the same reason.
        """
        return [
            f'<geom class="collision" name="{label}" type="box"'
            f' pos="{fmt(0.5 * (lo + hi) - origin)}" size="{fmt(0.5 * (hi - lo))}"/>'
            for label, lo, hi in self.module_bounds.get(seg, [])
        ]

    def sweep_clearance(self, nk: int = 0, na: int = 0) -> dict:
        """
        Over the whole joint envelope, how much room is there between the parts
        that CAN collide and the parts that cannot?

        The four module boxes and the blade mesh are the only geometry with
        contype set.  Every other part of the leg is a visual mesh MuJoCo will
        happily drive through the floor, so the collision model is sufficient
        only if no visual mesh can get below the lowest collidable point.  The
        measurement is exactly that: sweep knee and ankle over their full ranges
        and, at each pose, take the lowest point of any visual mesh minus the
        lowest point of any collision geom.  A positive worst case means no mesh
        can reach the floor before a collision geom does.  A negative one means
        the collision model has a hole and the boxes need rethinking.

        Two margins come out, because they answer different questions.  `margin`
        is over every visual mesh; it is the strict form of the claim and the one
        scripts/check_model.py can reproduce knowing nothing about how the boxes
        were built.  But most parts sit inside a box carried by their own body,
        and a convex box cannot rise above a part it contains when the two rotate
        together -- so those parts are safe by construction, and their margin
        only measures how close a box corner passes to the part that defined it.
        `unboxed` restricts to the parts no box contains, which is where the
        sweep can actually discover something: on this model that is the foot's 46
        non-blade parts, since the foot's only collision geom is the blade mesh
        and a mesh bounds nothing.  Those are the parts the sweep exists for.

        Cost is kept down by reducing each rigid group to its sagittal convex
        hull once -- see sagittal_hull -- so this is a few thousand 2x2 matrix
        products rather than a sweep over 1.3 M triangles.

        This used to be a number in a comment with no code behind it.  Both
        margins go into the sidecar so check_model.py can re-derive them from
        MuJoCo's own kinematics and hold the two to agreement.
        """
        nk, na = nk or SWEEP_GRID[0], na or SWEEP_GRID[1]
        boxes = [(seg, label, lo, hi) for seg in SEGMENTS
                 for label, lo, hi in self.module_bounds.get(seg, [])]
        coll = [(seg, label, sagittal_hull(np.array(
                    [[x, 0.0, z] for x in (lo[0], hi[0]) for z in (lo[2], hi[2])])))
                for seg, label, lo, hi in boxes]
        coll.append(("foot", "sole", sagittal_hull(self.world_tris(FOOT_LINK))))

        unc = []
        for seg in SEGMENTS:
            for n in sorted(self.kept.get(seg, [])):
                if n == FOOT_LINK:
                    continue
                t = self.world_tris(n)
                lo_, hi_ = t.min(0), t.max(0)
                # Containment is tested on the world AABB at qpos = 0, which is
                # exact here because the boxes are axis aligned in that same
                # frame, and it stays true at every pose only because the box
                # and the part belong to the same body.
                boxed = any(bseg == seg
                            and bool((lo_ >= lo - BOX_CONTAIN_TOL).all()
                                     and (hi_ <= hi + BOX_CONTAIN_TOL).all())
                            for bseg, _, lo, hi in boxes)
                unc.append((seg, n, sagittal_hull(t), boxed))
        if not unc or not boxes:
            return {}

        K = np.array([KNEE_POS[0], KNEE_POS[2]])
        A = np.array([ANKLE_POS[0], ANKLE_POS[2]])

        def place(seg: str, H: np.ndarray, qk: float, qa: float) -> np.ndarray:
            """Hull points of a group at a given pose, in the sagittal plane.

            The knee axis is -Y and the ankle axis is +Y (see JOINTS), so a
            positive knee angle turns by -qk and a positive ankle angle by +qa.
            The foot takes both, ankle first, because the ankle pivot itself is
            carried by the knee.
            """
            if seg == "knee_prox":
                return H
            if seg == "shank":
                return (H - K) @ rot_y(-qk).T + K
            P = (H - A) @ rot_y(qa).T + A
            return (P - K) @ rot_y(-qk).T + K

        worst, free = dict(margin=math.inf), dict(margin=math.inf)
        for qk in np.linspace(*JOINTS["knee"]["range"], nk):
            for qa in np.linspace(*JOINTS["ankle"]["range"], na):
                zc, cname = math.inf, ""
                for seg, label, H in coll:
                    z = float(place(seg, H, qk, qa)[:, 1].min())
                    if z < zc:
                        zc, cname = z, label
                za, aname = math.inf, ""
                zb, bname = math.inf, ""
                for seg, n, H, boxed in unc:
                    z = float(place(seg, H, qk, qa)[:, 1].min())
                    if z < za:
                        za, aname = z, n
                    if not boxed and z < zb:
                        zb, bname = z, n
                pose = dict(knee_deg=math.degrees(qk), ankle_deg=math.degrees(qa),
                            lowest_collision=cname)
                if za - zc < worst["margin"]:
                    worst = dict(pose, margin=za - zc, lowest_uncovered=aname)
                if zb - zc < free["margin"]:
                    free = dict(pose, margin=zb - zc, lowest_uncovered=bname)

        out = dict(worst, unboxed=free, grid=[nk, na], nparts=len(unc),
                   nunboxed=sum(1 for *_, b in unc if not b),
                   knee_range_deg=[math.degrees(x) for x in JOINTS["knee"]["range"]],
                   ankle_range_deg=[math.degrees(x) for x in JOINTS["ankle"]["range"]])
        if worst["margin"] <= 0.0:
            self.notes.append(
                f"COLLISION MODEL HAS A HOLE: at knee {worst['knee_deg']:+.1f} deg, "
                f"ankle {worst['ankle_deg']:+.1f} deg the visual mesh "
                f"{worst['lowest_uncovered']} sits {-1e3 * worst['margin']:.1f} mm "
                f"BELOW the lowest collision geom ({worst['lowest_collision']}), so it "
                f"can pass through the floor"
            )
        else:
            self.notes.append(
                f"collision sufficiency: over {nk}x{na} poses every one of the "
                f"{out['nparts']} visual meshes stays >= {1e3 * worst['margin']:.1f} mm "
                f"above the lowest collision geom, and the {out['nunboxed']} that no box "
                f"contains stay >= {1e3 * free['margin']:.1f} mm above it "
                f"(worst at knee {free['knee_deg']:+.1f}, ankle {free['ankle_deg']:+.1f} deg)"
            )
        return out

    def _sole_contact_edge(self) -> tuple:
        """
        The face of the blade the leg actually stands on, and the ankle angle
        that levels it.

        This exists because the ground scene had a real bug: at ankle = 0 the
        sole is 2.4 deg toe-down, so the leg balances on the keel of the
        forefoot blade instead of standing flat, and the smallest disturbance --
        or any commanded ankle motion -- tips it.  Rather than assert a flat
        pose, derive it: take the blade's vertices in the foot body frame,
        project to the sagittal plane, and compute the LOWER convex hull.  That
        hull is what a floor plane can touch.

        Its longest edge runs 149.5 mm from the distal tip of the heel blade at
        x = -0.0894 to the heel pad at x = +0.0600, rising 6.3 mm, i.e.
        +2.3971 deg.  The next longest edge is 34.7 mm, so this one dominates by
        4.3x and there is no ambiguity about which face is the sole.  Rotating
        the foot by +2.3971 deg (dorsiflexion, well inside the +20 deg limit)
        levels it, and the result is a real patch rather than a line: 173 blade
        vertices then lie within 2 mm of the lowest point, spanning 161 mm fore-
        aft and 64 mm across, flat to 0.49 mm rms, with residual tilts of
        +0.29 deg sagittal and under 0.01 deg frontal.  So the flat-foot keyframe
        stands the leg on both blades, not on an edge, and the frontal component
        of the build's 2.18 deg alignment tilt turns out to be negligible --
        which matters, because a single ankle hinge could not have corrected it.
        Those patch numbers are regenerated by tools/cad_findings.py, not typed.

        Returns ((x0, z0), (x1, z1), length) and the ankle angle in radians.
        """
        H = sagittal_hull(self.world_tris(FOOT_LINK) - ANKLE_POS, lower_only=True)

        d = np.diff(H, axis=0)
        L = np.hypot(d[:, 0], d[:, 1])
        i = int(np.argmax(L))
        a, b = H[i], H[i + 1]
        alpha = math.atan2(b[1] - a[1], b[0] - a[0])

        second = float(np.sort(L)[-2]) if len(L) > 1 else 0.0
        if L[i] < 3.0 * second:
            self.notes.append(
                f"SOLE CONTACT FACE IS AMBIGUOUS: longest lower-hull edge "
                f"{L[i] * 1000:.1f} mm vs {second * 1000:.1f} mm for the next "
                f"one -- the flat-foot pose below may be standing on the wrong "
                f"face"
            )
        self.notes.append(
            f"sole contact face: lower-hull edge {L[i] * 1000:.1f} mm from "
            f"x={a[0]:+.4f} to x={b[0]:+.4f} (next longest {second * 1000:.1f} "
            f"mm), {math.degrees(alpha):+.4f} deg toe-down at ankle=0 -> "
            f"flat-foot ankle {math.degrees(alpha):+.4f} deg"
        )
        return (tuple(a), tuple(b), float(L[i])), float(alpha)

    def _foot_collision(self, used: dict) -> list:
        """
        The sole: the blade's own mesh, at the blade's own visual pose.  See the
        MODULE_SEEDS comment for why this replaced a tilted box, and note that
        the pose here is deliberately identical to the visual geom's -- if the
        two ever drift apart, the leg is standing on something the viewer does
        not draw, which is what went wrong last time.
        """
        p, R = self.geom_pose(FOOT_LINK)
        mesh = used[self.cad.links[FOOT_LINK].mesh_file]
        return [
            f'<geom class="collision" name="sole" type="mesh" mesh="{mesh}"'
            f' pos="{fmt(p - ANKLE_POS)}" quat="{fmt(mat_to_quat(R))}"/>'
        ]

    def _sole_site(self) -> str:
        """
        A box site over the whole blade, so the `sole_touch` sensor integrates
        normal force across the real footprint.  It used to be a 6 mm sphere
        4 mm above the lowest point of the sole, which sat in the arch -- a
        region that never touches anything -- so the sensor read ~0 through
        stance.  MuJoCo's touch sensor sums the normal forces of contacts whose
        position falls inside the site volume, so the volume has to cover
        everywhere the sole can make contact, which is the blade.
        """
        v = self.world_tris(FOOT_LINK)
        lo, hi = v.min(0), v.max(0)
        return (
            f'<site name="sole_site" type="box"'
            f' pos="{fmt(0.5 * (lo + hi) - ANKLE_POS)}"'
            f' size="{fmt(0.5 * (hi - lo))}" rgba="0.9 0.3 0.3 0.15"/>'
        )

    def lowest_z(self) -> float:
        """
        Lowest point of anything the ground scene can collide with or draw.

        Now simply the mesh minimum.  It used to have to consider the sole box
        separately, because that box dipped 4.6 mm below the blade it was
        supposed to hug; the mesh collision geom is the blade, and every other
        collision geom is a bounding box of meshes that are already in this
        minimum, so there is nothing left to take a max over.
        """
        return self.model_min_z

    def flat_foot_lift(self) -> float:
        """
        Ground-scene base height for the flat-foot keyframe.  Levelling the sole
        raises its lowest point by 3.8 mm relative to qpos = 0 -- the toe comes
        up more than the heel goes down -- so the keyframe has to lower the base
        by the same amount or the leg starts 3.8 mm in the air and drops.
        """
        c, s = math.cos(self.flat_foot_ankle), math.sin(self.flat_foot_ankle)
        R = np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])
        lo = np.inf
        for n in self.kept.get("foot", []):
            t = (self.world_tris(n) - ANKLE_POS) @ R.T + ANKLE_POS
            lo = min(lo, float(t[:, 2].min()))
        for seg in ("knee_prox", "shank"):
            for n in self.kept.get(seg, []):
                lo = min(lo, float(self.world_tris(n)[:, 2].min()))
        return -lo + SPAWN_CLEARANCE

    def highest_z(self) -> float:
        """Top of the proximal pyramid, i.e. where a socket would bolt on."""
        return max(float(self.world_tris(n)[:, 2].max())
                   for n in self.kept.get("knee_prox", []))

    def _lift_for(self, scene: str) -> float:
        """Vertical offset applied to the whole leg: 0 on the bench, and enough
        to put the lowest collidable point SPAWN_CLEARANCE above the floor on
        the ground."""
        return 0.0 if scene == "bench" else -self.lowest_z() + SPAWN_CLEARANCE

    # -- primitive (non-CAD) inertia, for the human scaffold ---------------

    @staticmethod
    def _box_diag(m: float, half) -> tuple:
        """Diagonal inertia of a solid box about its centre, half-extents `half`."""
        hx, hy, hz = half
        return (m / 3.0 * (hy * hy + hz * hz),
                m / 3.0 * (hx * hx + hz * hz),
                m / 3.0 * (hx * hx + hy * hy))

    @staticmethod
    def _cyl_diag(m: float, r: float, length: float) -> tuple:
        """Diagonal inertia of a solid cylinder about its centre, long axis = z."""
        transverse = m * (3.0 * r * r + length * length) / 12.0
        axial = 0.5 * m * r * r
        return (transverse, transverse, axial)

    @staticmethod
    def _prim_inertial(com, mass: float, diag) -> str:
        """An explicit <inertial> for a primitive scaffold body (axis-aligned)."""
        return (f'<inertial pos="{fmt(com)}" quat="1 0 0 0" mass="{fmt(mass)}"'
                f' diaginertia="{fmt(diag)}"/>')

    # -- the CAD device sub-tree, shared by every scene --------------------

    def _emit_device(self, add, used: dict, d0: int, *, freejoint: bool,
                     collision: bool, knee_prox_pos: str) -> None:
        """
        Emit the CAD-derived device knee_prox -> shank -> foot, rooted at
        indentation depth d0.

        The geometry is identical in every scene; only three things vary, all
        passed in: where the root sits (knee_prox_pos), whether it carries a
        freejoint (ground only), and whether the collision geometry is emitted
        (any scene with a floor).  Factoring it out lets the walk scene nest the
        very same device under the socket, and keeps the bench/ground output
        byte-for-byte identical to before -- call with d0=2.
        """
        j = JOINTS
        add(d0, f'<body name="knee_prox" pos="{knee_prox_pos}">')
        if freejoint:
            add(d0 + 1, '<freejoint name="root"/>')
        add(d0 + 1, self._inertial("knee_prox", np.zeros(3)),
                    self._geoms("knee_prox", np.zeros(3), used),
            f'<site name="imu_knee_prox" pos="{fmt(KNEE_POS + [0, 0, 0.05])}"'
            ' size="0.005" rgba="0.1 0.8 0.9 1"/>')
        if collision:
            add(d0 + 1, self._module_boxes("knee_prox", np.zeros(3)))

        add(d0 + 1, f'<body name="shank" pos="{fmt(KNEE_POS)}">')
        add(d0 + 2, f'<joint name="knee" type="hinge" axis="{fmt(j["knee"]["axis"])}" pos="0 0 0"'
               f' range="{fmt(j["knee"]["range"])}" damping="{j["knee"]["damping"]}"'
               f' armature="{j["knee"]["armature"]}"'
               f' frictionloss="{j["knee"]["frictionloss"]}"/>',
               self._inertial("shank", KNEE_POS),
               self._geoms("shank", KNEE_POS, used),
               '<site name="imu_shank" pos="0 0 -0.12" size="0.005" rgba="0.1 0.8 0.9 1"/>',
               '<site name="loadcell" pos="0 0 -0.184" size="0.005" rgba="0.9 0.9 0.2 1"/>')
        if collision:
            add(d0 + 2, self._module_boxes("shank", KNEE_POS))

        add(d0 + 2, f'<body name="foot" pos="{fmt(ANKLE_POS - KNEE_POS)}">')
        add(d0 + 3, f'<joint name="ankle" type="hinge" axis="{fmt(j["ankle"]["axis"])}" pos="0 0 0"'
               f' range="{fmt(j["ankle"]["range"])}" damping="{j["ankle"]["damping"]}"'
               f' armature="{j["ankle"]["armature"]}"'
               f' frictionloss="{j["ankle"]["frictionloss"]}"/>',
               self._inertial("foot", ANKLE_POS),
               self._geoms("foot", ANKLE_POS, used))
        if collision:
            add(d0 + 3, self._foot_collision(used), self._sole_site())
        add(d0 + 2, "</body>")
        add(d0 + 1, "</body>")
        add(d0, "</body>")

    # -- the walk scene: human scaffold + the device it bolts onto ---------

    def _emit_walk(self, add, used: dict) -> None:
        """
        Emit the walk-scene body tree:

            pelvis (suspended, welded to the world -- the gantry)
              +- residual_thigh   via `hip`         (1 sagittal DOF)
              |    +- socket       via 4 PASSIVE DOFs (piston + 3 rotations)
              |         +- knee_prox -> shank -> foot   (the CAD device)
              +- contra_thigh      via `contra_hip`  (1 sagittal DOF, lumped leg)

        Only knee_prox/shank/foot are CAD.  Everything else is an anthropometric
        placeholder (see the PHASE 2 constants block) and is flagged at runtime.
        The hip height is DERIVED, not chosen, so the device sole rests
        SPAWN_CLEARANCE above the floor at the neutral pose -- the same trick the
        ground scene uses for its flat keyframe.
        """
        hi_z = self.highest_z()                       # pyramid top, device frame
        knee_prox_z = -SOCKET_LEN - hi_z              # knee_prox origin in socket frame
        # Derive the hip height from the UNLEVELLED (qpos=0, CAD-pose) clearance,
        # not flat_foot_lift(): the pelvis is welded, so its height is fixed for
        # every pose.  Pinning it to the leveled lift would leave the CAD-pose
        # forefoot ~1.8 mm below the floor at the default reset (qpos=0) -- the
        # "leg through the ground" failure mode.  Using the qpos=0 lift keeps the
        # device sole SPAWN_CLEARANCE above the floor at the default pose; the
        # `stand` keyframe then levels the ankle, which only raises it further.
        device_lift = -self.lowest_z() + SPAWN_CLEARANCE      # == _lift_for("ground")
        hip_z = device_lift + hi_z + SOCKET_LEN + RES_THIGH_LEN
        l_contra = hip_z - SPAWN_CLEARANCE            # intact-leg length, hip -> sole
        self._walk = dict(hip_z=hip_z, knee_prox_z=knee_prox_z, l_contra=l_contra)

        # ---- pelvis: the suspended gantry (welded; a trajectory drives it later)
        add(2, f'<body name="pelvis" pos="0 0 {fmt(hip_z)}">')
        add(3, self._prim_inertial(np.zeros(3), PELVIS_MASS,
                                   self._box_diag(PELVIS_MASS, PELVIS_HALF)),
               f'<geom type="box" size="{fmt(np.array(PELVIS_HALF))}" contype="0"'
               ' conaffinity="0" group="2" density="0" rgba="0.75 0.72 0.68 1"/>',
               '<site name="pelvis_center" pos="0 0 0" size="0.01"'
               ' rgba="0.9 0.6 0.1 1"/>')

        # ---- residual (amputated) thigh
        add(3, f'<body name="residual_thigh" pos="0 {fmt(HIP_HALF_WIDTH)} 0">')
        add(4, f'<joint name="hip" type="hinge" axis="{fmt(np.array(HIP["axis"]))}"'
               f' pos="0 0 0" range="{fmt(HIP["range"])}" damping="{HIP["damping"]}"'
               f' armature="{HIP["armature"]}" frictionloss="{HIP["frictionloss"]}"/>',
               self._prim_inertial(np.array([0.0, 0.0, -0.5 * RES_THIGH_LEN]),
                                   RES_THIGH_MASS,
                                   self._cyl_diag(RES_THIGH_MASS, RES_THIGH_RADIUS,
                                                  RES_THIGH_LEN)),
               f'<geom type="capsule" fromto="0 0 0 0 0 {fmt(-RES_THIGH_LEN)}"'
               f' size="{fmt(RES_THIGH_RADIUS)}" contype="0" conaffinity="0"'
               ' group="2" density="0" rgba="0.82 0.66 0.60 1"/>')

        # ---- socket: 4 PASSIVE spring-damper DOFs (piston + 3 rotations)
        add(4, f'<body name="socket" pos="0 0 {fmt(-RES_THIGH_LEN)}">')
        for nm, jt, ax, rng, stiff, damp in SOCKET_DOF:
            add(5, f'<joint name="{nm}" type="{jt}" axis="{fmt(np.array(ax))}"'
                   f' pos="0 0 0" range="{fmt(rng)}" stiffness="{fmt(stiff)}"'
                   f' damping="{fmt(damp)}" springref="0"/>')
        add(5, self._prim_inertial(np.array([0.0, 0.0, -0.5 * SOCKET_LEN]),
                                   SOCKET_MASS,
                                   self._cyl_diag(SOCKET_MASS, SOCKET_RADIUS,
                                                  SOCKET_LEN)),
               f'<geom type="capsule" fromto="0 0 0 0 0 {fmt(-SOCKET_LEN)}"'
               f' size="{fmt(SOCKET_RADIUS)}" contype="0" conaffinity="0"'
               ' group="2" density="0" rgba="0.35 0.55 0.75 0.6"/>')

        # ---- the CAD device, hung from the socket (no joint: a rigid bolt)
        self._emit_device(add, used, 5, freejoint=False, collision=True,
                          knee_prox_pos=f"0 0 {fmt(knee_prox_z)}")

        add(4, "</body>")          # socket
        add(3, "</body>")          # residual_thigh

        # ---- contralateral (intact) leg: one lumped rigid body + a box foot
        foot_cz = -(l_contra - CONTRA_FOOT_HALF[2])
        cap_end = -(l_contra - 2.0 * CONTRA_FOOT_HALF[2])
        add(3, f'<body name="contra_thigh" pos="0 {fmt(-HIP_HALF_WIDTH)} 0">')
        add(4, f'<joint name="contra_hip" type="hinge"'
               f' axis="{fmt(np.array(HIP["axis"]))}" pos="0 0 0"'
               f' range="{fmt(HIP["range"])}" damping="{HIP["damping"]}"'
               f' armature="{HIP["armature"]}" frictionloss="{HIP["frictionloss"]}"/>',
               self._prim_inertial(np.array([0.0, 0.0, -0.45 * l_contra]),
                                   CONTRA_MASS,
                                   self._cyl_diag(CONTRA_MASS, CONTRA_RADIUS,
                                                  l_contra)),
               f'<geom type="capsule" fromto="0 0 0 0 0 {fmt(cap_end)}"'
               f' size="{fmt(CONTRA_RADIUS)}" contype="0" conaffinity="0"'
               ' group="2" density="0" rgba="0.60 0.60 0.66 1"/>',
               f'<geom class="collision" name="contra_foot" type="box"'
               f' pos="{fmt(CONTRA_FOOT_XOFF)} 0 {fmt(foot_cz)}"'
               f' size="{fmt(np.array(CONTRA_FOOT_HALF))}"/>',
               f'<site name="contra_sole_site" type="box"'
               f' pos="{fmt(CONTRA_FOOT_XOFF)} 0 {fmt(foot_cz)}"'
               f' size="{fmt(np.array(CONTRA_FOOT_HALF))}" rgba="0.9 0.3 0.3 0.15"/>')
        add(3, "</body>")          # contra_thigh
        add(2, "</body>")          # pelvis

        self.notes.append(
            f"WALK SCENE IS A SCAFFOLD: knee_prox/shank/foot are the CAD device, "
            f"but pelvis ({PELVIS_MASS} kg), residual_thigh ({RES_THIGH_MASS} kg, "
            f"{RES_THIGH_LEN} m), socket ({SOCKET_MASS} kg, 4 passive DOFs) and "
            f"contra_thigh ({CONTRA_MASS} kg) are anthropometric PLACEHOLDERS, "
            f"not CAD.  Hip derived at z={hip_z:.4f} m so the device sole rests "
            f"{SPAWN_CLEARANCE * 1000:.0f} mm above the floor at the default pose "
            f"(qpos=0); the `stand` keyframe levels the ankle, raising it further. "
            f"Socket stiffness/damping are unfitted.  Reconcile with myoOSL."
        )

    def _walk_controls(self, add) -> None:
        """Actuators, sensors and the neutral-stance keyframe for the walk scene.

        Actuated joints: hip, knee, ankle, contra_hip (a trajectory or controller
        drives these).  The four socket DOFs are PASSIVE -- no actuator -- and are
        only observed, via jointpos sensors, so socket compliance is visible.
        """
        a = self.flat_foot_ankle
        add(1, "<actuator>")
        add(2, f'<position name="hip_pos" joint="hip" kp="{HIP["kp"]}"'
               f' ctrlrange="{fmt(HIP["range"])}" forcerange="{fmt(HIP["forcerange"])}"/>')
        for nm in ("knee", "ankle"):
            add(2, f'<position name="{nm}_pos" joint="{nm}" kp="{JOINTS[nm]["kp"]}"'
                   f' ctrlrange="{fmt(JOINTS[nm]["range"])}"'
                   f' forcerange="{fmt(JOINTS[nm]["forcerange"])}"/>')
        add(2, f'<position name="contra_hip_pos" joint="contra_hip" kp="{HIP["kp"]}"'
               f' ctrlrange="{fmt(HIP["range"])}" forcerange="{fmt(HIP["forcerange"])}"/>')
        add(1, "</actuator>", "<sensor>")
        for nm in ("hip", "knee", "ankle", "contra_hip"):
            add(2, f'<jointpos name="{nm}_q" joint="{nm}"/>',
                   f'<jointvel name="{nm}_qd" joint="{nm}"/>',
                   f'<actuatorfrc name="{nm}_tau" actuator="{nm}_pos"/>')
        for nm, *_ in SOCKET_DOF:
            add(2, f'<jointpos name="{nm}_q" joint="{nm}"/>')
        for s in ("imu_knee_prox", "imu_shank"):
            add(2, f'<framequat name="{s}_quat" objtype="site" objname="{s}"/>',
                   f'<gyro name="{s}_gyro" site="{s}"/>',
                   f'<accelerometer name="{s}_acc" site="{s}"/>')
        add(2, '<force name="loadcell_f" site="loadcell"/>',
               '<torque name="loadcell_t" site="loadcell"/>',
               '<touch name="sole_touch" site="sole_site"/>',
               '<touch name="contra_touch" site="contra_sole_site"/>')
        add(1, "</sensor>")
        # neutral stance: feet flat (ankle at the sole-levelling angle), socket
        # relaxed.  qpos doc order = hip, socket_piston, socket_flex, socket_abad,
        # socket_rot, knee, ankle, contra_hip (nq=8, no freejoint).  ctrl order =
        # hip, knee, ankle, contra_hip.
        add(1, "<keyframe>",
            f'  <key name="stand" qpos="0 0 0 0 0 0 {fmt(a)} 0" ctrl="0 0 {fmt(a)} 0"/>',
            "</keyframe>")

    def build(self, scene: str) -> str:

        assert scene in ("bench", "ground", "walk")
        used, assets, tri = self._asset_block()
        # walk places the device via the socket, not a world-frame lift.
        lift = 0.0 if scene == "walk" else self._lift_for(scene)

        j = JOINTS
        L: list = []

        def add(depth: int, *rows) -> None:
            pad = "  " * depth
            for r in rows:
                for line in (r if isinstance(r, list) else [r]):
                    L.append(pad + line)

        add(0, f'<mujoco model="osl_v2_{scene}">')
        add(1, "<!-- Generated by tools/build_mjcf.py from the untouched Onshape export.",
               "     Do not edit by hand: re-run the generator instead. -->",
            f'<compiler angle="radian" meshdir="{MESHDIR_REL}" autolimits="true"'
            ' inertiafromgeom="false" balanceinertia="false"/>',
            f'<option timestep="{fmt(TIMESTEP)}" integrator="implicitfast"'
            ' gravity="0 0 -9.81"/>',
            '<visual><scale forcewidth="0.02" contactwidth="0.05" contactheight="0.02"/>'
            "</visual>",
            "<default>")
        add(2, '<default class="visual">'
               '<geom type="mesh" contype="0" conaffinity="0" group="2" density="0"/></default>',
               # `contact` holds everything that has to be IDENTICAL on both
               # sides of a contact, because MuJoCo averages solref/solimp
               # between the two geoms rather than taking the stiffer one.  The
               # floor inherits from here for exactly that reason.
               '<default class="contact">',
               f'  <geom condim="{CONDIM}" friction="{fmt(np.array(FRICTION))}"'
               f' solref="{fmt(np.array(SOLREF))}"'
               f' solimp="{fmt(np.array(SOLIMP))}"/>',
               '  <default class="collision">'
               '<geom group="3" rgba="0.8 0.2 0.2 0.3"/></default>',
               '  <default class="floor">'
               '<geom type="plane" size="3 3 0.05" material="grid"/></default>',
               "</default>")
        add(1, "</default>", "<asset>")
        add(2, '<texture name="grid" type="2d" builtin="checker" width="512" height="512"'
               ' rgb1="0.22 0.24 0.27" rgb2="0.28 0.30 0.34"/>',
               '<material name="grid" texture="grid" texrepeat="4 4" reflectance="0.05"/>',
               assets)
        add(1, "</asset>", "<worldbody>")
        add(2, '<light pos="0.4 -0.6 1.2" dir="-0.3 0.45 -1" directional="true"/>')
        if scene in ("ground", "walk"):
            add(2, '<geom name="floor" class="floor"/>')
        else:
            # marker on the pyramid's own top face, where a socket bolts on --
            # not an arbitrary offset above it
            add(2, f'<site name="bench_mount"'
                   f' pos="{fmt(np.array([KNEE_POS[0], 0.0, self.highest_z()]))}"'
                   ' size="0.006" rgba="0.9 0.6 0.1 1"/>')

        # ---- the body tree.  bench/ground root the CAD device directly at the
        #      world; walk hangs the identical device off the human scaffold. ----
        if scene == "walk":
            self._emit_walk(add, used)
        else:
            self._emit_device(add, used, 2, freejoint=(scene == "ground"),
                              collision=(scene == "ground"),
                              knee_prox_pos=f"0 0 {fmt(lift)}")
        add(1, "</worldbody>")

        if scene == "walk":
            self._walk_controls(add)
        else:
            add(1, "<actuator>")
            for nm in ("knee", "ankle"):
                add(2, f'<position name="{nm}_pos" joint="{nm}" kp="{j[nm]["kp"]}"'
                       f' ctrlrange="{fmt(j[nm]["range"])}"'
                       f' forcerange="{fmt(j[nm]["forcerange"])}"/>')
            add(1, "</actuator>", "<sensor>")
            for nm in ("knee", "ankle"):
                add(2, f'<jointpos name="{nm}_q" joint="{nm}"/>',
                       f'<jointvel name="{nm}_qd" joint="{nm}"/>',
                       f'<actuatorfrc name="{nm}_tau" actuator="{nm}_pos"/>')
            for s in ("imu_knee_prox", "imu_shank"):
                add(2, f'<framequat name="{s}_quat" objtype="site" objname="{s}"/>',
                       f'<gyro name="{s}_gyro" site="{s}"/>',
                       f'<accelerometer name="{s}_acc" site="{s}"/>')
            add(2, '<force name="loadcell_f" site="loadcell"/>',
                   '<torque name="loadcell_t" site="loadcell"/>')
            if scene == "ground":
                add(2, '<touch name="sole_touch" site="sole_site"/>')
            add(1, "</sensor>")
            add(1, self._keyframe(scene))
        add(0, "</mujoco>")

        self._ntri = tri
        self._nmesh = len(assets)
        self._lift[scene] = lift
        return "\n".join(L) + "\n"

    def _keyframe(self, scene: str) -> list:
        """
        A named starting pose.

        `flat` puts the ankle at the angle that levels the sole (see
        _sole_contact_edge) and, in the ground scene, lowers the base by the
        3.8 mm that levelling gains, so the leg starts flat-footed and just
        clear of the floor instead of balanced on the forefoot keel.  qpos = 0
        is kept as the default reset pose because it is the CAD pose and every
        cross-check in the repo is stated at it; `flat` is what you load to
        stand the leg up:

            mujoco.mj_resetDataKeyframe(model, data, 0)

        `ctrl` is set to match, because the position servos would otherwise
        command the ankle straight back to zero and undo the pose on the first
        step.
        """
        a = self.flat_foot_ankle
        if scene == "ground":
            base = np.array([0.0, 0.0, self.flat_foot_lift()])
            qpos = f"{fmt(base)} 1 0 0 0 0 {fmt(a)}"
        else:
            qpos = f"0 {fmt(a)}"
        return ["<keyframe>",
                f'  <key name="flat" qpos="{qpos}" ctrl="0 {fmt(a)}"/>',
                "</keyframe>"]

    def predictions(self, scene: str) -> dict:
        """
        What this generator believes MuJoCo will report once it compiles the
        scene, in world coordinates at qpos = 0.  Written beside each XML as
        `<scene>.pred.json` and checked by scripts/check_model.py.

        The point is not redundancy.  Every measurement in this repo is made by
        the same pure-Python code, so a convention shared between the generator
        and its own validator would be invisible to both.  The specific worry:
        MuJoCo recentres mesh vertices on the mesh centroid at compile time and
        is supposed to compensate in the geom frame, so that geometry lands
        where it was authored.  If it did not compensate, every visual geom
        would be displaced by its own centroid offset -- tens of millimetres
        here -- and nothing in this repo would notice.  Comparing these numbers
        against MuJoCo's own `mesh_vert` placed by its own resolved geom frames
        settles it from outside the shared assumption.
        """
        if scene == "walk":
            return self._walk_predictions()
        lift = self._lift.get(scene, self._lift_for(scene))
        z = np.array([0.0, 0.0, lift])
        lo, hi = self.mesh_bbox
        a, b, length = self.sole_edge
        return {
            "scene": scene,
            "lift": float(lift),
            "total_mass": float(sum(self.inertia[s]["mass"] for s in SEGMENTS)),
            "mass": {s: float(self.inertia[s]["mass"]) for s in SEGMENTS},
            "com_world": {s: (self.inertia[s]["com"] + z).tolist() for s in SEGMENTS},
            "joint_world": {"knee": (KNEE_POS + z).tolist(),
                            "ankle": (ANKLE_POS + z).tolist()},
            "mesh_bbox_world": {"lo": (lo + z).tolist(), "hi": (hi + z).tolist()},
            "nmesh": int(self._nmesh),
            "ntri": int(self._ntri),
            # -- contact, added with the Phase 1 collision geometry ----------
            "flat_foot_ankle": float(self.flat_foot_ankle),
            "flat_lift": (float(self.flat_foot_lift()) if scene == "ground"
                          else 0.0),
            "sole_edge": {"a": list(a), "b": list(b), "length": length},
            # Where the lowest collidable point should sit at spawn.  Emitted so
            # check_model.py can confirm the leg starts just above the floor
            # rather than intersecting it -- the failure mode that produced the
            # original "leg falls through the ground" report.
            "spawn_clearance": SPAWN_CLEARANCE if scene == "ground" else 0.0,
            "collision_geoms": ([lab for s in SEGMENTS
                                 for lab, _, _ in self.module_bounds.get(s, [])]
                                + ["sole"]) if scene == "ground" else [],
            # Collision sufficiency, Phase 1c.  Unlike the contact stiffness
            # below, this one IS a prediction and is meant to be checked: it is
            # pure kinematics, so MuJoCo's own forward kinematics must reproduce
            # the same clearance to within mesh-vertex rounding.  check_model.py
            # re-runs the identical sweep and fails if it does not.
            "sweep": self.sweep_clearance() if scene == "ground" else {},
            # -- contact stiffness, Phase 1b.  There is deliberately no
            # penetration *prediction* here: solref sets a time constant, the
            # resting depth follows from it only up to a coefficient not worth
            # reconstructing from the docs, and check_model.py can read
            # contact.dist and measure it.  `penetration_limit` is a bound.
            "contact": {
                "timestep": float(TIMESTEP),
                "solref": list(SOLREF),
                "solimp": list(SOLIMP),
                "friction": list(FRICTION),
                "condim": int(CONDIM),
                "penetration_limit": float(PENETRATION_LIMIT),
            },
        }

    def _walk_predictions(self) -> dict:
        """
        Sidecar for the walk scene.  Deliberately partial: the device masses and
        joint axes are the same CAD numbers cross-checked in the bench/ground
        sidecars, so they are not re-asserted here as if independent.  The human
        scaffold numbers are PLACEHOLDERS, not measurements, and are reported
        under `human_mass` purely so check_model.py can see them and exclude them
        from the CAD MASS_RANGE test.  The device pose is given in world at the
        neutral qpos (all joints 0) so the walk-chain validator has something to
        reproduce with MuJoCo's own forward kinematics.
        """
        w = getattr(self, "_walk", {}) or {}
        hip_z = float(w.get("hip_z", 0.0))
        kp_z = float(w.get("knee_prox_z", 0.0))
        # device root (knee_prox) in world at the neutral pose:
        # pelvis(hip_z) -> thigh(+y, 0) -> socket(-RES_THIGH_LEN) -> knee_prox(kp_z)
        kp_world = np.array([0.0, HIP_HALF_WIDTH,
                             hip_z - RES_THIGH_LEN + kp_z])
        return {
            "scene": "walk",
            "note": ("Phase 2 scaffold.  knee_prox/shank/foot are the CAD device; "
                     "pelvis, residual_thigh, socket and contra_thigh are "
                     "anthropometric PLACEHOLDERS (not CAD, unfitted).  Device "
                     "mass/axes are cross-checked in the bench/ground sidecars. "
                     "Full validator + check_model coverage is added in the "
                     "walk-chain task."),
            "device_mass": float(sum(self.inertia[s]["mass"] for s in SEGMENTS)),
            "mass": {s: float(self.inertia[s]["mass"]) for s in SEGMENTS},
            "human_mass": {"pelvis": PELVIS_MASS, "residual_thigh": RES_THIGH_MASS,
                           "socket": SOCKET_MASS, "contra_thigh": CONTRA_MASS},
            "hip_z": hip_z,
            "hip_half_width": HIP_HALF_WIDTH,
            "residual_thigh_len": RES_THIGH_LEN,
            "socket_len": SOCKET_LEN,
            "knee_prox_pos_in_socket": [0.0, 0.0, kp_z],
            "knee_prox_world_neutral": kp_world.tolist(),
            "joint_world_neutral": {
                "hip": [0.0, HIP_HALF_WIDTH, hip_z],
                "knee": (kp_world + KNEE_POS).tolist(),
                "ankle": (kp_world + ANKLE_POS).tolist(),
                "contra_hip": [0.0, -HIP_HALF_WIDTH, hip_z],
            },
            "qpos_order": ["hip", "socket_piston", "socket_flex", "socket_abad",
                           "socket_rot", "knee", "ankle", "contra_hip"],
            "nq": 8,
            "actuators": ["hip_pos", "knee_pos", "ankle_pos", "contra_hip_pos"],
            "passive_socket_dof": [nm for nm, *_ in SOCKET_DOF],
            "flat_foot_ankle": float(self.flat_foot_ankle),
            "spawn_clearance": SPAWN_CLEARANCE,
            "collision_geoms": ([lab for s in SEGMENTS
                                 for lab, _, _ in self.module_bounds.get(s, [])]
                                + ["sole", "contra_foot"]),
            "contact": {
                "timestep": float(TIMESTEP),
                "solref": list(SOLREF),
                "solimp": list(SOLIMP),
                "friction": list(FRICTION),
                "condim": int(CONDIM),
                "penetration_limit": float(PENETRATION_LIMIT),
            },
        }


# ---------------------------------------------------------------------------


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--report", action="store_true", help="print the cluster table")
    args = ap.parse_args()

    os.makedirs(MODELS, exist_ok=True)
    m = OslModel()

    if args.report:
        print(f"{'seg':6s} {'z̄':>9s} {'vol cm3':>9s} {'n':>4s}  largest part / reason")
        for seg, vol, z, n, big, why in m.cluster_report:
            print(f"{seg:6s} {z:+9.4f} {vol * 1e6:9.1f} {n:4d}  {big[:38]:38s} {why}")
        print()

    for scene in ("bench", "ground", "walk"):
        xml = m.build(scene)
        path = os.path.join(MODELS, f"osl_v2_{scene}.xml")
        with open(path, "w", encoding="utf-8") as fh:
            fh.write(xml)
        pred = os.path.join(MODELS, f"osl_v2_{scene}.pred.json")
        with open(pred, "w", encoding="utf-8") as fh:
            json.dump(m.predictions(scene), fh, indent=2)
            fh.write("\n")
        print(f"wrote models/osl_v2_{scene}.xml  ({len(xml.splitlines())} lines)"
              f"  + osl_v2_{scene}.pred.json")

    print(f"\nmeshes emitted : {m._nmesh}  ({m._ntri:,} triangles)")
    tot = 0.0
    for s in SEGMENTS:
        d = m.inertia[s]
        tot += d["mass"]
        print(f"  {s:5s} mass={d['mass']:.4f} kg  com={np.round(d['com'], 4)}  "
              f"parts={d['nparts']:3d}  (kept {len(m.kept.get(s, []))}, "
              f"culled {len(m.culled.get(s, []))})")
    verdict = ("within" if MASS_RANGE[0] <= tot <= MASS_RANGE[1]
               else "ABOVE" if tot > MASS_RANGE[1] else "BELOW")
    print(f"  TOTAL {tot:.4f} kg   ({verdict} the expected "
          f"{MASS_RANGE[0]}-{MASS_RANGE[1]} kg for the CAD-derived total -- "
          f"see MASS_RANGE)")
    print(f"        published OSL V2 mass {PUBLISHED_MASS:.3f} kg "
          f"(Tan et al. 2025); shortfall {PUBLISHED_MASS - tot:+.4f} kg, "
          f"i.e. {1e3 * (PUBLISHED_MASS - tot) / 2:.0f} g per missing motor "
          f"-- see MOTOR_MASS, which is {MOTOR_MASS} kg (CAD only)")
    for n in m.notes:
        print(f"  note: {n}")


if __name__ == "__main__":
    main()
