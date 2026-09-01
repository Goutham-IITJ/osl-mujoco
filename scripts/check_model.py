#!/usr/bin/env python3
"""
check_model.py -- compile the generated scenes in MuJoCo and print the facts
worth checking.  This is the one command to run in the Windows venv after
regenerating the models; it is the only step the analysis pipeline cannot do
for itself, because MuJoCo is not installed in the build sandbox.

    python scripts/check_model.py

Beyond "does it compile", this cross-checks MuJoCo's own numbers against
`models/osl_v2_<scene>.pred.json`, which tools/build_mjcf.py wrote from the
CAD.  That comparison is the only check in the repo that escapes the shared
assumptions of the pure-Python tooling -- in particular it settles whether
MuJoCo's compile-time mesh recentring is compensated for in the geom frame.
It is done by placing MuJoCo's *own* stored vertices with MuJoCo's *own*
resolved geom frames, so no convention is taken on trust.

For the ground scene it also runs the flat-stance contact test: load the `flat`
keyframe, step 0.2 s, and read contact.dist to find how far the sole actually
sank into the floor.  That number is deliberately measured rather than predicted
-- the model sets a contact time constant, and the resting depth that follows
from it is not worth deriving from the docs when it can be read directly here.

Two kinematic checks run alongside it, and they are the ones that would catch a
return of the original "the leg goes underground" bug rather than just describe
it.  The sweep test re-derives, over 676 poses spanning the whole joint envelope,
whether any part of the leg WITHOUT a collision geom can get below the parts that
have one; the answer has to be positive, and has to match what the CAD sweep
predicted, both over every visual mesh and over the subset that no bounding box
already contains.  The flat-keyframe test puts the two endpoints of the sole's
contact edge through MuJoCo's forward kinematics and asks whether the keyframe
really does level them -- a check on the pair of sidecar fields that were derived
together and would otherwise agree with each other for free.

Exits non-zero if either scene fails to compile, diverges, disagrees with its
predictions, sinks into the floor past the limit in the sidecar, or turns out to
have geometry that can slip below its own collision model.
"""

from __future__ import annotations

import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mujoco  # noqa: E402
from mjcommon import ROOT, nid  # noqa: E402

SCENES = ("bench", "ground")
SETTLE = 2000    # steps; at 0.5 ms that is exactly 1.000 s
LOAD = 400       # steps for the flat-stance penetration test; 0.200 s
TOL_POS = 1e-4   # m.  mesh_vert is float32, so ~1e-5 m of noise is expected
TOL_MASS = 1e-6  # kg


def contact_depths(data) -> np.ndarray:
    """Penetration depth of every active contact, in metres, positive = into.

    contact.dist is the signed gap: negative means the two geoms overlap.  This
    is MuJoCo's own number for how far the sole has sunk, which is the whole
    point: build_mjcf.py picks a contact TIME constant and deliberately declines
    to predict the resting depth that follows from it, so this reads the depth
    off the solver instead of deriving it.
    """
    n = int(data.ncon)
    if n == 0:
        return np.zeros(0)
    try:
        d = np.asarray(data.contact.dist[:n], dtype=np.float64)
    except (AttributeError, TypeError):
        d = np.array([data.contact[i].dist for i in range(n)], dtype=np.float64)
    return np.maximum(0.0, -d)


def stand_test(model, data, pred: dict, fails: list) -> None:
    """
    Put the leg on the floor in the pose that levels the sole, let the contact
    come to equilibrium, and measure how far it sank.

    Why the `flat` keyframe and not qpos = 0: at ankle = 0 the blade is 2.4 deg
    toe-down and the leg balances on the forefoot keel, so a penetration read
    there measures a line contact under the full weight, which is not the case
    the number is supposed to describe.

    Why only 0.2 s: the contact time constant is 2 ms, so equilibrium is reached
    within a few tens of milliseconds, while toppling -- if it topples -- takes
    far longer.  The tilt is reported alongside so it is visible whether the
    measurement was taken while the leg was still standing.  This is a contact
    stiffness test, not a balance test.
    """
    c = pred.get("contact")
    if c is None:
        print("  (no contact block in the sidecar; re-run tools/build_mjcf.py)")
        return
    try:
        kid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "flat")
    except Exception:  # noqa: BLE001 - old bindings without mjOBJ_KEY
        kid = -1
    if kid < 0:
        print("  (no `flat` keyframe; skipping the stance test)")
        return

    mujoco.mj_resetDataKeyframe(model, data, kid)
    mujoco.mj_forward(model, data)
    z0, worst, ncon = float(data.qpos[2]), 0.0, 0
    for _ in range(LOAD):
        mujoco.mj_step(model, data)
        d = contact_depths(data)
        worst = max(worst, float(d.max()) if d.size else 0.0)
        ncon = max(ncon, int(data.ncon))
    root = nid(model, mujoco.mjtObj.mjOBJ_BODY, "knee_prox")
    tilt = math.degrees(math.acos(min(1.0, abs(float(data.xmat[root].reshape(3, 3)[2, 2])))))

    print(f"  --- flat-stance contact test, {LOAD * model.opt.timestep:.3f} s "
          f"from the `flat` keyframe ---")
    print(f"    contacts {ncon}   base height {z0:+.4f} -> {float(data.qpos[2]):+.4f} m"
          f"   root tilt {tilt:5.2f} deg")
    limit = c["penetration_limit"]
    mark = "ok  " if worst <= limit else "FAIL"
    print(f"    {mark} deepest penetration {1e3 * worst:7.4f} mm   "
          f"(limit {1e3 * limit:.3f} mm, set against the 0.49 mm rms flatness "
          f"of the sole)")
    print(f"    solref timeconst is {1e3 * c['solref'][0]:.1f} ms, so this number "
          f"is a measurement of that choice, not a prediction from it")
    if worst > limit:
        fails.append(f"sole penetrates the floor by {1e3 * worst:.3f} mm, over the "
                     f"{1e3 * limit:.3f} mm limit -- contact is too soft")
    if ncon == 0:
        fails.append("no contacts at all in the flat pose: the leg is not "
                     "touching the floor, so this measured nothing")
    if tilt > 10.0:
        print(f"    note: the root tilted {tilt:.1f} deg within {LOAD * model.opt.timestep:.2f} s, "
              f"so it is falling over; the penetration number above is still the\n"
              f"          peak seen while loaded, but balance is a separate problem.")


def geom_verts(model, g: int) -> np.ndarray:
    """Vertices of one geom in its own frame, for the types this model uses.

    Meshes come from MuJoCo's own vertex array so the answer includes whatever
    compile-time recentring it did.  Boxes are expanded to their eight corners.
    Nothing else needs handling -- the model is meshes, boxes and one plane -- so
    an unexpected type returns empty rather than being guessed at.
    """
    t = int(model.geom_type[g])
    if t == int(mujoco.mjtGeom.mjGEOM_MESH):
        i = int(model.geom_dataid[g])
        a, n = int(model.mesh_vertadr[i]), int(model.mesh_vertnum[i])
        return np.asarray(model.mesh_vert[a:a + n], dtype=np.float64)
    if t == int(mujoco.mjtGeom.mjGEOM_BOX):
        s = model.geom_size[g]
        return np.array([[sx * s[0], sy * s[1], sz * s[2]]
                         for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
    return np.zeros((0, 3))


def lowest_z(data, g: int, v: np.ndarray) -> float:
    """Lowest world z of a vertex set carried by geom `g`, at the current pose."""
    if not len(v):
        return math.nan
    return float((v @ data.geom_xmat[g].reshape(3, 3).T
                  + data.geom_xpos[g])[:, 2].min())


def sweep_candidates(model, data, g: int) -> np.ndarray:
    """
    The only vertices of geom `g` that can ever be its lowest point during the
    sweep.  Reducing to these is what makes the sweep affordable: four of the
    foot's screws carry 193k vertices each, and transforming all of them at all
    676 poses takes the better part of a minute.

    The reduction is exact, not a sample.  Both joints rotate about world Y, so
    the geom's world orientation is always Ry(t) * R0, for R0 its orientation
    here at qpos = 0.  Being lowest means minimising (Ry(t) R0 v)_z, which is the
    linear functional d(t) . (R0 v) with d(t) = (-sin t, 0, cos t).  Every d lies
    in the world xz plane, so the functional never sees the y component of R0 v,
    and its minimum over the cloud is attained at a vertex of the convex hull of
    R0 v projected to (x, z).  Those hull vertices are therefore the complete
    candidate set for every pose at once -- a few dozen points instead of 193k.

    Translation is left out above because it shifts every vertex of a geom
    equally and so cannot change which one is lowest.

    Note this is the same argument build_mjcf.sagittal_hull makes on the CAD
    side, but applied to MuJoCo's vertices through MuJoCo's resolved geom frames,
    so the two remain independent measurements of the same quantity.
    """
    v = geom_verts(model, g)
    if len(v) < 3:
        return v
    p = (v @ data.geom_xmat[g].reshape(3, 3).T)[:, [0, 2]]
    order = np.lexsort((p[:, 1], p[:, 0]))
    hull: list = []
    for chain in (order, order[::-1]):
        base = len(hull)
        for i in chain:
            while len(hull) - base >= 2:
                (x1, z1), (x2, z2) = p[hull[-2]], p[hull[-1]]
                if (x2 - x1) * (p[i][1] - z1) - (z2 - z1) * (p[i][0] - x1) <= 0:
                    hull.pop()
                else:
                    break
            hull.append(int(i))
    return v[np.unique(hull)]


def boxed_geoms(model, data, coll: list, unc: list) -> set:
    """
    Which of the uncovered meshes are contained in a collision box carried by
    their own body?

    A box and a mesh on the same body keep their relative pose at every joint
    angle, so a mesh inside such a box can never get below it: the box's image
    under any rotation still contains the mesh's image.  Those parts are safe by
    construction, and their clearance measures nothing but how close a box corner
    passes to the part that defined it.  Splitting them off is what leaves a
    margin with information in it -- on this model the remainder is the foot's
    non-blade hardware, whose only collision geom is the blade mesh.

    Containment is tested on every vertex, in the box's own frame, with the same
    0.1 mm slack build_mjcf.BOX_CONTAIN_TOL uses -- the boxes are the bounding
    boxes of their own members, so the extremal parts sit exactly on the faces and
    an exact comparison would be decided by the float32 rounding of `mesh_vert`.
    A wrong verdict cannot hide a hole: the strict margin covers every visual mesh
    however it is classified.

    Geoms whose body carries no box skip the transform entirely, which is what
    keeps this cheap: that is where all the heavy meshes are.
    """
    out = set()
    for g in unc:
        boxes = [b for b in coll
                 if int(model.geom_type[b]) == int(mujoco.mjtGeom.mjGEOM_BOX)
                 and int(model.geom_bodyid[b]) == int(model.geom_bodyid[g])]
        v = geom_verts(model, g) if boxes else np.zeros((0, 3))
        if not len(v):
            continue
        w = v @ data.geom_xmat[g].reshape(3, 3).T + data.geom_xpos[g]
        for b in boxes:
            local = (w - data.geom_xpos[b]) @ data.geom_xmat[b].reshape(3, 3)
            if bool((np.abs(local) <= model.geom_size[b] + TOL_POS).all()):
                out.add(g)
                break
    return out


def sweep_test(model, data, pred: dict, fails: list) -> None:
    """
    Re-derive the collision-sufficiency sweep from MuJoCo's own kinematics.

    build_mjcf.py claims that over the whole joint envelope, no part of the leg
    that LACKS a collision geom can get below the lowest part that has one -- so
    nothing can reach the floor before a collision geom does.  That claim is pure
    kinematics, which means MuJoCo has to agree with it exactly; if it does not,
    one of the two is wrong about where the geometry is.

    Both of the sidecar's margins are checked: the strict one over every visual
    mesh, and the one restricted to the meshes no box contains, which is the one
    that carries information (see boxed_geoms).  They are checked against a
    different code path, not a different set of numbers: the collidable geoms are
    found by contype, the floor plane is excluded because it is not part of the
    leg, and the blade is identified as whatever mesh the `sole` geom points at --
    so the visual copy of the blade is excluded without needing a list of part
    names.  That last point is what makes the test work at all: the sole's
    collision and visual geoms share a mesh, so the blade would otherwise report
    a margin of exactly zero and mask everything else.
    """
    sw = pred.get("sweep")
    if sw is None:
        print("  (no sweep block in the sidecar; re-run tools/build_mjcf.py)")
        return
    if not sw:
        return  # bench: no collision geometry, so nothing to be sufficient for

    sole = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "sole")
    if sole < 0:
        print("  (no `sole` geom; skipping the sweep)")
        return
    blade_mesh = int(model.geom_dataid[sole])

    coll, unc = [], []
    for g in range(model.ngeom):
        t = int(model.geom_type[g])
        if t == int(mujoco.mjtGeom.mjGEOM_PLANE):
            continue
        if model.geom_contype[g]:
            coll.append(g)
        elif (t == int(mujoco.mjtGeom.mjGEOM_MESH)
              and int(model.geom_dataid[g]) != blade_mesh):
            unc.append(g)
    if not coll or not unc:
        fails.append(f"sweep found {len(coll)} collision and {len(unc)} uncovered "
                     f"geoms, so it measured nothing")
        return

    mujoco.mj_kinematics(model, data)
    boxed = boxed_geoms(model, data, coll, unc)
    free = [g for g in unc if g not in boxed]
    if not free:
        fails.append("sweep: every visual mesh is inside a collision box, which "
                     "cannot be right -- the foot has no box")
        return
    cand = {g: sweep_candidates(model, data, g) for g in coll + unc}
    kept = sum(len(v) for v in cand.values())
    full = sum(len(geom_verts(model, g)) for g in coll + unc)

    kq = model.jnt_qposadr[nid(model, mujoco.mjtObj.mjOBJ_JOINT, "knee")]
    aq = model.jnt_qposadr[nid(model, mujoco.mjtObj.mjOBJ_JOINT, "ankle")]
    nk, na = sw["grid"]
    worst = dict(margin=math.inf, knee=0.0, ankle=0.0)
    wfree = dict(margin=math.inf, knee=0.0, ankle=0.0)
    for qk in np.radians(np.linspace(*sw["knee_range_deg"], nk)):
        for qa in np.radians(np.linspace(*sw["ankle_range_deg"], na)):
            data.qpos[kq], data.qpos[aq] = qk, qa
            mujoco.mj_kinematics(model, data)
            zc = min(lowest_z(data, g, cand[g]) for g in coll)
            za = min(lowest_z(data, g, cand[g]) for g in unc)
            zb = min(lowest_z(data, g, cand[g]) for g in free)
            if za - zc < worst["margin"]:
                worst = dict(margin=za - zc, knee=float(qk), ankle=float(qa))
            if zb - zc < wfree["margin"]:
                wfree = dict(margin=zb - zc, knee=float(qk), ankle=float(qa))

    # The hull reduction is an argument, so check it where it matters most: redo
    # the two worst poses with every vertex and require the same numbers.  If the
    # reasoning were wrong the reduced sweep would report a margin that is too
    # LARGE, which is the direction that would hide a hole rather than invent one.
    def exact_at(pose: dict, group: list) -> float:
        data.qpos[kq], data.qpos[aq] = pose["knee"], pose["ankle"]
        mujoco.mj_kinematics(model, data)
        return (min(lowest_z(data, g, geom_verts(model, g)) for g in group)
                - min(lowest_z(data, g, geom_verts(model, g)) for g in coll))

    checks = ((worst, unc, sw, "every visual mesh", True),
              (wfree, free, sw["unboxed"], "the meshes no box contains", False))

    print(f"  --- collision sufficiency, {nk}x{na} poses over the full envelope ---")
    print(f"    {len(coll)} collision geoms; {len(unc)} visual meshes, "
          f"{len(free)} of them inside no box "
          f"(CAD counted {sw['nparts']} and {sw['nunboxed']})")
    print(f"    hull reduction kept {kept} of {full} vertices as candidates for "
          f"lowest point")
    if len(unc) != sw["nparts"] or len(free) != sw["nunboxed"]:
        fails.append(f"sweep counted {len(unc)} visual meshes ({len(free)} unboxed) "
                     f"but the CAD sweep counted {sw['nparts']} ({sw['nunboxed']}), so "
                     f"the two are not looking at the same geometry")

    for got, group, want, label, strict in checks:
        kd, ad = math.degrees(got["knee"]), math.degrees(got["ankle"])
        mark = "ok  " if got["margin"] > 0 else "FAIL"
        print(f"    {mark} {label}: worst clearance {1e3 * got['margin']:7.3f} mm at "
              f"knee {kd:+.1f} deg, ankle {ad:+.1f} deg")
        print(f"         CAD said {1e3 * want['margin']:7.3f} mm at knee "
              f"{want['knee_deg']:+.1f}, ankle {want['ankle_deg']:+.1f} deg, binding on "
              f"{want['lowest_uncovered']}")
        # Only the strict group reports the hole.  It is a minimum over a superset,
        # so it is the smaller of the two margins and cannot miss one.
        if strict and got["margin"] <= 0:
            fails.append(f"collision model has a hole: at knee {kd:+.1f} deg, ankle "
                         f"{ad:+.1f} deg a visual mesh is {-1e3 * got['margin']:.1f} mm "
                         f"below the lowest collision geom, so it can pass through the "
                         f"floor")
        exact = exact_at(got, group)
        if abs(exact - got["margin"]) > 1e-9:
            fails.append(f"the hull reduction in sweep_candidates is wrong for {label}: "
                         f"at the worst pose it gives {1e3 * got['margin']:.4f} mm but "
                         f"all vertices give {1e3 * exact:.4f} mm")
        err = abs(got["margin"] - want["margin"])
        if err > TOL_POS:
            fails.append(f"sweep clearance over {label}: MuJoCo {1e3 * got['margin']:.3f} "
                         f"mm vs CAD {1e3 * want['margin']:.3f} mm, off by "
                         f"{1e3 * err:.3f} mm -- the two disagree about where the "
                         f"geometry is")
        else:
            print(f"         ok   agrees with the CAD sweep to {1e6 * err:.1f} um, and "
                  f"with an all-vertex recheck at that pose")


def flat_test(model, pred: dict, fails: list) -> None:
    """
    Check the `flat` keyframe against the two sidecar fields it was derived from,
    and check that it does what it was derived FOR.

    The interesting one is the last: sole_edge is the longest edge of the blade's
    lower convex hull, flat_foot_ankle is the angle that is supposed to level it,
    and the two were computed by the same script from the same mesh -- so they
    would agree with each other even if both were wrong.  Putting the edge's two
    endpoints through MuJoCo's forward kinematics at the keyframe pose and asking
    whether they come out at the same height tests the pair against something
    that did not compute them.

    sole_edge is stored as (x, z) in the foot body frame, which is why it can be
    handed to MuJoCo directly: the foot body origin IS the ankle axis, the frame
    the hull was taken in.
    """
    kid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "flat")
    if kid < 0 or "sole_edge" not in pred:
        return
    d = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, d, kid)
    mujoco.mj_kinematics(model, d)

    aq = model.jnt_qposadr[nid(model, mujoco.mjtObj.mjOBJ_JOINT, "ankle")]
    print("  --- the `flat` keyframe against what it was derived from ---")

    def chk(label: str, got: float, want: float, tol: float, unit: str, scale: float):
        err = abs(got - want)
        mark = "ok  " if err <= tol else "FAIL"
        print(f"    {mark} {label:28s} {got * scale:+9.4f} vs {want * scale:+9.4f} "
              f"{unit}")
        if err > tol:
            fails.append(f"{label}: {got * scale:.4f} vs predicted "
                         f"{want * scale:.4f} {unit}")

    chk("keyframe ankle angle", float(d.qpos[aq]), pred["flat_foot_ankle"],
        1e-9, "deg", 180.0 / math.pi)
    # The bench scene is welded to the world, so there is no base height to check.
    if any(int(model.jnt_type[j]) == int(mujoco.mjtJoint.mjJNT_FREE)
           for j in range(model.njnt)):
        chk("keyframe base height", float(d.qpos[2]), pred["flat_lift"], 1e-9, "mm", 1e3)

    fb = nid(model, mujoco.mjtObj.mjOBJ_BODY, "foot")
    R, p = d.xmat[fb].reshape(3, 3), d.xpos[fb]
    a, b = pred["sole_edge"]["a"], pred["sole_edge"]["b"]
    za, zb = (R @ np.array([a[0], 0.0, a[1]]) + p)[2], (R @ np.array([b[0], 0.0, b[1]]) + p)[2]
    tilt = math.degrees(math.atan2(zb - za, pred["sole_edge"]["length"]))
    mark = "ok  " if abs(tilt) < 0.01 else "FAIL"
    print(f"    {mark} sole edge is level          residual tilt {tilt:+.4f} deg "
          f"over {1e3 * pred['sole_edge']['length']:.1f} mm")
    if abs(tilt) >= 0.01:
        fails.append(f"the `flat` keyframe leaves the sole edge tilted {tilt:+.4f} deg, "
                     f"so flat_foot_ankle does not level sole_edge in MuJoCo")

    if pred.get("collision_geoms"):
        have = sorted(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g)
                      for g in range(model.ngeom)
                      if model.geom_contype[g]
                      and int(model.geom_type[g]) != int(mujoco.mjtGeom.mjGEOM_PLANE))
        want = sorted(pred["collision_geoms"])
        mark = "ok  " if have == want else "FAIL"
        print(f"    {mark} collidable geoms            {len(have)}: {', '.join(have)}")
        if have != want:
            fails.append(f"collidable geoms are {have}, sidecar says {want}")

    if "spawn_clearance" in pred:
        lows = [lowest_z(d, g, geom_verts(model, g)) for g in range(model.ngeom)
                if model.geom_contype[g]
                and int(model.geom_type[g]) != int(mujoco.mjtGeom.mjGEOM_PLANE)]
        chk("lowest collision point", min(lows), pred["spawn_clearance"],
            TOL_POS, "mm", 1e3)


def mesh_bbox_world(model, data) -> tuple:
    """
    World bounding box of every mesh geom, built from MuJoCo's own vertex
    arrays and its own resolved geom frames.

    This is deliberately not read from the XML.  MuJoCo translates mesh
    vertices at compile time so the mesh frame sits on the mesh centroid, and
    is supposed to fold the same translation into the geom frame so geometry
    lands where it was authored.  Reading mesh_vert (post-recentring) through
    geom_xpos/geom_xmat (post-compensation) reproduces exactly what the
    renderer and the collision engine see, so if the compensation were missing
    this bbox would drift away from the CAD prediction by the per-mesh centroid
    offsets -- tens of millimetres on this model.
    """
    lo, hi = np.full(3, np.inf), np.full(3, -np.inf)
    for g in range(model.ngeom):
        if model.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH:
            continue
        i = int(model.geom_dataid[g])
        a, n = int(model.mesh_vertadr[i]), int(model.mesh_vertnum[i])
        v = np.asarray(model.mesh_vert[a:a + n], dtype=np.float64)
        w = v @ data.geom_xmat[g].reshape(3, 3).T + data.geom_xpos[g]
        lo, hi = np.minimum(lo, w.min(0)), np.maximum(hi, w.max(0))
    return lo, hi


def compare(pred: dict, model, data, fails: list) -> None:
    """Hold MuJoCo to what build_mjcf.py predicted from the CAD."""
    print("  --- cross-check against models/osl_v2_"
          f"{pred['scene']}.pred.json (CAD-derived) ---")

    def cmp(label: str, got, want, tol: float, unit: str, scale: float) -> None:
        got, want = np.atleast_1d(np.asarray(got, float)), np.atleast_1d(np.asarray(want, float))
        err = float(np.max(np.abs(got - want)))
        mark = "ok  " if err <= tol else "FAIL"
        if err > tol:
            fails.append(f"{label}: MuJoCo {np.round(got, 6)} vs CAD "
                         f"{np.round(want, 6)}, off by {err * scale:.4g} {unit}")
        print(f"    {mark} {label:24s} max err {err * scale:8.4f} {unit}")

    cmp("total mass", model.body_mass.sum(), pred["total_mass"], TOL_MASS, "mg", 1e6)
    for seg, m in pred["mass"].items():
        i = nid(model, mujoco.mjtObj.mjOBJ_BODY, seg)
        cmp(f"{seg} mass", model.body_mass[i], m, TOL_MASS, "mg", 1e6)
    for seg, c in pred["com_world"].items():
        i = nid(model, mujoco.mjtObj.mjOBJ_BODY, seg)
        cmp(f"{seg} COM (world)", data.xipos[i], c, TOL_POS, "mm", 1e3)
    for jn, p in pred["joint_world"].items():
        j = nid(model, mujoco.mjtObj.mjOBJ_JOINT, jn)
        cmp(f"{jn} anchor (world)", data.xanchor[j], p, TOL_POS, "mm", 1e3)

    cmp("mesh count", model.nmesh, pred["nmesh"], 0.5, "", 1.0)
    lo, hi = mesh_bbox_world(model, data)
    cmp("mesh bbox lo (world)", lo, pred["mesh_bbox_world"]["lo"], TOL_POS, "mm", 1e3)
    cmp("mesh bbox hi (world)", hi, pred["mesh_bbox_world"]["hi"], TOL_POS, "mm", 1e3)

    shift = getattr(model, "mesh_pos", None)
    if shift is not None and model.nmesh:
        print(f"    note: MuJoCo recentred mesh vertices by up to "
              f"{1e3 * np.abs(shift).max():.1f} mm (mesh_pos); the bbox agreeing "
              f"above is what proves it compensated in the geom frames.")

    # Contact parameters, read off the compiled model rather than the XML.  The
    # trap being checked is MuJoCo's averaging of solref/solimp across a contact
    # pair: the generator puts both the floor and the leg in one default class so
    # they cannot disagree, and this confirms the compiler actually resolved them
    # that way on every collidable geom.
    cp = pred.get("contact")
    if cp is not None:
        cmp("timestep", model.opt.timestep, cp["timestep"], 1e-12, "s", 1.0)
        ids = [g for g in range(model.ngeom) if model.geom_contype[g]]
        if ids:
            sr = np.array([model.geom_solref[g] for g in ids])
            si = np.array([model.geom_solimp[g] for g in ids])
            cmp("solref (all collidable)", [sr[:, 0].min(), sr[:, 0].max(),
                                            sr[:, 1].min(), sr[:, 1].max()],
                [cp["solref"][0], cp["solref"][0], cp["solref"][1], cp["solref"][1]],
                1e-9, "", 1.0)
            cmp("solimp (all collidable)",
                np.r_[si.min(0), si.max(0)], np.r_[cp["solimp"], cp["solimp"]],
                1e-9, "", 1.0)
            print(f"    note: {len(ids)} collidable geoms, all resolving to "
                  f"solref {np.round(sr[0], 5).tolist()} -- so nothing averages "
                  f"down against a softer partner.")


def report(scene: str, fails: list) -> bool:
    path = os.path.join(ROOT, "models", f"osl_v2_{scene}.xml")
    name = os.path.basename(path)
    try:
        model = mujoco.MjModel.from_xml_path(path)
    except Exception as e:  # noqa: BLE001 - we want the message verbatim
        print(f"\n{name}: FAILED TO COMPILE\n  {e}")
        fails.append(f"{name} did not compile")
        return False

    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    print(f"\n{name}: compiled")
    print(f"  bodies {model.nbody}   dofs nq={model.nq} nv={model.nv}   "
          f"geoms {model.ngeom}   meshes {model.nmesh}   "
          f"actuators {model.nu}   sensors {model.nsensor}")
    print(f"  total mass {model.body_mass.sum():.4f} kg")
    for i in range(model.nbody):
        nm = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i)
        if nm in ("world", None):
            continue
        # xipos is the COM in world coordinates, which is what the README quotes
        print(f"    {nm:6s} mass {model.body_mass[i]:7.4f} kg   "
              f"com(world) {np.round(data.xipos[i], 4)}   "
              f"inertia {np.round(model.body_inertia[i], 6)}")

    for i in range(model.njnt):
        nm = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        jt = int(model.jnt_type[i])
        if jt == mujoco.mjtJoint.mjJNT_HINGE:
            lo, hi = model.jnt_range[i]
            print(f"  joint {nm:6s} hinge  axis {model.jnt_axis[i]}  "
                  f"pos {np.round(model.jnt_pos[i], 5)}  "
                  f"range {math.degrees(lo):+.1f} .. {math.degrees(hi):+.1f} deg")
        else:
            print(f"  joint {nm:6s} {mujoco.mjtJoint(jt).name}")

    pred_path = os.path.join(ROOT, "models", f"osl_v2_{scene}.pred.json")
    pred = None
    if os.path.exists(pred_path):
        with open(pred_path, encoding="utf-8") as fh:
            pred = json.load(fh)
        compare(pred, model, data, fails)
        flat_test(model, pred, fails)
        sweep_test(model, mujoco.MjData(model), pred, fails)
    else:
        print(f"  (no {os.path.basename(pred_path)}; re-run tools/build_mjcf.py "
              f"to get the CAD cross-check)")

    z0 = float(data.qpos[2]) if scene == "ground" else None
    for _ in range(SETTLE):
        mujoco.mj_step(model, data)
    ok = bool(np.all(np.isfinite(data.qpos)) and np.all(np.isfinite(data.qvel)))
    if not ok:
        fails.append(f"{name} diverged during settling")
    print(f"  {SETTLE * model.opt.timestep:.3f} s of free simulation: "
          f"{'stable' if ok else 'DIVERGED'}   |qvel|max {np.abs(data.qvel).max():.4g}")

    if scene == "ground":
        z1 = float(data.qpos[2])
        aj = nid(model, mujoco.mjtObj.mjOBJ_JOINT, "ankle")
        ankle_z = float(data.xanchor[aj][2])
        print(f"  contacts after settling: {data.ncon}")
        print(f"  base height {z0:+.4f} -> {z1:+.4f} m  (drop {1e3 * (z0 - z1):+.1f} mm)")
        print(f"  ankle axis {1e3 * ankle_z:+.1f} mm above the floor")
        print("  note: that settle starts at qpos = 0, where the sole is 2.4 deg")
        print("        toe-down, so it is a drop onto the forefoot keel.")
        if pred is not None:
            stand_test(model, mujoco.MjData(model), pred, fails)
        print("  upright? check whether the leg is still standing in the viewer:")
        print("    python scripts/view_osl.py --scene ground --flat")
    return ok


def main() -> None:
    fails: list = []
    allok = True
    for s in SCENES:
        if not os.path.exists(os.path.join(ROOT, "models", f"osl_v2_{s}.xml")):
            print(f"osl_v2_{s}.xml: not found -- run  python tools/build_mjcf.py")
            allok = False
            continue
        allok &= report(s, fails)
    if fails:
        print("\nPROBLEMS")
        for f in fails:
            print(f"  {f}")
    print()
    sys.exit(0 if allok and not fails else 1)


if __name__ == "__main__":
    main()
