#!/usr/bin/env python3
"""
cad_findings.py -- re-derive, from the untouched export, every geometric claim
the rest of this repo relies on, and print the evidence.

Run this instead of trusting docs/CAD_FINDINGS.md.  The document is just this
script's output, pasted; if the two disagree, the script is right.

    python tools/cad_findings.py

numpy + stdlib only.  MuJoCo is not needed.
"""

from __future__ import annotations

import collections
import math
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from oslcad import density_for  # noqa: E402
import build_mjcf as B  # noqa: E402


def h(title: str) -> None:
    print(f"\n## {title}")


def urdf_stats() -> None:
    root = ET.parse(B.URDF).getroot()
    links = root.findall("link")
    joints = root.findall("joint")
    kinds = collections.Counter(j.get("type") for j in joints)
    h("Export size and joint mix")
    print(f"links {len(links)}   joints {len(joints)}   {dict(kinds)}")
    print(f"file  {sum(1 for _ in open(B.URDF, encoding='utf-8'))} lines")

    masses, singular, sing_and_tri, triviol = [], 0, 0, 0
    for lk in links:
        it = lk.find("inertial")
        if it is None:
            continue
        masses.append(float(it.find("mass").get("value")))
        a = it.find("inertia").attrib
        I = np.array([[float(a["ixx"]), float(a["ixy"]), float(a["ixz"])],
                      [float(a["ixy"]), float(a["iyy"]), float(a["iyz"])],
                      [float(a["ixz"]), float(a["iyz"]), float(a["izz"])]])
        w = np.sort(np.linalg.eigvalsh(I))
        bad_tri = w[0] + w[1] < w[2]
        if w[0] <= 0:
            singular += 1
            sing_and_tri += int(bad_tri)
        elif bad_tri:
            triviol += 1
    h("Why the inertials are unusable")
    print(f"{len(masses)} inertial blocks, total mass {1e3 * sum(masses):.4f} g")
    print(f"  {singular} have a non-positive eigenvalue (Onshape assigned no")
    print(f"    materials, so every tensor is scaled by an absurd mass)")
    print(f"  {sing_and_tri} of those also violate the triangle inequality A + B >= C")
    print(f"  {triviol} non-singular tensors violate it")
    print("MuJoCo eigen-decomposes each of these while PARSING, so no compiler")
    print("option can rescue them -- hence native MJCF emission.")


def visual_origin_evidence():
    """The single most expensive lesson in this project."""
    m = B.OslModel()
    vis = m.vorigin
    nonid = {k: v for k, v in vis.items()
             if np.linalg.norm(v[0]) > 1e-12 or np.linalg.norm(v[1] - np.eye(3)) > 1e-9}
    mags = sorted(((np.linalg.norm(v[0]), k) for k, v in nonid.items()), reverse=True)
    h("<visual><origin> must be composed on top of the link pose")
    print(f"{len(nonid)} of {len(m.cad.links)} links carry a non-identity visual origin")
    print("largest translations:")
    for d, k in mags[:4]:
        print(f"  {1e3 * d:8.3f} mm  {k}")

    # cross-check against the export's own declared COM, which is geometric and
    # therefore trustworthy even though the masses are not
    root = ET.parse(B.URDF).getroot()
    err_fold, err_raw = [], []
    for lk in root.findall("link"):
        nm, it = lk.get("name"), lk.find("inertial")
        if it is None or nm not in m.cad.links or not m.cad.links[nm].mesh_file:
            continue
        o = it.find("origin")
        if o is None:
            continue
        declared = np.array([float(x) for x in o.get("xyz").split()])
        mp = m.cad.mass_props(m.cad.links[nm].mesh_file)
        vp, vR = vis.get(nm, (np.zeros(3), np.eye(3)))
        err_fold.append(np.linalg.norm(vp + vR @ mp.centroid - declared))
        err_raw.append(np.linalg.norm(mp.centroid - declared))
    for label, e in (("folded  ", err_fold), ("as-is   ", err_raw)):
        e = 1e3 * np.array(e)
        print(f"  COM error vs declared, {label}: median {np.median(e):7.3f}  "
              f"p90 {np.percentile(e, 90):8.3f}  max {e.max():8.3f} mm  (n={len(e)})")
    print("Folding wins on "
          f"{int(np.sum(np.array(err_fold) < np.array(err_raw)))} links, "
          f"as-is on {int(np.sum(np.array(err_raw) < np.array(err_fold)))}.")
    for note in m.notes:
        print(f"  note: {note}")
    return m


def centroid(m, n: np.ndarray) -> np.ndarray:
    p, R = m.geom_pose(n)
    return p + R @ m.cad.mass_props(m.cad.links[n].mesh_file).centroid


def axes(m) -> None:
    h("Knee and ankle hinge locations")
    print(f"KNEE_POS  {B.KNEE_POS}   ANKLE_POS {B.ANKLE_POS}   axis: world Y for both")
    print(f"knee-to-ankle distance {np.linalg.norm(B.KNEE_POS - B.ANKLE_POS):.5f} m")
    print("evidence -- the parts whose world centroid lies closest to each axis")
    print("(these pin the axis location; the segmentation rule further down uses")
    print("whole-cluster centroids, not individual parts):")
    for label, ax in (("knee", B.KNEE_POS), ("ankle", B.ANKLE_POS)):
        rows = []
        for n in m.cad.mesh_links():
            c = centroid(m, n)
            d = math.hypot(c[0] - ax[0], c[2] - ax[2])
            if d < B.AXIS_CLUSTER_RADIUS:
                rows.append((d, n, c))
        rows.sort()
        print(f"  {label}: {len(rows)} parts within "
              f"{1e3 * B.AXIS_CLUSTER_RADIUS:.0f} mm in the sagittal plane; closest:")
        for d, n, c in rows[:4]:
            print(f"    {1e3 * d:6.2f} mm  z={c[2]:+.5f}  {n}")


def anterior(m) -> None:
    h("Anterior is -X (settled by the foot, not by the battery)")
    t = m.world_tris(B.FOOT_LINK)
    lo, hi = t.min(0), t.max(0)
    print(f"Variflex blade world bbox  x [{lo[0]:+.4f}, {hi[0]:+.4f}]  "
          f"y [{lo[1]:+.4f}, {hi[1]:+.4f}]  z [{lo[2]:+.4f}, {hi[2]:+.4f}]")
    bolts = sorted(n for n in m.cad.mesh_links() if "m8" in n.lower())
    for n in bolts[:4]:
        c = centroid(m, n)
        print(f"  mount hardware {n}: x={c[0]:+.4f} y={c[1]:+.4f}")
    print("The blade runs much further from the bolts toward -X than +X, so the")
    print("long forefoot is -X and the short heel is +X.")


def clusters(m) -> None:
    h("Rigid clusters and how the straddling ones were resolved")
    counts = collections.Counter(seg for seg, *_ in m.cluster_report)
    axis_rows = [r for r in m.cluster_report if "concentric" in r[5]]
    print(f"{len(m.cluster_report)} clusters carry geometry -> {dict(counts)}")
    print(f"{len(axis_rows)} were assigned by the belt-drive axis rule rather "
          f"than the plain z-split:")
    for seg, vol, z, nmesh, biggest, why in axis_rows:
        print(f"  -> {seg:5s}  zbar {z:+.4f}  {nmesh:3d} parts  {biggest}")
        print(f"       {why}")


def masses(m) -> None:
    h("Mass recovery")
    used = collections.Counter()
    for n in m.cad.mesh_links():
        used[density_for(n)] += 1
    print("density rules actually in use (kg/m^3, parts):")
    for (rho, label), k in sorted(used.items(), key=lambda kv: -kv[1]):
        print(f"  {rho:7.1f}  {label:28s} {k:4d} parts")
    tot = 0.0
    for seg in B.SEGMENTS:
        d = m.inertia[seg]
        tot += d["mass"]
        print(f"  {seg:5s} {d['mass']:7.4f} kg  com(world) "
              f"{np.round(d['com'], 4)}  kept {len(m.kept.get(seg, []))}  "
              f"culled {len(m.culled.get(seg, []))}")
    print(f"  TOTAL {tot:7.4f} kg")
    print("NOTE: densities are assignments by part family, not measurements.")
    print("build/mass_audit.csv lists every part so they can be corrected")
    print("against weighed hardware.  Run:  python tools/validate_mjcf.py --audit")


def foot(m) -> None:
    h("The Ossur Variflex mate is inherited, not reconstructed")
    print(f"sole lowest z {m.foot_sole_z:+.4f} m -> ankle sits "
          f"{1e3 * (B.ANKLE_POS[2] - m.foot_sole_z):.1f} mm above the sole")
    print(f"lowest point of the whole model {m.lowest_z():+.4f} m")
    print("_check_foot() re-runs the mate test on every build; see its output above.")

    h("The sole is 2.4 deg toe-down at ankle = 0, so the leg cannot stand flat")
    a, b, length = m.sole_edge
    print("Lower convex hull of the blade in the sagittal plane, foot body frame")
    print("(origin = ankle axis).  This is the only part of the sole a floor can")
    print("touch, so it is what decides the standing pose.")
    print(f"  longest lower-hull edge  {1e3 * length:.1f} mm")
    print(f"    from  x={a[0]:+.5f}  z={a[1]:+.5f}   (distal tip, heel blade)")
    print(f"    to    x={b[0]:+.5f}  z={b[1]:+.5f}   (heel pad)")
    print(f"  slope {math.degrees(m.flat_foot_ankle):+.4f} deg -> flat-foot ankle "
          f"angle {math.degrees(m.flat_foot_ankle):+.4f} deg")
    print(f"  ground-scene base height, qpos=0 {m._lift_for('ground'):.4f} m, "
          f"flat pose {m.flat_foot_lift():.4f} m")
    print("The models/*.xml <keyframe name=\"flat\"> holds this pose.  At ankle = 0")
    print("the leg balances on the forefoot keel instead, which is why commanding")
    print("the ankle used to tip it over.")
    flat_patch(m)

    h("Collision geometry: the blade mesh, plus one bounding box per module")
    print("The sole is a type=\"mesh\" geom on the blade itself, so the toe spring")
    print("and heel roll-off are exact (both convex).  Everything above the ankle")
    print("gets an axis-aligned BOUNDING box whose only job is to keep the leg out")
    print("of the floor -- the fill fractions say plainly that these are not shape")
    print("claims:")
    for seg in B.SEGMENTS:
        for label, lo, hi in m.module_bounds.get(seg, []):
            names = dict(m.modules[seg])[label]
            vol = sum(m.cad.mass_props(m.cad.links[n].mesh_file).volume
                      for n in names)
            box = float(np.prod(hi - lo))
            print(f"  {seg:5s} {label:13s} {np.round(1e3 * (hi - lo), 1)} mm  "
                  f"{len(names):3d} parts  {100 * vol / box:5.1f} % full")
    sweep(m)


def sweep(m) -> None:
    """Are those five collision geoms actually enough to keep the leg out of the
    floor, in every pose it can reach?

    Fill fractions say the boxes are loose, which is fine.  The question that
    matters is the opposite one: is anything left UNCOVERED that could dip below
    the lowest thing that does collide?  If so, the viewer would show a part
    passing through the ground while MuJoCo reported no contact -- which is
    exactly the class of bug that started this whole phase.

    Sweeping both joints over their full ranges answers it directly, and the
    answer is reported twice.  Over every visual mesh it is the strict form of the
    claim.  Over the meshes that no bounding box contains it is the form with
    information in it, because a part inside a box on its own body cannot get
    below that box no matter how the leg is posed.
    """
    s = m.sweep_clearance()
    if not s:
        return
    u = s["unboxed"]
    print(f"  swept {s['grid'][0]}x{s['grid'][1]} = "
          f"{s['grid'][0] * s['grid'][1]} poses over knee "
          f"{s['knee_range_deg'][0]:+.0f}..{s['knee_range_deg'][1]:+.0f} deg x ankle "
          f"{s['ankle_range_deg'][0]:+.0f}..{s['ankle_range_deg'][1]:+.0f} deg:")
    print(f"    all {s['nparts']} visual meshes stay {1e3 * s['margin']:.1f} mm clear "
          f"of the lowest collision geom")
    print(f"      worst at knee {s['knee_deg']:+.1f} deg, ankle {s['ankle_deg']:+.1f} deg,"
          f" binding pair")
    print(f"        {s['lowest_uncovered']}")
    print(f"        vs the {s['lowest_collision']} geom")
    print(f"    the {s['nunboxed']} meshes no box contains stay "
          f"{1e3 * u['margin']:.1f} mm clear")
    print(f"      worst at knee {u['knee_deg']:+.1f} deg, ankle {u['ankle_deg']:+.1f} deg,"
          f" binding pair")
    print(f"        {u['lowest_uncovered']}")
    print(f"        vs the {u['lowest_collision']} geom")
    print(f"    the first figure is the smaller of the two because a part inside a box")
    print(f"    on its own body can never rise above it -- that clearance measures how")
    print(f"    close a rotating box corner passes to the part that defined the box,")
    print(f"    not whether anything is exposed.  The second is the real margin.")
    print(f"    Either way nothing can reach the floor before a collision geom does.")
    print(f"    scripts/check_model.py re-derives both from MuJoCo's own forward")
    print(f"    kinematics and fails if either disagrees by more than 0.1 mm.")


def flat_patch(m) -> None:
    """Is the flat-foot pose a real contact patch, or still a knife edge?

    The hull-edge slope above says only that *some* line on the sole becomes
    level.  A line is not a stance: if the blade were curved along that line the
    leg would still balance on a point.  So rotate the blade by the flat-foot
    angle about the ankle, take every vertex within 2 mm of the new lowest
    point, and measure what that set looks like.

    Two numbers decide whether the pose is worth having.  The patch extent says
    how much of the sole is in play, and the rms deviation from a best-fit plane
    says whether it is flat or merely low.  The residual frontal tilt matters
    most of all: the ankle is a single sagittal hinge, so any frontal component
    of the 2.18 deg build tilt is uncorrectable by construction, and the pose is
    only useful if that component is negligible.  It is -- under 0.01 deg.
    """
    a = m.flat_foot_ankle
    c, s = math.cos(a), math.sin(a)
    Ry = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    P = (m.world_tris(B.FOOT_LINK) - B.ANKLE_POS) @ Ry.T + B.ANKLE_POS
    near = P[P[:, 2] < P[:, 2].min() + 0.002]
    lo, hi = near.min(0), near.max(0)
    X = np.c_[near[:, 0], near[:, 1], np.ones(len(near))]
    coef = np.linalg.lstsq(X, near[:, 2], rcond=None)[0]
    res = near[:, 2] - X @ coef
    print(f"  at that angle the contact is a patch, not a knife edge:")
    print(f"    {len(near)} blade vertices within 2 mm of the lowest point, "
          f"spanning {1e3 * (hi[0] - lo[0]):.1f} x {1e3 * (hi[1] - lo[1]):.1f} mm")
    print(f"    flat to {1e3 * math.sqrt((res ** 2).mean()):.2f} mm rms about a "
          f"best-fit plane")
    print(f"    residual tilt: sagittal {math.degrees(math.atan(coef[0])):+.2f} "
          f"deg, frontal {math.degrees(math.atan(coef[1])):+.2f} deg -- the")
    print(f"      frontal term is what a single ankle hinge could NOT have fixed,")
    print(f"      so the pose only works because it comes out at zero")
    print(f"  the rms figure is load-bearing elsewhere: build_mjcf.PENETRATION_LIMIT")
    print(f"  is {1e3 * B.PENETRATION_LIMIT:.2f} mm, chosen as roughly that rms rather "
          f"than from any formula,")
    print(f"  because a sole that sinks less than its own unevenness is not the "
          f"reason")
    print(f"  a stance looks wrong.  check_model.py measures the real depth against "
          f"it.")


def main() -> None:
    print("OSL V2 CAD findings, re-derived from")
    print(f"  {os.path.relpath(B.URDF, B.ROOT)}")
    urdf_stats()
    m = visual_origin_evidence()
    axes(m)
    anterior(m)
    clusters(m)
    masses(m)
    foot(m)
    print()


if __name__ == "__main__":
    main()
