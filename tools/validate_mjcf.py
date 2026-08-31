#!/usr/bin/env python3
"""
validate_mjcf.py -- pre-flight check for the generated OSL scenes.

MuJoCo cannot be installed in the analysis sandbox (no outbound network), so
this replicates the compile-time checks MuJoCo actually performs, plus a few
model-specific sanity checks.  It is not a substitute for compiling -- it is
there so that when you do run MuJoCo on Windows, it compiles first try.

Checks
  XML       well-formed; no NaN/inf; every numeric attribute parses
  assets    every <mesh file=...> exists with EXACTLY that case on disk
            (the URDF's references differ in case from the real filenames, so
            a wrong case here works on Windows and breaks on Linux)
  meshes    positive volume, watertight
  names     unique within each element class; every reference resolves
            (geom->mesh, joint/actuator/sensor->target, class->default)
  frames    every quat normalised to 1e-6
  inertia   mass > 0, all three diagonal terms > 0, triangle inequality A+B>=C
  limits    joint range lo < hi, ctrlrange inside the joint range
  physics   total mass plausible; COM inside the geometry bbox; the foot
            clears the floor in the ground scene
  kinematics knee/ankle hinge positions match the CAD-measured axes

Usage:  python tools/validate_mjcf.py [--audit build/mass_audit.csv]
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from oslcad import OslCad, density_for  # noqa: E402
import build_mjcf as B  # noqa: E402

ROOT = B.ROOT
MODELS = B.MODELS

# Expected total mass of the CAD-derived model -- defined once, in
# build_mjcf.py, so the repo cannot quote three different ranges.  Note this is
# the range for the MODEL, not for the hardware: the published hardware figure
# is B.PUBLISHED_MASS = 5.377 kg and the export is missing both motors, so the
# model is expected to come out a few hundred grams light.  See MOTOR_MASS.
MASS_RANGE = B.MASS_RANGE


class Report:
    def __init__(self) -> None:
        self.fail: list = []
        self.warn: list = []
        self.ok = 0

    def check(self, cond: bool, msg: str, warn_only: bool = False) -> bool:
        if cond:
            self.ok += 1
        elif warn_only:
            self.warn.append(msg)
        else:
            self.fail.append(msg)
        return bool(cond)

    def summary(self, label: str) -> bool:
        print(f"\n=== {label} ===")
        print(f"  {self.ok} checks passed")
        for w in self.warn:
            print(f"  WARN  {w}")
        for f in self.fail:
            print(f"  FAIL  {f}")
        if not self.fail:
            print("  -> would compile")
        return not self.fail


def nums(el: ET.Element, attr: str) -> np.ndarray | None:
    v = el.get(attr)
    if v is None:
        return None
    return np.array([float(x) for x in v.split()])


def quat_to_mat(q: np.ndarray) -> np.ndarray:
    """MuJoCo quaternion order (w, x, y, z) -> rotation matrix."""
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def validate(path: str, cad: OslCad, model: B.OslModel) -> bool:
    r = Report()
    scene = "ground" if "ground" in os.path.basename(path) else "bench"

    try:
        root = ET.parse(path).getroot()
    except ET.ParseError as e:
        print(f"FAIL  {path} is not well-formed XML: {e}")
        return False
    r.check(True, "")

    # ---- no NaN / inf, every numeric attribute parses ---------------------
    numeric = ("pos", "quat", "axis", "size", "mass", "diaginertia", "range",
               "ctrlrange", "forcerange", "rgba", "kp", "damping", "armature",
               "frictionloss", "friction", "timestep", "gravity",
               "solref", "solimp")
    for el in root.iter():
        for a in numeric:
            v = el.get(a)
            if v is None:
                continue
            try:
                x = np.array([float(t) for t in v.split()])
            except ValueError:
                r.check(False, f"<{el.tag} {el.get('name')}> {a}='{v}' is not numeric")
                continue
            r.check(bool(np.all(np.isfinite(x))),
                    f"<{el.tag} {el.get('name')}> {a} has NaN/inf")

    # ---- meshdir + asset files -------------------------------------------
    comp = root.find("compiler")
    meshdir = os.path.normpath(os.path.join(os.path.dirname(path), comp.get("meshdir")))
    r.check(os.path.isdir(meshdir), f"meshdir does not exist: {meshdir}")
    ondisk = set(os.listdir(meshdir)) if os.path.isdir(meshdir) else set()

    meshes = {}
    for el in root.iterfind("asset/mesh"):
        nm, f = el.get("name"), el.get("file")
        r.check(nm not in meshes, f"duplicate mesh asset name {nm!r}")
        meshes[nm] = f
        # exact-case existence: the reason this matters is that the URDF's
        # references are mis-cased relative to the real filenames
        r.check(f in ondisk, f"mesh file missing or mis-cased: {f!r}")
        if f in ondisk:
            mp = cad.mass_props(f)
            r.check(mp.volume > 0, f"{f}: non-positive volume {mp.volume:.3g}")
            r.check(mp.closed, f"{f}: not watertight", warn_only=True)

    # ---- declared default classes ----------------------------------------
    # Defaults nest, and MuJoCo resolves a class name from anywhere in the tree,
    # so this has to recurse.  It did not, until the contact parameters were
    # factored into a `contact` parent that `collision` and `floor` inherit
    # from -- at which point a flat scan silently stopped seeing `collision` and
    # every collision geom would have been reported as referencing an undeclared
    # class.
    classes: set = set()

    def collect_classes(el: ET.Element) -> None:
        for d in el.findall("default"):
            if d.get("class"):
                classes.add(d.get("class"))
            collect_classes(d)

    for d in root.findall("default"):
        collect_classes(d)
        if d.get("class"):
            classes.add(d.get("class"))

    # ---- contact parameters ----------------------------------------------
    # These are the numbers that decide whether the leg stands on the floor or
    # sinks into it, and one of them has a hard stability limit that is easy to
    # cross by accident, so they are checked rather than trusted.
    ts = nums(root.find("option"), "timestep")
    timestep = float(ts[0]) if ts is not None else 0.002

    # every <geom> that carries contact parameters, labelled by where it lives,
    # so a failure names the default class to go and edit
    def contact_carriers(el: ET.Element, label: str) -> list:
        out = []
        g = el.find("geom")
        if g is not None:
            out.append((label, g))
        for d in el.findall("default"):
            out += contact_carriers(d, f'default class "{d.get("class")}"')
        return out

    carriers = []
    for d in root.findall("default"):
        carriers += contact_carriers(d, "the root <default>")
    for el in root.find("worldbody").iter("geom"):
        carriers.append((f'geom "{el.get("name") or el.get("class")}"', el))

    for who, el in carriers:
        sr = nums(el, "solref")
        if sr is not None:
            r.check(len(sr) == 2, f"{who}: solref wants 2 values, got {len(sr)}")
            if len(sr) == 2 and sr[0] > 0:
                # MuJoCo: a timeconst below 2*timestep is stiffer than the
                # integrator can follow.  The compiler does not refuse it; the
                # contact just rings or blows up at run time.
                r.check(sr[0] >= 2 * timestep - 1e-12,
                        f"{who}: solref timeconst {sr[0]:g} s is below the "
                        f"2*timestep floor of {2 * timestep:g} s")
                r.check(sr[1] > 0, f"{who}: solref dampratio {sr[1]:g} must be > 0")
        si = nums(el, "solimp")
        if si is not None:
            r.check(len(si) == 5, f"{who}: solimp wants 5 values, got {len(si)}")
            if len(si) == 5:
                r.check(0 < si[0] < 1, f"{who}: solimp d0 {si[0]:g} not in (0,1)")
                r.check(0 < si[1] < 1, f"{who}: solimp dwidth {si[1]:g} not in (0,1)")
                r.check(si[0] <= si[1],
                        f"{who}: solimp d0 {si[0]:g} > dwidth {si[1]:g}; impedance "
                        f"must not fall as the contact gets deeper")
                r.check(si[2] > 0, f"{who}: solimp width {si[2]:g} must be > 0")
                r.check(0 < si[3] < 1,
                        f"{who}: solimp midpoint {si[3]:g} not in (0,1)")
                r.check(si[4] >= 1, f"{who}: solimp power {si[4]:g} must be >= 1")

    # Resolve the contact parameters each *instantiated* collidable geom will
    # actually end up with, by walking its default-class chain, and require them
    # to agree.  This is the check the nesting exists for: MuJoCo does not take
    # the stiffer of two contacting geoms, it takes a solmix-weighted average of
    # solref and solimp, so a floor left at the compiler default would quietly
    # halve the stiffness of every footfall and nothing would look wrong.
    CONTACT_ATTRS = ("solref", "solimp", "condim", "friction")
    cls_attr: dict = {}

    def resolve(el: ET.Element, inherited: dict) -> None:
        g = el.find("geom")
        cur = dict(inherited)
        if g is not None:
            cur.update({a: g.get(a) for a in CONTACT_ATTRS if g.get(a) is not None})
        if el.get("class"):
            cls_attr[el.get("class")] = cur
        for d in el.findall("default"):
            resolve(d, cur)

    for d in root.findall("default"):
        resolve(d, {})

    eff: dict = {}
    for el in root.find("worldbody").iter("geom"):
        cls = el.get("class")
        if cls == "visual" or el.get("contype") == "0":
            continue
        a = dict(cls_attr.get(cls, {}))
        a.update({k: el.get(k) for k in CONTACT_ATTRS if el.get(k) is not None})
        eff[el.get("name") or cls or "geom"] = a

    names = sorted(eff)
    for nm in names:
        r.check(eff[nm].get("solref") is not None,
                f"{nm}: no solref anywhere in its class chain, so it falls back "
                f"to MuJoCo's soft 0.02 s default")
    if len(names) > 1:
        base, bn = eff[names[0]], names[0]
        for nm in names[1:]:
            for a in ("solref", "solimp"):
                r.check(eff[nm].get(a) == base.get(a),
                        f"contact parameter {a} differs between {bn} "
                        f"({base.get(a)!r}) and {nm} ({eff[nm].get(a)!r}); MuJoCo "
                        f"averages these across a contact pair, so the mismatch "
                        f"softens every contact between them")

    # ---- names, references, quats, inertials -----------------------------
    seen: dict = {}

    def uniq(kind: str, nm: str | None) -> None:
        if nm is None:
            return
        key = (kind, nm)
        r.check(key not in seen, f"duplicate {kind} name {nm!r}")
        seen[key] = True

    bodies, joints, sites, actuators = {}, {}, {}, {}
    geom_bbox_lo, geom_bbox_hi = [], []
    qorder: list = []          # joints in qpos order, i.e. document order
    collidable: list = []      # (geom name, owning body, world points)

    def walk(body: ET.Element, parent_world: np.ndarray) -> None:
        nm = body.get("name")
        uniq("body", nm)
        # this walker composes translations only, so assert the generator did
        # not emit a rotated body frame (all three segment frames are
        # world-aligned at qpos = 0 by construction)
        r.check(body.get("quat") is None and body.get("euler") is None,
                f"body {nm!r} has a rotated frame; the validator assumes "
                f"translation-only body poses")
        wpos = parent_world + (nums(body, "pos") if body.get("pos") else np.zeros(3))
        bodies[nm] = wpos

        inert = body.find("inertial")
        if r.check(inert is not None, f"body {nm!r} has no <inertial> "
                                     f"(compiler inertiafromgeom is off)"):
            m = float(inert.get("mass"))
            d = nums(inert, "diaginertia")
            q = nums(inert, "quat")
            r.check(m > 0, f"body {nm!r}: mass {m} is not positive")
            r.check(bool(np.all(d > 0)),
                    f"body {nm!r}: diaginertia {d} must be strictly positive")
            a, b, c = np.sort(d)
            r.check(a + b >= c * (1 - 1e-12),
                    f"body {nm!r}: inertia violates the triangle inequality "
                    f"({a:.4g}+{b:.4g} < {c:.4g})")
            r.check(abs(np.linalg.norm(q) - 1) < 1e-6,
                    f"body {nm!r}: inertial quat norm {np.linalg.norm(q):.9f} != 1")

        # document order matters: qpos is laid out in it
        for j in body:
            if j.tag == "freejoint":
                uniq("joint", j.get("name"))
                joints[j.get("name")] = dict(body=nm, free=True)
                qorder.append((j.get("name"), 7))
            elif j.tag == "joint":
                jn = j.get("name")
                uniq("joint", jn)
                joints[jn] = dict(body=nm, wpos=wpos + nums(j, "pos"),
                                  axis=nums(j, "axis"), range=nums(j, "range"))
                r.check(abs(np.linalg.norm(joints[jn]["axis"]) - 1) < 1e-9,
                        f"joint {jn!r}: axis is not unit length")
                lo, hi = joints[jn]["range"]
                r.check(lo < hi, f"joint {jn!r}: range {lo} .. {hi} is not increasing")
                qorder.append((jn, 1))

        for g in body.findall("geom"):
            uniq("geom", g.get("name"))
            cls = g.get("class")
            r.check(cls is None or cls in classes,
                    f"geom in {nm!r} uses undeclared class {cls!r}")
            q = nums(g, "quat")
            if q is not None:
                r.check(abs(np.linalg.norm(q) - 1) < 1e-6,
                        f"geom {g.get('mesh') or g.get('name')} in {nm!r}: "
                        f"quat norm {np.linalg.norm(q):.9f} != 1")
            mref = g.get("mesh")
            pts = None
            if mref is not None:
                r.check(mref in meshes, f"geom in {nm!r} references undeclared "
                                        f"mesh {mref!r}")
                if mref in meshes and meshes[mref] in ondisk:
                    gp = nums(g, "pos") if g.get("pos") else np.zeros(3)
                    gR = quat_to_mat(q) if q is not None else np.eye(3)
                    pts = cad.triangles(meshes[mref]).reshape(-1, 3) @ gR.T + gp + wpos
                    geom_bbox_lo.append(pts.min(0))
                    geom_bbox_hi.append(pts.max(0))
            elif g.get("type") == "box":
                gp = nums(g, "pos") if g.get("pos") else np.zeros(3)
                gR = quat_to_mat(q) if q is not None else np.eye(3)
                half = nums(g, "size")
                # boxes may be rotated (the old sole box was), so bound the
                # eight rotated corners rather than pos +/- size
                sgn = np.array([[a, b, c] for a in (-1, 1) for b in (-1, 1)
                                for c in (-1, 1)], float)
                pts = (sgn * half) @ gR.T + gp + wpos
                geom_bbox_lo.append(pts.min(0))
                geom_bbox_hi.append(pts.max(0))
            if cls == "collision" and pts is not None:
                collidable.append((g.get("name"), nm, pts))

        for s in body.findall("site"):
            uniq("site", s.get("name"))
            sites[s.get("name")] = wpos + (nums(s, "pos") if s.get("pos") else 0)

        for child in body.findall("body"):
            walk(child, wpos)

    wb = root.find("worldbody")
    for s in wb.findall("site"):
        uniq("site", s.get("name"))
        sites[s.get("name")] = nums(s, "pos") if s.get("pos") else np.zeros(3)
    for g in wb.findall("geom"):
        uniq("geom", g.get("name"))
    for body in wb.findall("body"):
        walk(body, np.zeros(3))

    # ---- actuators --------------------------------------------------------
    for el in root.iterfind("actuator/*"):
        nm = el.get("name")
        uniq("actuator", nm)
        actuators[nm] = el
        jn = el.get("joint")
        r.check(jn in joints, f"actuator {nm!r} references unknown joint {jn!r}")
        cr, fr = nums(el, "ctrlrange"), nums(el, "forcerange")
        if cr is not None:
            r.check(cr[0] < cr[1], f"actuator {nm!r}: ctrlrange is not increasing")
            if jn in joints and "range" in joints[jn]:
                jr = joints[jn]["range"]
                r.check(cr[0] >= jr[0] - 1e-9 and cr[1] <= jr[1] + 1e-9,
                        f"actuator {nm!r}: ctrlrange {cr} escapes joint range {jr}")
        if fr is not None:
            r.check(fr[0] < fr[1], f"actuator {nm!r}: forcerange is not increasing")

    # ---- sensors ----------------------------------------------------------
    for el in root.iterfind("sensor/*"):
        nm = el.get("name")
        uniq("sensor", nm)
        for attr, pool, label in (("joint", joints, "joint"),
                                  ("site", sites, "site"),
                                  ("actuator", actuators, "actuator")):
            v = el.get(attr)
            if v is not None:
                r.check(v in pool, f"sensor {nm!r} references unknown {label} {v!r}")
        if el.get("objtype") == "site":
            r.check(el.get("objname") in sites,
                    f"sensor {nm!r} references unknown site {el.get('objname')!r}")

    # ---- keyframes --------------------------------------------------------
    #
    # Worth checking rather than trusting, for two reasons.  A qpos of the wrong
    # LENGTH is a hard compile error in MuJoCo, and it is easy to get wrong
    # because the free joint contributes 7 and the two hinges 1 each -- 9 here,
    # 2 in the bench scene.  And a keyframe whose whole purpose is "stand the
    # leg flat on the floor" is exactly the kind of claim that can be silently
    # false: the previous sole geometry sat 4.6 mm below the mesh it was meant
    # to hug, and nothing noticed.  So the flat pose is re-derived here from the
    # numbers actually in the XML -- rotate the emitted sole geom about the
    # emitted ankle anchor by the emitted keyframe angle, shift by the emitted
    # base height -- and required to touch down.
    nq = sum(n for _, n in qorder)
    for k in root.iterfind("keyframe/key"):
        kn = k.get("name")
        uniq("key", kn)
        qp = nums(k, "qpos")
        if not r.check(qp is not None and len(qp) == nq,
                       f"keyframe {kn!r}: qpos has "
                       f"{0 if qp is None else len(qp)} entries, nq is {nq}"):
            continue
        ct = nums(k, "ctrl")
        if ct is not None:
            r.check(len(ct) == len(actuators),
                    f"keyframe {kn!r}: ctrl has {len(ct)} entries, "
                    f"{len(actuators)} actuators")
        q, ang = 0, {}
        for jn, n in qorder:
            if n == 1:
                ang[jn] = float(qp[q])
                jr = joints[jn]["range"]
                r.check(jr[0] - 1e-9 <= qp[q] <= jr[1] + 1e-9,
                        f"keyframe {kn!r}: {jn} = {math.degrees(qp[q]):.3f} deg "
                        f"is outside its range "
                        f"[{math.degrees(jr[0]):.1f}, {math.degrees(jr[1]):.1f}]")
            else:
                r.check(abs(np.linalg.norm(qp[q + 3:q + 7]) - 1) < 1e-6,
                        f"keyframe {kn!r}: free-joint quat is not normalised")
            q += n

        if scene != "ground" or not collidable:
            continue
        root_body = joints["root"]["body"]
        dz = float(qp[2]) - float(bodies[root_body][2])
        r.check(abs(float(qp[0]) - bodies[root_body][0]) < 1e-9
                and abs(float(qp[1]) - bodies[root_body][1]) < 1e-9,
                f"keyframe {kn!r}: free-joint x/y {qp[0:2]} do not match the "
                f"{root_body} body pos {bodies[root_body][0:2]}")
        # only the ankle is allowed to move in this pose; a bent knee would
        # need real forward kinematics, so say so instead of guessing
        if abs(ang.get("knee", 0.0)) > 1e-9:
            r.check(True, "")  # nothing to verify geometrically
            continue
        th = ang.get("ankle", 0.0)
        ax = joints["ankle"]["axis"] / np.linalg.norm(joints["ankle"]["axis"])
        K = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
        R = np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * (K @ K)
        anchor = joints["ankle"]["wpos"]
        low = np.inf
        for gname, owner, pts in collidable:
            p = (pts - anchor) @ R.T + anchor if owner == joints["ankle"]["body"] else pts
            low = min(low, float(p[:, 2].min()) + dz)
        r.check(low > -1e-6,
                f"keyframe {kn!r}: collision geometry starts {-low * 1000:.2f} mm "
                f"below the floor")
        r.check(low < 0.004,
                f"keyframe {kn!r}: collision geometry starts {low * 1000:.2f} mm "
                f"above the floor, so the leg is dropped rather than placed")
        print(f"\n{os.path.basename(path)} keyframe {kn!r}: nq={nq}, ankle "
              f"{math.degrees(ang.get('ankle', 0.0)):+.4f} deg, lowest collision "
              f"point {low * 1000:+.2f} mm above the floor")

    # ---- kinematics against the CAD measurement ---------------------------
    lift = bodies["thigh"][2]
    for jn, want in (("knee", B.KNEE_POS), ("ankle", B.ANKLE_POS)):
        got = joints[jn]["wpos"] - np.array([0, 0, lift])
        r.check(float(np.linalg.norm(got - want)) < 1e-6,
                f"joint {jn!r} sits at {np.round(got, 6)}, CAD says {want}")

    # ---- physics plausibility --------------------------------------------
    total = sum(float(b.find("inertial").get("mass"))
                for b in root.iter("body") if b.find("inertial") is not None)
    r.check(MASS_RANGE[0] <= total <= MASS_RANGE[1],
            f"total mass {total:.3f} kg is outside the expected "
            f"{MASS_RANGE[0]}-{MASS_RANGE[1]} kg for the CAD-derived model "
            f"(published hardware figure {B.PUBLISHED_MASS} kg, motors absent "
            f"from the export)", warn_only=True)

    lo = np.min(np.array(geom_bbox_lo), axis=0)
    hi = np.max(np.array(geom_bbox_hi), axis=0)
    for b in root.iter("body"):
        inert = b.find("inertial")
        if inert is None:
            continue
        com = bodies[b.get("name")] + nums(inert, "pos")
        r.check(bool(np.all(com >= lo - 1e-6) and np.all(com <= hi + 1e-6)),
                f"body {b.get('name')!r} COM {np.round(com, 4)} lies outside the "
                f"model bbox")

    if scene == "ground":
        r.check(lo[2] > -1e-6,
                f"geometry dips below the floor by {-lo[2] * 1000:.1f} mm")
        r.check(lo[2] < 0.02,
                f"the leg floats {lo[2] * 1000:.1f} mm above the floor", warn_only=True)

    print(f"\n{os.path.basename(path)}: bbox "
          f"{np.round(lo, 4)} .. {np.round(hi, 4)}  total mass {total:.4f} kg")
    return r.summary(os.path.basename(path))


def audit(path: str, model: B.OslModel) -> None:
    rows = []
    for seg in B.SEGMENTS:
        for kind, names in (("kept", model.kept.get(seg, [])),
                            ("culled", model.culled.get(seg, []))):
            for n in names:
                mp = model.cad.mass_props(model.cad.links[n].mesh_file)
                rho, rule = density_for(n)
                rows.append(dict(segment=seg, status=kind, link=n,
                                 mesh=model.cad.links[n].mesh_file,
                                 volume_cm3=round(mp.volume * 1e6, 4),
                                 density_kg_m3=rho, density_rule=rule,
                                 mass_g=round(rho * mp.volume * 1000, 3),
                                 triangles=mp.ntri, watertight=mp.closed))
    rows.sort(key=lambda d: -d["mass_g"])
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    print(f"\nwrote {os.path.relpath(path, ROOT)}  ({len(rows)} parts) -- weigh the "
          f"real parts and correct DENSITY_RULES in tools/oslcad.py against it")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--audit", nargs="?", const=os.path.join(ROOT, "build", "mass_audit.csv"))
    args = ap.parse_args()

    model = B.OslModel()
    cad = model.cad
    allok = True
    for scene in ("bench", "ground"):
        p = os.path.join(MODELS, f"osl_v2_{scene}.xml")
        if not os.path.exists(p):
            print(f"FAIL  {p} not found -- run tools/build_mjcf.py first")
            allok = False
            continue
        allok &= validate(p, cad, model)
    if args.audit:
        audit(args.audit, model)
    print()
    sys.exit(0 if allok else 1)


if __name__ == "__main__":
    main()
