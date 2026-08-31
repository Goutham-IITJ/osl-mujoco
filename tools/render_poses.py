#!/usr/bin/env python3
"""
render_poses.py -- offscreen check that the emitted MJCF articulates correctly.

Reads models/osl_v2_bench.xml (so it checks the *generated file*, not the
in-memory model), applies the knee and ankle hinge rotations itself, and
rasterises the result with the pure-numpy renderer.  This catches the errors a
compile check cannot: a segment assigned to the wrong body, a flipped axis
sign, a hinge in the wrong place.

MuJoCo is not needed and is not available in the analysis sandbox.

Usage:  python tools/render_poses.py [--out build/verify/poses.png]
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from oslcad import OslCad  # noqa: E402
from softrender import ChunkRenderer, hstack_images, write_png  # noqa: E402
from validate_mjcf import nums, quat_to_mat  # noqa: E402
import build_mjcf as B  # noqa: E402

# knee, ankle in degrees.  Positive = flexion / dorsiflexion.
POSES = [
    (0, 0),
    (30, 0),
    (60, 0),
    (90, 0),
    (0, -30),
    (0, 20),
    (60, -15),
]

SEG_TINT = {"thigh": (1.00, 0.55, 0.15), "shank": None, "foot": (0.20, 0.75, 0.35)}


def axis_angle(axis: np.ndarray, ang: float) -> np.ndarray:
    a = axis / np.linalg.norm(axis)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(ang) * K + (1 - math.cos(ang)) * (K @ K)


def load(path: str) -> tuple:
    """-> (bodies, meshdir).  bodies: name -> dict(origin, joint, geoms, parent)."""
    root = ET.parse(path).getroot()
    meshdir = os.path.normpath(os.path.join(os.path.dirname(path),
                                            root.find("compiler").get("meshdir")))
    files = {m.get("name"): m.get("file") for m in root.iterfind("asset/mesh")}
    bodies: dict = {}

    def walk(el: ET.Element, parent: str | None, world: np.ndarray) -> None:
        nm = el.get("name")
        origin = world + (nums(el, "pos") if el.get("pos") else np.zeros(3))
        j = el.find("joint")
        geoms = []
        for g in el.findall("geom"):
            if g.get("mesh") is None or g.get("class") == "collision":
                continue  # boxes have no mesh; the sole shares the blade's
            q = nums(g, "quat")
            rgba = nums(g, "rgba")
            geoms.append((files[g.get("mesh")],
                          nums(g, "pos") if g.get("pos") else np.zeros(3),
                          quat_to_mat(q) if q is not None else np.eye(3),
                          rgba[:3] if rgba is not None else np.array([0.7, 0.7, 0.7])))
        bodies[nm] = dict(parent=parent, origin=origin, geoms=geoms,
                          axis=nums(j, "axis") if j is not None else None)
        for c in el.findall("body"):
            walk(c, nm, origin)

    for b in root.find("worldbody").findall("body"):
        walk(b, None, np.zeros(3))
    return bodies, meshdir


def place(bodies: dict, q: dict) -> dict:
    """Forward kinematics through the emitted tree -> name -> (offset, R)."""
    out: dict = {}
    for nm, b in bodies.items():
        p, R = np.zeros(3), np.eye(3)
        chain = []
        cur = nm
        while cur is not None:
            chain.append(cur)
            cur = bodies[cur]["parent"]
        for link in reversed(chain):
            bb = bodies[link]
            # body origin is expressed in the parent frame
            local = bb["origin"] - (np.zeros(3) if bb["parent"] is None
                                    else bodies[bb["parent"]]["origin"])
            p = p + R @ local
            if bb["axis"] is not None:
                R = R @ axis_angle(bb["axis"], q.get(link, 0.0))
        out[nm] = (p, R, bodies[nm]["origin"])
    return out


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", default=os.path.join(B.MODELS, "osl_v2_bench.xml"))
    ap.add_argument("--out", default=os.path.join(B.ROOT, "build", "verify", "poses.png"))
    ap.add_argument("--width", type=int, default=300)
    ap.add_argument("--height", type=int, default=560)
    args = ap.parse_args()

    bodies, meshdir = load(args.model)
    cad = OslCad(B.URDF, B.MESHES)  # mesh cache only
    print(f"{os.path.basename(args.model)}: bodies "
          f"{ {k: len(v['geoms']) for k, v in bodies.items()} }")

    # a generous shared bbox so every pose is drawn at the same scale
    lo = np.array([-0.34, -0.20, -0.34])
    hi = np.array([0.20, 0.20, 0.32])
    imgs = []
    for kdeg, adeg in POSES:
        q = {"shank": math.radians(kdeg), "foot": math.radians(adeg)}
        fk = place(bodies, q)
        ctr = np.array([-0.06, 0.0, 0.0])
        r = ChunkRenderer(eye=ctr + np.array([0.0, -1.6, 0.0]), target=ctr,
                          bbox_min=lo, bbox_max=hi,
                          width=args.width, height=args.height)
        for nm, b in bodies.items():
            p, R, _ = fk[nm]
            tint = SEG_TINT.get(nm)
            for f, gp, gR, rgba in b["geoms"]:
                v = cad.triangles(f)
                t = v @ (R @ gR).T + (p + R @ gp)
                c = np.array(tint) if tint else np.array(rgba)
                r.add(t, np.tile(c, (len(t), 1)))
        imgs.append(r.image())
        print(f"  rendered knee={kdeg:+4d} deg  ankle={adeg:+4d} deg")

    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    write_png(args.out, hstack_images(imgs, gap=6, bg=(1, 1, 1)))
    lbl = "  ".join(f"k{k:+d}/a{a:+d}" for k, a in POSES)
    print(f"\nwrote {os.path.relpath(args.out, B.ROOT)}\n  panels: {lbl}"
          f"\n  thigh tinted orange, foot green, shank as-modelled; "
          f"sagittal view, anterior to the LEFT (-X)")


if __name__ == "__main__":
    main()
