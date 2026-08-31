"""
verify_frames.py -- settle, by looking at it, which frame the Onshape STLs
are written in.  This was the FIRST of the two frame questions; see the note
at the bottom of this docstring.

Hypothesis A: each STL is in its own link frame, so the URDF forward
              kinematics must be applied to place it.
Hypothesis B: each STL is already baked in the assembly (global) frame, so
              applying the FK would double-transform it.

Whichever produces a slender, connected, leg-shaped assembly is the truth.
Writes build/verify/frames_A.png and frames_B.png plus a numeric report.

A wins (slenderness 1.63 vs 1.26), and that conclusion still stands.  But note
that "hypothesis A" here is the link pose ALONE, which is only half the
transform: 207 links also carry a `<visual><origin>` that has to be composed on
top, and that was not discovered until later.  So frames_A.png is a correct
answer to this question and a WRONG picture of the leg -- it is the same
211 x 327 x 532 mm jumble that appears as the "before" in
build/verify/visorigin_AB.png.  For the finished frame convention, read
tools/verify_visorigin.py instead; this script is kept because it is what ruled
hypothesis B out, and that still needed ruling out.
"""

from __future__ import annotations

import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from oslcad import OslCad  # noqa: E402
from softrender import ChunkRenderer, hstack_images, write_png  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
URDF = os.path.join(ROOT, "osl_v2_0_assembly", "urdf", "osl_v2_0_assembly.urdf")
MESHES = os.path.join(ROOT, "osl_v2_0_assembly", "meshes")
OUT = os.path.join(ROOT, "build", "verify")


def bbox_streaming(cad: OslCad, links: list, apply_pose: bool):
    lo = np.full(3, np.inf)
    hi = np.full(3, -np.inf)
    for n in links:
        v = cad.triangles(cad.links[n].mesh_file).reshape(-1, 3)
        if apply_pose:
            p, R = cad.pose[n]
            v = v @ R.T + p
        lo = np.minimum(lo, v.min(0))
        hi = np.maximum(hi, v.max(0))
    return lo, hi


def main() -> None:
    os.makedirs(OUT, exist_ok=True)
    cad = OslCad(URDF, MESHES)
    links = cad.mesh_links()
    print(f"{len(cad.links)} links, {len(links)} with a visual mesh")

    for label, apply_pose in (("A", True), ("B", False)):
        t0 = time.time()
        lo, hi = bbox_streaming(cad, links, apply_pose)
        size = hi - lo
        slender = size[2] / max(size[0], size[1])
        print(
            f"\nhypothesis {label} ({'FK applied' if apply_pose else 'meshes as-is'}):"
            f"\n  bbox min = {lo.round(4)}"
            f"\n  bbox max = {hi.round(4)}"
            f"\n  size     = {size.round(4)}  (m)"
            f"\n  slenderness Z / max(X,Y) = {slender:.2f}"
        )

        ctr = 0.5 * (lo + hi)
        rad = float(np.linalg.norm(size))
        eyes = {
            "sagittal (-Y)": ctr + np.array([0.0, -rad, 0.0]),
            "frontal (+X)": ctr + np.array([rad, 0.0, 0.0]),
            "iso": ctr + np.array([rad * 0.75, -rad * 0.75, rad * 0.30]),
        }
        imgs = []
        for eye in eyes.values():
            r = ChunkRenderer(eye, ctr, lo, hi, 480, 720, samples_per_px=2.0)
            # stream link by link: peak memory stays at one mesh
            for n in links:
                tri = cad.triangles(cad.links[n].mesh_file)
                if apply_pose:
                    p, R = cad.pose[n]
                    tri = tri @ R.T + p
                col = np.tile(np.asarray(cad.links[n].rgba[:3]), (len(tri), 1))
                r.add(tri, col)
            imgs.append(r.image())
        write_png(os.path.join(OUT, f"frames_{label}.png"), hstack_images(imgs))
        print(f"  -> build/verify/frames_{label}.png   ({time.time() - t0:.1f}s)")

    print(
        "\nViews are, left to right: sagittal (looking along -Y, the flexion axis),"
        "\nfrontal (looking along +X), and isometric."
    )


if __name__ == "__main__":
    main()
