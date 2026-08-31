#!/usr/bin/env python3
"""
verify_visorigin.py -- regenerate build/verify/visorigin_AB.png, the image that
settles whether <visual><origin> must be composed onto the link pose.

Four panels: sagittal and frontal views with the mesh placed by link pose
ALONE, then the same two views with the visual origin folded in.  The first
pair has the knee and ankle modules interpenetrating; the second is a clean
OSL V2 with a distinct pylon between them.

This existed as an ad-hoc analysis before; it is a script now so the claim in
README.md is reproducible rather than asserted.

    python tools/verify_visorigin.py

numpy + stdlib only.  MuJoCo is not needed.
"""

from __future__ import annotations

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from oslcad import OslCad  # noqa: E402
from softrender import ChunkRenderer, hstack_images, write_png  # noqa: E402
from build_mjcf import MESHES, ROOT, URDF, visual_origins  # noqa: E402


def placed(cad: OslCad, vis: dict, n: str, fold: bool) -> np.ndarray:
    """World-space triangles of one link, (N, 3, 3)."""
    p, R = cad.pose[n]
    if fold:
        vp, vR = vis[n]
        p, R = p + R @ vp, R @ vR
    t = cad.triangles(cad.links[n].mesh_file)
    return t.reshape(-1, 3) @ R.T + p


def bbox(cad: OslCad, vis: dict, fold: bool) -> tuple:
    lo, hi = np.full(3, np.inf), np.full(3, -np.inf)
    for n in cad.mesh_links():
        t = placed(cad, vis, n, fold)
        lo, hi = np.minimum(lo, t.min(0)), np.maximum(hi, t.max(0))
    return lo, hi


def render(cad: OslCad, vis: dict, fold: bool, lo, hi, eye_dir,
           width: int, height: int) -> np.ndarray:
    """Stream every link through the rasteriser; nothing is held in memory."""
    ctr = 0.5 * (lo + hi)
    span = float(np.max(hi - lo))
    r = ChunkRenderer(eye=ctr + span * 2.0 * np.asarray(eye_dir, float), target=ctr,
                      bbox_min=lo, bbox_max=hi, width=width, height=height)
    for n in cad.mesh_links():
        t = placed(cad, vis, n, fold).reshape(-1, 3, 3)
        rgba = cad.links[n].rgba
        c = np.array(rgba[:3] if rgba else (0.6, 0.6, 0.65))
        r.add(t, np.tile(c, (len(t), 1)))
    return r.image()


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=os.path.join(ROOT, "build", "verify",
                                                  "visorigin_AB.png"))
    ap.add_argument("--width", type=int, default=260)
    ap.add_argument("--height", type=int, default=620)
    args = ap.parse_args()

    cad = OslCad(URDF, MESHES)
    vis = visual_origins(URDF)

    imgs = []
    for fold in (False, True):
        lo, hi = bbox(cad, vis, fold)
        size = 1e3 * (hi - lo)
        print(f"{'visual origin FOLDED' if fold else 'link pose ONLY   '}: "
              f"bbox {size[0]:.0f} x {size[1]:.0f} x {size[2]:.0f} mm")
        for eye_dir in ((0.0, -1.0, 0.0), (-1.0, 0.0, 0.0)):
            imgs.append(render(cad, vis, fold, lo, hi, eye_dir,
                               args.width, args.height))

    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    write_png(args.out, hstack_images(imgs, gap=8, bg=(1, 1, 1)))
    print(f"\nwrote {os.path.relpath(args.out, ROOT)}")
    print("panels: [link pose only: sagittal, frontal] "
          "[visual origin folded: sagittal, frontal]")
    print("The first pair should look jumbled -- the knee and ankle modules")
    print("interpenetrating -- and the second should show a clean leg with a")
    print("distinct pylon.  If it does not, stop and re-derive before trusting")
    print("anything else in this repo.")


if __name__ == "__main__":
    main()
