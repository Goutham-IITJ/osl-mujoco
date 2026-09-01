#!/usr/bin/env python3
"""
view_osl.py -- open the OSL V2 model in the MuJoCo viewer.

    python scripts/view_osl.py                 # bench: pyramid welded to world
    python scripts/view_osl.py --scene ground   # freejoint + floor
    python scripts/view_osl.py --scene ground --flat   # standing, sole level
    python scripts/view_osl.py --knee 60 --ankle -15

Viewer keys worth knowing: space pauses, Tab shows the control panel (drag the
knee/ankle sliders under "Control").  The Group toggles hide geom groups -- the
visual meshes are group 2; group 3 is the collision geometry, which only exists
in the ground scene: the blade mesh at the sole, plus four bounding boxes that
stop the shank and knee_prox reaching the floor.

--flat loads the `flat` keyframe, which puts the ankle at +2.40 deg.  That is
the angle that levels the sole: at ankle = 0 the blade is toe-down and the leg
balances on the forefoot keel.
"""

from __future__ import annotations

import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mujoco  # noqa: E402
from mjcommon import joint_limits, nid, scene_path, viewer_loop  # noqa: E402


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--scene", choices=("bench", "ground", "walk"), default="bench")
    ap.add_argument("--knee", type=float, default=0.0, help="initial knee angle, deg")
    ap.add_argument("--ankle", type=float, default=0.0, help="initial ankle angle, deg")
    ap.add_argument("--flat", action="store_true",
                    help="start from the `flat` keyframe: sole level on the floor")
    args = ap.parse_args()

    path = scene_path(args.scene)
    model = mujoco.MjModel.from_xml_path(path)
    data = mujoco.MjData(model)

    if args.flat:
        mujoco.mj_resetDataKeyframe(model, data,
                                    nid(model, mujoco.mjtObj.mjOBJ_KEY, "flat"))
        mujoco.mj_forward(model, data)
        aid = nid(model, mujoco.mjtObj.mjOBJ_JOINT, "ankle")
        print(f"keyframe 'flat': ankle "
              f"{math.degrees(data.qpos[model.jnt_qposadr[aid]]):+.2f} deg, "
              f"which is the angle that levels the sole")
    else:
        for nm, deg in (("knee", args.knee), ("ankle", args.ankle)):
            jid = nid(model, mujoco.mjtObj.mjOBJ_JOINT, nm)
            q = math.radians(deg)
            lim = joint_limits(model, jid)
            if lim is not None and not (lim[0] <= q <= lim[1]):
                q = min(max(q, lim[0]), lim[1])
                print(f"--{nm} {deg:+.1f} deg is outside the joint limit "
                      f"{math.degrees(lim[0]):+.1f} .. {math.degrees(lim[1]):+.1f}; "
                      f"clamped to {math.degrees(q):+.1f}")
            data.qpos[model.jnt_qposadr[jid]] = q
            data.ctrl[nid(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{nm}_pos")] = q
        mujoco.mj_forward(model, data)
        if args.scene == "ground" and args.knee == 0.0 and args.ankle == 0.0:
            print("note: at ankle = 0 the sole is 2.4 deg toe-down, so the leg "
                  "starts on the forefoot keel.\n      Add --flat to stand it "
                  "level.")

    print(f"{os.path.basename(path)}: {model.nbody - 1} bodies, {model.ngeom} geoms, "
          f"{model.body_mass.sum():.3f} kg.  Tab -> Control for the joint sliders.")

    viewer_loop(model, data)


if __name__ == "__main__":
    main()
