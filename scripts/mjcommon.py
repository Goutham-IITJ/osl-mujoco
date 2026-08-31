#!/usr/bin/env python3
"""
mjcommon.py -- the few things check_model / view_osl / demo_sweep all need.

Kept deliberately small.  The only non-obvious piece is `viewer_loop`: syncing
the viewer after every 0.5 ms physics step asks the renderer for 2000 fps, and
because a sync costs on the order of a millisecond the sleep-to-real-time
arithmetic never fires and the simulation ends up running *slower* than real
time.  So we step in batches and sync once per display frame.
"""

from __future__ import annotations

import os
import sys
import time

try:
    import mujoco
except ImportError:
    sys.exit("mujoco is not installed in this interpreter.\n"
             "  .venv\\Scripts\\activate  &&  pip install mujoco")

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FRAME_DT = 1.0 / 60.0


def scene_path(scene: str) -> str:
    p = os.path.join(ROOT, "models", f"osl_v2_{scene}.xml")
    if not os.path.exists(p):
        sys.exit(f"{p} not found -- run  python tools/build_mjcf.py")
    return p


def nid(model, objtype, name: str) -> int:
    """mj_name2id, but a missing name is an error instead of index -1.

    Without this, a typo silently indexes the *last* element: with exactly two
    actuators, a mistyped "knee_pos" would command the ankle and every number
    printed afterwards would still look plausible.
    """
    i = mujoco.mj_name2id(model, objtype, name)
    if i < 0:
        kind = mujoco.mjtObj(objtype).name.replace("mjOBJ_", "").lower()
        sys.exit(f"no {kind} named {name!r} in this model")
    return i


def joint_limits(model, jid: int) -> tuple:
    """(lo, hi) in radians, or None if the joint is unlimited."""
    if not model.jnt_limited[jid]:
        return None
    lo, hi = model.jnt_range[jid]
    return float(lo), float(hi)


def viewer_loop(model, data, on_step=None, after_step=None, until=None) -> None:
    """Step in real time with one viewer sync per display frame.

    on_step(t) is called immediately before each physics step, with the
    simulation time that step starts from.  after_step() is called immediately
    after it, when sensordata holds the state at t + dt.  until(t) stops the run.
    """
    import mujoco.viewer

    steps_per_frame = max(1, int(round(FRAME_DT / model.opt.timestep)))
    with mujoco.viewer.launch_passive(model, data) as v:
        while v.is_running():
            t0 = time.perf_counter()
            for _ in range(steps_per_frame):
                if until is not None and until(data.time):
                    v.sync()
                    return
                if on_step is not None:
                    on_step(data.time)
                mujoco.mj_step(model, data)
                if after_step is not None:
                    after_step()
            v.sync()
            slack = steps_per_frame * model.opt.timestep - (time.perf_counter() - t0)
            if slack > 0:
                time.sleep(slack)
