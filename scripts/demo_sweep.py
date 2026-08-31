#!/usr/bin/env python3
"""
demo_sweep.py -- drive the knee and ankle through a sinusoidal sweep and log
every sensor, so the whole loop (actuator -> physics -> sensor) is exercised.

    python scripts/demo_sweep.py                       # live viewer
    python scripts/demo_sweep.py --headless --csv build/sweep.csv

This is the "does it actually run" demo.  It drives both joints across most of
their range and records joint angles, velocities, actuator torques, the two IMU
signals and the load cell -- exactly the data a gait-phase estimator or an IMU
filter would later consume.

CSV timing, stated explicitly so nobody has to guess: each row describes one
physics step.  `t_cmd` is the time the command was applied and the time the
actuator torque was computed for; `t_state` is `t_cmd + dt`, the time the sensor
columns refer to, because MuJoCo populates sensordata after integrating.  The
two differ by one timestep (0.5 ms) and conflating them puts a half-sample bias
into anything fitted to this log.
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mujoco  # noqa: E402
from mjcommon import nid, scene_path, viewer_loop  # noqa: E402

# sweep shape, in degrees.  Knee stays flexion-only; the ankle swings either
# side of neutral.  Both stay inside the ranges declared in the model
# (knee -5..+120, ankle -30..+20), with margin at each end.
KNEE_MID, KNEE_AMP = 55.0, 55.0
ANKLE_MID, ANKLE_AMP = -5.0, 20.0
PERIOD = 2.0  # s, roughly one gait cycle


def sensor_layout(model) -> list:
    """[(name, address, dim)] for every sensor, in declaration order."""
    return [(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SENSOR, i),
             int(model.sensor_adr[i]), int(model.sensor_dim[i]))
            for i in range(model.nsensor)]


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--scene", choices=("bench", "ground"), default="bench")
    ap.add_argument("--headless", action="store_true")
    ap.add_argument("--duration", type=float, default=6.0)
    ap.add_argument("--csv", default=None)
    args = ap.parse_args()

    path = scene_path(args.scene)
    model = mujoco.MjModel.from_xml_path(path)
    data = mujoco.MjData(model)

    knee_a = nid(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "knee_pos")
    ankle_a = nid(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "ankle_pos")
    knee_j = nid(model, mujoco.mjtObj.mjOBJ_JOINT, "knee")
    ankle_j = nid(model, mujoco.mjtObj.mjOBJ_JOINT, "ankle")
    sensors = sensor_layout(model)
    dt = model.opt.timestep

    def target(t: float) -> tuple:
        ph = 2 * math.pi * t / PERIOD
        return (math.radians(KNEE_MID - KNEE_AMP * math.cos(ph)),
                math.radians(ANKLE_MID + ANKLE_AMP * math.sin(ph)))

    rows: list = []
    peak = np.zeros(2)

    def on_step(t: float) -> None:
        """Apply the command for time t.  Called just before each mj_step."""
        k, a = target(t)
        data.ctrl[knee_a], data.ctrl[ankle_a] = k, a
        if args.csv:
            rows.append({"t_cmd": round(t, 6), "t_state": round(t + dt, 6),
                         "knee_cmd_deg": round(math.degrees(k), 4),
                         "ankle_cmd_deg": round(math.degrees(a), 4)})

    def after_step() -> None:
        peak[0] = max(peak[0], abs(data.actuator_force[knee_a]))
        peak[1] = max(peak[1], abs(data.actuator_force[ankle_a]))
        if args.csv:
            row = rows[-1]
            for nm, adr, dim in sensors:
                for j in range(dim):
                    key = nm if dim == 1 else f"{nm}[{j}]"
                    row[key] = round(float(data.sensordata[adr + j]), 8)

    if args.headless:
        while data.time < args.duration:
            on_step(data.time)
            mujoco.mj_step(model, data)
            after_step()
    else:
        print("viewer open -- close the window to stop early")
        viewer_loop(model, data, on_step=on_step, after_step=after_step,
                    until=lambda t: t >= args.duration)

    print(f"simulated {data.time:.3f} s at {dt * 1e3:.2f} ms")
    print(f"peak actuator torque: knee {peak[0]:.2f} Nm, ankle {peak[1]:.2f} Nm")
    print(f"final angles: knee {math.degrees(data.qpos[model.jnt_qposadr[knee_j]]):+.2f} deg, "
          f"ankle {math.degrees(data.qpos[model.jnt_qposadr[ankle_j]]):+.2f} deg")

    if not args.csv:
        return
    if not rows:
        print("no samples recorded -- nothing written")
        return
    ncol = len(rows[0])
    rows = [r for r in rows if len(r) == ncol]  # drop a half-filled last row
    out = os.path.abspath(args.csv)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, "w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    print(f"wrote {args.csv}  ({len(rows)} samples, {ncol} columns)")


if __name__ == "__main__":
    main()
