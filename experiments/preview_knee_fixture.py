"""View/render the fixture at a prescribed angle; not a PID response experiment."""
import argparse
from pathlib import Path
import math
import time

import mujoco

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--angle', type=float, default=15., help='Relative knee angle in degrees')
    parser.add_argument('--view', action='store_true')
    parser.add_argument('--png', type=Path)
    args = parser.parse_args()
    model = mujoco.MjModel.from_xml_path(str(ROOT/'models/osl_v2_knee_fixture.xml'))
    model.vis.headlight.ambient[:] = [.45, .45, .45]
    model.vis.headlight.diffuse[:] = [.8, .8, .8]
    data = mujoco.MjData(model)
    data.qpos[0] = math.radians(args.angle)
    mujoco.mj_forward(model, data)
    print('POSE PREVIEW ONLY: angle prescribed; no PID or hardware connection.')
    print(f'nq={model.nq}, moving mass={model.body_mass[-1]:.6f} kg')
    def camera(cam):
        cam.lookat[:] = [0., 0., .13]
        cam.distance = .48
        cam.azimuth = 110
        cam.elevation = -10
    if args.png:
        from PIL import Image
        args.png.parent.mkdir(parents=True, exist_ok=True)
        model.vis.global_.offwidth = 960
        model.vis.global_.offheight = 720
        with mujoco.Renderer(model, height=720, width=960) as renderer:
            cam = mujoco.MjvCamera()
            camera(cam)
            renderer.update_scene(data, camera=cam)
            Image.fromarray(renderer.render()).save(args.png)
    if args.view:
        from mujoco import viewer as mj_viewer
        with mj_viewer.launch_passive(model, data) as viewer:
            camera(viewer.cam)
            while viewer.is_running():
                viewer.sync()
                time.sleep(.03)


if __name__ == '__main__':
    main()
