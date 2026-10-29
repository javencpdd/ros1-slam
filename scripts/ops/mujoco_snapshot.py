#!/usr/bin/env python3
"""Render one MuJoCo Lite3 frame on a generated 3-step world to PNG.

The world is composed from the supplied flat MJCF; no third-party file is edited.
"""
import argparse
import os
import shutil
from pathlib import Path

import numpy as np

os.environ.setdefault('MUJOCO_GL', 'osmesa')


def main():
    import mujoco
    from PIL import Image

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--flat-xml', required=True)
    parser.add_argument('--out', required=True)
    parser.add_argument('--step-h', type=float, default=.15)
    parser.add_argument('--steps', type=int, default=3)
    parser.add_argument('--base', nargs=3, type=float, default=[1.75, 0., .64])
    parser.add_argument('--pose', nargs=12, type=float, default=[0., -1., 1.8] * 4)
    parser.add_argument('--lookat', nargs=3, type=float, default=[1.6, 0., .4])
    parser.add_argument('--distance', type=float, default=2.4)
    parser.add_argument('--azimuth', type=float, default=90.)
    parser.add_argument('--elevation', type=float, default=-5.)
    parser.add_argument('--width', type=int, default=640)
    parser.add_argument('--height', type=int, default=480)
    args = parser.parse_args()
    dynamics = Path(__file__).resolve().parents[2] / 'ws/src/95_simulation/lite3_gazebo_sim/dynamics'
    import sys
    sys.path.insert(0, str(dynamics))
    import mj_dreamwaq_walk as walk

    world = walk.make_stairs_xml(args.flat_xml, args.step_h, args.steps)
    try:
        model = mujoco.MjModel.from_xml_path(world)
        data = mujoco.MjData(model)
        data.qpos[:3] = args.base
        data.qpos[3:7] = [1., 0., 0., 0.]
        data.qpos[7:19] = args.pose
        mujoco.mj_forward(model, data)
        renderer = mujoco.Renderer(model, args.height, args.width)
        camera = mujoco.MjvCamera()
        camera.lookat[:] = args.lookat
        camera.distance = args.distance
        camera.azimuth = args.azimuth
        camera.elevation = args.elevation
        renderer.update_scene(data, camera=camera)
        image = renderer.render()
        Image.fromarray(image).save(args.out)
    finally:
        shutil.rmtree(Path(world).parent, ignore_errors=True)
    print(args.out)


if __name__ == '__main__':
    main()
