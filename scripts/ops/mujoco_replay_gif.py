#!/usr/bin/env python3
"""Render a recorded V8-X1 MuJoCo qpos trace to an animated GIF for the web page.

The trace is produced by a5_v8x1_bridge --trace; this tool only replays qpos
from that evidence, not a new controller run. It is a visualization of an
existing test replay, not a formal/real-robot acceptance artifact.
"""
import argparse
import gzip
import json
import os
import shutil
import sys
from pathlib import Path

os.environ.setdefault('MUJOCO_GL', 'osmesa')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trace', required=True, help='*.jsonl or *.jsonl.gz')
    parser.add_argument('--out', required=True)
    parser.add_argument('--flat-xml', default='/tmp/mjcf/Lite3.xml')
    parser.add_argument('--step-h', type=float, default=.1988)
    parser.add_argument('--steps', type=int, default=5)
    parser.add_argument('--depth', type=float, default=.72)
    parser.add_argument('--landing', type=float, default=2.0)
    parser.add_argument('--stride', type=int, default=10)
    parser.add_argument('--fps', type=float, default=12.5)
    parser.add_argument('--width', type=int, default=640)
    parser.add_argument('--height', type=int, default=400)
    parser.add_argument('--distance', type=float, default=2.8)
    parser.add_argument('--elevation', type=float, default=-10.)
    parser.add_argument('--colors', type=int, default=64,
                        help='palette size; 0 keeps full RGB')
    parser.add_argument('--label', default='MuJoCo V8-X1 replay',
                        help='overlay label; tick progress is appended')
    args = parser.parse_args()
    if args.stride < 1 or args.fps <= 0:
        raise SystemExit('stride >= 1 and fps > 0 required')

    import mujoco
    import numpy as np
    from PIL import Image, ImageDraw

    opener = gzip.open if args.trace.endswith('.gz') else open
    with opener(args.trace, 'rt', encoding='utf-8') as stream:
        rows = [json.loads(line) for line in stream if line.strip()]
    if not rows:
        raise SystemExit('empty trace')

    dynamics = Path(__file__).resolve().parents[2] / 'ws/src/95_simulation/lite3_gazebo_sim/dynamics'
    sys.path.insert(0, str(dynamics))
    import mj_dreamwaq_walk as walk

    world = walk.make_stairs_xml(args.flat_xml, args.step_h, args.steps, args.depth,
                                 3.0, 1.0, args.landing)
    frames = []
    try:
        model = mujoco.MjModel.from_xml_path(world)
        data = mujoco.MjData(model)
        renderer = mujoco.Renderer(model, args.height, args.width)
        camera = mujoco.MjvCamera()
        camera.azimuth = 90.0
        camera.elevation = args.elevation
        camera.distance = args.distance
        selected = rows[::args.stride]
        if selected[-1] is not rows[-1]:
            selected.append(rows[-1])
        for index, row in enumerate(selected):
            qpos = np.asarray(row['qpos'], dtype=float)
            if qpos.shape != (model.nq,):
                raise SystemExit('qpos shape %r != model.nq %d at tick %s' %
                                 (qpos.shape, model.nq, row.get('tick')))
            data.qpos[:] = qpos
            mujoco.mj_forward(model, data)
            camera.lookat[:] = [float(qpos[0]), float(qpos[1]), float(qpos[2])]
            renderer.update_scene(data, camera=camera)
            image = Image.fromarray(renderer.render())
            draw = ImageDraw.Draw(image)
            draw.text((10, 8), '%s  tick %s/%s' %
                      (args.label, row.get('tick'), rows[-1].get('tick')),
                      fill=(235, 245, 255))
            if args.colors > 0:
                image = image.quantize(colors=args.colors, dither=Image.NONE)
            frames.append(image)
        duration = max(1, int(round(1000.0 / args.fps)))
        frames[0].save(args.out, save_all=True, append_images=frames[1:],
                       duration=duration, loop=0, optimize=True, disposal=2)
    finally:
        shutil.rmtree(Path(world).parent, ignore_errors=True)
    print(args.out, 'frames=%d' % len(frames))


if __name__ == '__main__':
    main()
