#!/usr/bin/env python3
"""scan_cloud_ground_segment 的纯函数单测（无 ROS；合成点云）。"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'scripts'))
from scan_cloud_ground_segment import segment_ground  # noqa: E402

FAIL = 0


def check(cond, what):
    global FAIL
    print(('%s  %s' % ('PASS' if cond else 'FAIL', what)))
    if not cond:
        FAIL += 1


def make_scene(tilt=0.2, offset=-0.3, seed=0):
    rng = np.random.default_rng(seed)
    xs = np.arange(-2.0, 2.0 + 1e-9, 0.05)
    ys = np.arange(-2.0, 2.0 + 1e-9, 0.05)
    gx, gy = np.meshgrid(xs, ys)
    ground = np.column_stack([gx.ravel(), gy.ravel(), tilt * gx.ravel() + offset])

    # 0.8m 立方体（中心 1,0），6 面采样；地面处为基准
    c = np.array([1.0, 0.0, tilt * 1.0 + offset + 0.4])
    faces = []
    for axis in range(3):
        for sign in (-1.0, 1.0):
            u = np.linspace(-0.4, 0.4, 9)
            v = np.linspace(-0.4, 0.4, 9)
            uu, vv = np.meshgrid(u, v)
            pts = np.zeros((uu.size, 3))
            other = [i for i in range(3) if i != axis]
            pts[:, axis] = c[axis] + sign * 0.4
            pts[:, other[0]] = uu.ravel() + c[other[0]]
            pts[:, other[1]] = vv.ravel() + c[other[1]]
            faces.append(pts)
    box = np.vstack(faces)

    # 墙 x=2, y∈[-0.8,0.8], z 从地面到 2m
    wx, wy = np.meshgrid(np.linspace(-0.8, 0.8, 17), np.linspace(0.0, 2.0, 21))
    wall = np.column_stack([np.full(wx.size, 2.0), wx.ravel(), wy.ravel()])

    self_pts = rng.normal(0.0, 0.05, size=(20, 3))
    return np.vstack([ground, box, wall, self_pts]), ground, box, wall, self_pts


def main():
    for tilt, name in ((0.2, 'tilted 11.3deg'), (0.0, 'flat')):
        points, ground, box, wall, self_pts = make_scene(tilt=tilt)
        kept, info = segment_ground(points, distance=0.08, iterations=80, self_radius=0.35)
        # 地面（含平面内点）保留率应很低
        kept_set = set(map(tuple, np.round(kept, 6)))
        ground_kept = sum(1 for p in np.round(ground, 6) if tuple(p) in kept_set)
        box_above = np.round(box[box[:, 2] > (tilt * box[:, 0] - 0.3) + 0.08], 6)
        wall_above = np.round(wall[wall[:, 2] > (tilt * wall[:, 0] - 0.3) + 0.08], 6)
        box_kept = sum(1 for p in box_above if tuple(p) in kept_set)
        wall_kept = sum(1 for p in wall_above if tuple(p) in kept_set)
        check(info['input'] == len(points), '%s: input count' % name)
        check(ground_kept / max(1, len(ground)) < 0.10,
              '%s: ground removed (kept=%d/%d)' % (name, ground_kept, len(ground)))
        # 贴地但上方有障碍的列（箱底/墙脚）应整列保留
        box_near = np.round(box[np.abs(box[:, 2] - (tilt * box[:, 0] - 0.3)) < 0.08], 6)
        box_near_kept = sum(1 for p in box_near if tuple(p) in kept_set)
        check(len(box_near) > 0 and box_near_kept / len(box_near) > 0.8,
              '%s: near-ground obstacle base kept (%.0f%%)'
              % (name, 100.0 * box_near_kept / max(1, len(box_near))))
        check(info.get('near_kept', 0) > 0, '%s: near_kept counted' % name)
        check(box_kept / max(1, len(box_above)) > 0.8,
              '%s: box above ground kept (%.0f%%)' % (name, 100.0 * box_kept / max(1, len(box_above))))
        check(wall_kept / max(1, len(wall_above)) > 0.9,
              '%s: wall above ground kept (%.0f%%)' % (name, 100.0 * wall_kept / max(1, len(wall_above))))
        # 机体圆柱也会带走原点附近的地面格，故 self_removed 会大于 20；
        # 这里断言 20 个自车点一个都不在 kept 里。
        self_kept = sum(1 for p in np.round(self_pts, 6) if tuple(p) in kept_set)
        check(self_kept == 0, '%s: self points removed' % name)
        check(info['self_removed'] >= 20, '%s: self removal counted' % name)
        check(info['plane'] is not None, '%s: plane reported' % name)
        if info['plane'] is not None:
            n = np.array(info['plane'])
            # 平面法向应接近 +z（坡面 11° 时 z 分量 ~0.98）
            check(n[2] > 0.9, '%s: plane normal up (nz=%.3f)' % (name, n[2]))

    empty = np.zeros((0, 3))
    kept, info = segment_ground(empty)
    check(len(kept) == 0 and info['kept'] == 0, 'empty cloud')
    few = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
    kept, info = segment_ground(few)
    check(len(kept) == 2, 'too-few points pass through')

    print('FAILED (%d)' % FAIL if FAIL else 'ALL PASS')
    return 1 if FAIL else 0


if __name__ == '__main__':
    raise SystemExit(main())
