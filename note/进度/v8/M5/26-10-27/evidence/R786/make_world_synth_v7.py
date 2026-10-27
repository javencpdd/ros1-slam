"""m1_world_v2 几何合成图 v5：v4 基础上加 12 肋＋4 柱（R786 方案 A）。"""
import math

import numpy as np

BOXES = [
    (1.5, 1.2, 0.4, 0.8, 0.8, 0.8, 0.4),
    (3.0, -1.1, 0.45, 0.9, 0.7, 0.9, -0.3),
    (4.6, 0.9, 0.35, 0.7, 0.7, 0.7, 0.9),
    (2.2, -1.6, 0.5, 1.0, 0.6, 1.0, 0.2),
    (5.8, -1.0, 0.4, 0.8, 0.8, 0.8, -0.6),
    (0.8, 1.8, 0.5, 0.6, 0.6, 1.0, 0.1),
]
WALLS = [(2.5, 4.5, 9.0, 0.2, 1.0), (2.5, -4.5, 9.0, 0.2, 1.0)]

# R786 方案 A：v2 世界的非对称定位特征（与 m1_world_v2.world 逐字对齐）
RIBS = [(0.4, 4.15), (1.7, 4.15), (3.1, 4.15), (4.4, 4.15), (5.9, 4.15), (7.0, 4.15),
        (-0.3, -4.15), (1.1, -4.15), (2.4, -4.15), (3.8, -4.15), (5.1, -4.15), (6.5, -4.15)]
PYLONS = [(2.2, 3.2), (4.8, 3.4), (3.4, -3.2), (6.0, -3.4)]
STEP = 0.025
LAYERS = (0.0, 0.07, 0.14)
GROUND_STEP = 0.10


def rot(yaw, x, y):
    c, s = math.cos(yaw), math.sin(yaw)
    return c * x - s * y, s * x + c * y


pts = []
for (cx, cy, cz, sx, sy, sz, yaw) in BOXES:
    hx, hy = sx / 2, sy / 2
    z0, z1 = cz - sz / 2 + 0.02, cz + sz / 2 - 0.02
    for depth in LAYERS:
        ox, oy = hx - depth, hy - depth
        if ox <= 0 or oy <= 0:
            continue
        for x in np.arange(-ox, ox + 1e-9, STEP):
            for z in np.arange(z0, z1 + 1e-9, STEP):
                for yy in (oy, -oy):
                    px, py = rot(yaw, x, yy)
                    pts.append((cx + px, cy + py, z))
        for y in np.arange(-oy, oy + 1e-9, STEP):
            for z in np.arange(z0, z1 + 1e-9, STEP):
                for xx in (ox, -ox):
                    px, py = rot(yaw, xx, y)
                    pts.append((cx + px, cy + py, z))
for (cx, cy, sx, sy, sz) in WALLS:
    sign = -1 if cy > 0 else 1
    y_face = cy + sign * sy / 2
    for depth in LAYERS:
        y = y_face + sign * depth
        for x in np.arange(cx - sx / 2, cx + sx / 2 + 1e-9, STEP):
            for z in np.arange(0.02, sz + 1e-9, STEP):
                pts.append((x, y, z))



for (cx, cy) in RIBS:
    for depth in LAYERS:
        ox, oy = 0.125 - depth, 0.15 - depth
        if ox <= 0 or oy <= 0:
            continue
        z0, z1 = 0.02, 1.38
        for x in np.arange(-ox, ox + 1e-9, STEP):
            for z in np.arange(z0, z1 + 1e-9, STEP):
                for yy in (oy, -oy):
                    pts.append((cx + x, cy + yy, z))
        for y in np.arange(-oy, oy + 1e-9, STEP):
            for z in np.arange(z0, z1 + 1e-9, STEP):
                for xx in (ox, -ox):
                    pts.append((cx + xx, cy + y, z))
for (cx, cy) in PYLONS:
    for depth in LAYERS:
        ox, oy = 0.15 - depth, 0.15 - depth
        if ox <= 0 or oy <= 0:
            continue
        z0, z1 = 0.02, 1.48
        for x in np.arange(-ox, ox + 1e-9, STEP):
            for z in np.arange(z0, z1 + 1e-9, STEP):
                for yy in (oy, -oy):
                    pts.append((cx + x, cy + yy, z))
        for y in np.arange(-oy, oy + 1e-9, STEP):
            for z in np.arange(z0, z1 + 1e-9, STEP):
                for xx in (ox, -ox):
                    pts.append((cx + xx, cy + y, z))


JUTS = [(1.2, -3.2, 0.20, 2.4), (3.6, -3.2, 0.20, 2.4), (6.0, -3.2, 0.20, 2.4)]
for (cx, cy, sxw, syw) in JUTS:
    hx, hy = sxw / 2, syw / 2
    for depth in LAYERS:
        ox, oy = hx - depth, hy - depth
        if ox <= 0 or oy <= 0:
            continue
        z0, z1 = 0.02, 1.38
        for x in np.arange(-ox, ox + 1e-9, STEP):
            for z in np.arange(z0, z1 + 1e-9, STEP):
                for yy in (oy, -oy):
                    pts.append((cx + x, cy + yy, z))
        for y in np.arange(-oy, oy + 1e-9, STEP):
            for z in np.arange(z0, z1 + 1e-9, STEP):
                for xx in (ox, -ox):
                    pts.append((cx + xx, cy + y, z))


PILC = [(0.6, -1.0), (2.6, 1.0), (4.0, -1.0), (5.4, 1.0)]
for (cx, cy) in PILC:
    for depth in LAYERS:
        ox, oy = 0.18 - depth, 0.18 - depth
        if ox <= 0 or oy <= 0:
            continue
        z0, z1 = 0.02, 1.58
        for x in np.arange(-ox, ox + 1e-9, STEP):
            for z in np.arange(z0, z1 + 1e-9, STEP):
                for yy in (oy, -oy):
                    pts.append((cx + x, cy + yy, z))
        for y in np.arange(-oy, oy + 1e-9, STEP):
            for z in np.arange(z0, z1 + 1e-9, STEP):
                for xx in (ox, -ox):
                    pts.append((cx + xx, cy + y, z))

arr = np.array(pts, dtype=np.float32)
out = np.zeros((arr.shape[0], 4), dtype=np.float32)
out[:, :3] = arr
print('points', out.shape)
path = '/work/maps/m1_world_synth_v7.pcd'
with open(path, 'wb') as f:
    f.write(b'# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n')
    f.write(('WIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA binary\n' % (out.shape[0], out.shape[0])).encode())
    f.write(out.tobytes())
print('written', path)
