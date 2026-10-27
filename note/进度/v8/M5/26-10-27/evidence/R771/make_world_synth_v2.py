"""m1 世界几何合成图 v2：加密表面＋体积厚度层，适配 NDT 分布。"""
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
STEP = 0.03
LAYERS = (0.0, 0.08, 0.16)
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
for x in np.arange(-2.0, 7.0 + 1e-9, GROUND_STEP):
    for y in np.arange(-4.4, 4.4 + 1e-9, GROUND_STEP):
        pts.append((x, y, 0.0))

arr = np.array(pts, dtype=np.float32)
out = np.zeros((arr.shape[0], 4), dtype=np.float32)
out[:, :3] = arr
print('points', out.shape)
path = '/work/maps/m1_world_synth_v2.pcd'
with open(path, 'wb') as f:
    f.write(b'# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n')
    f.write(('WIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA binary\n' % (out.shape[0], out.shape[0])).encode())
    f.write(out.tobytes())
print('written', path)
