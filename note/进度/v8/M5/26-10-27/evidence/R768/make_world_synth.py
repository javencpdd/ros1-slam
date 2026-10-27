"""从 m1_world.world 几何生成合成先验图（世界帧，含墙/盒立面＋稀疏地面）。"""
import math

import numpy as np

BOXES = [
    # cx, cy, cz, sx, sy, sz, yaw
    (1.5, 1.2, 0.4, 0.8, 0.8, 0.8, 0.4),
    (3.0, -1.1, 0.45, 0.9, 0.7, 0.9, -0.3),
    (4.6, 0.9, 0.35, 0.7, 0.7, 0.7, 0.9),
    (2.2, -1.6, 0.5, 1.0, 0.6, 1.0, 0.2),
    (5.8, -1.0, 0.4, 0.8, 0.8, 0.8, -0.6),
    (0.8, 1.8, 0.5, 0.6, 0.6, 1.0, 0.1),
]
WALLS = [
    # cx, cy, sx, sy, sz
    (2.5, 4.5, 9.0, 0.2, 1.0),
    (2.5, -4.5, 9.0, 0.2, 1.0),
]
STEP = 0.05
GROUND_STEP = 0.20


def face(center, yaw, normal, half, z0, z1):
    """在盒子的一个立面上采样：normal 取 +x/-x/+y/-y。"""
    c, s = math.cos(yaw), math.sin(yaw)
    xs = np.arange(-half[0], half[0] + 1e-9, STEP)
    ys = np.arange(-half[1], half[1] + 1e-9, STEP)
    zs = np.arange(z0, z1 + 1e-9, STEP)
    pts = []
    if normal in ('+x', '-x'):
        off = half[0] if normal == '+x' else -half[0]
        for y in ys:
            for z in zs:
                lx, ly = off, y
                pts.append((center[0] + c * lx - s * ly, center[1] + s * lx + c * ly, z))
    else:
        off = half[1] if normal == '+y' else -half[1]
        for x in xs:
            for z in zs:
                lx, ly = x, off
                pts.append((center[0] + c * lx - s * ly, center[1] + s * lx + c * ly, z))
    return pts


pts = []
for (cx, cy, cz, sx, sy, sz, yaw) in BOXES:
    center = (cx, cy)
    half = (sx / 2, sy / 2)
    for normal in ('+x', '-x', '+y', '-y'):
        pts += face(center, yaw, normal, half, cz - sz / 2 + 0.01, cz + sz / 2 - 0.01)
    # 顶面
    c, s = math.cos(yaw), math.sin(yaw)
    for lx in np.arange(-half[0], half[0] + 1e-9, STEP * 2):
        for ly in np.arange(-half[1], half[1] + 1e-9, STEP * 2):
            pts.append((cx + c * lx - s * ly, cy + s * lx + c * ly, cz + sz / 2))

for (cx, cy, sx, sy, sz) in WALLS:
    # 朝向走廊的内立面：北墙取 y=cy-sy/2，南墙取 y=cy+sy/2
    y_inner = cy - sy / 2 if cy > 0 else cy + sy / 2
    for x in np.arange(cx - sx / 2, cx + sx / 2 + 1e-9, STEP):
        for z in np.arange(0.02, sz + 1e-9, STEP):
            pts.append((x, y_inner, z))

for x in np.arange(-2.0, 7.0 + 1e-9, GROUND_STEP):
    for y in np.arange(-4.4, 4.4 + 1e-9, GROUND_STEP):
        pts.append((x, y, 0.0))

arr = np.array(pts, dtype=np.float32)
print('points', arr.shape)
out = '/work/maps/m1_world_synth_v1.pcd'
with open(out, 'wb') as f:
    f.write(b'# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\n')
    f.write(b'FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n')
    f.write(('WIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA binary\n' % (arr.shape[0], arr.shape[0])).encode())
    f.write(arr.tobytes())
print('written', out)
