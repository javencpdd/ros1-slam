"""m1 真值占据图生成（与 R665 障碍几何一致）：200x200@0.05，origin (-2.5,-5.0)。"""
import struct

OBSTACLES = [(1.5, 1.2, 0.8, 0.8, 0.4), (3.0, -1.1, 0.9, 0.7, -0.3),
             (4.6, 0.9, 0.8, 0.8, 0.9), (2.2, -1.6, 0.8, 0.8, 0.2),
             (5.8, -1.0, 0.8, 0.8, -0.6), (0.8, 1.8, 0.8, 0.8, 0.1)]
WALLS = [(2.5, 4.5, 9.0, 0.2, 0.0), (2.5, -4.5, 9.0, 0.2, 0.0)]

RES = 0.05
W = H = 200
OX, OY = -2.5, -5.0


def inside(px, py, cx, cy, sx, sy, yaw):
    import math
    c, s = math.cos(-yaw), math.sin(-yaw)
    dx, dy = px - cx, py - cy
    lx, ly = dx * c - dy * s, dx * s + dy * c
    return abs(lx) <= sx / 2 and abs(ly) <= sy / 2


pixels = bytearray()
for row in range(H):
    y = OY + (H - row - 0.5) * RES
    for col in range(W):
        x = OX + (col + 0.5) * RES
        hit = any(inside(x, y, *box) for box in OBSTACLES + WALLS)
        pixels.append(0 if hit else 254)

with open('/work/pilot_map.pgm', 'wb') as f:
    f.write(b'P5\n%d %d\n255\n' % (W, H))
    f.write(bytes(pixels))

with open('/work/pilot_map.yaml', 'w') as f:
    f.write('image: /work/pilot_map.pgm\nresolution: %.2f\norigin: [%.2f, %.2f, 0.0]\n'
            'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n' % (RES, OX, OY))
print('map written', W, H)
