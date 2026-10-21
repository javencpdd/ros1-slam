#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""矩阵W2 每图场地勘察（小明 2026-10-07）：mockamap 独立起 → 抓 /map_generator/global_cloud →
按 sim_map_bridge 同款约定（origin=-size/2, res 0.1, z∈[0.05,2.0]）物化 2D 栅格 →
膨胀 0.4m 求保守自由区（机器人半径+裕度）→ 连通域 BFS → 选 start+5 goals（互达可证）。
产出：<map>_goals.json（start/goals/占据统计）+ traj_<map>.yaml（静止轨迹源，hold_start 超长）。
用法: python3 w2_survey.py <mapname> <size> <res>   # size=grid_size（≥地图最长边）"""
import sys, json, math
import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

name, size, res = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
topic = sys.argv[4] if len(sys.argv) > 4 else '/mock_map'
rospy.init_node('w2_survey_%s' % name)
ZMIN, ZMAX = 0.05, 2.0
CLEAR = 0.35            # 保守自由判据：距占据 ≥0.35m（机器人半径+裕度）
msg = rospy.wait_for_message(topic, PointCloud2, timeout=120)
pts = np.array(list(pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)), dtype=np.float64)
n = int(round(size / res)); origin = -size / 2.0
zok = (pts[:, 2] >= ZMIN) & (pts[:, 2] <= ZMAX)
a = pts[zok]
occ = np.zeros((n, n), dtype=bool)
ix = np.floor((a[:, 0] - origin) / res).astype(np.int64)
iy = np.floor((a[:, 1] - origin) / res).astype(np.int64)
ok = (ix >= 0) & (ix < n) & (iy >= 0) & (iy < n)
occ[iy[ok], ix[ok]] = True
# 边界外全占（防选到图外）：圈外 1 格封边
occ[0, :] = occ[-1, :] = occ[:, 0] = occ[:, -1] = True
# 膨胀 r=CLEAR/res
r = int(math.ceil(CLEAR / res))
yy, xx = np.mgrid[-r:r + 1, -r:r + 1]
disk = (xx * xx + yy * yy) <= r * r
from scipy.ndimage import binary_dilation
occ_d = binary_dilation(occ, structure=disk)
free = ~occ_d
# 连通域（4 邻接）
from scipy.ndimage import label
lab, nl = label(free)
if nl == 0:
    print('SURVEY %s FAIL: 无自由区' % name); sys.exit(1)
sizes = np.bincount(lab.ravel()); sizes[0] = 0
comp = sizes.argmax().item()
comp_mask = (lab == comp)
cells = np.argwhere(comp_mask)   # [iy, ix]
print('SURVEY %s: 点=%d 占据格=%d 保守自由格=%d 最大连通域=%d 格'
      % (name, pts.shape[0], int(occ.sum()), int(free.sum()), int(comp_mask.sum())))
def cell_of(x, y):
    return int((y - origin) / res), int((x - origin) / res)
def coord_of(iy, ix):
    return origin + (ix + 0.5) * res, origin + (iy + 0.5) * res
# start=连通域内最靠近 (0,0) 的格；goals=按角度 5 等分采样（最远格），保证散布
c0 = np.argmin((cells[:, 1] * res + origin) ** 2 + (cells[:, 0] * res + origin) ** 2)
sy, sx = cells[c0]
ang = np.arctan2(cells[:, 0] * res + origin - (sy * res + origin),
                 cells[:, 1] * res + origin - (sx * res + origin))
goals = []
for k in range(5):
    lo, hi = -math.pi + k * 2 * math.pi / 5, -math.pi + (k + 1) * 2 * math.pi / 5
    sel = cells[(ang >= lo) & (ang < hi)]
    if len(sel) == 0: continue
    d = (sel[:, 1] - sx) ** 2 + (sel[:, 0] - sy) ** 2
    gy, gx = sel[d.argmax()]
    goals.append([round(float(coord_of(gy, gx)[0]), 3), round(float(coord_of(gy, gx)[1]), 3)])
sxy = [round(float(coord_of(sy, sx)[0]), 3), round(float(coord_of(sy, sx)[1]), 3)]
out = dict(map=name, size=size, res=res, occ_cells=int(occ.sum()), free_cells=int(free.sum()),
           comp_cells=int(comp_mask.sum()), start=sxy, goals=goals)
json.dump(out, open('/tmp/matrix_w2/%s_goals.json' % name, 'w'), ensure_ascii=False, indent=1)
# 静止轨迹源：hold_start_s 超长（测试窗内恒驻 start），单航点
traj = dict(frame='world', hold_start_s=100000.0, hold_end_s=5.0, speed=0.5,
            turn_omega=1.0, v_ramp_s=0.5, align_gate=0.15,
            waypoints=[sxy])
import yaml
yaml.safe_dump(traj, open('/tmp/matrix_w2/traj_%s.yaml' % name, 'w'), allow_unicode=True)
print('SURVEY %s start=%s goals=%s' % (name, sxy, goals))
