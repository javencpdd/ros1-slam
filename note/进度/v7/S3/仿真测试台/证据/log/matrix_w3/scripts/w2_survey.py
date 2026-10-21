#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""矩阵W3 勘察修法版 v3（小明 2026-10-07）：W2 N1 修法——free 判定与 mockamap 实际范围/规划语义一致。
W2 缺陷（小美 Wave2 验收 N1）：`free=~occ_d` 把云不覆盖区全判自由（封边只封栅格边非地形边），
perlin3d 地形 y∈[−10,+10]（y_length=20）而勘察格 40×40 → y=−19.45 的 goals 落「云外假自由带」。
修法三层（v1/v2 现场实证演进，log 留底 survey_perlin3d_w3_v1_degenerate.log/_v2_comp605.log）：
  ①模板地形封边（小美裁定「按模板 x/y_length 封地形边」）：栅格中心落在
    [−x_len/2,+x_len/2]×[−y_len/2,+y_len/2] 之外一律 non-free——mockamap 实际世界=模板盒体，
    盒内空柱=真实自由空间（perlin3d 体积噪声语义），盒外=不存在地形（N1 假自由带本体）；
  ②连通域/选点图与 planner 语义对齐：global_costmap 插件=static_layer+stvl_obstacle_layer，
    **无 inflation 层**（global_costmap_params.yaml:6-8 实证）→ planner 可通行集=非占据格（cost<253）；
    选点连通域图=~occ（模板封边后），v2 实证 0.35m 保守膨胀图在本图最大连通域仅 605 格与
    planner 语义脱节（goals 挤 start 4m 内=测试空转）；
  ③选点舒适间隙：start/goal 自身格距占据 ≥0.15m（防 goal 嵌缝）；0.35m 保守自由格数仍统计上报
    （与 W2 口径连续），不再作选点连通图。
其余约定与 W2 一致：origin=−size/2、res=grid_res、occ 切片 z∈[0.05,2.0]、按角度等分扇区取最远格
（空扇回退补齐）、start=选点图连通域内最靠 (0,0) 格。
用法: python3 w2_survey.py <mapname> <grid_size> <grid_res> <n_goals> <x_length> <y_length> [topic]
产出: /tmp/matrix_w3/<map>_goals.json + /tmp/matrix_w3/traj_<map>.yaml（waypoints 双重复航点，sim_adapter ≥2 校验）
      + /tmp/matrix_w3/<map>_cloud.npy（点云存档，离线复核用，不入 note/ 归档）
"""
import sys, json, math
import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

name, size, res = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
n_goals = int(sys.argv[4])
x_len, y_len = float(sys.argv[5]), float(sys.argv[6])
topic = sys.argv[7] if len(sys.argv) > 7 else '/mock_map'
TX, TY = x_len / 2.0, y_len / 2.0   # 模板地形半宽（封边①）
rospy.init_node('w3_survey_%s' % name)
ZMIN, ZMAX = 0.05, 2.0
CLEAR_REPORT = 0.35      # 保守自由统计口径（W2 连续性，仅报告）
CLEAR_SEL = 0.15         # 选点舒适间隙（start/goal 自身格）
msg = rospy.wait_for_message(topic, PointCloud2, timeout=120)
pts = np.array(list(pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)), dtype=np.float64)
np.save('/tmp/matrix_w3/%s_cloud.npy' % name, pts)
n = int(round(size / res)); origin = -size / 2.0
zok = (pts[:, 2] >= ZMIN) & (pts[:, 2] <= ZMAX)
a = pts[zok]
occ = np.zeros((n, n), dtype=bool)
ix = np.floor((a[:, 0] - origin) / res).astype(np.int64)
iy = np.floor((a[:, 1] - origin) / res).astype(np.int64)
ok = (ix >= 0) & (ix < n) & (iy >= 0) & (iy < n)
occ[iy[ok], ix[ok]] = True
# ── 修法封边①：模板地形边外全占（W2 此处只封栅格边 1 格——N1 缺陷本体）──
cxx = origin + (np.arange(n) + 0.5) * res
out_rect = (np.abs(cxx) > TX)[None, :] | (np.abs(cxx) > TY)[:, None]
occ |= out_rect
occ[0, :] = occ[-1, :] = occ[:, 0] = occ[:, -1] = True   # 栅格边封边（保留）
from scipy.ndimage import binary_dilation, label
def dilate(mask, r):
    yy, xx = np.mgrid[-r:r + 1, -r:r + 1]
    return binary_dilation(mask, structure=(xx * xx + yy * yy) <= r * r)
occ_rep = dilate(occ, int(math.ceil(CLEAR_REPORT / res)))   # 0.35m 保守口径（仅统计）
occ_sel = dilate(occ, int(math.ceil(CLEAR_SEL / res)))      # 0.15m 选点舒适口径
free_conn = ~occ                                            # planner 语义连通图（无 inflation 层）
free_rep = ~occ_rep                                         # 0.35m 保守自由（统计）
# 连通域（4 邻接，planner 语义图）
lab, nl = label(free_conn)
if nl == 0:
    print('SURVEY %s FAIL: 无自由区' % name); sys.exit(1)
sizes = np.bincount(lab.ravel()); sizes[0] = 0
comp = sizes.argmax().item()
comp_mask = (lab == comp)
cells = np.argwhere(comp_mask)   # [iy, ix]
cand_mask = comp_mask & (~occ_sel)
cand_cells = np.argwhere(cand_mask)
print('SURVEY %s: 点=%d 占据格=%d 保守自由格(0.35m)=%d planner连通域=%d 格 选点候选(0.15m)=%d 格 '
      '地形=[%.1f,%.1f]x[%.1f,%.1f]'
      % (name, pts.shape[0], int(occ.sum()), int(free_rep.sum()), int(comp_mask.sum()),
         len(cand_cells), -TX, TX, -TY, TY))
# ── 云足迹对账（报告+断言，不封锁）──
ext = (np.percentile(pts[:, 0], 0.1), np.percentile(pts[:, 0], 99.9),
       np.percentile(pts[:, 1], 0.1), np.percentile(pts[:, 1], 99.9))
quad = [int(((pts[:, 0] >= 0) & (pts[:, 1] >= 0)).sum()), int(((pts[:, 0] < 0) & (pts[:, 1] >= 0)).sum()),
        int(((pts[:, 0] < 0) & (pts[:, 1] < 0)).sum()), int(((pts[:, 0] >= 0) & (pts[:, 1] < 0)).sum())]
assert ext[0] >= -TX - 1.0 and ext[1] <= TX + 1.0 and ext[2] >= -TY - 1.0 and ext[3] <= TY + 1.0, \
    '云足迹越出模板盒（P0.1/P99.9=%s）' % (ext,)
assert all(q > 0 for q in quad), '模板盒四象限存在零云点象限=%s' % (quad,)
# 封边自检：选点图自由格必须全部落在模板地形内（N1 回归断言）
iyv, ixv = np.nonzero(free_conn)
assert np.all(np.abs(cxx[ixv]) <= TX + 1e-9) and np.all(np.abs(cxx[iyv]) <= TY + 1e-9), \
    '封边①失效：自由格越出模板地形'
if len(cand_cells) == 0:
    print('SURVEY %s FAIL: 无 0.15m 舒适候选格' % name); sys.exit(1)
def coord_of(iy, ix):
    return origin + (ix + 0.5) * res, origin + (iy + 0.5) * res
# start=候选格内最靠近 (0,0) 的格；goals=按角度 n_goals 等分扇区取最远候选格（空扇回退候选全局最远）
c0 = np.argmin((cand_cells[:, 1] * res + origin) ** 2 + (cand_cells[:, 0] * res + origin) ** 2)
sy, sx = cand_cells[c0]
ang = np.arctan2(cand_cells[:, 0] * res + origin - (sy * res + origin),
                 cand_cells[:, 1] * res + origin - (sx * res + origin))
goals = []
chosen = {(sy, sx)}
def pick(pool, exclude):
    d = (pool[:, 1] - sx) ** 2 + (pool[:, 0] - sy) ** 2
    for oi in np.argsort(d)[::-1]:
        gy, gx = pool[oi]
        if (gy, gx) not in exclude:
            exclude.add((gy, gx))
            return [round(float(coord_of(gy, gx)[0]), 3), round(float(coord_of(gy, gx)[1]), 3)]
    return None
for k in range(n_goals):
    lo, hi = -math.pi + k * 2 * math.pi / n_goals, -math.pi + (k + 1) * 2 * math.pi / n_goals
    sel = cand_cells[(ang >= lo) & (ang < hi)]
    if len(sel):
        g = pick(sel, chosen)
        if g: goals.append(g)
while len(goals) < n_goals:
    g = pick(cand_cells, chosen)
    if g is None: break
    goals.append(g)
sxy = [round(float(coord_of(sy, sx)[0]), 3), round(float(coord_of(sy, sx)[1]), 3)]
assert len(goals) == n_goals, 'goals 不足（%d/%d）' % (len(goals), n_goals)
out = dict(map=name, size=size, res=res, occ_cells=int(occ.sum()), free_rep_cells=int(free_rep.sum()),
           comp_cells=int(comp_mask.sum()), cand_cells=len(cand_cells), terrain=[-TX, TX, -TY, TY],
           cloud_extent_xy=list(ext), quad_points=quad, start=sxy, goals=goals)
json.dump(out, open('/tmp/matrix_w3/%s_goals.json' % name, 'w'), ensure_ascii=False, indent=1)
# 静止轨迹源：hold_start_s 超长（测试窗内恒驻 start）；waypoints 双重复航点（sim_adapter 校验 ≥2）
traj = dict(frame='world', hold_start_s=100000.0, hold_end_s=5.0, speed=0.5,
            turn_omega=1.0, v_ramp_s=0.5, align_gate=0.15,
            waypoints=[sxy, sxy])
import yaml
yaml.safe_dump(traj, open('/tmp/matrix_w3/traj_%s.yaml' % name, 'w'), allow_unicode=True)
print('SURVEY %s start=%s goals=%s' % (name, sxy, goals))
