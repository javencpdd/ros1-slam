#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""矩阵W3-R2 goals 再生成（小明 2026-10-07；施工包 /tmp/rework_w3_r2_xiaozhang_manifest.md 方案 A）。
只做 goals 数据文件再生成——harness/traj yaml/launch/planner yaml 零改动（B 否决，planner 参数禁动）。
网格重建逐字复刻 w2_survey.py v3（f44277a4）勘察段：同 size/res/ZMIN/ZMAX/封边①/连通域/0.15m 选点间隙，
输入=已存档点云 /tmp/matrix_w3/perlin3d_cloud.npy（W3 v3 勘察原件，离线复核不启 mockamap）。
选点改环带制（方案 A）：4 等分角扇区沿用，扇区内取「欧氏距 start 4–8m 环带内最远候选」（原制=全图最远，
即远角本体；环带上限 8m 覆盖 7.9m 已知好包络、下限 4m 避开 start 邻域空转）。远角 4 对整体剔除标准集。
用法:
  python3 w3r2_goals_regen.py candidates      # 阶段1：重建网格断言对账+产出环带候选排序表
  python3 w3r2_goals_regen.py finalize        # 阶段2：读预验结果 json → 写正式 perlin3d_goals.json + 留证
产出（阶段1）: /tmp/matrix_w3/w3r2_ring_candidates.json
产出（阶段2）: /tmp/matrix_w3/perlin3d_goals.json（新版，schema 与旧版逐键一致）+ /tmp/matrix_w3/logs/w3r2_goals_regen_evidence.log
"""
import sys, json, math
import numpy as np

name, size, res = 'perlin3d', 40.0, 0.1
x_len, y_len = 40.0, 20.0
TX, TY = x_len / 2.0, y_len / 2.0
ZMIN, ZMAX = 0.05, 2.0
CLEAR_REPORT = 0.35
CLEAR_SEL = 0.15
RING_LO, RING_HI = 4.0, 8.0     # 方案 A：中程环带（m，欧氏距 start）
N_GOALS = 4
N_ALT = 20                      # 每扇区候选排序表深度（预验回炉用；主键=舒适间隙，次键=中程偏差）

CLOUD = '/tmp/matrix_w3/perlin3d_cloud.npy'
OLD = '/tmp/matrix_w3/perlin3d_goals.json'
CAND = '/tmp/matrix_w3/w3r2_ring_candidates.json'
PRE = '/tmp/matrix_w3/w3r2_preverify_result.json'
EVLOG = '/tmp/matrix_w3/logs/w3r2_goals_regen_evidence.log'


def rebuild():
    pts = np.load(CLOUD)
    n = int(round(size / res)); origin = -size / 2.0
    zok = (pts[:, 2] >= ZMIN) & (pts[:, 2] <= ZMAX)
    a = pts[zok]
    occ = np.zeros((n, n), dtype=bool)
    ix = np.floor((a[:, 0] - origin) / res).astype(np.int64)
    iy = np.floor((a[:, 1] - origin) / res).astype(np.int64)
    ok = (ix >= 0) & (ix < n) & (iy >= 0) & (iy < n)
    occ[iy[ok], ix[ok]] = True
    cxx = origin + (np.arange(n) + 0.5) * res
    out_rect = (np.abs(cxx) > TX)[None, :] | (np.abs(cxx) > TY)[:, None]
    occ |= out_rect
    occ[0, :] = occ[-1, :] = occ[:, 0] = occ[:, -1] = True
    from scipy.ndimage import binary_dilation, label
    def dilate(mask, r):
        yy, xx = np.mgrid[-r:r + 1, -r:r + 1]
        return binary_dilation(mask, structure=(xx * xx + yy * yy) <= r * r)
    occ_rep = dilate(occ, int(math.ceil(CLEAR_REPORT / res)))
    occ_sel = dilate(occ, int(math.ceil(CLEAR_SEL / res)))
    free_conn = ~occ
    lab, nl = label(free_conn)
    assert nl > 0, '无自由区'
    sizes = np.bincount(lab.ravel()); sizes[0] = 0
    comp = sizes.argmax().item()
    comp_mask = (lab == comp)
    cand_mask = comp_mask & (~occ_sel)
    cand_cells = np.argwhere(cand_mask)
    return pts, occ, occ_rep, comp_mask, cand_cells, cxx, origin


def coord_of(iy, ix):
    return -size / 2.0 + (ix + 0.5) * res, -size / 2.0 + (iy + 0.5) * res


def phase_candidates():
    pts, occ, occ_rep, comp_mask, cand_cells, cxx, origin = rebuild()
    old = json.load(open(OLD))
    # ── 网格重建对账断言：与旧 json（W3 v3 勘察产物）逐键一致=重建忠实性铁证 ──
    assert int(occ.sum()) == old['occ_cells'], 'occ 对账失败 %d vs %d' % (int(occ.sum()), old['occ_cells'])
    assert int((~occ_rep).sum()) == old['free_rep_cells'], 'free_rep 对账失败'
    assert int(comp_mask.sum()) == old['comp_cells'], 'comp 对账失败'
    assert len(cand_cells) == old['cand_cells'], 'cand 对账失败'
    iyv, ixv = np.nonzero(~occ)
    assert np.all(np.abs(cxx[ixv]) <= TX + 1e-9) and np.all(np.abs(cxx[iyv]) <= TY + 1e-9), '封边①失效'
    c0 = np.argmin((cand_cells[:, 1] * res + origin) ** 2 + (cand_cells[:, 0] * res + origin) ** 2)
    sy, sx = cand_cells[c0]
    sxy = [round(float(coord_of(sy, sx)[0]), 3), round(float(coord_of(sy, sx)[1]), 3)]
    assert sxy == old['start'], 'start 对账失败 %s vs %s' % (sxy, old['start'])
    ext = (float(np.percentile(pts[:, 0], 0.1)), float(np.percentile(pts[:, 0], 99.9)),
           float(np.percentile(pts[:, 1], 0.1)), float(np.percentile(pts[:, 1], 99.9)))
    quad = [int(((pts[:, 0] >= 0) & (pts[:, 1] >= 0)).sum()), int(((pts[:, 0] < 0) & (pts[:, 1] >= 0)).sum()),
            int(((pts[:, 0] < 0) & (pts[:, 1] < 0)).sum()), int(((pts[:, 0] >= 0) & (pts[:, 1] < 0)).sum())]
    assert ext == tuple(old['cloud_extent_xy']) and quad == old['quad_points'], '云足迹对账失败'
    # ── 环带候选（方案 A）──
    gx = cand_cells[:, 1] * res + origin
    gy = cand_cells[:, 0] * res + origin
    d = np.hypot(gx - sxy[0], gy - sxy[1])
    ang = np.arctan2(gy - sxy[1], gx - sxy[0])
    ring = (d >= RING_LO) & (d <= RING_HI)
    # 舒适间隙（到最近占据格的欧氏距离，m）：预验 run1-3 实证 0.15m 间隙候选在 stvl 动态加厚下
    # 处于可行性边界（GP fwd 双峰 0.06s↔55@3.0s，8 候选×8 新栈全 55），而已知好对（宽走廊）
    # 跨栈 6/6 ≤0.15s → 排序主键=间隙降序（开阔口袋优先），次键=|d−6.0| 升序（中程优先）
    from scipy.ndimage import distance_transform_edt
    clear_m = distance_transform_edt(~occ) * res
    cdx = clear_m[cand_cells[:, 0], cand_cells[:, 1]]
    sectors = []
    for k in range(N_GOALS):
        lo, hi = -math.pi + k * 2 * math.pi / N_GOALS, -math.pi + (k + 1) * 2 * math.pi / N_GOALS
        m = ring & (ang >= lo) & (ang < hi)
        sel = cand_cells[m]
        order = np.lexsort((np.abs(d[m] - 6.0), -cdx[m]))   # 主键 clearance 降序，次键 |d−6| 升序
        ranked = []
        for oi in order[:N_ALT]:
            gyi, gxi = sel[oi]
            c = coord_of(gyi, gxi)
            ranked.append([round(float(c[0]), 3), round(float(c[1]), 3), round(float(d[m][oi]), 2),
                           round(float(cdx[m][oi]), 2)])
        sectors.append(dict(sector=k, angle_range=[round(lo, 3), round(hi, 3)],
                            ring_cells=int(m.sum()), ranked=ranked))
        print('SECTOR%d 环带格=%d 排序前%d: %s' % (k, int(m.sum()), N_ALT,
              ' '.join('(%.2f,%.2f)d=%.2f clr=%.2f' % (r[0], r[1], r[2], r[3]) for r in ranked)))
    assert all(len(s['ranked']) > 0 for s in sectors), '有扇区环带为空'
    out = dict(map=name, ring_m=[RING_LO, RING_HI], start=sxy,
               rebuild_check=dict(occ_cells=int(occ.sum()), free_rep_cells=int((~occ_rep).sum()),
                                  comp_cells=int(comp_mask.sum()), cand_cells=len(cand_cells)),
               sectors=sectors)
    json.dump(out, open(CAND, 'w'), ensure_ascii=False, indent=1)
    print('CANDIDATES -> %s' % CAND)


def phase_finalize():
    old = json.load(open(OLD))
    pre = json.load(open(PRE))
    cand = json.load(open(CAND))
    chosen = pre['chosen']            # [[x,y],...×4] 按 sector 序
    assert len(chosen) == N_GOALS, 'chosen 不足 %d' % len(chosen)
    assert all(p['ok'] for p in pre['attempts'] if p['selected']), '被选候选预验未全过'
    pts, occ, occ_rep, comp_mask, cand_cells, cxx, origin = rebuild()
    out = dict(map=name, size=old['size'], res=old['res'],
               occ_cells=old['occ_cells'], free_rep_cells=old['free_rep_cells'],
               comp_cells=old['comp_cells'], cand_cells=old['cand_cells'],
               terrain=old['terrain'], cloud_extent_xy=old['cloud_extent_xy'],
               quad_points=old['quad_points'], start=old['start'], goals=[[g[0], g[1]] for g in chosen])
    json.dump(out, open(OLD, 'w'), ensure_ascii=False, indent=1)
    lines = []
    lines.append('W3-R2 goals 再生成留证（小明 2026-10-07，方案 A）')
    lines.append('旧 goals（远角 4 对，整体剔除标准集）：' + ' '.join('(%.2f,%.2f)' % (g[0], g[1]) for g in old['goals']))
    lines.append('旧欧氏距: ' + ' '.join('%.1fm' % math.hypot(g[0] - old['start'][0], g[1] - old['start'][1]) for g in old['goals']))
    lines.append('新 goals（中程环带 %.0f–%.0fm×4，逐对预验 GP→navfn 正反向 outcome=0 且 ≤1s）：' % (RING_LO, RING_HI)
                 + ' '.join('(%.2f,%.2f)' % (g[0], g[1]) for g in chosen))
    lines.append('新欧氏距: ' + ' '.join('%.1fm' % math.hypot(g[0] - old['start'][0], g[1] - old['start'][1]) for g in chosen))
    for p in pre['attempts']:
        lines.append('PREV sector=%d cand=(%.2f,%.2f) %s->%s planner=%s outcome=%d dt=%.3fs ok=%s selected=%s'
                     % (p['sector'], p['cand'][0], p['cand'][1], tuple(p['a']), tuple(p['b']), p['planner'],
                        p['outcome'], p['dt'], p['ok'], p['selected']))
    lines.append('网格重建对账: occ=%d free_rep=%d comp=%d cand=%d start=%s（与旧 json 逐键全等）'
                 % (out['occ_cells'], out['free_rep_cells'], out['comp_cells'], out['cand_cells'], out['start']))
    txt = '\n'.join(lines) + '\n'
    open(EVLOG, 'w').write(txt)
    print(txt)
    print('FINAL goals json -> %s' % OLD)


if __name__ == '__main__':
    dict(candidates=phase_candidates, finalize=phase_finalize)[sys.argv[1]]()
