#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""_zwin_grid.py —— 批63：z 窗 (h,θ) 候选清单（★离线、★只调用在册件、★不另写第二套几何）。

★用法纪律＝地面对雷达系的斜率与 k0／truncated 判据一律取 t3 的在册函数（ground_z_radar／k0_of／is_truncated），
  本件只做"组合与列表"，不自设任何判据形状。★不写盘、不改任何在册件。
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REG = os.path.join('note', '进度', 'SCAN集成', '设计', 'scripts')
ROOT = HERE
while ROOT and ROOT != os.sep:
    if os.path.isdir(os.path.join(ROOT, REG)):
        break
    ROOT = os.path.dirname(ROOT)
else:
    ROOT = ''
if not ROOT:
    sys.exit('★前置缺失：从 %s 往上找不到含 %s 的仓根（★不硬写宿主绝对路径＝坑66 同族，两代须同一件可跑）' % (HERE, REG))
sys.path.insert(0, os.path.join(ROOT, REG))

import math

import t3_decidable_input_offline as t3   # ★在册件，只读调用

import t1_multilevel_raster_offline as t1  # ★在册件，只取甲档常量

JIA_MIN, JIA_MAX, SLAB = t1.Z_MIN, t1.Z_MAX, t1.Z_SLAB
Z_LOW = -7.21 / 180.0 * math.pi            # 发射仰角下界（在册 CSV 现读，见 r406 留底）
TAN_LOW = math.tan(-Z_LOW)                 # ＝tan(7.21°)


def row(h, th):
    zg = t3.ground_z_radar(0.0, h, th)                 # 正下方地面（雷达系 z）
    k0 = t3.k0_of(zg, JIA_MIN)
    trunc = t3.is_truncated(zg, JIA_MIN)
    s = math.sin(math.radians(th))
    # 地面降到甲档下沿之前的前向距离（θ=0 ⇒ 地面恒定 ⇒ 永不出窗）
    x_exit = float('inf') if s <= 0 else (abs(JIA_MIN) - h * math.cos(math.radians(th))) / s
    # 地面在 [0, X] 前向范围内跨几块板（★这才是 15° 的真代价：地面不再"一块板"）
    spread = dict((X, int(round((abs(t3.ground_z_radar(X, h, th) - zg)) / SLAB))) for X in (0.5, 1.0, 2.0, 4.0, 6.0))
    # 要地面在 X m 内都不出窗，需要把下沿压到哪儿／于是共几板
    need = {}
    for X in (1.0, 2.0, 4.0, 6.0):
        zmin_x = min(JIA_MIN, t3.ground_z_radar(X, h, th) - SLAB)   # ★留一整板垫在下面（在册贴地法形状，t3 §B.1.4）
        need[X] = (round(zmin_x, 4), int(round((JIA_MAX - zmin_x) / SLAB)))
    return dict(h=h, th=th, zg=round(zg, 4), k0=k0, trunc=trunc, x_exit=x_exit, spread=spread, need=need,
                band_above=round(JIA_MAX - zg, 3))


def blind_world(h, th):
    """世界系近场无地面回波半径（批61 那张表的式子，供与雷达系读法对照）。θ>0 下倾 ⇒ 仰角下界更向下 ⇒ 盲区缩小。"""
    e = -7.21 - th
    return None if e >= -0.5 else h / math.tan(math.radians(-e))


print('★甲档在册常量（取 t1，非本件自设）＝Z_MIN=%.2f Z_MAX=%.2f Z_SLAB=%.2f ⇒ 板数=%d'
      % (JIA_MIN, JIA_MAX, SLAB, int(round((JIA_MAX - JIA_MIN) / SLAB))))
print('★用法校验＝ground_z_radar(0/0.5/1.0, 0.45, 15) ＝ %s ‖ 在册 ANCHOR_IN_REG ＝ %s ‖ 全等=%s'
      % ([round(t3.ground_z_radar(x, 0.45, 15.0), 4) for x in (0.0, 0.5, 1.0)],
         list(t3.ANCHOR_IN_REG),
         [round(t3.ground_z_radar(x, 0.45, 15.0), 4) for x in (0.0, 0.5, 1.0)] == list(t3.ANCHOR_IN_REG)))
print('★DROP_PER_M 在册＝%s ‖ 我的 sin15＝%s' % (t3.DROP_PER_M, round(math.sin(math.radians(15.0)), 4)))
print()
hdr = 'h     θ     z_ground  k0  trunc  地面出窗前向距离  地面跨板(0.5/1/2/4/6m)  甲档内可判高度带  世界系盲区m'
print(hdr)
print('-' * len(hdr))
CANDS = [(0.49, 0.0), (0.45, 0.0), (0.45, 5.0), (0.45, 10.0), (0.45, 15.0), (0.53, 0.0), (0.49, 15.0)]
for h, th in CANDS:
    r = row(h, th)
    sp = '/'.join(str(r['spread'][X]) for X in (0.5, 1.0, 2.0, 4.0, 6.0))
    xe = '∞（θ=0 地面恒定）' if r['x_exit'] == float('inf') else '%.2f m' % r['x_exit']
    bl = blind_world(h, th)
    print('%.2f  %2.0f°  %8.4f  %2d  %-5s  %-19s  %-24s  %6.3f m          %s'
          % (h, th, r['zg'], r['k0'], str(r['trunc']), xe, sp, r['band_above'],
             '—' if bl is None else '%.2f' % bl))
print()
print('★候选档（★窗＝真人定的判据档，施工侧只列形状与代价、★不自改）——按"要地面在 X m 内不出甲档窗"反推下沿与板数')
print('   h=0.45, θ=15°（乙 目标态）：')
r = row(0.45, 15.0)
for X in (1.0, 2.0, 4.0, 6.0):
    zmin, nz = r['need'][X]
    print('     容到 %2.0f m ⇒ z_min=%.2f ‖ 板数=%d（★甲档＝13 板 ⇒ 增 %d 板）‖ 该范围内地面跨 %d 板'
          % (X, zmin, nz, nz - 13, r['spread'][X]))
print('   h=0.45, θ=0°（★只改高不改俯仰）：z_min=%.2f 不动即可容到任意距离（地面恒定 %.4f、跨 0 板）' % (JIA_MIN, row(0.45, 0.0)['zg']))
print('   h=0.49, θ=0°（★当期实测态）：z_ground=%.4f ⇒ k0=%d（★在册 R310 锚的 k0=%d ⇒ 差 %d 板）'
      % (row(0.49, 0.0)['zg'], row(0.49, 0.0)['k0'], 1, row(0.49, 0.0)['k0'] - 1))
print()
print('★读法提醒＝雷达系里"地面恒定"只在 θ=0 成立；θ>0 时地面是一条斜线 ⇒ 单一 k0（接地板号）这个概念本身开始失效，')
print('  代价随 sinθ 线性增长（在册 DROP_PER_M=%.4f＝sin15°）。⇒ 勾"加宽窗"与勾"回转点云再用甲档"是两条不同的路，★后者属判定链结构改动、须明令。' % t3.DROP_PER_M)
