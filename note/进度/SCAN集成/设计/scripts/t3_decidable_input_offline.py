#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# t3_decidable_input_offline.py —— 单层高程栅格路径的**结构断言**可判输入件（B.1.3；零窗、不自装闸）
#
# 性质：本件只出**结构断言与读数**（载体契约／三态语义／分辨率／z 窗），★不出代价映射、不出"可上/不可上"判据、
#       不出任何达成结论 —— A47＝丙 三步的第三步形状，结论权在主理人＋小张＋小美。
# 边界：零窗、零编译目标、不发 ROS、不连 costmap 插件链、不改任何在册件（★含已搁置 S1 留盘件 w3_elevation_node.py）。
# 路线：按真人 2026-10-17 推荐路线，3D 闭环优先走**单层高程栅格**路径（v6 §B.1.3 窄义裁定 R347：
#       /elevation_grid 对单层不违令）；★本件不绕去多层、★不因此撤 X1 其余禁令。
# 并存关系：B.1.3 的 X1 多层结构断言已由 an 稿（R303，2026-10-14）交付并入库（判：单标量载体表达不了多层、
#       单高度化要丢七成板事实）—— 本件与 an 稿**并存不覆盖**：an 稿判"多层"，本件立"单层形状"。
# 分档：本件产出＝**甲-离**（离线几何/真数据读数），★不得顶替甲-活（05 追加三十二）。
# 参数依据：分辨率 0.10 有在册依据（spike3d §4.3）；z 窗按 R310 工作值（h=0.45／θ=15°、Mid-360 量具实测、
#       ★只对静止站立态成立）重推，几何式与四个锚点值均在册（R310 收编记录四）；旧工作值（h=0.38／θ=18°）
#       仅作对照，不回改。
# P8：计划书未定稿、现行以 v5.3（含 R294／R310 收编段）＋v6 §B.1（含 R347 窄义裁定）为准、本件不是终稿。
#
# 用法：
#   python3 t3_decidable_input_offline.py --selftest          # 自测（合成档＋阴性对照）
#   python3 t3_decidable_input_offline.py --emit              # 出具可判输入块（结构断言＋读数）
#   python3 t3_decidable_input_offline.py --zwin              # 只跑 B.1.4 z 窗重推并逐字并报
#   （三者皆不传 ⇒ rc=9 前置缺失）

import argparse
import math
import sys

# ---- 在册常量（★只引用、不自造）----
UNKNOWN, FREE, BLOCKED = -1, 0, 100       # #46 三态在册常量（与 t1 同源）
RES = 0.10                                # 分辨率在册依据（spike3d §4.3）
SLAB = 0.10                               # z 板厚（丙档设计值）
Z_RANGE = 1.2                             # 丙档窗宽（v6 §B.1.1）
C_BING_MIN, C_BING_MAX = -0.45, 0.75      # 丙档设计值窗（v6 §B.1.1；由旧 38cm 推导、非实测）
W3_NODE_Z_MIN, W3_NODE_Z_MAX = -2.0, 2.0  # 现读（已搁置 S1 留盘件 w3_elevation_node.py 参数缺省）
H_R310, THETA_R310 = 0.45, 15.0           # R310 工作值（Mid-360 量具实测；只对静止站立态成立）
H_OLD, THETA_OLD = 0.38, 18.0             # 旧工作值（仅对照）
ANCHOR_IN_REG = (-0.4347, -0.5641, -0.6935)  # R310 在册锚点（x_b＝0／0.5／1.0 m，雷达系）
DROP_PER_M = 0.2588                       # R310 在册：每前进 1 m 地面在雷达系再降（＝sin15°）
GROUND_OLD = -0.3614                      # R310 在册：旧工作值下正下方地面（雷达系）

# ---- B.1.1 结构断言族（单层高程栅格路径；★断言"在册形状"，不自设契约）----
CARRIER_TOPIC = '/elevation_grid'         # R280 落盘节点发布位（w3_elevation_node.py 第 231 行现查）
CARRIER_TYPE = 'nav_msgs/OccupancyGrid'   # 同上（latch=True 同现）
CARRIER_LATCH = True


def ground_z_radar(x_b, h, theta_deg):
    """R310 在册几何式：机身系正前方 x_b 处的地面，在雷达系 z_l ＝ −(h·cosθ ＋ x_b·sinθ)。

    ★x_b 向前为正、h＝离地高度、θ＝前倾角（光轴相对机身水平面向下倾）。
    ★返回值为雷达系 z（地面在校正后为负）。
    """
    t = math.radians(theta_deg)
    return -(h * math.cos(t) + x_b * math.sin(t))


def k0_of(z_ground, z_min, slab=SLAB):
    """地面板序号 k0 ＝ floor((z_ground − z_min)/slab)。

    ★实现级注意（浮点陷阱）：当 z_min 恰等于 z_ground − slab 时，浮点减法可能得
    0.9999999999999998 ⇒ 裸 floor 得 0（差一板）。本件加 1e-9 容差（与 t1 的 truncated
    判据同族容差），★该容差只此一处、只治"恰好贴板"这一形状，不放宽任何其他计算。
    """
    return int(math.floor((float(z_ground) - float(z_min)) / slab + 1e-9))


def is_truncated(z_ground, z_min, slab=SLAB):
    """截断判据（沿用 t1 在册形状）：窗下沿高于"地面所在整板"⇒ 地面那板被截半。

    判据＝z_min > z_ground − slab ＋ 1e-9（小张修正④：要求地面所在整板在窗内）。
    """
    return bool(float(z_min) > float(z_ground) - slab + 1e-9)


def z_window_rederive(h=H_R310, theta_deg=THETA_R310, slab=SLAB, z_range=Z_RANGE,
                      upper_keep=C_BING_MAX, pad=1, mode='keep-upper'):
    """B.1.4：按 R310 工作值重推雷达系 z 窗（贴地法，小张修正④同款）。

    形状＝下沿贴地（地面所在整板在窗内、整一板垫在下面）：z_min ≤ z_ground − pad·slab；
    本件先把下沿向 0.05 m 栅格向下取整（设计值取整、★非实测），再向**上沿的板栅格**对齐
    （keep-upper 档：上沿固定 ⇒ 窗宽必须是板厚整数倍 ⇒ 下沿只许向下微动找板栅格；
    ★旧工作值那组若不做这步会落在 1.25 板＝非整数倍上，自测 M1 当场逮到）。
    上沿两个候选（★都是丙档设计值、★定档权不在施工侧、★以小美／真人裁为准）：
      mode='keep-upper'（缺省）＝上沿沿用丙档 +0.75（R310 只给地面带几何、上沿无重推依据
              ⇒ 保守保留、与丙档保持可比）；
      mode='keep-width'＝保丙档窗宽（z_max ＝ z_min ＋ z_range，12 板）。
    返回 dict（★非实测、★引用必须连带安装位 (h,θ) 与站立态）。
    """
    z_g = ground_z_radar(0.0, h, theta_deg)                    # 正下方地面（雷达系）
    raw_min = z_g - pad * slab
    z_min = round(math.floor(raw_min / 0.05) * 0.05, 2)        # 向 0.05 m 栅格向下取整
    if mode == 'keep-upper':
        z_max = upper_keep
        nz = int(math.ceil((z_max - z_min) / slab - 1e-9))     # 板数向上取整（浮点误差用 1e-9 抵）
        z_min = round(z_max - nz * slab, 2)                    # 下沿向板栅格对齐（只下不上）
    elif mode == 'keep-width':
        z_max = round(z_min + z_range, 2)
    else:
        raise ValueError('mode 只认 keep-upper／keep-width：%r' % (mode,))
    nz = int(round((z_max - z_min) / slab))
    if abs((z_max - z_min) / slab - nz) > 1e-9:
        raise ValueError('窗宽不是板厚整数倍：z_max-z_min=%.4f slab=%.2f' % (z_max - z_min, slab))
    return {'h': h, 'theta_deg': theta_deg, 'stance': '静止站立态（R310 限定）',
            'mode': mode,
            'z_ground': z_g, 'pad_slabs': pad, 'z_min': z_min, 'z_max': z_max,
            'z_range': z_max - z_min, 'nz': nz, 'slab': slab,
            'k0': k0_of(z_g, z_min, slab), 'truncated': is_truncated(z_g, z_min, slab),
            'grade': '丙档设计值（非实测）'}


# ---------- 结构断言（可判输入的断言面；★不是闸）----------
def assert_carrier_contract(topic, msg_type, latch):
    """载体契约三件一套：话题名／消息类型／latch（形状照 R280 在册节点，R347 窄义只放行单层）。"""
    return (topic == CARRIER_TOPIC and msg_type == CARRIER_TYPE and bool(latch) == CARRIER_LATCH)


def assert_tristate_constants(unknown_v, free_v, blocked_v):
    """#46 三态常量逐字（未覆盖＝未知、默认分支不产出自由）。"""
    return (unknown_v == UNKNOWN and free_v == FREE and blocked_v == BLOCKED)


def assert_single_layer_only(claims_multi_layer):
    """窄义前提断言：本路径只立单层形状；★宣称多层 ⇒ 断言必须红（an 稿 R303 已判单标量载体表达不了多层）。"""
    return not bool(claims_multi_layer)


# ---------- 自测（合成分档：每条都带"怎么当场验"）----------
def selftest():
    ok = fail = 0
    lines = []

    def check(tag, cond, detail=''):
        nonlocal ok, fail
        if cond:
            ok += 1
            lines.append('  [PASS] %s %s' % (tag, detail))
        else:
            fail += 1
            lines.append('  [FAIL] %s %s' % (tag, detail))

    # T3-a R310 几何复算对在册三锚点（x_b＝0／0.5／1.0 m；旧值锚点在 T3-c）
    got = [round(ground_z_radar(x, H_R310, THETA_R310), 4) for x in (0.0, 0.5, 1.0)]
    check('T3-a R310 三锚点复算（h=0.45／θ=15°）', tuple(got) == ANCHOR_IN_REG,
          '复算=%s 在册=%s' % (got, list(ANCHOR_IN_REG)))

    # T3-b 每米降幅对在册（＝sin15°）
    check('T3-b 每前进 1 m 再降 0.2588（在册）', abs(abs(math.sin(math.radians(15.0))) - DROP_PER_M) < 5e-5,
          '复算=%.4f 在册=%.4f' % (abs(math.sin(math.radians(15.0))), DROP_PER_M))

    # T3-c 旧工作值对照（证明式子跨两个在册时点都成立，不是只拟合新值）
    g_old = round(ground_z_radar(0.0, H_OLD, THETA_OLD), 4)
    check('T3-c 旧工作值正下方 −0.3614（h=0.38／θ=18°）', g_old == GROUND_OLD,
          '复算=%.4f 在册=%.4f' % (g_old, GROUND_OLD))

    # T3-d ★B.1.4 回核发现：丙档窗在 R310 锚下截断（地面点距窗下沿仅 0.0153 m、不足一板）
    z_g = ground_z_radar(0.0, H_R310, THETA_R310)
    check('T3-d 丙档 [−0.45,+0.75] 按 R310 锚判 truncated=True（回核发现）',
          is_truncated(z_g, C_BING_MIN) is True,
          'z_min=%.2f z_ground=%.4f（贴地法要求下沿 ≤ %.4f）' % (C_BING_MIN, z_g, z_g - SLAB))
    check('T3-d 丙档窗 k0=0（地面落在 0 号板、板下无垫板 ⇒ 这正是截断的几何形状）',
          k0_of(z_g, C_BING_MIN) == 0, 'k0=%d' % k0_of(z_g, C_BING_MIN))

    # T3-e 重推窗形状（贴地一板、板数整数、不截断）
    w = z_window_rederive()
    check('T3-e 重推窗 [%.2f, %.2f] 板数=%d truncated=False k0=%d'
          % (w['z_min'], w['z_max'], w['nz'], w['k0']),
          w['truncated'] is False and w['nz'] == int(round(w['z_range'] / SLAB)) and w['k0'] == 1,
          'z_min=%.2f（≤ %.4f 取整）z_max=%.2f nz=%d' % (w['z_min'], z_g - SLAB, w['z_max'], w['nz']))
    check('T3-e 重推窗下沿整板垫地（z_min ≤ z_ground − slab）', w['z_min'] <= z_g - SLAB + 1e-12,
          'z_min=%.2f vs 界限=%.4f' % (w['z_min'], z_g - SLAB))

    # T3-e2 保宽候选（上沿随下沿动、保丙档 12 板）：两个候选都合法、★定档权不在施工侧
    ww = z_window_rederive(mode='keep-width')
    check('T3-e2 保宽候选 [%.2f, %.2f] %d 板 truncated=False k0=%d'
          % (ww['z_min'], ww['z_max'], ww['nz'], ww['k0']),
          ww['truncated'] is False and ww['nz'] == 12 and ww['k0'] == 1 and ww['z_max'] == 0.65,
          'z_max=%.2f（保丙档窗宽 %.1f）' % (ww['z_max'], Z_RANGE))

    # T3-f ★浮点陷阱实现级对照：恰好贴板时裸 floor 会少一板（证明 1e-9 容差不是装饰）
    bare = int(math.floor((z_g - (z_g - SLAB)) / SLAB))
    check('T3-f 恰好贴板时裸 floor＝%d（陷阱复现）⇒ 本件 k0 用容差后＝1' % bare,
          bare in (0, 1) and k0_of(z_g, z_g - SLAB) == 1,
          '裸=%d 容差后=%d' % (bare, k0_of(z_g, z_g - SLAB)))

    # T3-g 三态常量断言（含"未覆盖≠自由"的形状：未知不是 0）
    check('T3-g 三态常量逐字（−1／0／100）', assert_tristate_constants(-1, 0, 100) is True)
    check('T3-g 未知≠自由（UNKNOWN 不是 FREE）', UNKNOWN != FREE and UNKNOWN == -1)

    # T3-h 载体契约断言（合成结构验断言函数本身，不连 ROS）
    check('T3-h 载体契约三件一套过（/elevation_grid｜OccupancyGrid｜latch）',
          assert_carrier_contract('/elevation_grid', 'nav_msgs/OccupancyGrid', True) is True)
    check('T3-h 话题名错一个字符即红（断言有牙）',
          assert_carrier_contract('/elevation_gridx', 'nav_msgs/OccupancyGrid', True) is False)
    check('T3-h latch 丢失即红', assert_carrier_contract('/elevation_grid', 'nav_msgs/OccupancyGrid', False) is False)

    # M1 阴性对照：换回旧工作值 ⇒ 重推窗必须回到旧锚一带（证明窗跟着安装位走、不是写死）
    w_old = z_window_rederive(h=H_OLD, theta_deg=THETA_OLD)
    check('M1 旧工作值下重推窗贴旧地面（z_min ≤ %.4f）' % (GROUND_OLD - SLAB),
          w_old['z_min'] <= GROUND_OLD - SLAB + 1e-12 and abs(w_old['z_ground'] - GROUND_OLD) < 5e-5,
          'z_ground=%.4f z_min=%.2f' % (w_old['z_ground'], w_old['z_min']))

    # M2 阴性对照：宣称多层 ⇒ 窄义前提断言必须红（单层路径不许拿单标量载体声称多层）
    check('M2 宣称多层 ⇒ 断言红（窄义前提有牙）', assert_single_layer_only(True) is False)
    check('M2 不宣称多层 ⇒ 断言绿', assert_single_layer_only(False) is True)

    # M3 阴性对照：把窗下沿抬到地面板之上 ⇒ 截断判据必须红（证明 truncated 有分辨力）
    check('M3 下沿截半地面板 ⇒ truncated=True（判据有牙）',
          is_truncated(z_g, z_g - SLAB / 2) is True, 'z_min=%.4f' % (z_g - SLAB / 2))

    lines.append('  合计 PASS=%d FAIL=%d' % (ok, fail))
    return ok, fail, lines


def emit_contract():
    """出具可判输入块（结构断言＋读数；★甲-离、★不是闸、★入门成闸须主理人／验收侧明令）。"""
    w = z_window_rederive()
    z_g = ground_z_radar(0.0, H_R310, THETA_R310)
    print('T3_CONTRACT 载体＝%s｜%s｜latch=%s（形状照 R280 在册节点现查；R347 窄义只放行单层）'
          % (CARRIER_TOPIC, CARRIER_TYPE, CARRIER_LATCH))
    print('T3_CONTRACT 三态＝UNKNOWN=%d／FREE=%d／BLOCKED=%d（#46 在册常量；★未覆盖＝未知、绝不塌陷成可上）'
          % (UNKNOWN, FREE, BLOCKED))
    print('T3_CONTRACT 分辨率＝%.2f m（在册依据 spike3d §4.3）；z 板厚＝%.2f m（丙档设计值）' % (RES, SLAB))
    print('T3_ZWIN 安装位＝(h=%.2f m, θ=%.0f° 前倾)＋%s（R310 工作值；★任何 z 窗引用必须连带这两件）'
          % (H_R310, THETA_R310, '静止站立态'))
    print('T3_ZWIN 几何（R310 在册式）＝正下方 %.4f｜x_b=0.5 m 处 %.4f｜x_b=1.0 m 处 %.4f｜每前进 1 m 再降 %.4f（单位：m，雷达系）'
          % tuple([round(ground_z_radar(x, H_R310, THETA_R310), 4) for x in (0.0, 0.5, 1.0)] + [DROP_PER_M]))
    print('T3_ZWIN 重推窗候选甲（keep-upper）＝[%.2f, %.2f]（%d 板；k0=%d；truncated=%s；%s）'
          % (w['z_min'], w['z_max'], w['nz'], w['k0'], w['truncated'], w['grade']))
    ww = z_window_rederive(mode='keep-width')
    print('T3_ZWIN 重推窗候选乙（keep-width）＝[%.2f, %.2f]（%d 板；k0=%d；truncated=%s；%s）'
          '★两候选下沿同值、上沿二选一，定档权归小美／真人，施工侧不自选'
          % (ww['z_min'], ww['z_max'], ww['nz'], ww['k0'], ww['truncated'], ww['grade']))
    print('T3_ZWIN 并报一＝丙档设计值 [%.2f, %.2f] 按 R310 锚判 truncated=%s（★地面那板被截半 ⇒ 丙档下沿按 R310 回核不成立）'
          % (C_BING_MIN, C_BING_MAX, is_truncated(z_g, C_BING_MIN)))
    print('T3_ZWIN 并报二＝现读 [%.2f, %.2f]（已搁置 S1 留盘件参数缺省；★该件无坐标系注记、窗宽 %d 板，与丙档/重推值均不可比，仅登记）'
          % (W3_NODE_Z_MIN, W3_NODE_Z_MAX, int(round((W3_NODE_Z_MAX - W3_NODE_Z_MIN) / SLAB))))
    print('T3_ZWIN 帧系纪律＝上两行雷达系；map 系数据（lite3.pcd）按 t1 --anchor ground 用数据自报地面分位，★雷达系值直接用会把大部分点判成窗外')
    print('T3_GRADE 分档＝甲-离（离线几何读数；★不得顶替甲-活）；★本件不是闸：入门成闸须主理人／验收侧明令，施工侧不自装')
    print('T3_BOUNDARY 与 an 稿（R303）并存不覆盖：an 稿判多层（单标量载体表达不了多层、丢七成板事实），本件立单层形状（R347 窄义）')


def main(argv=None):
    ap = argparse.ArgumentParser(description='B.1.3 单层高程栅格结构断言可判输入件（零窗）')
    ap.add_argument('--selftest', action='store_true')
    ap.add_argument('--emit', action='store_true')
    ap.add_argument('--zwin', action='store_true')
    a = ap.parse_args(argv)
    rc = 0
    if a.selftest:
        ok, fail, lines = selftest()
        for ln in lines:
            print(ln)
        print('T3_SELFTEST PASS=%d FAIL=%d rc=%d' % (ok, fail, 0 if fail == 0 else 1))
        rc = 0 if fail == 0 else 1
    if a.zwin:
        w = z_window_rederive()
        print('T3_ZWIN_REDERIVE (h=%.2f, θ=%.0f°, 站立态) ⇒ z_ground=%.4f 重推窗=[%.2f, %.2f] nz=%d k0=%d truncated=%s（%s）'
              % (w['h'], w['theta_deg'], w['z_ground'], w['z_min'], w['z_max'], w['nz'], w['k0'],
                 w['truncated'], w['grade']))
    if a.emit:
        emit_contract()
    if not a.selftest and not a.emit and not a.zwin:
        print('用法：--selftest／--emit／--zwin（三者皆不传 ⇒ rc=9 前置缺失）')
        rc = 9
    return rc


if __name__ == '__main__':
    sys.exit(main())
