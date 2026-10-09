#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""n_selfmask_offline_compare.py —— 用**已落盘的原始帧**离线比任意过滤方案（R82；开一次链，之后不再开链）

为什么要有它：主理人要的是"换更适合 lite3 的过滤模型"，而候选形状是一族而不是一个（盒外扩多少、
要不要并一个小球、要不要按腿的最大外张做成圆角柱）。若每种想法都开一次链，就是拿独占窗试参数——
本件把这件事变成纯离线：读 `p4_selfmask_probe.py` 落的 `.npy`（雷达系原始坐标，float16）＋ `sm.json`
（各站机体位姿回读值＋障碍物位姿），对同一批点做任意过滤变体的对照。

三个口径同时报，缺一个就会被单面读数骗：
  · `obst`  = 落在**障碍物自身 OBB** 内的保留点数（这才是"换模型救回多少真障碍"）；
  · `self`  = 保留点里仍落在**零外扩躯干盒**内的个数（这是"换完还剩多少自回波进云"，越小越干净）；
  · `grnd`  = 保留点里世界系 z<0.10 且距雷达 <1.0m 的个数（近地面点回来多少——好事还是噪声**本件不判**，
              留给 #68 之后的 SCAN 栅格对照，见 n 稿 §5 的 Q3 第三问）。

零改点：只读证据目录，不碰任何源码/配置/门值；不起 ROS。
用法：python3 note/进度/SCAN集成/设计/scripts/n_selfmask_offline_compare.py <证据目录> [每站取前几帧]
"""
import json
import math
import os
import sys

import numpy as np

BANDS = (0.0, 0.27, 0.56, 1.00, 1.50, 3.00, 1e9)
HALF = (0.210, 0.155, 0.110)
LID_OFF_Z = 0.380
EPS = 0.01
Z_FLOOR = -0.35


def world_of(x, y, z, mp):
    qx, qy, qz, qw = mp["qx"], mp["qy"], mp["qz"], mp["qw"]
    vx, vy, vz = x, y, z + LID_OFF_Z
    tx, ty, tz = 2.0 * (qy * vz - qz * vy), 2.0 * (qz * vx - qx * vz), 2.0 * (qx * vy - qy * vx)
    return (vx + (qw * tx + (qy * tz - qz * ty)) + mp["x"],
            vy + (qw * ty + (qz * tx - qx * tz)) + mp["y"],
            vz + (qw * tz + (qx * ty - qy * tx)) + mp["z"])


def box_mask(x, y, z, margin):
    hx, hy, hz = HALF
    cz = -LID_OFF_Z
    return ((np.abs(x) <= hx + margin + EPS) & (np.abs(y) <= hy + margin + EPS)
            & (np.abs(z - cz) <= hz + margin + EPS))


def cyl_mask(x, y, z, rad, zlo, zhi):
    """按腿最大外张的候选形状：半径 rad 的柱，只在 zlo<z<zhi 段生效。"""
    return (np.sqrt(x * x + y * y) <= rad) & (z > zlo) & (z < zhi)


def obb_mask(x, y, z, cx, cy, cz, yaw, half=0.40):
    dx, dy, dz = x - cx, y - cy, z - cz
    c, s = math.cos(-yaw), math.sin(-yaw)
    lx, ly = c * dx - s * dy, s * dx + c * dy
    return (np.abs(lx) <= half + EPS) & (np.abs(ly) <= half + EPS) & (dz >= Z_FLOOR) & (dz <= half + EPS)


def zero_mask(x, y, z):
    """插件的**无命中占位点**：实测 (0,0,0) 且**不是**躯干盒内的自回波（盒心 z=−0.38，z=0 在盒外）。
    R82 实测：仿真每帧 2 万点里 **64–75% 是这种占位点**，现网的球形 blind 顺手把它们全删了 ⇒
    **"换盒"必须同时显式剔零**，否则这 1.4 万点会带着机体轨迹往地图上堆一坨原点。"""
    return (np.abs(x) < 1e-9) & (np.abs(y) < 1e-9) & (np.abs(z) < 1e-9)


def variants():
    """候选过滤（返回 (名字, 保留掩码函数)）。名字里带数值，表就是自说明的。

    列的含义：`tot`＝**下游要吃的点数**（负载面），`障碍`＝救回的真障碍点（收益面），
    `自回波`＝残留机体点（干净面）。R82 在仿真里 `自回波` 恒 0 ⇒ 该列**仿真测不了**，见 n 稿 §5。"""
    v = [("现网 球1.0", lambda x, y, z, r: r >= 1.00),
         ("simyaml 球0.2（从未生效）", lambda x, y, z, r: r >= 0.20),
         ("球0.3", lambda x, y, z, r: r >= 0.30),
         ("盒 m=0.05（不剔零＝会淹）", lambda x, y, z, r: ~box_mask(x, y, z, 0.05)),
         ("盒m=0.05 ∪ 剔零", lambda x, y, z, r: (~box_mask(x, y, z, 0.05)) & (~zero_mask(x, y, z))),
         ("盒m=0.05∪剔零∪球0.3", lambda x, y, z, r: (~box_mask(x, y, z, 0.05)) & (~zero_mask(x, y, z)) & (r >= 0.30)),
         ("盒 m=0.00", lambda x, y, z, r: ~box_mask(x, y, z, 0.00)),
         ("盒 m=0.05", lambda x, y, z, r: ~box_mask(x, y, z, 0.05)),
         ("盒 m=0.15", lambda x, y, z, r: ~box_mask(x, y, z, 0.15)),
         ("盒 m=0.25", lambda x, y, z, r: ~box_mask(x, y, z, 0.25)),
         ("盒m=0.05 ∪ 球0.20", lambda x, y, z, r: (~box_mask(x, y, z, 0.05)) & (r >= 0.20)),
         ("盒m=0.05 ∪ 球0.30", lambda x, y, z, r: (~box_mask(x, y, z, 0.05)) & (r >= 0.30)),
         ("柱 r=0.45（腿外张）", lambda x, y, z, r: ~cyl_mask(x, y, z, 0.45, -0.60, -0.20)),
         ("柱r=0.45 ∪ 球0.20", lambda x, y, z, r: (~cyl_mask(x, y, z, 0.45, -0.60, -0.20)) & (r >= 0.20))]
    return v


def do_grid(j, tags, load):
    """外扩 m × 兜底小球 s 的**网格**（--grid 模式）：同时出「收益＝障碍 OBB 内保留点」与
    「负载＝下游总点数」，并给每档外扩的**盒阴影半径**＝盒八角点在雷达系里的最近／最远距离。

    为什么要盒阴影半径这个数：真机侧真正的问题是"外扩 m 之后，正面多大范围内的障碍会被我自己
    划掉的盒吞掉"，那个范围就是盒角点的距离——它是**几何量的上界**，不需要真回波也能算，
    而仿真的 `自回波` 列恒 0（无腿、无地面），所以这一列是**当前唯一能给的定标输入**。
    """
    Ms = [round(0.00 + 0.05 * i, 2) for i in range(10)]
    Ss = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 1.0]
    ob = j["obstacle"]
    yaw_o = 2.0 * math.atan2(ob["qz"], ob["qw"])
    cs = []
    for t in tags:
        fr = load(t)
        if fr is None:
            continue
        mp = j["rows"][t]["pose"]
        w = world_of(fr[:, 0], fr[:, 1], fr[:, 2], mp)
        r = np.sqrt(fr[:, 0] ** 2 + fr[:, 1] ** 2 + fr[:, 2] ** 2)
        om = obb_mask(w[0], w[1], w[2], ob["x"], ob["y"], ob["z"], yaw_o)
        cs.append((fr[:, 0], fr[:, 1], fr[:, 2], r, om))
    if not cs:
        print("无可用站 ⇒ INVALID|网格算不出来")
        return 9
    print("== 网格（每格 收益＝障碍内保留点 ｜ 负载＝下游总点数；跨 %d 站合计）==" % len(cs))
    print("%-6s %s" % ("外扩m\\小球s", " ".join("%14.1f" % sv for sv in Ss)))
    best = {}
    for m in Ms:
        cells = []
        for sv in Ss:
            o = g = 0
            for (x, y, z, r, om) in cs:
                kp = (~box_mask(x, y, z, m)) & (~zero_mask(x, y, z)) & (r >= sv)
                o += int(np.sum(kp & om))
                g += int(np.sum(kp))
            cells.append("%6d|%8d" % (o, g))
            best[(m, sv)] = (o, g)
        print("%-6.2f %s" % (m, " ".join(cells)))
    mx = max(v[0] for v in best.values())
    print("\n== 读法一：收益上限 %d 点；达到 99%% 上限所需的**最小兜底球**（同外扩下）==" % mx)
    for m in Ms:
        hit = [sv for sv in Ss if best[(m, sv)][0] >= 0.99 * mx]
        lo = min(hit) if hit else None
        print("  外扩 %.2f：99%% 收益的最小兜底球 %s；该点负载=%s" % (
            m, ("%.1f m" % lo) if lo is not None else "无（被盒自己吃掉）",
            best[(m, lo)][1] if lo is not None else "—"))
    print("\n== 读法二：盒阴影半径（八角点在雷达系的最近／最远距离＝正面会被吞到的范围）==")
    hx, hy, hz = HALF
    cz = -LID_OFF_Z
    for m in Ms:
        cor = [(sx * (hx + m), sy * (hy + m), cz + sz * (hz + m))
               for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]
        dd = [math.sqrt(a * a + b * b + c * c) for (a, b, c) in cor]
        print("  外扩 %.2f ⇒ 阴影 %.3f–%.3f m（正面 x 向 %.3f m）" % (m, min(dd), max(dd), hx + m))
    print("\n边界声明：本表全部由**仿真落盘帧**算出 ⇒ 收益与负载两列可信；"
          "但『外扩该取多少』的真约束是自回波覆盖（腿＋支架），**仿真无腿**（n 稿 §7.3）⇒ "
          "本表给的是**上界侧的几何证据**，不是最终尺寸；最终尺寸等 B15 结合真机安装位定。")
    return 0


def main():
    if len(sys.argv) < 2:
        print("用法：python3 n_selfmask_offline_compare.py <证据目录> [帧数=全部] [--grid]")
        return 2
    d = sys.argv[1]
    nf = int(next((a for a in sys.argv[2:] if not a.startswith("-")), 0))   # 位置参数可跳过 --flag
    jp = os.path.join(d, "sm.json")
    if not os.path.isfile(jp):
        print("读不到 %s ⇒ 没有实跑落盘就没法离线比（本件不造数）" % jp)
        return 2
    j = json.load(open(jp, encoding="utf-8"))
    ob = j["obstacle"]
    yaw_o = 2.0 * math.atan2(ob["qz"], ob["qw"])
    tags = sorted([k for k in j["rows"] if os.path.isfile(os.path.join(d, k + ".npy"))])
    if not tags:
        print("sm.json 有 %d 站但无可读 .npy ⇒ 判 INVALID|证据不完整" % len(j["rows"]))
        return 9
    def load(t):
        a = np.load(os.path.join(d, t + ".npy")).astype(np.float64)
        if a.shape[0] != 3:
            print("  站 %-16s .npy 形状 %s 不是 (3,N) ⇒ 跳过，不猜" % (t, a.shape))
            return None
        return a.T

    if "--grid" in sys.argv:
        return do_grid(j, tags, load)

    V = variants()
    print("被检站=%d（障碍物 %s 原位未动：x=%.2f y=%.2f yaw=%.0f°）" % (len(tags), "obs_5", ob["x"], ob["y"],
                                                                        math.degrees(yaw_o)))
    print("%-16s %8s %8s ｜ %s" % ("站(近面m/方向)", "障碍内点", "自回波",
                                   "  ".join("%s⇒障碍/自/地/总" % n for n, _ in V)))
    tot = dict((n, [0.0, 0.0, 0.0, 0.0]) for n, _ in V)
    for t in tags:
        arr = np.load(os.path.join(d, t + ".npy")).astype(np.float64)
        if arr.shape[0] != 3:
            print("  站 %-16s .npy 形状 %s 不是 (3,N) ⇒ 跳过，不猜" % (t, arr.shape))
            continue
        if nf:
            arr = arr[:, :nf]
        fr = arr.T
        mp = j["rows"][t]["pose"]
        w = world_of(fr[:, 0], fr[:, 1], fr[:, 2], mp)
        r = np.sqrt(fr[:, 0] ** 2 + fr[:, 1] ** 2 + fr[:, 2] ** 2)
        om = obb_mask(w[0], w[1], w[2], ob["x"], ob["y"], ob["z"], yaw_o)
        body0 = box_mask(fr[:, 0], fr[:, 1], fr[:, 2], 0.0)
        nearg = (w[2] < 0.10) & (r < 1.0)
        cells = []
        for name, fn in V:
            kp = fn(fr[:, 0], fr[:, 1], fr[:, 2], r)
            o = int(np.sum(kp & om))
            sl = int(np.sum(kp & body0))
            g = int(np.sum(kp & nearg))
            tt = int(np.sum(kp))
            cells.append("%5d/%4d/%4d/%6d" % (o, sl, g, tt))
            for i, val in enumerate((o, sl, g, tt)):
                tot[name][i] += val
        print("%-16s %8d %8d ｜ %s" % (t, int(np.sum(om)), int(np.sum(body0)), "  ".join(cells)))
    print("\n== 合计（跨 %d 站；格式 障碍保留/残留自回波/近地面点）==" % len(tags))
    base = tot["现网 球1.0"][0] or 1.0
    baset = tot["现网 球1.0"][3] or 1.0
    for name, fn in V:
        o, sl, g, tt = tot[name]
        print("  %-28s 障碍=%7.0f(×%.2f)  自回波=%6.0f  近地面=%6.0f  下游总点数=%8.0f(×%.2f)"
              % (name, o, o / base, sl, g, tt, tt / baset))
    print("\n读法：`障碍` 越大越好＝收益；`自回波` 越小越好＝干净；`下游总点数` 越小越好＝负载（在册 CPU 已超载过一次"
          "　faster_lio_outdoor.yaml:15 的 filter_size_surf 0.15→0.3 回退注记）。")
    print("三者不可同时最优 ⇒ 本表是给裁定的**输入**，不是结论。仿真侧 `自回波` 与 `近地面` 两列恒 0 ⇒ "
          "这两列必须由真机或换带腿/地面的仿真补测，**不能因为读到 0 就当通过**（坑45 同族）。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
