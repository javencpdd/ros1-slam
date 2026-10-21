#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""o_fov_check.py —— 从**已落盘帧**复算「仿真侧视场到底照得到什么」（o 稿 §2 的可复算件；零开链、零 ROS）

它回答三个能被现有数据回答的问题，并把不能回答的问题明确留在纸面上：
  F1 已发布点云里，**朝下**的最深俯角是多少？近场地面（世界 z≈0）出现在哪些量程？
  F2 光束图案 CSV 的角宽是多少？与厂商标称 59° 对不对得上？（宽度对得上 ≠ 取向对得上）
  F3 给定雷达离地高 h 与下沿角 E，**最近地面点**应在多远？现网 `blind` 会砍掉这段环里的多少？
     ⇒ 这一问是纯几何外推，用来告诉裁定者"真机若下沿更低，会多出什么"，**不是仿真实测**。

用法：python3 o_fov_check.py <r82 证据目录> [h=0.53]
"""
import json
import math
import os
import sys

import numpy as np

CSV = ("ws/src/95_simulation/Mid360_simulation_plugin/livox_laser_simulation/"
       "scan_mode/mid360-real-centr.csv")
Z_GROUND = 0.06
LID_OFF_Z = 0.380


def rot(qx, qy, qz, qw, vx, vy, vz):
    tx, ty, tz = 2.0 * (qy * vz - qz * vy), 2.0 * (qz * vx - qx * vz), 2.0 * (qx * vy - qy * vx)
    return (vx + (qw * tx + (qy * tz - qz * ty)),
            vy + (qw * ty + (qz * tx - qx * tz)),
            vz + (qw * tz + (qx * ty - qy * tx)))


def frames(d, j):
    for t in sorted(j["rows"]):
        p = os.path.join(d, t + ".npy")
        if not os.path.isfile(p):
            continue
        a = np.load(p).astype(np.float64)
        if a.shape[0] != 3:
            print("  站 %s 形状 %s 非 (3,N) ⇒ 跳过" % (t, a.shape))
            continue
        yield t, a, j["rows"][t]["pose"]


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    d = sys.argv[1].rstrip("/")
    h = float(sys.argv[2]) if len(sys.argv) > 2 else 0.53
    jp = os.path.join(d, "sm.json")
    if not os.path.isfile(jp):
        print("读不到 %s ⇒ 没有落盘帧就没法复算（本件不造数）" % jp)
        return 2
    j = json.load(open(jp, encoding="utf-8"))

    print("== F1 已发布点云的俯角与地面带（源：%s，%d 站）==" % (os.path.basename(d), len(j["rows"])))
    all_el, all_gz, all_gr = [], [], []
    n_all = 0
    for t, a, mp in frames(d, j):
        r = np.linalg.norm(a, axis=0)
        m = r > 1e-6                                   # 剔掉插件的 (0,0,0) 无命中占位点（R82 Q2）
        wx, wy, wz = rot(mp["qx"], mp["qy"], mp["qz"], mp["qw"], a[0], a[1], a[2] + LID_OFF_Z)
        wz = wz + mp["z"]
        el = np.degrees(np.arcsin(np.clip(-a[2][m] / r[m], -1.0, 1.0)))   # 正＝朝下
        all_el.append(el)
        gm = m & (np.abs(wz) < Z_GROUND)
        all_gz.append(np.degrees(np.arcsin(np.clip(-a[2][gm] / r[gm], -1.0, 1.0))) if gm.any() else np.array([]))
        all_gr.append(r[gm])
        n_all += int(m.sum())
    el = np.concatenate(all_el)
    gz = np.concatenate([v for v in all_gz if len(v)]) if any(len(v) for v in all_gz) else np.array([])
    gr = np.concatenate([v for v in all_gr if len(v)]) if any(len(v) for v in all_gr) else np.array([])
    print("  非零点合计=%d（占位点已剔）" % n_all)
    print("  全体点俯角：%.1f° … %.1f°（正＝朝下）  p99=%.1f°" % (el.min(), el.max(), np.percentile(el, 99)))
    if len(gz):
        print("  地面高度带（|z_world|<%.2fm）的点=%d 占非零 %.1f%%；其量程 %.2f…%.2fm；其俯角 %.1f…%.1f°"
              % (Z_GROUND, len(gz), 100.0 * len(gz) / n_all, gr.min(), gr.max(), gz.min(), gz.max()))
        print("  ⇒ 近场（r<1.0m）地面点=%d  r<1.5m=%d  ⇒ 现网 blind=1.0 砍到的地面点=0"
              % (int((gr < 1.0).sum()), int((gr < 1.5).sum())))
    else:
        print("  【注意】一个地面高度带的点都没有 ⇒ 本域仿真确实照不到地面（与 o 稿 §1 的另一种可能一致）")

    print("\n== F2 光束图案 CSV 的角宽 ==")
    if os.path.isfile(CSV):
        z = np.loadtxt(CSV, delimiter=",", skiprows=1)[:, 2]
        print("  光束数=%d  zenith 极轴角=%.2f…%.2f ⇒ **角宽=%.2f°**（厂商标称 59°）"
              % (len(z), z.min(), z.max(), z.max() - z.min()))
        print("  换算（插件 livox_points_plugin.cpp:41 用 zenith−90° 当 pitch）⇒ 相对 boresight %.2f°…%.2f°"
              % (z.min() - 90.0, z.max() - 90.0))
        print("  ⇒ **宽度保真**；至于这 59° 在机体系里朝上多还是朝下多，取决于 x 轴朝向约定，**本件不结论**（o 稿 §4①）")
    else:
        print("  读不到 %s ⇒ 不猜" % CSV)

    print("\n== F3 几何外推：真机若下沿更低，近场地面环从哪里开始 ==")
    print("  式：最近地面水平距离 = h / tan(E)；现网 blind=%.1f 会把 r<%.1f 的这段一起砍掉" % (1.0, 1.0))
    for E in (7.2, 15.0, 22.5, 29.5):
        dh = h / math.tan(math.radians(E))
        cut = "有（环落在 r<1.0m 内的一部分被砍）" if h / math.sin(math.radians(E)) < 1.0 else "无（最近地面点已在 1.0m 外）"
        print("   h=%.2fm 下沿 −%.1f° ⇒ 最近地面水平 %.2fm（对应斜距 %.2fm）⇒ blind 砍地面：%s"
              % (h, E, dh, h / math.sin(math.radians(E)), cut))
    print("  h 的取值本身未证：仿真 xacro 是 base_link 上 0.38m＋spawn z=0.15 ⇒ 0.53m；"
          "真机是**安装高×身体高度挡位**的函数（X25），0.38 属约定值 ⇒ **本表按 h 参数化，不宣称已定**")
    print("\n本件只出数：F1/F2 是实测与实算，F3 是外推；结论归属见 o 稿 §4–§5。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
