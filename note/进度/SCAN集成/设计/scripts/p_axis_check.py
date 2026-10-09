#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""p_axis_check.py —— 轴向对账（p 稿 §2）：**用实测点云反证插件的换算我读对了没有**

为什么这是可证伪的、而不是再读一遍代码：
  插件把 CSV 的 (azimuth, zenith) 变成射线方向的那两行是
      `ray.Euler(Vector3d(0.0, rotate_info.zenith, rotate_info.azimuth));  axis = offset.Rot() * ray * Vector3d(1,0,0)`
  其中 `zenith = CSV度 − 90°`（`livox_points_plugin.cpp:41`）。ignition 的 `Euler()` 用 **固定轴 XYZ 序**
  ⇒ R = Rz(yaw)·Ry(pitch)·Rx(roll)，roll=0。作用在 (1,0,0) 上得
      axis = ( sin z·cos a , sin z·sin a , cos z )      ← 我读出来的结果（**约定 C1**）
  也就是说：**极轴（zenith=0 的方向）落在传感器 frame 的 +Z 上**，而不是直觉上的"前面＝+X"。
  若我把 Euler 的次序或 zenith 的偏移读错了，实际应该是**约定 C2**（极轴＝+X）：
      axis = ( cos z , sin z·cos a , sin z·sin a )
  ⇒ C1 与 C2 会给出**不同**的方向集合，而实测点云只有一个真值 ⇒ **这一跑就能判我读对没有**。

判法：把 14 站实测非零点的单位方向，分别与 C1／C2 的候选方向集合做最近邻角差，**角差小的一套就是我读对的那套**。
零开链、只读已落盘帧与 CSV。

用法：python3 p_axis_check.py <r82 证据目录> [CSV 路径]
"""
import json
import os
import sys

import numpy as np

DEF_CSV = ("ws/src/95_simulation/Mid360_simulation_plugin/livox_laser_simulation/"
           "scan_mode/mid360-real-centr.csv")
CHUNK = 4000


def cand_sets(CSV):
    d = np.loadtxt(CSV, delimiter=",", skiprows=1)
    a = np.radians(d[:, 1])
    z = np.radians(d[:, 2])
    c1 = np.stack([np.sin(z) * np.cos(a), np.sin(z) * np.sin(a), np.cos(z)], axis=1)
    c2 = np.stack([np.cos(z), np.sin(z) * np.cos(a), np.sin(z) * np.sin(a)], axis=1)
    return c1, c2, np.degrees(z)


def nearest_err(obs, cand, k=3000):
    """obs／cand 都是单位向量阵；返回每个 obs 到 cand 最近邻的角差（度）。subsample 控制代价。"""
    if len(obs) > k:
        obs = obs[np.random.RandomState(0).choice(len(obs), k, replace=False)]
    if len(cand) > 60000:
        cand = cand[np.random.RandomState(1).choice(len(cand), 60000, replace=False)]
    cand = cand / np.linalg.norm(cand, axis=1, keepdims=True)
    out = []
    for i in range(0, len(obs), CHUNK):
        b = obs[i:i + CHUNK]
        s = np.max(b @ cand.T, axis=1)
        out.append(np.degrees(np.arccos(np.clip(s, -1.0, 1.0))))
    return np.concatenate(out)


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    d = sys.argv[1].rstrip("/")
    CSV = sys.argv[2] if len(sys.argv) > 2 else DEF_CSV
    if not os.path.isfile(CSV):
        print("读不到 CSV（%s）⇒ 不猜，退出" % CSV)
        return 2
    jp = os.path.join(d, "sm.json")
    if not os.path.isfile(jp):
        print("读不到 %s ⇒ 没有落盘帧就没法对账" % jp)
        return 2
    j = json.load(open(jp, encoding="utf-8"))
    c1, c2, zen = cand_sets(CSV)
    print("CSV 光束数=%d zenith=%.2f…%.2f°（角宽 %.2f°）" % (len(zen), zen.min(), zen.max(), zen.max() - zen.min()))
    for nm, c in (("C1 极轴＝+Z（我读代码的结果）", c1), ("C2 极轴＝+X（直觉那套）", c2)):
        el = np.degrees(np.arcsin(np.clip(c[:, 2], -1.0, 1.0)))
        print("  %-30s ⇒ 该约定下图案仰角范围 %+0.1f° … %+0.1f°" % (nm, el.min(), el.max()))

    obs, n = [], 0
    for t in sorted(j["rows"]):
        p = os.path.join(d, t + ".npy")
        if not os.path.isfile(p):
            continue
        a = np.load(p).astype(np.float64)
        r = np.linalg.norm(a, axis=0)
        m = r > 1e-6
        u = a[:, m] / r[m]
        obs.append(u.T)
        n += int(m.sum())
    if not obs:
        print("无可读 .npy ⇒ 退出（不造数）")
        return 9
    O = np.concatenate(obs)
    print("\n实测非零点=%d（14 站合并，单位方向）" % len(O))
    el = np.degrees(np.arcsin(np.clip(O[:, 2], -1.0, 1.0)))
    print("  实测点仰角（正＝朝上，雷达系 +Z）：min=%+0.1f° max=%+0.1f°  p1=%+0.1f° p99=%+0.1f°"
          % (el.min(), el.max(), np.percentile(el, 1), np.percentile(el, 99)))
    print("\n== 最近邻角差（越小＝该套约定越贴合实测）==")
    res = {}
    for nm, c in (("C1", c1), ("C2", c2)):
        e = nearest_err(O, c)
        res[nm] = e
        print("  %s：中位=%.2f°  p90=%.2f°  p99=%.2f°  max=%.2f°  角差<1° 的占比=%.1f%%"
              % (nm, np.median(e), np.percentile(e, 90), np.percentile(e, 99), e.max(), 100.0 * (e < 1.0).mean()))
    win = min(res, key=lambda k: np.median(res[k]))
    lose = "C2" if win == "C1" else "C1"
    ratio = np.median(res[lose]) / max(np.median(res[win]), 1e-9)
    print("\n判定：**%s 更贴合实测**（中位角差是另一套的 1/%.1f）⇒ %s"
          % (win, ratio,
             "我读的那套成立：插件确实把 zenith 的极轴放在**传感器 +Z** 上，图案在机体系里上偏＝设备/CSV 属性，不是我把 Euler 次序读反。"
             if win == "C1" else
             "我读错了：极轴其实落在 +X，前面所有关于图案上下分布的推算要重做（p 稿要跟着改）。"))
    print("\n边界声明（本件不能答的部分）：本件只证**插件在仿真里怎么摆这 59°**，"
          "不证**真机 Mid-360 相对机械安装面的取向**——那要厂商资料或真机原始点云，"
          "而**仓内没有任何真机 Mid-360 记录**（已查：*.bag 无；仓内 .pcd 全是仿真产物或上游示例）。⇒ 归 B17。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
