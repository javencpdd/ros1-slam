#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""s1_moving_ghost_probe.py —— R288：占位零点的「移动态幽灵格拖尾」数量级估算（零窗、离线）

★件头限定句（逐字抄自部署架构师-小张 2026-10-14 裁定，签文留底
  `note/进度/SCAN集成/证据/log/raw/r283_signs/20_xiaozhang_177_176.txt`，限定＝依据施工侧转录证据、未自盘）＝
  「本件为 yaw=0 近似、仅用机体 (x,y) 平移重投的幽灵拖尾数量级估算，非实测；
    不构成 S1 已在 map 系验证之证据，下游不得引用作该验证凭据。」

量什么（只量这一件）
  R281 静止态实测＝占位零点 `点级占比=74.6%（208836／280000）` 而 `占位零点落格数=1` ⇒ 格级只动 1／14000 格；
  成因＝所有占位零点都是雷达原点 (0,0,0)，静止时全塌进同一格。
  本件把逐站机体 (x,y) 平移量加回去（yaw 恒取 0 ⇒ 旋转＝单位阵），量这些占位零点在 map 系散成多少格。

三条近似（全部如实写出，不许被读成实测）
  ①yaw=0：R82 留底逐站**只落盘 x,y、未落 yaw** ⇒ 只平移不旋转。
    ★对本件主判据**不敏感**：占位零点转任何角度仍是 (0,0)，落哪一格只由机体平移量决定；
      而"真实障碍点落哪一格"对 yaw **敏感** ⇒ 那部分本件不据以下任何结论。
  ②z 不平移（雷达安装高度偏移在 R82 留底里没有逐站数值）：★整段 z 平移不改变任何点落在哪个 xy 格 ⇒ 对本件主判据无影响。
  ③14 站是离散进近位姿、不是连续轨迹 ⇒ 连续运动下的拖尾只能外推，**只报量级并标为推断**。

★本件不得输出「已排除／未排除」「S1 已在 map 系验证」这类句子；三通道计数分因并报、不合成单值。
"""
import glob
import importlib.util
import math
import os
import re
import sys

import numpy as np

R82_DIR = "note/进度/SCAN集成/证据/log/raw/r82_selfmask_20261010_042738"
NODE = "ws/src/30_perception/lite3_perception/scripts/w3_elevation_node.py"
SLOPE_DEG = 25.0
STEP_M = 0.15
ROUGH_M = 0.05
RES = 0.1
STATION_RE = re.compile(r"站\s+(\S+)\s+机体\(\s*(-?[0-9.]+),\s*(-?[0-9.]+)\)")
TRISTATE = set([-1, 0, 100])


def find_root():
    d = os.path.abspath(os.path.dirname(__file__))
    for _ in range(12):
        if os.path.isdir(os.path.join(d, "ws", "src")) and os.path.isdir(os.path.join(d, "note", "进度")):
            return d
        d = os.path.dirname(d)
    return None


def load_station_xy(root):
    p = os.path.join(root, R82_DIR, "probe_stdout.txt")
    if not os.path.isfile(p):
        return {}
    out = {}
    with open(p, encoding="utf-8", errors="replace") as f:
        for line in f:
            m = STATION_RE.search(line)
            if m:
                out[m.group(1)] = (float(m.group(2)), float(m.group(3)))
    return out


def load_frames(root):
    out = []
    for f in sorted(glob.glob(os.path.join(root, R82_DIR, "*.npy"))):
        out.append((os.path.basename(f)[:-4], np.asarray(np.load(f).T, dtype=np.float64)))
    return out


def run_once(mod, frames, xy, move, window):
    """move=False＝雷达系原样（静止态，＝R281 那一形态）；move=True＝逐站按机体 (x,y) 平移重投（yaw 恒 0）。"""
    g = mod.ElevGrid(RES, window[0], window[1], window[2], window[3], z_min=-2.0, z_max=2.0)
    real = np.zeros(g.count.shape, dtype=bool)
    ghost = np.zeros(g.count.shape, dtype=bool)
    tot = zeros = 0
    for name, pts in frames:
        xyz = pts
        if move:
            bx, by = xy[name]
            xyz = pts.copy()
            xyz[:, 0] += bx
            xyz[:, 1] += by
        zp = (np.abs(xyz) == 0).all(axis=1)
        zeros += int(zp.sum())
        tot += int(len(xyz))
        g.add_points(xyz)
        r, c = g._cells(xyz[zp][:, :2])
        if len(r):
            ghost[r, c] = True
        r2, c2 = g._cells(xyz[~zp][:, :2])
        if len(r2):
            real[r2, c2] = True
    cm, _ = mod.cost_map(g, math.radians(SLOPE_DEG), STEP_M, ROUGH_M)
    surface = g.p50()
    live = g.count > 0
    m_sl = live & (g.slope(surface) > math.tan(math.radians(SLOPE_DEG)))
    m_st = live & (g.step(surface) > STEP_M)
    m_rg = live & (g.rms() > ROUGH_M)
    any_m = m_sl | m_st | m_rg
    return {
        "tot": tot, "zeros": zeros, "covered": int(live.sum()), "clipped": g.clipped,
        "ratio": 1.0 - float(live.sum()) / float(g.count.size),
        "vals": sorted(set(int(v) for v in np.unique(cm).tolist())),
        "ghost": int(ghost.sum()), "ghost_only": int((ghost & ~real).sum()),
        "ghost_live_both": int((ghost & real).sum()),
        "slope_only": int((m_sl & ~m_st & ~m_rg).sum()), "step_only": int((m_st & ~m_sl & ~m_rg).sum()),
        "rough_only": int((m_rg & ~m_sl & ~m_st).sum()), "multi": int((any_m & ((m_sl.astype(int) + m_st.astype(int) + m_rg.astype(int)) > 1)).sum()),
        "blocked": int((cm == mod.BLOCKED).sum()), "free": int((cm == mod.FREE).sum()),
        "unknown": int((cm == mod.UNKNOWN).sum()), "ny": g.ny, "nx": g.nx,
    }


def main():
    root = find_root()
    if root is None:
        print("[前置缺失] 找不到同时含 ws/src 与 note/进度 的工作根 ⇒ 不猜路径、退出 9")
        return 9
    xy = load_station_xy(root)
    if not xy:
        print("[前置缺失] R82 `probe_stdout.txt` 不在位或读不出逐站 x,y ⇒ 不猜位姿、退出 9")
        return 9
    frames = load_frames(root)
    if not frames:
        print("[前置缺失] 找不到 R82 原始帧目录 ⇒ 不猜数据、退出 9")
        return 9
    miss = [n for n, _ in frames if n not in xy]
    if miss:
        print("[前置缺失] 有帧无对应站名（%d 帧）＝%s ⇒ 半件数据不跑、退出 9" % (len(miss), ",".join(miss)))
        return 9
    spec = importlib.util.spec_from_file_location("w3_elev", os.path.join(root, NODE))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)

    xs = np.concatenate([p[:, 0] + xy[n][0] for n, p in frames])
    ys = np.concatenate([p[:, 1] + xy[n][1] for n, p in frames])
    window = (math.floor(xs.min() - 1.0), math.ceil(xs.max() + 1.0),
              math.floor(ys.min() - 1.0), math.ceil(ys.max() + 1.0))
    a = run_once(mod, frames, xy, False, window)
    b = run_once(mod, frames, xy, True, window)
    pts = sorted(xy.values())
    pl = sum(math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1]) for i in range(1, len(pts)))

    print("S1_MOVING_GHOST 帧数=%d 站数=%d res=%.2f 窗口=%s 格=%dx%d"
          % (len(frames), len(xy), RES, window, b["ny"], b["nx"]))
    print("  [A 雷达系原样（静止态对照＝R281 形态）] 送入点=%d 占位零点=%d 落格数=%d 覆盖格=%d uncovered_ratio=%.4f 取值集合=%s"
          % (a["tot"], a["zeros"], a["ghost"], a["covered"], a["ratio"], a["vals"]))
    print("  [B yaw=0 仅 (x,y) 平移重投（近似、非实测）] 送入点=%d 占位零点=%d 落格数=%d 覆盖格=%d uncovered_ratio=%.4f 取值集合=%s"
          % (b["tot"], b["zeros"], b["ghost"], b["covered"], b["ratio"], b["vals"]))
    print("  ★主判据：占位零点落格数 A=%d → B=%d（本站数=%d ⇒ 上界＝站数）｜B 中「仅占位点独占」格=%d｜与真点同格=%d"
          % (a["ghost"], b["ghost"], len(xy), b["ghost_only"], b["ghost_live_both"]))
    print("  三通道分因（B 态，分因并报不合成单值）：slope_only=%d step_only=%d rough_only=%d 多因同拦=%d blocked=%d free=%d unknown=%d clipped=%d"
          % (b["slope_only"], b["step_only"], b["rough_only"], b["multi"], b["blocked"], b["free"], b["unknown"], b["clipped"]))
    print("  ★推断（非实测、不外推即不成立）：连续运动下拖尾格数 ≈ 站点折线长／res ＝ %.2f m／%.2f m ≈ %d 格量级；"
          "本件**只测得**上一行的 %d→%d" % (pl, RES, int(round(pl / RES)), a["ghost"], b["ghost"]))

    bad = []
    if (b["tot"], b["zeros"]) != (a["tot"], a["zeros"]):
        bad.append("点数不守恒：A(tot=%d,zeros=%d) ≠ B(tot=%d,zeros=%d)" % (a["tot"], a["zeros"], b["tot"], b["zeros"]))
    if not b["ghost"] > a["ghost"]:
        bad.append("★移动态落格数未增（B=%d ≤ A=%d）⇒ 平移未生效，本件没在测拖尾" % (b["ghost"], a["ghost"]))
    if b["ghost"] > len(xy):
        bad.append("★B 落格数=%d 超过站数=%d ⇒ 占位点不该散到站外，投影逻辑有误" % (b["ghost"], len(xy)))
    for k, r in (("A", a), ("B", b)):
        if set(r["vals"]) - TRISTATE:
            bad.append("%s 违反三值约束（取值集合=%s）" % (k, r["vals"]))
    if b["clipped"]:
        bad.append("B 有越界裁点 clipped=%d ⇒ z_range 不适配本数据集，z 面一律不取" % b["clipped"])
    for t in bad:
        print("  [FAIL] %s" % t)
    print("S1_MOVING_GHOST VERDICT: %s（数量级估算；不构成 S1 已在 map 系验证之证据）"
          % ("rc=0 无违反项" if not bad else "rc=5 有违反项"))
    return 0 if not bad else 5


if __name__ == "__main__":
    sys.exit(main())
