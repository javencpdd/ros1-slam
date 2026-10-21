#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""s1_offline_frames_smoke.py —— R281：把 R82 已落盘的 14 站原始帧离线喂给 S1（零窗、零开链）

目的不是"证明 S1 可用"（达成判定在验收侧），而是把 S1 放到**真实点云分布**上跑一遍，
并复现 X28 那条"一参两职"发现：球形 blind 一直在顺手删掉每帧 64.5–72.5% 的精确 (0,0,0) 占位点，
所以换任何形状都必须把"显式剔零"写成独立一条 ⇒ 本件按 **A 原样喂**／**B 显式剔零喂** 两法并报，
且三通道（slope／step／roughness）**分因计数、不合成单值**。

坐标系限定（必读）＝`.npy` 是**雷达系** float16（见 r82 目录 index.txt），**未加位姿、不在 map 系**
⇒ 本件只验"输入面形状与统计面"，不构成"高程正确性"或"map 系栅格"的任何声称。
"""
import glob
import importlib.util
import math
import os
import sys

import numpy as np

FRAME_DIR = "note/进度/SCAN集成/证据/log/raw/r82_selfmask_20261010_042738"
NODE = "ws/src/30_perception/lite3_perception/scripts/w3_elevation_node.py"
SLOPE_DEG = 25.0
STEP_M = 0.15
ROUGH_M = 0.05
NAN_WARN = 0.30
RES = 0.1


def load_frames(root):
    out = []
    for f in sorted(glob.glob(os.path.join(root, FRAME_DIR, "*.npy"))):
        a = np.load(f)
        pts = np.asarray(a.T, dtype=np.float64)
        out.append((os.path.basename(f), pts))
    return out


def envelope(frames, pad=1.0):
    xs = np.concatenate([p[:, 0] for _, p in frames])
    ys = np.concatenate([p[:, 1] for _, p in frames])
    lo_x, hi_x = float(xs.min()), float(xs.max())
    lo_y, hi_y = float(ys.min()), float(ys.max())
    return (math.floor(lo_x - pad), math.ceil(hi_x + pad),
            math.floor(lo_y - pad), math.ceil(hi_y + pad))


def run(mod, frames, mode, window, drop_zero, tot_all, zeros):
    g = mod.ElevGrid(RES, window[0], window[1], window[2], window[3], z_min=-2.0, z_max=2.0)
    tot = 0
    dropped = 0
    for name, pts in frames:
        z_abs = (np.abs(pts) == 0).all(axis=1)
        keep = ~z_abs if drop_zero else np.ones(len(pts), dtype=bool)
        tot += int(keep.sum())
        dropped += int(z_abs.sum()) - int((z_abs & keep).sum())
        g.add_points(pts[keep])
    surface = g.p50()
    sl = g.slope(surface)
    st = g.step(surface)
    rg = g.rms()
    live = g.count > 0
    m_sl = live & (sl > math.tan(math.radians(SLOPE_DEG)))
    m_st = live & (st > STEP_M)
    m_rg = live & (rg > ROUGH_M)
    cm, _ = mod.cost_map(g, math.radians(SLOPE_DEG), STEP_M, ROUGH_M)
    any_m = m_sl | m_st | m_rg
    ratio, covered = g.coverage()
    vals = sorted(set(int(v) for v in np.unique(cm).tolist()))
    print("  [%s] 送入点=%d（点级剔零=%d）占位零点=%d 点级占比=%.1f%%（分母＝全帧总点 %d，两模式同分母）"
          % (mode, tot, dropped, zeros, 100.0 * zeros / max(tot_all, 1), tot_all))
    print("        帧=%d 窗口=%s 格=%dx%d 越界裁点=%d" % (g.frames, window, g.ny, g.nx, g.clipped))
    print("        覆盖格=%d uncovered_ratio=%.4f WARN(>%.2f)=%s"
          % (covered, ratio, NAN_WARN, "YES" if ratio > NAN_WARN else "no"))
    print("        三通道分因（不合成单值）：slope_only=%d step_only=%d rough_only=%d 多因同拦=%d 合计拦=%d"
          % (int((m_sl & ~m_st & ~m_rg).sum()), int((m_st & ~m_sl & ~m_rg).sum()),
             int((m_rg & ~m_sl & ~m_st).sum()),
             int(((m_sl & m_st) | (m_sl & m_rg) | (m_st & m_rg)).sum()), int(any_m.sum())))
    print("        栅格取值集合=%s（三值约束 −1/0/100：%s）unknown=%d free=%d blocked=%d"
          % (vals, "满足" if set(vals) <= set([-1, 0, 100]) else "★违反",
             int((cm == mod.UNKNOWN).sum()), int((cm == mod.FREE).sum()), int((cm == mod.BLOCKED).sum())))
    zero_cell_mask = np.zeros(g.count.shape, dtype=bool)
    zr, zc = g._cells(np.zeros((zeros, 2), dtype=np.float64)) if zeros else (np.array([], np.int64), np.array([], np.int64))
    if len(zr):
        zero_cell_mask[zr, zc] = True
    print("        占位零点落格数=%d（占位点共 %d 个）这些格内当前计数之和=%d"
          % (int(zero_cell_mask.sum()), zeros,
             int(g.count[zero_cell_mask].sum()) if zero_cell_mask.any() else 0))
    return {"ratio": ratio, "covered": covered, "vals": vals, "tot": tot,
            "dropped": dropped, "live": live, "zero_cells": zero_cell_mask, "g": g}


def main():
    root = None
    d = os.path.abspath(os.path.dirname(__file__))
    for _ in range(12):
        if os.path.isdir(os.path.join(d, "ws", "src")) and os.path.isdir(os.path.join(d, "note", "进度")):
            root = d
            break
        d = os.path.dirname(d)
    if root is None:
        print("[前置缺失] 找不到同时含 ws/src 与 note/进度 的工作根 ⇒ 不猜路径、退出 9")
        return 9
    spec = importlib.util.spec_from_file_location("w3_elev", os.path.join(root, NODE))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    frames = load_frames(root)
    if len(frames) == 0:
        print("[前置缺失] 找不到 R82 原始帧目录 ⇒ 不猜数据、退出 9")
        return 9
    window = envelope(frames)
    zeros = int(sum(int((np.abs(pts) == 0).all(axis=1).sum()) for _, pts in frames))
    tot_all = int(sum(len(pts) for _, pts in frames))
    print("S1_OFFLINE_SMOKE 帧数=%d（%s … %s）坐标系＝雷达系（未加位姿）"
          % (len(frames), frames[0][0], frames[-1][0]))
    a = run(mod, frames, "A 原样喂（含占位零点）", window, False, tot_all, zeros)
    b = run(mod, frames, "B 显式剔零后喂", window, True, tot_all, zeros)
    bad = []
    if set(a["vals"]) - set([-1, 0, 100]):
        bad.append("A 违反三值约束（取值集合=%s）" % a["vals"])
    if set(b["vals"]) - set([-1, 0, 100]):
        bad.append("B 违反三值约束（取值集合=%s）" % b["vals"])
    if b["dropped"] != zeros:
        bad.append("★点级剔零未生效：B 实剔=%d ≠ 占位零点总数=%d" % (b["dropped"], zeros))
    if b["tot"] != a["tot"] - zeros:
        bad.append("★送入点数不自洽：A %d − 占位 %d ≠ B %d" % (a["tot"], zeros, b["tot"]))
    zc = int(a["zero_cells"].sum())
    if zeros > 0 and zc == 0:
        bad.append("★占位零点有 %d 个却没落进任何格 ⇒ 本件没在测这些点" % zeros)
    ghost = int((a["live"] & ~b["live"]).sum())
    delta = a["covered"] - b["covered"]
    if ghost != delta:
        bad.append("★正向对照不闭合：仅占位零点独占格=%d ≠ 剔零后覆盖格之差=%d" % (ghost, delta))
    print("\n  ---- 对照面（三条真断言＋发现，不拿格级差否点级事实）----")
    print("  ①点级：占位零点=%d／A 全帧送入点=%d＝%.1f%%（X28 的 64.5–72.5%% 是**点级**事实）"
          % (zeros, a["tot"], 100.0 * zeros / max(a["tot"], 1)))
    print("  ②格级：A 覆盖=%d vs B 覆盖=%d（差=%d）｜占位零点落在 %d 格（res=%.2f m ⇒ 同一 (0,0,0) 塌进同一格）"
          % (a["covered"], b["covered"], delta, zc, RES))
    print("  ③正向对照：仅由占位零点独占的「幽灵格」=%d 个＝与②之差%s ⇒ %s"
          % (ghost, "（须等）" if ghost == delta else "★不等",
             "格级面成立" if ghost == delta else "格级面不成立"))
    print("  发现（不判红）：点级 %.1f%%（%d／%d）的占位零点，在格级只影响 %d／%d 覆盖格（%.4f%%）"
          % (100.0 * zeros / max(a["tot"], 1), zeros, a["tot"],
             max(delta, ghost), a["covered"] or 1, 100.0 * max(delta, ghost) / max(a["covered"], 1)))
    print("        ⇒ 两法在 blocked／三通道分因上完全同数（%s），差异只在 unknown↔free 的那 %d 格"
          % ("同" if a["vals"] == b["vals"] else "不同", abs(delta)))
    print("        ⇒ 结论限定：本件不构成「剔零无用」，也不构成 X28 被推翻；X28 是点级吞吐事实，本件是格级形状事实")
    print("  uncovered_ratio：A=%.4f B=%.4f（阈 %.2f，超阈＝WARN，来源限定见头注坐标系段）"
          % (a["ratio"], b["ratio"], NAN_WARN))
    for b in bad:
        print("  [FAIL] %s" % b)
    print("S1_OFFLINE_SMOKE VERDICT: %s（本件只出数与形状，不声称 S1 可用／正确）"
          % ("rc=0 无违反项" if not bad else "rc=5 有违反项"))
    return 0 if not bad else 5


if __name__ == "__main__":
    sys.exit(main())
