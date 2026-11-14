#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""stairstudy/sweep.py —— 「在第几米停下来才看得清」：可感知距离窗离线扫描（H 稿 §4 的补全）

为何存在：precheck.py 只在 d=1.5 一个姿态上给分辨力，另加一个对照臂的解析距离带（0.36m 宽）。
判据要落地必须先知道**距离窗在哪**——太近会被整锥越过梯脚看不见，太远则竖向线摊不开、
立面只剩一两列。本件把距离当自变量扫一遍，产出可判距离窗（上下界）与失配方向。
性质：纯离线，零 ROS 节点、零在册端口、不改任何门值与在册件。

调用姿势自证：`python3 stairstudy/sweep.py`（零 ROS 依赖，不需 env 三步；复用 precheck 的同一套
features/points/grid，故两件的读数口径必然同源——不自立第二套定义，02 坑43）。
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import fixtures as FX                                       # noqa: E402
import precheck as PC                                       # noqa: E731,E402

DS = (0.4, 0.6, 0.8, 1.0, 1.2, 1.5, 2.0, 2.5)
ARMS = ("mid360", "hdl32")
R = 0.05
AZ = 2.0            # 两臂同用 2° 方位步（360 个方位那档太慢）；命中点数逐行打出以保证比较公平
PAIRS = (("stairs_mid", "wall_h080"), ("stairs_easy", "wall_h045"), ("stairs_steep", "wall_h080"))
NAMES = ("stairs_easy", "stairs_mid", "stairs_steep", "wall_h080", "wall_h045", "ramp_envelope")


def main():
    fxmap = dict(FX.scenarios())
    feat = {}
    dens = {}
    for arm in ARMS:
        for nm in NAMES:
            for d in DS:
                pts = PC.points(arm, fxmap[nm], -d, AZ)
                dens[(arm, nm, d)] = len(pts)
                g, h = PC.grid(pts, fxmap[nm], R)
                feat[(arm, nm, d)] = PC.feats(g, h, fxmap[nm], R)

    print("=== 可感知距离窗（r=%.2f，方位步 %.1f°；surf＝有实体面列比，run＝最长连续有面，"
          "gap＝与配对矮墙的最大相对差）===" % (R, AZ))
    for arm in ARMS:
        print("\n--- 臂 %s %s ---" % (arm, "（在册仿真近似）" if arm == "mid360" else "（真机 32 线口径对照）"))
        print("  d      " + "".join("%-44s" % nm for nm in NAMES))
        for d in DS:
            cells = []
            for nm in NAMES:
                f = feat[(arm, nm, d)]
                cells.append("surf%.2f run%.2f fc%d slope%.0f hs%.3f n%d"
                             % (f["surf_frac"], f["run_len_m"], f["faces"], f["slope_deg"],
                                f["h_spread"], dens[(arm, nm, d)]))
            print("  %.1fm  " % d + "".join("%-44s" % c for c in cells))

    print("\n=== 与矮墙配对的可分性随距离的变化（gap≥%.2f 记可分）===" % PC.SEP)
    for arm in ARMS:
        for a, b in PAIRS:
            row = []
            ok = []
            for d in DS:
                fa, fb = feat[(arm, a, d)], feat[(arm, b, d)]
                if min(fa["ret_frac"], fb["ret_frac"]) < 0.05:
                    row.append(" %.1fn/a" % d)
                    ok.append(False)
                    continue
                gp = max(PC.relgap(fa, fb).values())
                row.append(" %.1f%s(%.2f)" % (d, "+" if gp >= PC.SEP else "-", gp))
                ok.append(gp >= PC.SEP)
            wins = []
            i = 0
            while i < len(ok):
                if ok[i]:
                    j = i
                    while j + 1 < len(ok) and ok[j + 1]:
                        j += 1
                    wins.append("%.1f–%.1fm" % (DS[i], DS[j]))
                    i = j + 1
                else:
                    i += 1
            print("  %-7s %s↔%s：%s｜可判窗=%s"
                  % (arm, a, b, "".join(row), "、".join(wins) if wins else "（本扫描内无可判窗）"))

    print("\n=== 误拒方向检查：楼梯在哪些距离上读起来像「立面没铺开」（run<0.20m ⇒ 会被当墙/不当楼梯）===")
    for arm in ARMS:
        for nm in ("stairs_easy", "stairs_mid", "stairs_steep"):
            miss2 = ["%.1f" % d for d in DS if feat[(arm, nm, d)]["run_len_m"] < 0.20]
            print("  %-7s %-12s run<0.20m 的距离=%s"
                  % (arm, nm, "、".join(miss2) if miss2 else "（无）"))

    print("\n=== 自检（不起新射线，只核已算读数）===")
    bad = 0

    def ck(name, cond, detail):
        nonlocal bad
        print("[%s] %s ｜ %s" % ("PASS" if cond else "FAIL", name, detail))
        if not cond:
            bad += 1

    miss = [(arm, nm, d) for arm in ARMS for nm in NAMES for d in DS if (arm, nm, d) not in feat]
    ck("S0 读数齐全", not miss, "格数=%d/%d，缺 %d" % (len(feat), len(ARMS) * len(NAMES) * len(DS), len(miss)))

    near = [feat[("hdl32", "stairs_mid", d)]["slope_deg"] for d in (0.4, 0.6, 0.8, 1.0, 1.2)]
    far = [feat[("hdl32", "stairs_mid", d)]["slope_deg"] for d in (2.0, 2.5)]
    ck("S1 可判窗的存在性（坡度读数在窗内≥10°、窗外＜10°）",
       all(v >= 10.0 for v in near) and all(v < 10.0 for v in far),
       "对照臂 stairs_mid 坡度 0.4–1.2m=%s，2.0–2.5m=%s（真值 19.7°）"
       % (["%.0f" % v for v in near], ["%.0f" % v for v in far]))

    thin = [(arm, b, d) for arm in ARMS for b in ("wall_h080", "wall_h045") for d in DS
            if feat[(arm, b, d)]["run_len_m"] > 0.10]
    ck("S2 薄墙在所有距离都读成薄面（公共 x 窗生效）", not thin,
       "违规格=%d %s" % (len(thin), thin[:3]))
    print("SWEEP SELFTEST: FAIL=%d" % bad)
    return 0 if bad == 0 else 5


if __name__ == "__main__":
    sys.exit(main())
