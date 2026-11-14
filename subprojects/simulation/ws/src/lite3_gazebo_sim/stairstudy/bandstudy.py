#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""stairstudy/bandstudy.py —— B6 出路的反事实预检：把竖向样本压进窄带，买到什么、花掉什么

为何存在：R65 的结论是「在册仿真图案的可判距离窗碎且非单调 ⇒ 现图案下写不出可复现的可上楼梯判据」，
而 B6 的两条出路里，①「把竖向 50 线摊到 ±20–25° 带」一直被当成免费午餐在传话。
本件把它**量化**：同一批实体、同一套特征定义（复用 precheck，绝不另立一套），只改竖向覆盖带宽度，
看 ① 是否真换来单连可判窗；并把它**必然**引入的代价算出来——窄带意味着不再有线打到机体脚下的地面，
于是「梯脚与地面的交界」这条参照线消失（几何上就是 d_ground＝安装高/tan(最低线俯角)）。

性质：纯离线。零 ROS 节点、零在册端口、不改 xacro、不改任何门值；
**fixtures.py / precheck.py / sweep.py 一字未动**（R64/R65 已归档读数因此不作废，本件是新增旁路）。

调用姿势自证：`python3 stairstudy/bandstudy.py`（零 ROS 依赖，不需 env 三步；约 3–4 分钟）。
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import fixtures as FX                                       # noqa: E402
import precheck as PC                                       # noqa: E402

NAMES = ("stairs_mid", "stairs_easy", "stairs_steep", "wall_h080", "wall_h045")
R = 0.05
AZSTEP = 4.0            # 四臂同用（90 方位 × 50 线＝4500 根/扫描），比较面公平；跑批 6 分钟级
DS = (0.4, 0.6, 0.8, 1.0, 1.2, 1.5, 2.0, 2.5)
PAIRS = (("stairs_mid", "wall_h080"), ("stairs_easy", "wall_h045"), ("stairs_steep", "wall_h080"))
MOUNT = FX.MID360_MOUNT_Z                                  # 0.38m，在册 xacro 实测

# 四臂：线数与方位密度全同，只有竖向带宽不同 ⇒ 配对差只可归因于"带有多窄"（而不是通量）
ARMS = [
    ("band90",  50, 90.0, "在册现状：50 线摊 ±90°（间隔 3.67°，楼梯立面带内只剩约 8 条）"),
    ("band45",  50, 45.0, "中间档：50 线摊 ±45°（间隔 1.84°）"),
    ("band25",  50, 25.0, "B6 出路①的字面版：50 线压进 ±25°（间隔 1.02°）"),
    ("band16",  50, 15.5, "极限档：50 线压进 ±15.5°（对齐真机 32 线 FOV）"),
]


def cloud_band(fx, ox, nlines, half_deg, oz=MOUNT, azstep=AZSTEP):
    els = [-half_deg + i * (2.0 * half_deg / (nlines - 1)) for i in range(nlines)]
    pts = []
    a = 0.0
    while a < 360.0:
        for e in els:
            hit = FX.cast_rays(fx, ox, 0.0, oz, a, e, max_range=8.0)
            if hit:
                pts.append(hit)
        a += azstep
    return pts, els


def lines_in_face_band(els, lo=-15.5, hi=15.5):
    return sum(1 for e in els if lo <= e <= hi)


def main():
    fxmap = dict(FX.scenarios())
    feat, dens, inband, dg = {}, {}, {}, {}
    for tag, nl, half, _ in ARMS:
        pts, els = None, None
        for nm in NAMES:
            for d in DS:
                pts, els = cloud_band(fxmap[nm], -d, nl, half)
                g, h = PC.grid(pts, fxmap[nm], R)
                feat[(tag, nm, d)] = PC.feats(g, h, fxmap[nm], R)
                dens[(tag, nm, d)] = len(pts)
        inband[tag] = lines_in_face_band(els)
        tg = math.tan(math.radians(half))
        dg[tag] = MOUNT / tg if tg > 0 else 0.0
        print("[%s] 竖向 %d 线 ±%.1f° ⇒ 落在楼梯立面 ±15.5° 带内 **%d 条**；"
              "地面盲区半径 d_ground=%.2fm" % (tag, nl, half, inband[tag], dg[tag]))

    print("\n=== ①结构面铺开程度 surf（越大＝越「看得清是楼梯」）===")
    print("  %-8s %s" % ("距离", "".join("%-14s" % t for t, _, _, _ in ARMS)))
    for nm in ("stairs_mid", "stairs_easy", "stairs_steep"):
        print("  实体 %s" % nm)
        for d in DS:
            print("   %.1fm  " % d + "".join("%-14s" % ("%.2f" % feat[(t, nm, d)]["surf_frac"])
                                             for t, _, _, _ in ARMS))

    print("\n=== ②最长连续有面 run_len（m）——判据的承重特征 ===")
    for nm in ("stairs_mid", "stairs_easy", "stairs_steep"):
        print("  %s" % nm)
        for d in DS:
            print("   %.1fm  " % d + "".join("%-14s" % ("%.2f" % feat[(t, nm, d)]["run_len_m"])
                                             for t, _, _, _ in ARMS))

    def window(tag, a, b):
        ok, row = [], []
        for d in DS:
            fa, fb = feat[(tag, a, d)], feat[(tag, b, d)]
            if min(fa["ret_frac"], fb["ret_frac"]) < 0.05:
                row.append("%.1fn/a" % d); ok.append(False); continue
            gp = max(PC.relgap(fa, fb).values())
            row.append("%.1f%s(%.2f)" % (d, "+" if gp >= PC.SEP else "-", gp))
            ok.append(gp >= PC.SEP)
        wins, i = [], 0
        while i < len(ok):
            if ok[i]:
                j = i
                while j + 1 < len(ok) and ok[j + 1]:
                    j += 1
                wins.append("%.1f–%.1fm" % (DS[i], DS[j])); i = j + 1
            else:
                i += 1
        return wins, row

    print("\n=== ③可判距离窗（gap≥%.2f 的连续段；碎＝非单调）===" % PC.SEP)
    for a, b in PAIRS:
        for tag, _, _, _ in ARMS:
            wins, row = window(tag, a, b)
            print("  %s↔%s @%-7s 窗=%-26s｜逐距 %s"
                  % (a, b, tag, "、".join(wins) if wins else "（无）", " ".join(row)))

    print("\n=== ④代价面：窄带把「脚下」让出去了 ===")
    for tag, nl, half, _ in ARMS:
        near = [feat[(tag, "stairs_mid", d)]["run_len_m"] for d in (0.4, 0.6)]
        print("  %-7s d_ground=%.2fm ⇒ 站距＜%.2fm 时机体脚下无点（梯脚与地面交界这条参照线丢失）；"
              "该两距的 run_len=%.2f/%.2f" % (tag, dg[tag], dg[tag], near[0], near[1]))

    print("\n=== 自检 ===")
    bad = 0

    def ck(name, cond, detail):
        nonlocal bad
        print("[%s] %s ｜ %s" % ("PASS" if cond else "FAIL", name, detail))
        if not cond:
            bad += 1

    ck("S0 四臂齐全", all((t, nm, d) in feat for t, _, _, _ in ARMS
                          for nm in NAMES for d in DS), "格数=%d（应 %d）" % (len(feat), len(ARMS) * len(NAMES) * len(DS)))
    ck("S1 带内线数随带宽单调（密度自证）",
       inband["band90"] < inband["band45"] < inband["band25"] <= inband["band16"],
       "立面 ±15.5° 带内线数：%s" % inband)
    gain = feat[("band25", "stairs_mid", 1.0)]["surf_frac"] - feat[("band90", "stairs_mid", 1.0)]["surf_frac"]
    ck("S2 窄带确实换来结构面（阳性对照，若不成立则 B6①是空话）", gain > 0,
       "stairs_mid @1.0m surf：band90=%.2f → band25=%.2f（差 %+.2f）"
       % (feat[("band90", "stairs_mid", 1.0)]["surf_frac"],
          feat[("band25", "stairs_mid", 1.0)]["surf_frac"], gain))
    w90, _ = window("band90", "stairs_mid", "wall_h080")
    w25, _ = window("band25", "stairs_mid", "wall_h080")
    print("     [对照] band90 窗=%s｜band25 窗=%s" % ("、".join(w90) or "（无）", "、".join(w25) or "（无）"))
    ck("S3 代价面成立（窄带必产生地面盲区）",
       dg["band25"] > dg["band90"], "d_ground：band90=%.2fm（无盲区）→ band25=%.2fm" % (dg["band90"], dg["band25"]))
    ck("S4 命中点数四臂同量级（差＜2 倍才可归因带宽）",
       max(dens[("band25", "stairs_mid", 1.5)], dens[("band90", "stairs_mid", 1.5)])
       < 2 * max(1, min(dens[("band25", "stairs_mid", 1.5)], dens[("band90", "stairs_mid", 1.5)])),
       "stairs_mid@1.5m 命中点：band90=%d band25=%d"
       % (dens[("band90", "stairs_mid", 1.5)], dens[("band25", "stairs_mid", 1.5)]))
    print("BANDSTUDY SELFTEST: FAIL=%d" % bad)
    return 0 if bad == 0 else 5


if __name__ == "__main__":
    sys.exit(main())
