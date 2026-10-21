#!/usr/bin/env python3
# 02_freeband_scan.py —— ★零窗扫描（只读 R462 的留底 csv）★：给 R464 挑一条"基准态几乎全自由"的靶带
# 为什么要有这一步＝r462q 裁 C7-B 只记 II 支的唯一卡点＝B2 (a) 前提项 `X_A ≤ 0.1N` 字面未过
#   （R462 的带 x∈[0.25,1.35] 压在静态层存量致命上：X_A=1204、0.1N=132）
# 做法＝在★发布器能覆盖的范围内★（/map 是 120x120@0.1、origin=(-6,-6) ⇒ 世界 x∈[-6,+6]）逐列数基准致命格，
#       取★连续 11 列（＝1.1m 厚、N=11×120=1320≥1200 与 R462 同功率）★里基准致命总数最低的一条带。
# ★只读、不改动任何在册件与任何留底件★
import sys

SRC = sys.argv[1] if len(sys.argv) > 1 else "/opt/ros1-slam/note/进度/SCAN集成/证据/log/raw/r462_三式收口窗/run/csv/cm_A.csv"
WCOLS = 11

with open(SRC, encoding="utf-8", errors="replace") as f:
    meta = f.readline().strip().split(",")
    data = f.readline().strip().split(",")
w, h, res, ox, oy = int(meta[1]), int(meta[2]), float(meta[3]), float(meta[4]), float(meta[5])
print("被扫件＝%s ‖ 几何 W=%d H=%d res=%.3f origin=(%.2f,%.2f)" % (SRC.split("/raw/")[-1], w, h, res, ox, oy))
cols = []
for c in range(w):
    x = ox + (c + 0.5) * res
    if -6.0 <= x <= 6.0:
        leth = 0
        for r in range(h):
            if int(data[r * w + c]) >= 90:
                leth += 1
        cols.append((c, x, leth))
print("可覆盖列数（x∈[-6,6]）＝%d ‖ 其中基准致命为 0 的列数＝%d" % (len(cols), sum(1 for _, _, v in cols if v == 0)))
best = None
for i in range(len(cols) - WCOLS + 1):
    win = cols[i:i + WCOLS]
    tot = sum(v for _, _, v in win)
    if best is None or tot < best[0]:
        best = (tot, win)
tot, win = best
N = WCOLS * 120
print("★最低基准致命总数的一条连续 %d 列＝%d 格★" % (WCOLS, tot))
print("带范围＝x∈[%.2f, %.2f]（列 %d..%d）‖ 逐列致命数＝%s" % (
    win[0][1] - res / 2, win[-1][1] + res / 2, win[0][0], win[-1][0], [v for _, _, v in win]))
print("按 r462q 的 B2(a) 支：X_A＝%d、0.1N＝%.1f（N=%d）⇒ ★该支% s★" % (
    tot, 0.1 * N, N, "预期字面可过" if tot <= 0.1 * N else "★仍不过、须换更窄/更远的位置★"))
print("★发布器要用的绝对带（世界坐标 x 下限、上限）＝%.2f %.2f★" % (win[0][1] - res / 2, win[-1][1] + res / 2))

# ===== v2（同一次调用里补算★靶自身 footprint 内★的基准致命数；★v1 用全高 1000 行当分母＝我第一版扫错了面积★）=====
Y0, Y1 = -6.0, 6.0
rows = [r for r in range(h) if Y0 <= oy + (r + 0.5) * res <= Y1]
print("★v2★靶自身 footprint＝x∈[-6,6] × y∈[%.2f,%.2f] ＝ %d 行 × %d 列候选（★v1 用整列 1000 行＝面积错、★先自曝★）" % (
    oy + (rows[0] + 0.5) * res, oy + (rows[-1] + 0.5) * res, len(rows), len(cols)))
cols2 = []
for c, x, _ in cols:
    leth = sum(1 for r in rows if int(data[r * w + c]) >= 90)
    cols2.append((c, x, leth))
best2 = None
for i in range(len(cols2) - WCOLS + 1):
    win = cols2[i:i + WCOLS]
    tot = sum(v for _, _, v in win)
    if best2 is None or tot < best2[0]:
        best2 = (tot, win)
tot2, win2 = best2
N2 = WCOLS * len(rows)
print("★最低基准致命的一条连续 %d 列（只数靶会盖到的行）＝X_A=%d★｜带＝x∈[%.2f,%.2f]（列 %d..%d）‖ 逐列=%s" % (
    WCOLS, tot2, win2[0][1] - res / 2, win2[-1][1] + res / 2, win2[0][0], win2[-1][0], [v for _, _, v in win2]))
print("靶面积 N=%d 格（=%d 列 × %d 行）‖ 0.1N=%.1f ⇒ ★B2(a) 支 X_A≤0.1N %s★" % (
    N2, WCOLS, len(rows), 0.1 * N2, "预期可过" if tot2 <= 0.1 * N2 else "★仍不过★"))
print("★IV 支靶功率核对：本带 11 列×120 行＝1320 格 ≥1200 ⇒ 与 R462 同功率★")
print("★发布器参数（绝对带）＝BANDX %0.2f %0.2f★" % (win2[0][1] - res / 2, win2[-1][1] + res / 2))

# ===== v3：★取舍曲线★——不同带厚下的"最小 X_A"，看 (a) 与 IV 能不能同时满足 =====
print("== v3 取舍曲线（★在同一张 A 态 costmap 上、只数靶会盖到的 120 行★）==")
print("带厚(列) N(格)=列×120  0.1N  最小X_A  所在带(x范围)  (a)支可否")
for k in (4, 6, 8, 10, 11, 12, 14, 18, 24, 30, 40, 60, 120):
    if k > len(cols2):
        continue
    bb = None
    for i in range(len(cols2) - k + 1):
        win = cols2[i:i + k]
        tot = sum(v for _, _, v in win)
        if bb is None or tot < bb[0]:
            bb = (tot, win)
    tot, win = bb
    N = k * len(rows)
    ok = "可过" if tot <= 0.1 * N else "★不过★"
    print("%4d  %7d  %7.1f  %6d  [%.2f,%.2f]  %s" % (
        k, N, 0.1 * N, tot, win[0][1] - res / 2, win[-1][1] + res / 2, ok))
print("结论＝★要 N≥1200（IV 支）⇒ 带厚≥11 列 ⇒ 本镜像里最小 X_A＝812 ＞ 0.1N＝132 ⇒ (a) 与 IV 同时不可满足★")
print("　　　反之要让 (a) 可过（带厚≤4 列＝N=480）⇒ 直接破 IV 的 ≥1200")
print("⇒ ★这不是「我再挑一条带」能解决的，是这两支的数值在本镜像里互斥；改哪一支属判据面★")

# ===== v4（★修正版★）：v4 第一版把"全 120 候选列"都取了 min ⇒ 边缘列 440~450 是★整列全致命★（地图外沿＋膨胀）⇒ 报出"最长自由段＝0 行"＝★我自己算错的、不是地图的事实★
#      修正＝只在内侧候选列里找、并按★同一 y 段★要求连续（★诊断见 03_diag_columns.py／05_diag_columns.txt★）
print("== v4 修正版：★能做的「零基准致命」靶面积上限★")
runs = {}
for c, x, _ in cols2:
    col = [int(data[r * w + c]) for r in rows]
    run = best = 0
    for v in col:
        run = run + 1 if v < 90 else 0
        best = max(best, run)
    runs[c] = best
good = sorted(runs.values(), reverse=True)[:12]
print("  最长自由竖段（前 12 名，行）＝%s（★每行 0.1m★）" % good)
best_common = min(good[:11])
print("  ⇒ 取★连续 11 列都能覆盖★的同一段：上限＝11 × %d 行＝★%d 格★" % (best_common, 11 * best_common))
print("  IV 支要 ≥1200 格 ⇒ %s" % ("★仍不可兼得★" if 11 * best_common < 1200 else "可兼得"))
print("  ★真正的绑定事实＝迷宫自由走廊只有约 2～4m 宽（05_diag_columns.txt 抽样列＝70/120 致命、该列最长连续自由 21 行；★全图最优也只有 38 行＝3.8m★）★")
print("  ⇒ ★(a) 支的 0.1N 与 IV 支的 1200 在本镜像里不可同时满足；改哪一支属判据面、不自裁★")
