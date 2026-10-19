#!/usr/bin/env python3
# 03_diag_columns.py —— ★诊断（本批 v4 那个"最长自由段＝0 行"读数可疑，先查自己的代码再下结论★）
import sys

f = sys.argv[1] if len(sys.argv) > 1 else "/opt/ros1-slam/note/进度/SCAN集成/证据/log/raw/r462_三式收口窗/run/csv/cm_A.csv"
with open(f, encoding="utf-8", errors="replace") as fh:
    meta = fh.readline().strip().split(",")
    data = fh.readline().strip().split(",")
w, h, res, ox, oy = int(meta[1]), int(meta[2]), float(meta[3]), float(meta[4]), float(meta[5])
rows = [r for r in range(h) if -6.0 <= oy + (r + 0.5) * res <= 6.0]
print("rows=%d（y∈[%.2f,%.2f]）" % (len(rows), oy + (rows[0] + 0.5) * res, oy + (rows[-1] + 0.5) * res))
print("列  世界x   致命  自由  最长连续自由(行)  该段 y 范围")
for c in (440, 441, 445, 450, 453, 455, 460, 463, 500, 520, 559):
    col = [int(data[r * w + c]) for r in rows]
    leth = sum(1 for v in col if v >= 90)
    run = best = bstart = start = 0
    for i, v in enumerate(col):
        if v < 90:
            if run == 0:
                start = i
            run += 1
            if run > best:
                best, bstart = run, start
        else:
            run = 0
    y0 = oy + (rows[bstart] + 0.5) * res if best else 0.0
    y1 = oy + (rows[bstart + best - 1] + 0.5) * res if best else 0.0
    print("%3d %7.2f %5d %5d %10d  y∈[%.2f,%.2f]" % (c, ox + (c + 0.5) * res, leth, len(col) - leth, best, y0, y1))
