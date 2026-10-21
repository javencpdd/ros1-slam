#!/usr/bin/env python3
# 06_neighbour_stat.py —— 值域与邻域统计（零窗、只读留底 csv）
#   目的＝把两件事做成可原样重跑的读数：
#     ① 发布侧网格在本镜像★只有 {0, 99, 100} 三个值★ ⇒ 我取样器里那枚 `v >= 90` 实际选中的是 {99,100} 两值合集、
#        与「严格致命＝100」不是同一个集合（差 6400 格＝A 态的 99 那一圈）
#     ② 99 这一圈★全部★贴着 100（切比雪夫距离<=2）⇒ 它与内部膨胀／内切标记同处、不是独立分布的占用格
#   用法：06_neighbour_stat.py <cm_*.csv>
import sys

p = sys.argv[1]
with open(p, encoding="utf-8", errors="replace") as f:
    meta = f.readline().strip().split(",")
    data = f.readline().strip().split(",")
w, h = int(meta[1]), int(meta[2])
flat = [int(v) for v in data]
print("件＝%s ‖ 尺寸=%dx%d" % (p.split("/")[-1], w, h))
print("值域 distinct＝%s" % sorted(set(flat)))
print("值为 100 的格数＝%d ‖ 值为 99 的格数＝%d ‖ 值为 0 的格数＝%d ‖ 落在 1..98 的格数＝%d" % (
    sum(1 for v in flat if v == 100), sum(1 for v in flat if v == 99),
    sum(1 for v in flat if v == 0), sum(1 for v in flat if 1 <= v <= 98)))
print("（★>=90 与 ==100 两个集合的差＝99 那圈＝%d 格；这就是本批要说清的谓词差★）" % sum(1 for v in flat if v == 99))


def near(idx, K):
    r, c = divmod(idx, w)
    for dr in range(-K, K + 1):
        for dc in range(-K, K + 1):
            if dr == 0 and dc == 0:
                continue
            rr, cc = r + dr, c + dc
            if 0 <= rr < h and 0 <= cc < w and flat[rr * w + cc] == 100:
                return True
    return False


i99 = [i for i, v in enumerate(flat) if v == 99]
print("★99 这一圈贴着 100 的厚度★（分母＝全部 99 格）")
for K in (1, 2, 3):
    print("  距任一 100 格切比雪夫距离<=%d 的＝%d / %d" % (K, sum(1 for i in i99 if near(i, K)), len(i99)))
