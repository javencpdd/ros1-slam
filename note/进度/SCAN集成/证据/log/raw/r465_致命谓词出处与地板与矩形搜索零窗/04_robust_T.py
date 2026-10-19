#!/usr/bin/env python3
# 04_robust_T.py —— 在★同一张 A 态 csv★ 上按「余量」而非「刚好压线」挑活体窗要用的单块矩形 T
#   为什么还要再扫一遍＝03 件里 P100 最优块 n_T=1475、X_A=145、上限 0.1n_T=147.5 ⇒ ★只剩 2.5 格余量★；
#   活体窗的 A 态与留底 A 态不会逐字相同（STVL 时变），压线靶会因两格之差把前提项判成不过 ⇒ 要按余量挑
#   搜索式＝对每个连续列区间 [c0,c1]（宽 W）与每个比率上限 k，求最长连续行段 L 使 sum(rowval) <= k*W*L
#   （＝X_A <= k*n_T；相邻性由「连续列区间 × 连续行区间」的构造保证）
# 用法：04_robust_T.py <cm_A.csv>
import sys

K_LIST = [0.0, 0.02, 0.05, 0.10]
MIN_AREA = 1200


def load(p):
    with open(p, encoding="utf-8", errors="replace") as f:
        meta = f.readline().strip().split(",")
        data = f.readline().strip().split(",")
    w, h, res, ox, oy = int(meta[1]), int(meta[2]), float(meta[3]), float(meta[4]), float(meta[5])

    OXREF = -50.0

    def key(v):
        return int(round((v - OXREF) / res - 0.5))
    cells = {}
    for r in range(h):
        cy = key(oy + (r + 0.5) * res)
        base = r * w
        for c in range(w):
            cells[(key(ox + (c + 0.5) * res), cy)] = int(data[base + c])
    return cells, res


A, res = load(sys.argv[1])
ctr = lambda k: -50.0 + (k + 0.5) * res
xs = sorted(set(k[0] for k in A if -6 <= ctr(k[0]) <= 6))
ys = sorted(set(k[1] for k in A if -6 <= ctr(k[1]) <= 6))
NX, NY = len(xs), len(ys)
print("footprint 格点＝%d 列 × %d 行（列 %d..%d、行 %d..%d）" % (NX, NY, xs[0], xs[-1], ys[0], ys[-1]))


def longest(rowval, W, k):
    n = len(rowval)
    P_ = [0.0] * (n + 1)
    for i in range(n):
        P_[i + 1] = P_[i] + rowval[i] - k * W
    stack = []
    for i in range(n + 1):
        if not stack or P_[i] > P_[stack[-1]]:
            stack.append(i)
    best, bi = 0, -1
    for j in range(n, -1, -1):
        while stack and P_[stack[-1]] >= P_[j] - 1e-9:
            i = stack.pop()
            if j - i > best:
                best, bi = j - i, i
    return best, bi


for pname, pred in (("P100 严格致命 v==100", lambda v: v == 100),
                    ("P90  旧扫描 v>=90", lambda v: v >= 90)):
    print("\n== %s ==" % pname)
    grid = [[1 if pred(A[(xs[c], ys[r])]) else 0 for c in range(NX)] for r in range(NY)]
    for k in K_LIST:
        best = None
        for c0 in range(NX):
            acc = [0] * NY
            for c1 in range(c0, NX):
                for r in range(NY):
                    acc[r] += grid[r][c1]
                W = c1 - c0 + 1
                L, r0 = longest(acc, W, k)
                if L and (best is None or W * L > best[0]):
                    best = (W * L, c0, c1, r0, r0 + L - 1)
        if best is None:
            print("  k=%.2f：无解" % k)
            continue
        area, c0, c1, r0, r1 = best
        bad = sum(sum(grid[r][c0:c1 + 1]) for r in range(r0, r1 + 1))
        print("  上限 X_A<=%.2f*n_T ⇒ 最大单块＝%d 格（%d×%d）‖ 列 %d..%d × 行 %d..%d ‖ 世界 x∈[%.2f,%.2f] y∈[%.2f,%.2f]"
              % (k, area, c1 - c0 + 1, r1 - r0 + 1, xs[c0], xs[c1], ys[r0], ys[r1],
                 ctr(xs[c0]), ctr(xs[c1]), ctr(ys[r0]), ctr(ys[r1])))
        print("     直接重数：n_T=%d、X_A=%d、k*n_T=%.1f、余量=%.1f 格、IV 支(>=1200)=%s"
              % (area, bad, k * area, k * area - bad, "★成立★" if area >= MIN_AREA else "不成立"))

print("\n== 面积>=1200 时★余量最大★的块（谓词 P100、k 固定 0.1）==")
grid = [[1 if A[(xs[c], ys[r])] == 100 else 0 for c in range(NX)] for r in range(NY)]
best = None
for c0 in range(NX):
    acc = [0] * NY
    for c1 in range(c0, NX):
        for r in range(NY):
            acc[r] += grid[r][c1]
        W = c1 - c0 + 1
bests = []
for c0 in range(NX):
    acc = [0] * NY
    for c1 in range(c0, NX):
        for r in range(NY):
            acc[r] += grid[r][c1]
        W = c1 - c0 + 1
        L, r0 = longest(acc, W, 0.10)
        if L and W * L >= MIN_AREA:
            area = W * L
            bad = sum(sum(grid[r][c0:c1 + 1]) for r in range(r0, r0 + L))
            bests.append((0.1 * area - bad, area, bad, c0, c1, r0, r0 + L - 1))
bests.sort(reverse=True)
print("  合规且面积>=1200 的块数＝%d；余量前 6 名：" % len(bests))
for slack, area, bad, c0, c1, r0, r1 in bests[:6]:
    print("   余量 %6.1f 格 ‖ n_T=%5d（%d×%d）‖ X_A=%4d ‖ 列 %d..%d × 行 %d..%d ‖ 世界 x∈[%.2f,%.2f] y∈[%.2f,%.2f]" % (
        slack, area, c1 - c0 + 1, r1 - r0 + 1, bad, xs[c0], xs[c1], ys[r0], ys[r1],
        ctr(xs[c0]), ctr(xs[c1]), ctr(ys[r0]), ctr(ys[r1])))
