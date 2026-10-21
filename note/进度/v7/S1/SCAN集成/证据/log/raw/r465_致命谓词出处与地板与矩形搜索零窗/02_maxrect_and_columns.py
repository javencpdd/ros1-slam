#!/usr/bin/env python3
# 02_maxrect_and_columns.py —— 零窗（只读 R462 留底 csv）：
#   ① 把「致命」两套谓词的出处与差异摆平：P100＝v==100（发布侧严格致命）／P90＝v>=90（我取样器自写的旧 cutoff，本镜像里恰等于 {99,100}）
#   ② 在发布器 footprint（世界 x,y∈[-6,6]）内搜★单块矩形★ T，使 C7-B(a-R) 的前提项 X_A<=0.1*n_T 与 IV 支 n_T>=1200 同时成立，
#      谓词与尺寸全部★按世界格格点★取，相邻性由「连续列区间 × 连续行区间」这个构造本身保证（05 追加五十七①）
#   ③ 对搜到的 T 与 R462 的旧靶带，各算 U／V／皆致命／U_f／V_f／X_A／X_C／D／D_floor（两套谓词并列）
# 用法：02_maxrect_and_columns.py <cm_A.csv> <cm_Ap.csv> <cm_C.csv>
import sys

MIN_AREA = 1200


def load(p):
    with open(p, encoding="utf-8", errors="replace") as f:
        meta = f.readline().strip().split(",")
        data = f.readline().strip().split(",")
    w, h, res, ox, oy = int(meta[1]), int(meta[2]), float(meta[3]), float(meta[4]), float(meta[5])
    # ★世界格点键＝以 A 的 origin 为锚的列／行号★（OXREF=-50）：格心 = OXREF+(k+0.5)*res
    # v1 这里写的是 key=round((v-res/2)/res)、筛选又用 (k-0.5)*res ⇒ 整域平移一格（0.1m）、报出的世界范围偏低 0.1
    # v2 修＝键与格心用同一个式子；★自证＝修后 footprint 内 P90 总数应等于直接按 A 列号数出来的 12060★
    OXREF = -50.0

    def key(v):
        return int(round((v - OXREF) / res - 0.5))
    cells = {}
    for r in range(h):
        cy = key(oy + (r + 0.5) * res)
        for c in range(w):
            cx = key(ox + (c + 0.5) * res)
            cells[(cx, cy)] = int(data[r * w + c])
    return cells, res, ox, oy, w, h


A, res, ox, oy, w, h = load(sys.argv[1])
Ap, _, _, _, _, _ = load(sys.argv[2])
C, _, _, _, _, _ = load(sys.argv[3])

print("三份件几何：A origin=(%.2f,%.2f)  Ap origin 见下  C origin 见下；res=%.3f  W=%d H=%d" % (ox, oy, res, w, h))
for nm, d in (("Ap", Ap), ("C", C)):
    ks = sorted(k for k in d)
    print("  %s 的格点键范围＝x 格 %d..%d、y 格 %d..%d（与 A 同键者 %d/%d 格）" % (
        nm, ks[0][0], ks[-1][0], min(k[1] for k in ks), max(k[1] for k in ks), sum(1 for k in d if k in A), len(d)))
    print("  %s 值域 distinct＝%s" % (nm, sorted(set(d.values()))))

LO, HI = -6, 6
ctr = lambda k: -50.0 + (k + 0.5) * res
xs = sorted(set(k[0] for k in A if LO <= ctr(k[0]) <= HI))
ys = sorted(set(k[1] for k in A if LO <= ctr(k[1]) <= HI))
print("\n发布器 footprint 世界 x∈[%d,%d]、y∈[%d,%d] ⇒ 落到的格点＝%d 列 × %d 行＝%d 格（列 %d..%d、行 %d..%d）" % (
    LO, HI, LO, HI, len(xs), len(ys), len(xs) * len(ys), xs[0], xs[-1], ys[0], ys[-1]))
ci = {x: i for i, x in enumerate(xs)}
ri = {y: i for i, y in enumerate(ys)}
P = {"P100 严格致命 v==100": lambda v: v == 100,
     "P90  旧扫描 v>=90（＝{99,100}）": lambda v: v >= 90}


def longest_ok(rowval, W):
    """rowval[r]＝该行在给定列区间内的坏格数；返回★满足 总坏数<=0.1*面积（＝sum(rowval)<=0.1*W*L）★的最长连续行段 (len, r0)
    形＝最长子段使前缀和满足 P[j] - P[i] <= 0 ⇔ P[i] >= P[j] ⇒ 候选 i 取★前缀和记录最大值★序列
    ★v1 这里取的是记录最小值、比较方向也反了 ⇒ 只找到 1×120＝面积上限严重偏低＝本件自曝的第一笔（对照件侧假读数，非活体读数）★"""
    n = len(rowval)
    P_ = [0.0] * (n + 1)
    for i in range(n):
        P_[i + 1] = P_[i] + rowval[i] - 0.1 * W
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


for pname, pred in P.items():
    print("\n== 谓词＝%s ==" % pname)
    tot_fp = sum(1 for x in xs for y in ys if pred(A[(x, y)]))
    tot_all = sum(1 for k in A if pred(A[k]))
    print("  A 态该类格总数＝footprint 内 %d／14400（%.1f%%）‖ 全图 %d／1000000" % (
        tot_fp, 100.0 * tot_fp / (len(xs) * len(ys)), tot_all))
    best = (0, None)
    for c0 in range(len(xs)):
        acc = [0] * len(ys)
        for c1 in range(c0, len(xs)):
            for r in range(len(ys)):
                acc[r] += 1 if pred(A[(xs[c1], ys[r])]) else 0
            W = c1 - c0 + 1
            L, r0 = longest_ok(acc, W)
            if L and W * L > best[0]:
                best = (W * L, (c0, c1, r0, r0 + L - 1))
            if W * len(ys) < MIN_AREA:
                continue
    area, (c0, c1, r0, r1) = best
    x0, x1, y0, y1 = xs[c0], xs[c1], ys[r0], ys[r1]
    bad = sum(1 for i in range(c0, c1 + 1) for j in range(r0, r1 + 1) if pred(A[(xs[i], ys[j])]))
    print("  ★满足前提项 X_A<=0.1*n_T 的最大单块矩形★＝%d 格＝(%d 列 × %d 行)" % (area, c1 - c0 + 1, r1 - r0 + 1))
    print("    格点范围＝列 %d..%d × 行 %d..%d ⇒ 世界★格心覆盖★ x∈[%.2f,%.2f] y∈[%.2f,%.2f]（外沿 %.2f/%.2f）" % (
        x0, x1, y0, y1, ctr(x0), ctr(x1), ctr(y0), ctr(y1), ctr(x0) - res / 2, ctr(x1) + res / 2))
    print("    复核（直接重数）：n_T＝%d、X_A＝%d、0.1*n_T＝%.1f ⇒ 前提项%s；IV 支 n_T>=1200 %s" % (
        area, bad, 0.1 * area, "★成立★" if bad <= 0.1 * area else "不成立",
        "★成立★" if area >= MIN_AREA else "不成立"))
    print("    该 T 内：Ap 该类格＝%d、C 该类格＝%d" % (
        sum(1 for i in range(c0, c1 + 1) for j in range(r0, r1 + 1) if pred(Ap[(xs[i], ys[j])])),
        sum(1 for i in range(c0, c1 + 1) for j in range(r0, r1 + 1) if pred(C[(xs[i], ys[j])]))))


def columns(name, T, PA=None, PB=None):
    """T＝(x 格区间, y 格区间)；PA＝基准态谓词源 dict、PB＝对照态；输出七栏（对给定谓词 pred）"""
    out = {}
    for pname, pred in P.items():
        ks = [(x, y) for x in T[0] for y in T[1]]
        up = sum(1 for k in ks if not pred(A[k]) and pred(C[k]))
        dn = sum(1 for k in ks if pred(A[k]) and not pred(C[k]))
        both = sum(1 for k in ks if pred(A[k]) and pred(C[k]))
        uf = sum(1 for k in ks if not pred(A[k]) and pred(Ap[k]))
        vf = sum(1 for k in ks if pred(A[k]) and not pred(Ap[k]))
        xa = sum(1 for k in ks if pred(A[k]))
        xc = sum(1 for k in ks if pred(C[k]))
        out[pname] = (len(ks), xa, xc, up, dn, both, uf, vf)
    print("  ── 靶块＝%s（n_T＝%d）" % (name, len([(x, y) for x in T[0] for y in T[1]])))
    print("     谓词                    n_T    X_A    X_C      U      V   皆致命    U_f    V_f")
    for pname, v in out.items():
        print("     %-24s %5d %6d %6d %6d %6d %7d %6d %6d" % (pname.split()[0], *v))
    return out


print("\n== 七栏并列（★同一张 csv、只换谓词★）==")
best100 = None
for pname, pred in P.items():
    if not pname.startswith("P100"):
        continue
    for c0 in range(len(xs)):
        acc = [0] * len(ys)
        for c1 in range(c0, len(xs)):
            for r in range(len(ys)):
                acc[r] += 1 if pred(A[(xs[c1], ys[r])]) else 0
            L, r0 = longest_ok(acc, c1 - c0 + 1)
            if L and (best100 is None or (c1 - c0 + 1) * L > best100[0]):
                best100 = ((c1 - c0 + 1) * L, xs[c0:c1 + 1], ys[r0:r0 + L])
area100, TX, TY = best100
columns("P100 搜到的最大合规单块 T", (TX, TY))
BAND = [x for x in xs if 0.25 <= ctr(x) <= 1.35]
columns("R462 的旧靶带 x∈[0.25,1.35]（全高 120 行）", (BAND, ys))

print("\n== D 与 D_floor（★全图逐格、与谓词无关★）==")
shared = set(A) & set(Ap)
d_floor_full = sum(1 for k in shared if A[k] != Ap[k])
d_ac_full = sum(1 for k in (set(A) & set(C)) if A[k] != C[k])
print("  D_floor（A↔Ap 全图逐格值不等）＝%d 格 ‖ D（A↔C 全图）＝%d 格（在册 r462q 报的 601／1300 对得上＝%s）" % (
    d_floor_full, d_ac_full, (d_floor_full == 601, d_ac_full == 1300)))
print("  注：R462 的 601／1300 是★交集格★上的逐格值不等数、与谓词无关 ⇒ 本栏只是复算自证，不是新数")
