#!/usr/bin/env python3
# _cmp_r467.py —— ★首次按「C7-B 内容栏『致命』谓词钉」下的对照件★（★只读留底、不起栈★）
#   硬规矩（R465q 立）＝①凡发布侧计数★带谓词号★ ②★行内谓词一致★（P100 行只用 P100 栏、P90 行只用 P90 栏）
#                 ③判词★分行并列★、★不写不带谓词号的「(a-R) 成立／不成立」 ④只报一套谓词＝该窗八栏按缺一取样点计 INVALID
#   被试体具名三项（R465q Q3 新增形）＝绝对列／行区间 ‖ n_T ‖ 与各态的★索引平移量★（由 origin 现推、不假设）
#   派生栏标注＝皆致命 ≡ X_A − V（V=0 时不增一格证据）⇒ 独立栏只有六枚
# 用法：_cmp_r467.py <smoke_A> <smoke_Ap> <smoke_C> <dump目录> <csv目录> <run目录>
import os
import re
import sys

A_SM, AP_SM, C_SM, DUMP, CSV, RUN = sys.argv[1:7]
TCOL = (476, 534)
TROW = (523, 543)
N_T = (TCOL[1] - TCOL[0] + 1) * (TROW[1] - TROW[0] + 1)
IV_MIN = 1200
PRED = (("P100", lambda v: v == 100), ("P90", lambda v: v >= 90))


def lines(p):
    return open(p, encoding="utf-8", errors="replace").read().splitlines() if os.path.isfile(p) else []


def checks(p):
    out = []
    for l in lines(p):
        if l.startswith("CHECK|"):
            f = l.split("|")
            out.append((f[1], f[2], "|".join(f[3:])))
    return out


def vec(v):
    return [(n, r) for n, r, _ in v]


def row12(a, b, la, lb):
    va, vb = vec(a), vec(b)
    same_names = [n for n, _ in va] == [n for n, _ in vb]
    diff = sum(1 for x, y in zip(va, vb) if x != y) + abs(len(va) - len(vb))
    print("〔%s vs %s〕名数=%d/%d ‖ 名集逐字同＝%s ‖ ★栏一（判定向量）异行数＝%d★" % (la, lb, len(va), len(vb), same_names, diff))
    fa = {n: msg for n, r, msg in a if r == "FAIL"}
    fb = {n: msg for n, r, msg in b if r == "FAIL"}
    common = sorted(set(fa) & set(fb))
    neq = [n for n in common if fa[n] != fb[n]]
    print("   ★栏二（C4）★＝共同 FAIL 行 %d 枚 ‖ outcome／message／连带句★逐字不等★的＝%d 枚%s‖ FAIL 集差 a−b=%s b−a=%s" % (
        len(common), len(neq), ("（★逐枚列出：" + "、".join(neq) + "）") if neq else "（＝全字等）",
        sorted(set(fa) - set(fb)), sorted(set(fb) - set(fa))))
    print("   判定计数＝%s: PASS=%d FAIL=%d ‖ %s: PASS=%d FAIL=%d" % (
        la, sum(1 for _, r, _ in a if r == "PASS"), len(fa), lb, sum(1 for _, r, _ in b if r == "PASS"), len(fb)))


def grid(tag):
    p = os.path.join(CSV, "cm_%s.csv" % tag)
    with open(p, encoding="utf-8", errors="replace") as f:
        meta = f.readline().strip().split(",")
        data = f.readline().strip().split(",")
    w, h, res, ox, oy = int(meta[1]), int(meta[2]), float(meta[3]), float(meta[4]), float(meta[5])
    vals = [int(v) for v in data]
    shift = int(round((ox + 50.0) / res)), int(round((oy + 50.0) / res))
    def g(c, r):
        if not (0 <= c < w and 0 <= r < h):
            raise SystemExit("★越界取格＝(%d,%d)‖ 件=%s 尺寸 %dx%d ⇒ 对照件停、不猜数★" % (c, r, tag, w, h))
        return vals[r * w + c]
    return g, w, h, res, ox, oy, shift, sorted(set(vals))


print("===〔零〕被试体具名（★三项齐★）===")
print("  T＝A 规范格点 列 %d..%d × 行 %d..%d ‖ n_T＝%d（=%d 列 × %d 行）‖ 世界格心 x∈[-2.35,3.45] y∈[2.35,4.35]" % (
    TCOL[0], TCOL[1], TROW[0], TROW[1], N_T, TCOL[1] - TCOL[0] + 1, TROW[1] - TROW[0] + 1))
G = {}
for tag in ("A", "Ap", "C"):
    g, w, h, res, ox, oy, sh, dv = grid(tag)
    G[tag] = g
    print("  %s 态：origin=(%.2f,%.2f) res=%.3f ‖ ★索引平移量＝(列%+d, 行%+d)★（该态文件下标＝A 规范格点 − 平移量） ‖ distinct=%s" % (tag, ox, oy, res, sh[0], sh[1], dv))

print("\n===〔一〕栏一＋栏二（判定面，与谓词无关）===")
ca, cap, cc = checks(A_SM), checks(AP_SM), checks(C_SM)
row12(ca, cap, "A", "Ap")
row12(ca, cc, "A", "C")

print("\n===〔二〕八栏 × 两谓词（★行内一致、判词分行并列★）===")
SIF = {}
for tag in ("A", "Ap", "C"):
    g, w, h, res, ox, oy, sh, dv = grid(tag)
    SIF[tag] = sh
res_map = {}
for pname, pred in PRED:
    ks = [(c, r) for c in range(TCOL[0], TCOL[1] + 1) for r in range(TROW[0], TROW[1] + 1)]
    vA = lambda c, r: G["A"](c - SIF["A"][0], r - SIF["A"][1])
    vP = lambda c, r: G["Ap"](c - SIF["Ap"][0], r - SIF["Ap"][1])
    vC = lambda c, r: G["C"](c - SIF["C"][0], r - SIF["C"][1])
    xa = sum(1 for c, r in ks if pred(vA(c, r)))
    xc = sum(1 for c, r in ks if pred(vC(c, r)))
    uf = sum(1 for c, r in ks if (not pred(vA(c, r))) and pred(vP(c, r)))
    vf = sum(1 for c, r in ks if pred(vA(c, r)) and (not pred(vP(c, r))))
    up = sum(1 for c, r in ks if (not pred(vA(c, r))) and pred(vC(c, r)))
    dn = sum(1 for c, r in ks if pred(vA(c, r)) and (not pred(vC(c, r))))
    res_map[pname] = dict(n_t=len(ks), xa=xa, xc=xc, u=up, v=dn, uf=uf, vf=vf, both=xa - dn)
for pname, _ in PRED:
    d = res_map[pname]
    print("  ──【%s 行】n_T=%d ‖ X_A=%d ‖ X_C=%d ‖ U（升）=%d ‖ V（反降）=%d ‖ U_f=%d ‖ V_f=%d ‖ 皆致命=%d（★派生＝X_A−V、不作第二证★）" % (
        pname, d["n_t"], d["xa"], d["xc"], d["u"], d["v"], d["uf"], d["vf"], d["both"]))
    print("     判词（★只用本行谓词★）：")
    print("       · IV 靶功率下限 n_T≥%d ⇒ %s" % (IV_MIN, "★成立★" if d["n_t"] >= IV_MIN else "不成立"))
    print("       · (a-R) 前提项 X_A≤0.1·n_T（＝%.1f）⇒ %s ‖ 余量＝%.1f 格" % (
        0.1 * d["n_t"], "★成立★" if d["xa"] <= 0.1 * d["n_t"] else "★不成立（合格读数、不是判过式失手）★", 0.1 * d["n_t"] - d["xa"]))
    print("       · P1 先决项 U_f==0 ∧ V_f==0 ⇒ %s" % ("★成立★" if d["uf"] == 0 and d["vf"] == 0 else "★不成立（U_f=%d、V_f=%d）★" % (d["uf"], d["vf"])))
    print("       · 净升支 U≥0.9·n_T（＝%.1f）⇒ %s ‖ 反降支 V==0 ⇒ %s" % (
        0.9 * d["n_t"], "★成立★" if d["u"] >= 0.9 * d["n_t"] else "不成立", "★成立★" if d["v"] == 0 else "★不成立★"))
    print("       · X_C≥0.9·n_T ⇒ %s" % ("★成立★" if d["xc"] >= 0.9 * d["n_t"] else "不成立"))

print("\n===〔三〕D 与地板（★两域并报＋漂移落点★）===")
for pname, pred in PRED:
    d_fl = sum(1 for c, r in [(c, r) for c in range(440, 560) for r in range(440, 560)] if pred(G["A"](c, r)) != pred(G["Ap"](c, r)))
    d_ac = sum(1 for c in range(440, 560) for r in range(440, 560) if pred(G["A"](c, r)) != pred(G["C"](c, r)))
    print("  【%s】footprint 内谓词成员不等：A↔Ap（＝T 外也计入的地板）=%d ‖ A↔C=%d" % (pname, d_fl, d_ac))
diff_cells = []
for c in range(440, 560):
    for r in range(440, 560):
        if G["A"](c, r) != G["Ap"](c, r):
            diff_cells.append((c, r))
raw = sum(1 for c in range(0, 1000) for r in range(0, 1000) if G["A"](c, r) != G["Ap"](c, r))
if diff_cells:
    print("  ★重量级 D_floor（全图逐格★值不等★）＝%d 格‖ footprint 内＝%d 格 ‖ 漂移落点＝列 %d..%d × 行 %d..%d（世界 x∈[%.2f,%.2f] y∈[%.2f,%.2f]）" % (
        raw, len(diff_cells), min(x for x, _ in diff_cells), max(x for x, _ in diff_cells),
        min(y for _, y in diff_cells), max(y for _, y in diff_cells),
        -50 + min(x for x, _ in diff_cells) * 0.1, -50 + (max(x for x, _ in diff_cells) + 1) * 0.1,
        -50 + min(y for _, y in diff_cells) * 0.1, -50 + (max(y for _, y in diff_cells) + 1) * 0.1))
    t_in = [(c, r) for c, r in diff_cells if TCOL[0] <= c <= TCOL[1] and TROW[0] <= r <= TROW[1]]
    print("  ★T 内 D_floor（逐格值不等）＝%d 格★（★地板是靶相关的、不是全局常数★）" % len(t_in))
d_ac_raw = sum(1 for c in range(0, 1000) for r in range(0, 1000) if G["A"](c, r) != G["C"](c, r))
print("  D（A↔C 全图逐格值不等）＝%d 格；在册容差只适用于★占据格计数★（±600）、★不适用★逐格差" % d_ac_raw)

print("\n===〔四〕B1 靶生效锁（★四件齐★）===")
pub = lines(os.path.join(RUN, "pub_C.txt"))
sl = [l for l in pub if l.startswith("SELFREPORT")]
for l in sl:
    print("   %s" % l)
for k, pat in (("Using plugin 行", "Using plugin"), ("Received a 行", "Received a"), ('含 elevation_layer 的 Using plugin 行', 'Using plugin "elevation_layer"')):
    n = sum(1 for l in lines(os.path.join(RUN, "roslaunch_C.log")) if pat in l)
    print("   %s＝%d" % (k, n))
mp = re.search(r"painted=(\d+)", "\n".join(sl))
print("   ★n_T==N 逐字比★＝painted=%s ‖ n_T=%d ⇒ %s" % (mp.group(1) if mp else "★缺 SELFREPORT★", N_T,
                                                        (mp and int(mp.group(1)) == N_T) if mp else False))

print("\n===〔五〕参数面 C1-C（★键路径展平、T1／T2 先自等★）===")
for tag in ("A", "Ap", "C"):
    p1 = os.path.join(DUMP, "dump_%s_T1.yaml" % tag)
    p2 = os.path.join(DUMP, "dump_%s_T2.yaml" % tag)
    if not (os.path.isfile(p1) and os.path.isfile(p2)):
        print("   %s：★dump 缺件★" % tag)
        continue
    a = [l for l in lines(p1) if l.strip() and not l.startswith("#")]
    b = [l for l in lines(p2) if l.strip() and not l.startswith("#")]
    print("   %s：T1 行数=%d T2 行数=%d ‖ 两采自等＝%s" % (tag, len(a), len(b), sorted(a) == sorted(b)))
print("   （★A↔C 的展平差与白名单命中＝本窗只在 03_delta.txt 报参数面形，判定面同值以〔一〕为准★）")

print("\n===〔六〕三态值域（★钉的「1..98 全空」在活体三态是否仍成立★）===")
for tag in ("A", "Ap", "C"):
    g, w, h, res, ox, oy, sh, dv = grid(tag)
    vals = [g(c, r) for r in range(440, 560) for c in range(440, 560)]
    mid = sum(1 for v in vals if 1 <= v <= 98)
    print("   %s：distinct=%s ‖ footprint 内 P90=%d ‖ P100=%d ‖ 1..98=%d" % (
        tag, dv, sum(1 for v in vals if v >= 90), sum(1 for v in vals if v == 100), mid))
print("\n★并报完整性自检★＝两套谓词行均打出＝%s（★缺一行 ⇒ 本窗八栏按在册钉计 INVALID★）" %
      (set(res_map) == {"P100", "P90"}))
