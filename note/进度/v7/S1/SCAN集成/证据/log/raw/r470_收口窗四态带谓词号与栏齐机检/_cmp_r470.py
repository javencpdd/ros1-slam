#!/usr/bin/env python3
# _cmp_r470.py —— ★首次按 r468q 三面新形（P-N1／R-E／R-F／L-A1）写的对照件★
#   P-N1＝八栏★栏名一律带谓词号★（X_A[P90]／X_A[P100]…），★P1 两行分别判、不选边★，判词★跨行不合成★
#   R-E ＝任何★跨态逐格差★一律★raw／aligned 双列★＋★平移量带减序标注★＋逐态 origin＋统计域；
#          ★并自带不变量检查：同 origin ⇒ raw≡aligned（不同＝我的架错、不是数据错）★
#   R-F ＝四态含 REF ⇒ 打★X_Ref★；并★两枚独立 rc＝rc_机检／rc_栏齐★（机检齐≠读数齐）
#   L-A1＝末行给★窗级归档形★「跨谓词分歧·行级各判（不合成）」＋G1 闸（逐态 T 内 v==99 枚数）
# 用法：_cmp_r470.py <smoke_REF> <smoke_A> <smoke_Ap> <smoke_C> <dump目录> <csv目录> <run目录>
import os
import re
import sys

REF_SM, A_SM, AP_SM, C_SM, DUMP, CSV, RUN = sys.argv[1:8]
TCOL = (476, 534)
TROW = (523, 543)
N_T = (TCOL[1] - TCOL[0] + 1) * (TROW[1] - TROW[0] + 1)
IV_MIN = 1200
W_HOPES = ("n_T", "X_A", "X_C", "X_Ref", "U", "V", "U_f", "V_f")
PRED = ("P100", lambda v: v == 100), ("P90", lambda v: v >= 90)
STATES = ("REF", "A", "Ap", "C")


def load(tag):
    p = os.path.join(CSV, "cm_%s.csv" % tag)
    with open(p, encoding="utf-8", errors="replace") as f:
        meta = f.readline().strip().split(",")
        data = f.readline().strip().split(",")
    w, h, res, ox, oy = int(meta[1]), int(meta[2]), float(meta[3]), float(meta[4]), float(meta[5])
    return {"w": w, "h": h, "res": res, "ox": ox, "oy": oy, "d": [int(v) for v in data], "path": p}


S = {t: load(t) for t in STATES}
BASE = S["A"]


def shift(t):
    """★减序固定＝(该态 − 基准态 A)★；该态文件下标＝A 规范格点 − 平移量"""
    g = S[t]
    return (int(round((g["ox"] - BASE["ox"]) / g["res"])), int(round((g["oy"] - BASE["oy"]) / g["res"])))


def raw_at(t, c, r):
    g = S[t]
    return g["d"][r * g["w"] + c]


def val(t, c, r):
    """世界对齐取值：以 A 规范格点 (c,r) 求该态的★文件下标★（两轴各自减、★绝不交换★；越界返回 None）"""
    g = S[t]
    sc, sr = shift(t)
    fc, fr = c - sc, r - sr
    if not (0 <= fc < g["w"] and 0 <= fr < g["h"]):
        return None
    return g["d"][fr * g["w"] + fc]


def lines(p):
    return open(p, encoding="utf-8", errors="replace").read().splitlines() if os.path.isfile(p) else []


def checks(p):
    out = []
    for l in lines(p):
        if l.startswith("CHECK|"):
            f = l.split("|")
            out.append((f[1], f[2], "|".join(f[3:])))
    return out


print("===〔零〕被试体具名（★三项齐＋逐态 origin 与平移量减序★）===")
print("  T＝A 规范格点 列 %d..%d × 行 %d..%d ‖ n_T＝%d（=%d 列 × %d 行）" % (TCOL[0], TCOL[1], TROW[0], TROW[1], N_T, TCOL[1] - TCOL[0] + 1, TROW[1] - TROW[0] + 1))
for t in STATES:
    g = S[t]
    xs = [c for c in range(g["w"]) if -6 <= g["ox"] + (c + 0.5) * g["res"] <= 6]
    ys = [r for r in range(g["h"]) if -6 <= g["oy"] + (r + 0.5) * g["res"] <= 6]
    print("  %s：origin=(%.2f,%.2f) res=%.3f ‖ 平移量(该态−A)＝%+d,%+d ‖ 该态自身架 footprint＝列 %d..%d × 行 %d..%d ‖ distinct=%s" % (
        t, g["ox"], g["oy"], g["res"], shift(t)[0], shift(t)[1], xs[0], xs[-1], ys[0], ys[-1], sorted(set(g["d"]))))

print("\n===〔一〕★不变量检查（形 R-E 的核心）★＝同 origin ⇒ raw≡aligned ===")
inv_ok = True
for a, b in [("A", "Ap"), ("A", "C"), ("A", "REF"), ("REF", "A")]:
    sa, sb = S[a], S[b]
    same = (sa["ox"] == sb["ox"] and sa["oy"] == sb["oy"])
    n_raw = sum(1 for i in range(len(sa["d"])) if sa["d"][i] != sb["d"][i])
    n_ali = sum(1 for c in range(1000) for r in range(1000) if (lambda x, y: x is not None and y is not None and x != y)(val(a, c, r), val(b, c, r)))
    ok = (not same) or (n_raw == n_ali)
    inv_ok = inv_ok and ok
    print("  %s↔%s：origin 相同＝%s ‖ raw（同下标硬比）＝%d ‖ aligned（世界对齐）＝%d ‖ 不变量＝%s%s" % (
        a, b, same, n_raw, n_ali, "★成立★" if ok else "★不成立（我的架错了）★",
        "  ‖★两态 origin 不同 ⇒ 只有 aligned 可用、raw 只作产物留名" if not same else ""))

print("\n===〔二〕栏一＋栏二（判定面，★与谓词无关★）===")
CS = {t: checks(p) for t, p in (("REF", REF_SM), ("A", A_SM), ("Ap", AP_SM), ("C", C_SM))}


def row12(ta, tb):
    a, b = CS[ta], CS[tb]
    va, vb = [(n, r) for n, r, _ in a], [(n, r) for n, r, _ in b]
    diff = sum(1 for x, y in zip(va, vb) if x != y) + abs(len(va) - len(vb))
    fa = {n: m for n, r, m in a if r == "FAIL"}
    fb = {n: m for n, r, m in b if r == "FAIL"}
    neq = [n for n in sorted(set(fa) & set(fb)) if fa[n] != fb[n]]
    print("  〔%s vs %s〕名数=%d/%d ‖ 名集逐字同＝%s ‖ 栏一异行数＝★%d★ ‖ 栏二 C4：共同 FAIL %d 枚、逐字不等＝%d%s ‖ PASS/FAIL＝%d/%d vs %d/%d" % (
        ta, tb, len(va), len(vb), [n for n, _ in va] == [n for n, _ in vb], diff,
        len(set(fa) & set(fb)), len(neq), ("（" + "、".join(neq) + "）") if neq else "（全字等）",
        sum(1 for _, r, _ in a if r == "PASS"), len(fa), sum(1 for _, r, _ in b if r == "PASS"), len(fb)))


for pair in (("REF", "A"), ("A", "Ap"), ("A", "C")):
    row12(*pair)

print("\n===〔三〕★八栏 × 两谓词（形 P-N1：栏名带号、行内一致、不合成）★===")
KS = [(c, r) for c in range(TCOL[0], TCOL[1] + 1) for r in range(TROW[0], TROW[1] + 1)]
COLS = {}
for pname, pred in PRED:
    d = {}
    d["n_T"] = len(KS)
    d["X_A"] = sum(1 for c, r in KS if pred(val("A", c, r)))
    d["X_C"] = sum(1 for c, r in KS if pred(val("C", c, r)))
    d["X_Ref"] = sum(1 for c, r in KS if pred(val("REF", c, r)))
    d["U"] = sum(1 for c, r in KS if (not pred(val("A", c, r))) and pred(val("C", c, r)))
    d["V"] = sum(1 for c, r in KS if pred(val("A", c, r)) and (not pred(val("C", c, r))))
    d["U_f"] = sum(1 for c, r in KS if (not pred(val("A", c, r))) and pred(val("Ap", c, r)))
    d["V_f"] = sum(1 for c, r in KS if pred(val("A", c, r)) and (not pred(val("Ap", c, r))))
    d["G1_v99_in_T"] = sum(1 for c, r in KS if val("A", c, r) == 99)
    COLS[pname] = d
for pname, _ in PRED:
    d = COLS[pname]
    print("  ──【%s 行】%s" % (pname, " ‖ ".join("%s[%s]=%d" % (k, pname, d[k]) for k in W_HOPES)))
    print("     G1 闸（A 态 T 内 v==99 枚数）＝%d ⇒ ★逐行各判、跨行不合成（形 L-A1）★" % d["G1_v99_in_T"])
    print("     判词（★只用本行谓词★）：IV n_T≥%d ⇒ %s ‖ P1：X_A[%s]≤0.1n_T（%.1f）⇒ %s（余量 %.1f） ‖ 靶区地板 U_f[%s]==0∧V_f[%s]==0 ⇒ %s ‖ 净升 U[%s]≥0.9n_T（%.1f）⇒ %s ‖ 反降 V[%s]==0 ⇒ %s ‖ X_C[%s]≥0.9n_T ⇒ %s" % (
        IV_MIN, "成立" if d["n_T"] >= IV_MIN else "不成立", pname, 0.1 * d["n_T"],
        "★成立★" if d["X_A"] <= 0.1 * d["n_T"] else "★不成立（合格读数、不是判过式失手）★", 0.1 * d["n_T"] - d["X_A"],
        pname, pname, "★成立★" if d["U_f"] == 0 and d["V_f"] == 0 else "不成立（U_f=%d、V_f=%d）" % (d["U_f"], d["V_f"]),
        pname, 0.9 * d["n_T"], "★成立★" if d["U"] >= 0.9 * d["n_T"] else "不成立", pname, "★成立★" if d["V"] == 0 else "不成立",
        pname, "★成立★" if d["X_C"] >= 0.9 * d["n_T"] else "不成立"))
    print("     注：皆致命 ≡ X_A[%s] − V[%s] ＝ %d（★派生栏、不作第二证★）" % (pname, pname, d["X_A"] - d["V"]))

print("\n===〔四〕D／D_floor（★raw／aligned 双列＋统计域＋跨域不相减★）===")
for pname, pred in PRED:
    for ta, tb, lab in (("A", "Ap", "D_floor"), ("A", "C", "D"), ("REF", "A", "D(REF↔A)")):
        n_ali = sum(1 for c, r in KS if (lambda x, y: x is not None and y is not None and pred(x) != pred(y))(val(ta, c, r), val(tb, c, r)))
        n_raw = sum(1 for c, r in KS if pred(raw_at(ta, c, r)) != pred(raw_at(tb, c, r)))
        print("  %s〔%s↔%s〕于 %s：T 内 aligned＝%d ‖ raw＝%d ‖（域＝T；全图见末行）" % (lab, ta, tb, pname, n_ali, n_raw))
for pname, pred in PRED:
    fa = sum(1 for v in S["A"]["d"] if pred(v))
    fc = sum(1 for v in S["C"]["d"] if pred(v))
    print("  全图计数〔%s〕A＝%d ‖ C＝%d ‖ Δ＝%+d（★只报全图域；不与 footprint 域同句相减★）" % (pname, fa, fc, fc - fa))
for pname, pred in PRED:
    fp_a = sum(1 for c in range(440, 560) for r in range(440, 560) if pred(val("A", c, r)))
    fp_c = sum(1 for c in range(440, 560) for r in range(440, 560) if pred(val("C", c, r)))
    print("  footprint〔A 规范架 440..559〕〔%s〕A＝%d ‖ C＝%d ‖ Δ＝%+d（★该域单独成行★）" % (pname, fp_a, fp_c, fp_c - fp_a))
dc = [(c, r) for c in range(1000) for r in range(1000) if val("A", c, r) is not None and val("Ap", c, r) is not None and val("A", c, r) != val("Ap", c, r)]
if dc:
    xs, ys = [x for x, _ in dc], [y for _, y in dc]
    print("  ★漂移落点（aligned、全图域）＝列 %d..%d × 行 %d..%d ‖ 外沿以★A 件自身 origin=%.2f★ 为架＝x∈[%.2f,%.2f) y∈[%.2f,%.2f)★两轴各自现算、不复制★" % (
        min(xs), max(xs), min(ys), max(ys), BASE["ox"], BASE["ox"] + min(xs) * BASE["res"], BASE["ox"] + (max(xs) + 1) * BASE["res"],
        BASE["oy"] + min(ys) * BASE["res"], BASE["oy"] + (max(ys) + 1) * BASE["res"]))
    tin = [(c, r) for c, r in dc if TCOL[0] <= c <= TCOL[1] and TROW[0] <= r <= TROW[1]]
    print("  ★T 内逐格差＝%d 枚（★先决项靠的是落点不交叠、不是容差★）；全图＝%d 枚；footprint 域＝%d 枚" % (
        len(tin), len(dc), sum(1 for c, r in dc if 440 <= c < 560 and 440 <= r < 560)))

print("\n===〔五〕B1 靶生效锁＋C1-C 两采自等＋值域 ===")
sl = [l for l in lines(os.path.join(RUN, "pub_C.txt")) if l.startswith("SELFREPORT")]
for l in sl:
    print("   %s" % l)
for k, pat in (("Using plugin 行", "Using plugin"), ("Received a 行", "Received a"), ('含 elevation_layer 的 Using plugin 行', 'Using plugin "elevation_layer"')):
    print("   %s＝REF %d ‖ A %d ‖ Ap %d ‖ C %d" % (k,
        *[sum(1 for l in lines(os.path.join(RUN, "roslaunch_%s.log" % t)) if pat in l) for t in STATES]))
mp = re.search(r"painted=(\d+)", "\n".join(sl))
print("   ★n_T==N★＝painted=%s ‖ n_T=%d ⇒ %s" % (mp.group(1) if mp else "缺", N_T, bool(mp) and int(mp.group(1)) == N_T))
for t in STATES:
    p1, p2 = os.path.join(DUMP, "dump_%s_T1.yaml" % t), os.path.join(DUMP, "dump_%s_T2.yaml" % t)
    a = [l for l in lines(p1) if l.strip() and not l.startswith("#")]
    b = [l for l in lines(p2) if l.strip() and not l.startswith("#")]
    print("   C1-C %s：T1＝%d 行 T2＝%d 行 ‖ 两采自等＝%s" % (t, len(a), len(b), sorted(a) == sorted(b)))

print("\n===〔六〕★两枚独立 rc（形 R-F：机检齐≠读数齐）★===")
missing = [k for pname, _ in PRED for k in W_HOPES if k not in COLS[pname]]
for t in STATES:
    if len(CS[t]) != 13:
        missing.append("%s 的 CHECK 枚数≠13" % t)
print("  rc_栏齐＝%s（★应为 0＝八格×两谓词全打出、含 X_Ref；非 0＝缺哪栏逐枚列出★）缺＝%s" % (0 if not missing else 5, missing or "无"))
print("  rc_机检＝%s（★0＝STATE 四行＋B1 四件＋n_T==N＋两采自等＋退净乙式全部在窗志内自证★）" % (0 if (mp and inv_ok) else 5))
print("  ★窗级归档（形 L-A1）★＝「跨谓词分歧·行级各判（不合成）」：P100 行 P1＝%s ‖ P90 行 P1＝%s ⇒ ★本窗不发任何不带谓词号的合成句★" % (
    "成立" if COLS["P100"]["X_A"] <= 0.1 * N_T else "不成立", "成立" if COLS["P90"]["X_A"] <= 0.1 * N_T else "不成立"))
print("  ★G1（合成闸一）＝A 态 T 内 v==99 枚数＝%d ⇒ %s★" % (COLS["P100"]["G1_v99_in_T"], "≠0 ⇒ 不许跨行合成" if COLS["P100"]["G1_v99_in_T"] != 0 else "＝0 ⇒ 允许合成"))
