#!/usr/bin/env python3
# 01_复读件_r495.py —— ★批145＝把 r492q 给的三处形修到能自证★（★零窗＝不取数、不开窗、不起节点★）
#   修①★真 open 被引件★：留底侧一律从被引件解析出来（R492 那版〔七〕段 open( 次数＝0、四枚全是手抄字面量＝"复读"是同族自我循环）
#   修②★两行都复★：P100 与 P90 分别判（在册 P-N1 要两行分别判、不选边；R492 只复了 P100 ⇒ 判据面逮到"P90 行复读＝0 枚"）
#   修③★同名位分家★：`rc_勾对` 在册有两枚同名（对照件那枚＝硬编码清单、预检件那枚＝真取外部物）⇒ 本件那枚改名 `rc_勾对_复读件` 且★清单从 r464q 真读解析★
#   修④★机检式自带面条件★：根路径由 argv 传入（容器＝/opt/ros1-slam、宿主＝本机仓根），★不绑字面容器根★（r492 那版 (c) 项绑 /opt/ros1-slam ⇒ 宿主面必假红）
#   修⑤★可执行替换测试★：把被引件路径换成不存在的件后重跑 ⇒ ★rc 必须翻红★；不翻＝这枚机检位的名与实没绑上（照 r492q Q2 补的"替换测试"形）
# 用法：python3 01_复读件_r495.py <仓根> [--swap]      # --swap ＝ 替换自测分支（把被引件换成不存在的名字）
import hashlib
import os
import re
import sys

ROOT = sys.argv[1]
SWAP = "--swap" in sys.argv
RAW = os.path.join(ROOT, "note/进度/SCAN集成/证据/log/raw")
SRC_WIN = "r472_收口窗重跑规范架与出处随行与勾对表"     # ★被复读的那一窗＝本件要真打开的件★
LIVE_WIN = "r492_四枚形自打活体窗_复读栏与lethal同印与勾对行与rc申报"
ANCH = -50.0
TCOL, TROW = (476, 534), (523, 543)
N_T = (TCOL[1] - TCOL[0] + 1) * (TROW[1] - TROW[0] + 1)
PRED = (("P100", lambda v: v == 100), ("P90", lambda v: v >= 90))
CITE = {"X_A": 25, "余量": 27, "G1_v99": 66, "n_T": 25}


def sha16(p):
    return hashlib.sha256(open(p, "rb").read()).hexdigest()[:16]


def load_csv(win, st):
    p = os.path.join(RAW, win, "run", "csv", "cm_%s.csv" % st)
    with open(p, encoding="utf-8", errors="replace") as f:
        m = f.readline().strip().split(",")
        d = [int(v) for v in f.readline().strip().split(",")]
    return dict(ox=float(m[4]), oy=float(m[5]), res=float(m[3]), d=d, w=int(m[1]), h=int(m[2]), p=p)


def cellset(g, keep):
    sc = int(round((g["ox"] - ANCH) / g["res"]))
    sr = int(round((g["oy"] - ANCH) / g["res"]))
    out = set()
    for c in range(TCOL[0], TCOL[1] + 1):
        for r in range(TROW[0], TROW[1] + 1):
            cc, rr = c - sc, r - sr
            if 0 <= cc < g["w"] and 0 <= rr < g["h"] and keep(g["d"][rr * g["w"] + cc]):
                out.add((c, r))
    return out, sc, sr


print("=== 面条件（修④）★根由 argv 传入、不绑容器字面★ ===")
print("   argv 根＝%s ‖ 存在＝%s ‖ 分支＝%s ‖ 本机 OS＝%s" % (ROOT, os.path.isdir(ROOT), "★替换自测★" if SWAP else "正测", sys.platform))

cited = os.path.join(RAW, SRC_WIN, "05_cmp.txt")
if SWAP:
    cited = os.path.join(RAW, SRC_WIN, "05_cmp_★本件不存在用来做替换自测★.txt")
print("\n=== 修①真 open 被引件（★留底侧不许是手抄常量★）===")
if not os.path.exists(cited):
    print("   被引件不在位＝%s ⇒ rc_复读=5（★替换自测分支期望的就是这个红★）" % cited)
    sys.exit(0 if SWAP else 5)
raw_txt = open(cited, encoding="utf-8", errors="replace").read().splitlines()
print("   被引件＝%s ‖ 行数＝%d ‖ sha 前 16＝%s" % (cited.replace(RAW, "RAW/…"), len(raw_txt), sha16(cited)))


def keep_from_file(key, ln):
    line = raw_txt[ln - 1]
    if key == "X_A":
        m = re.search(r"X_A\[P100\]=(\d+)", line) if "P100" in line else re.search(r"X_A\[P90\]=(\d+)", line)
    elif key == "n_T":
        m = re.search(r"n_T\[P100\]=(\d+)", line) if "P100" in line else re.search(r"n_T\[P90\]=(\d+)", line)
    elif key == "余量":
        m = re.search(r"余量 ([\d.\-]+)", line)
    else:
        m = re.search(r"v==99 枚数＝(\d+)", line)
    return (m.group(1) if m else "★该行无此字段★"), line[:66]


gl = load_csv(LIVE_WIN, "A")
ref = load_csv(SRC_WIN, "A")
cs_l = {pn: cellset(gl, kp)[0] for pn, kp in PRED}
cs_r = {pn: cellset(ref, kp)[0] for pn, kp in PRED}
print("\n=== 修②两行都复（P100 与 P90 分别判、不选边）===")
bad = []
for pn, kp in PRED:
    xa = len(cs_l[pn])
    v99 = len(cellset(gl, lambda v: v == 99)[0])
    for key, ln, live in (("X_A", 25 if pn == "P100" else 29, str(xa)), ("n_T", 25 if pn == "P100" else 29, str(N_T)), ("G1_v99", 66, str(v99))):
        cite_ln = ln
        if cite_ln > len(raw_txt):
            bad.append("%s/%s 行号越界" % (pn, key)); continue
        keepv, snip = keep_from_file(key, cite_ln)
        if key == "X_A" and pn == "P90":
            keepv, snip = keep_from_file(key, 29)
        if key == "n_T" and pn == "P90":
            keepv, snip = keep_from_file(key, 29)
        eq = (keepv == live)
        if not eq:
            bad.append("%s/%s 留底 %s vs 活体 %s" % (pn, key, keepv, live))
        print("   [%s] %s：★留底（从被引件第 %d 行真读）%s★ ‖ 活体 %s ‖ 同值＝%s" % (pn, key, cite_ln, keepv, live, "成立" if eq else "★不成立★"))
    print("   [%s] 域核对：本窗 T 内 %s 格集 vs 被引窗 同架★真对称差＝%d 枚★（★复读出处窗也进对照集＝修⑤的前半、不除 2★）" % (
        pn, pn, len(cs_l[pn] ^ cs_r[pn])))
print("\n=== 修③同名位分家：清单从 r464q 真读解析、不是硬编码 ===")
q464 = os.path.join(RAW, "r464q_L1两支互斥改哪一支独立裁定/00_独立裁定.txt")
present = []
if os.path.exists(q464):
    qt = open(q464, encoding="utf-8", errors="replace").read().splitlines()
    print("   清单出处＝r464q 00 件 ‖ 行数＝%d ‖ sha 前 16＝%s" % (len(qt), sha16(q464)))
    row108 = qt[107] if len(qt) >= 108 else ""
    found = re.findall(r"`?(n_T|X_A|X_C|X_Ref|U_f|V_f|U|V)`?", row108)
    order = []
    for x in found:
        if x not in order:
            order.append(x)
    print("   第 108 行原文＝%s" % row108[:120])
    print("   从该行★解析出的栏名枚数＝%d ‖ 名单＝%s" % (len(order), order))
    own = {pn: set() for pn, _ in PRED}
    print("   本件在场栏（★从被引件逐行真读、不看内存字典★）：")
    for pn in ("P100", "P90"):
        ln = 25 if pn == "P100" else 29
        for nm in ("n_T", "X_A", "X_C", "X_Ref", "U", "V", "U_f", "V_f"):
            if re.search(r"%s\[?%s?\]?=" % (re.escape(nm), pn), raw_txt[ln - 1]):
                own[pn].add(nm)
    for nm in order:
        if all(nm in own[pn] for pn in own):
            present.append(nm)
    miss = [nm for nm in order if nm not in present]
    print("   ★rc_勾对_复读件（新名、与预检件那枚分开）＝%s★ 在册清单枚数（真读 r464q）＝%d ‖ 在场（两谓词行齐）＝%d ‖ 缺项＝%s" % (
        0 if not miss else 5, len(order), len(present), miss or "无"))
else:
    print("   ★清单出处件不在位 ⇒ rc_勾对_复读件=5（不猜清单★）")
print("\n=== 总判定 ===")
print("   rc_复读＝%s（★缺项＝无 ⇒ 两谓词 × 三栏全部真读并同值；有缺即红、★不许用手抄常量补★）＝%s" % (0 if not bad else 5, bad or "无"))
print("   ★本件只报不拦、不进任何在册件的 rc；★未起节点、未起 master、无 sudo、未 kill★")
