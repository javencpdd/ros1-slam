#!/usr/bin/env python3
# 01_复读件_r517.py —— ★批145＝把 r492q 给的三处形修到能自证★（★零窗＝不取数、不开窗、不起节点★）
#   修①★真 open 被引件★：留底侧一律从被引件解析出来（R492 那版〔七〕段 open( 次数＝0、四枚全是手抄字面量＝"复读"是同族自我循环）
#   修②★两行都复★：P100 与 P90 分别判（在册 P-N1 要两行分别判、不选边；R492 只复了 P100 ⇒ 判据面逮到"P90 行复读＝0 枚"）
#   修③★同名位分家★：`rc_勾对` 在册有两枚同名（对照件那枚＝硬编码清单、预检件那枚＝真取外部物）⇒ 本件那枚改名 `rc_勾对_复读件` 且★清单从 r464q 真读解析★
#   修④★机检式自带面条件★：根路径由 argv 传入（容器＝/opt/ros1-slam、宿主＝本机仓根），★不绑字面容器根★（r492 那版 (c) 项绑 /opt/ros1-slam ⇒ 宿主面必假红）
#   修⑤★可执行替换测试★：把被引件路径换成不存在的件后重跑 ⇒ ★rc 必须翻红★；不翻＝这枚机检位的名与实没绑上（照 r492q Q2 补的"替换测试"形）
# 用法：python3 01_复读件_r517.py <仓根> <被引窗目录名> <本窗目录名> [--swap]   ★批146＝被引窗与本窗改由 argv 传入（一份形只实现一次、不复制逻辑）★
import hashlib
import os
import re
import sys

ROOT = sys.argv[1]
SWAP = "--swap" in sys.argv
RAW = os.path.join(ROOT, "note/进度/SCAN集成/证据/log/raw")
SRC_WIN = sys.argv[2]     # ★被复读的那一窗＝本件要真打开的件★
LIVE_WIN = sys.argv[3]   # ★本窗★
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
    if SWAP:
        print("   被引件不在位＝%s ⇒ rc_复读=5（★5＝替换自测分支＝设计来翻红的那支★）" % cited)
    else:
        print("   被引件不在位＝%s ⇒ rc_复读=9（★9＝正测前置缺失档★；★B5/R517＝不再与负例共用 5★）" % cited)
    if SWAP:
        print("   ★本件即将以 rc=5 退出（R517 改形⑤＝证件要把进程码打进件里；旧证件只有打印串、没有进程码）★")
        sys.exit(5)   # ★R517 改形②＝丁的边界写进代码字面：只有★负例分支★（SWAP＝设计来翻红的那支）的红进退出码＝5★
    sys.exit(9)       # ★正测走到这里＝承重前提缺失 ⇒ 走三态之外的「前置缺失」档＝9（与账构／VCC 同形）★、★不再与负例红共用 5★
PREMISS = 0   # ★R517 改形③＝前置缺失分档累加器（文件末尾据此退出，不再一路退 0）★
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
def _diff(a, b):
    # ★R517 改形①＝旧式 abs(int(float(a))-int(float(b))) 把亚 1.0 的差全抹成 0（r511q 延五：我现读 5.9 vs 5.4→0）
    #   ⇒ 同一行里会出现「差 0」与「同值＝不成立」并存＝自测件里的真·假绿路径；现在改★小数差★（保留两位）。
    try:
        fa = float(str(a).replace("\u2212", "-").replace("+", ""))
        fb = float(str(b).replace("\u2212", "-").replace("+", ""))
        return round(abs(fa - fb), 2)
    except Exception:
        return "不可数（非数值）"

import datetime
_utc = datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%d %H:%M:%S")
_cst = (datetime.datetime.now(datetime.timezone.utc) + datetime.timedelta(hours=8)).strftime("%Y-%m-%d %H:%M:%S")
print("   ★复读时刻＝%s UTC ‖ %s CST★（两侧同打＝追加五十六③；栏形＝留底值 ‖ 留底出处窗号＋行号 ‖ 活体复读值 ‖ ★值差·不合枚★ ‖ 复读时刻（★A3/R517＝旧名「差值枚」退役★））" % (_utc, _cst))
for pn, kp in PRED:
    xa = len(cs_l[pn])
    v99 = len(cellset(gl, lambda v: v == 99)[0])
    for key, ln, live in (("X_A", 25 if pn == "P100" else 29, str(xa)), ("n_T", 25 if pn == "P100" else 29, str(N_T)), ("G1_v99", 66, str(v99)), ("余量", 27 if pn == "P100" else 31, "%.1f" % (0.1 * N_T - xa))):   # ★改形③＝「余量」进差值循环、并按谓词分行（P100→第 27 行 ‖ P90→第 31 行＝R504 现取）★
        cite_ln = ln
        if cite_ln > len(raw_txt):
            bad.append("%s/%s 行号越界" % (pn, key)); continue
        keepv, snip = keep_from_file(key, cite_ln)
        if key == "余量":
            def _nz(z):
                try:
                    return round(float(str(z).replace("\u2212", "-").replace("+", "")), 1)
                except Exception:
                    return None
            eq = _nz(keepv) is not None and _nz(keepv) == _nz(live)   # ★余量比数值（留底侧全角减号、活体侧半角）★
        else:
            eq = (keepv == live)
        if not eq:
            bad.append("%s/%s 留底 %s vs 活体 %s" % (pn, key, keepv, live))
        print("   [%s] %s：★留底（从被引件第 %d 行真读）%s★ ‖ 活体 %s ‖ ★两数之差＝%s ‖ 该栏不合＝%s 枚★（★R517 改形④＋A3/R517＝旧名「差值枚」把★值差★报成★枚数★，两枚并报才分开；本批起栏形声明已改用新名「值差·不合枚」★）‖ 同值＝%s" % (pn, key, cite_ln, keepv, live, _diff(keepv, live), 0 if eq else 1, "成立" if eq else "★不成立★"))
        print("   [%s] 域核对：本窗 T 内 %s 格集 vs 被引窗 同架★真对称差＝%d 枚★（★复读出处窗也进对照集＝修⑤的前半、不除 2★）‖ ★两侧几何随句＝本窗 origin=(%.2f,%.2f) 平移(%+d,%+d) ‖ 被引窗 origin=(%.2f,%.2f) 平移(%+d,%+d)★＝★不报几何就不许发跨窗句★（r502q 第 9 枚限定落地）" % (
        pn, pn, len(cs_l[pn] ^ cs_r[pn]), gl["ox"], gl["oy"], int(round((gl["ox"] - ANCH) / gl["res"])), int(round((gl["oy"] - ANCH) / gl["res"])),
        ref["ox"], ref["oy"], int(round((ref["ox"] - ANCH) / ref["res"])), int(round((ref["oy"] - ANCH) / ref["res"]))))
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
    print("   ★清单出处件不在位 ⇒ rc_勾对_复读件=5（不猜清单）‖ 并记「前置缺失」档＝9★（★r505q 点过的那支没进码、本批补上★）")
    PREMISS = 9
print("\n=== 总判定 ===")
print("   ★不合栏枚数＝%d 枚★（★真枚数口径＝逐栏计 0/1；上面那枚「两数之差」是值差、不是枚数★）" % len(bad))
print("   rc_复读＝%s（★缺项＝无 ⇒ 两谓词 × 三栏全部真读并同值；有缺即红、★不许用手抄常量补★）＝%s" % (0 if not bad else 5, bad or "无"))
print("   ★本件只报不拦、不进任何在册件的 rc；★未起节点、未起 master、无 sudo、未 kill★")

# ★改形②＝读数件自身带〈第几遍／共几遍〉（r496q L-O；R502 时这件含「遍」＝0 枚＝B 半合的残留）★
import time as _tm
try:
    _rp = os.path.join(os.path.dirname(os.path.abspath(__file__)), ".runs")
    _rn = open(_rp, encoding="utf-8").read().strip()
except Exception as _e:
    _rp, _rn = "★读 .runs 失败＝%s：%s★" % (type(_e).__name__, _e), "★缺★"
print("   ★遍次（件自身带）＝本遍＝第 %s 遍 ‖ 共 %s 遍（★共几遍＝截至本遍已起过的窗数★） ‖ 出处＝%s ‖ 本件时刻＝%s（★容器面 UTC；宿主 CST＝+8★）" % (
    _rn, _rn, _rp, _tm.strftime("%Y-%m-%d %H:%M:%S", _tm.gmtime())))

# ★R517 改形③收尾＝前置缺失分档真要退出来（r505 那版只打印、进程退 0＝拿退出码当闸的消费者会给绿）★
print("   ★边界自陈（R517 改形③＝把丁的「只」字写进字面）＝本件 rc=0 ★只表示★跑通并取到全部读数、★不表示★留底与活体一致；"
      "正向红（留底与活体不一致）依丁★不进退出码★、只进打印串与上面那枚 rc_复读；5＝负例分支（SWAP）；9＝前置缺失（被引件或清单出处件不在位）★")
print("   ★本件退出码档＝%s（0＝正测取到全读数 ‖ 5＝负例分支红（仅 SWAP）‖ 9＝前置缺失＝被引件或清单出处件不在位)★" % (0 if PREMISS == 0 else PREMISS))
sys.exit(0 if PREMISS == 0 else PREMISS)
