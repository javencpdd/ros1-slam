#!/usr/bin/env python3
# 06_判读件_r530.py —— ★F4／F5／F9 的执行形＝判读面每一枚数与每一枚行号现取、★查不到必有红档★★
#   用法：python3 06_判读件_r530.py <仓根> <本窗目录名>      （★根由 argv 传入；只读窗内件、只写本目录 06_判读.txt★）
#   ★F4（r525q S27）★＝旧版三种「查不到」＝★崩★（IndexError／KeyError）或★空值上路★（打印「第 None 行」）＝都不算被检到
#     ⇒ 本件规则：任何一处取不到值 ⇒ 记入 miss[]、该枚★红档★（rc=5）＋点名；★全件禁止 print 出 None／空串冒充读数★
import io, os, re, subprocess, sys

ROOT, WIN = sys.argv[1], sys.argv[2]
W = os.path.join(ROOT, "note/进度/SCAN集成/证据/log/raw", WIN)
CMP, LOG, TSV = (os.path.join(W, x) for x in ("05_cmp.txt", "00_window_log.txt", "09_锚键.tsv"))
OUT = os.path.join(W, "06_判读.txt")
TOK = "本件时刻＝[0-9]{4}-[0-9]{2}-[0-9]{2} [0-9:{2}]（[^）]*）"   # ★F5＝覆盖到形★（★单一枚式子同时用于计数与替换＝不许两套★）
TOK = TOK.replace("[0-9:{2}]", "[0-9]{2}:[0-9]{2}:[0-9]{2}")
miss, O = [], []
P = O.append


def rd(p):
    if not os.path.exists(p):
        miss.append("★前置件不在位＝%s★" % os.path.basename(p))
        return ""
    return io.open(p, encoding="utf-8", errors="replace").read()


def lines(p):
    return rd(p).split("\n")


def hits(path, pat):
    """★F4＝查不到 ⇒ 红档点名★（旧版在这里返回空表再由调用方 [0] ⇒ IndexError）"""
    h = [i + 1 for i, l in enumerate(lines(path)) if pat in l]
    if not h:
        miss.append("★锚串未命中＝%s ‖ 件＝%s★" % (pat[:26], os.path.basename(path)))
        return ["★缺★"]   # ★G4/R534＝哨兵值、不是 None★（旧形＝打印面出「第 None 行」＝S27 残留）
    return h


def rows_of(txt, keyname):
    """★G3/R534＝表字段里出现非数字（如「无」「缺」）⇒★红档点名★、绝不 ValueError 崩★"""
    v = set()
    for x in txt.split(","):
        if x.isdigit():
            v.add(int(x))
        else:
            miss.append("★表字段非数字＝%s ‖ 键＝%s★（G3＝按红档、不按崩）" % (x, keyname))
    return v


def grab(path, pat, field):
    """★F4＝取不到字段 ⇒ 红档★；★绝不打印 None★"""
    for l in lines(path):
        if pat in l:
            m = re.search(field, l)
            if m:
                return m.group(1)
            miss.append("★字段取不到＝%s ‖ 行=%s★" % (field[:24], l[:26]))
            return "★缺★"
    miss.append("★整行取不到＝%s ‖ 件＝%s★" % (pat[:26], os.path.basename(path)))
    return "★缺★"


import datetime
_off = datetime.datetime.now().astimezone().utcoffset()
_hrs = int(_off.total_seconds() // 3600) if _off else 0
# ★G8/R534＝面判定★真取★＝取本机 UTC 偏移（＋8＝宿主 CST、0＝容器 UTC），★不再用路径子串猜★（旧形＝"jack" in ROOT＝S42）
FACE_TAG = ("宿主（UTC+%d）" % _hrs) if _hrs == 8 else ("容器（UTC%+d）" % _hrs if _hrs == 0 else "★偏移非 8/0＝面未定＝不发面名★")
P("=== 06_判读（★F4／F5／F9 执行形＝行号与数值全现取、查不到必有红档★）===")
P("   根＝%s ‖ 本窗＝%s ‖ 宿主面 CST 与容器面 UTC 差 8 小时＝本件跑在★%s★面" % (ROOT, WIN, FACE_TAG))

P("\n=== 〔一〕锚键表逐枚与现跑对表（★r525q 问④＝判读件真读该键★）===")
rows = [l.split("\t") for l in rd(TSV).rstrip("\n").split("\n") if l.strip()]
P("   表枚数＝%d（在册期望 6）" % len(rows))
if len(rows) != 6:
    miss.append("★表枚数＝%d≠6★" % len(rows))
for row in rows:
    if len(row) < 4:
        miss.append("★表行残缺＝%s★" % row)
        continue
    k, crows, lrows, pat = row[:4]
    cn = [i + 1 for i, l in enumerate(lines(CMP)) if pat in l]
    ln = [i + 1 for i, l in enumerate(lines(LOG)) if pat in l]
    okc = crows == ",".join(map(str, cn))
    okl = rows_of(lrows, k) <= set(ln)   # ★G3/R534＝字段非数字⇒记红档、不 int 崩★（旧形＝ValueError＠件:67）
    P("   %-20s ‖ cmp 表=%s 现跑=%s ⇒ %s ‖ 志 表=%s 现跑全=%s ⇒ 子集=%s ‖ 志面★多命中=%d★（★表是时点产物★）" % (
        k, crows, ",".join(map(str, cn)) or "★无★", "合" if okc else "★不合★", lrows,
        ",".join(map(str, ln)) or "★无★", "合" if okl else "★不合★", max(0, len(ln) - len(crows.split(",")))))
    if not okc:
        miss.append("★%s 的 cmp 面表值≠现跑★" % k)
    if not okl:
        miss.append("★%s 的志面表值不在现跑里★" % k)

P("\n=== 〔二〕★F5＝排除式三数并报★（★同一枚式子既计数又替换＝不许两套式子★）＋★F9＝事后复现★===")
cl = lines(CMP)
n_app = sum(1 for x in cl if "本件时刻＝" in x)
n_hit = sum(1 for x in cl if re.search(TOK, x))
bcl = lines(os.path.join(W, "05_cmp_复跑.txt"))
d0 = sum(1 for i in range(max(len(cl), len(bcl))) if (cl[i] if i < len(cl) else None) != (bcl[i] if i < len(bcl) else None))
d1 = sum(1 for i in range(max(len(cl), len(bcl)))
         if (re.sub(TOK, "★时刻已排除★", cl[i]) if i < len(cl) else None) != (re.sub(TOK, "★时刻已排除★", bcl[i]) if i < len(bcl) else None))
diffrows = [i + 1 for i in range(max(len(cl), len(bcl))) if (cl[i] if i < len(cl) else None) != (bcl[i] if i < len(bcl) else None)]
P("   出现枚数＝%d ‖ 命中枚数＝%d ‖ ★两者相等＝%s★（★不等⇒这枚式子没行使⇒下面的 0 不可信＝r525q S23★）" % (n_app, n_hit, n_hit == n_app))
P("   未排除逐行差＝%d 枚（行位=%s）‖ 排除后逐行差＝%d 枚 ‖ ★差集所在行与命中行的交＝%s★" % (
    d0, diffrows, d1, set(diffrows) <= set(i + 1 for i, x in enumerate(cl) if re.search(TOK, x))))
if n_hit != n_app:
    miss.append("★排除式命中 %d≠出现 %d＝死码★" % (n_hit, n_app))
lab = hits(LOG, "F9/R530＝同一枚检查★事后复现★")[0]
P("   ★F9＝同一枚检查的★当场值与落盘后复现值★已在志第 %s 行并报（★时点读数不许写成状态★）" % lab)

P("\n=== 〔三〕承重数逐枚具名（★值＋载体行号同句★）===")
for nm, pat in (("八栏 P100", "──【P100 行】"), ("八栏 P90", "──【P90 行】"), ("G1 闸", "G1 闸"),
                ("栏齐", "rc_栏齐"), ("机检", "rc_机检"), ("申报", "rc_申报"), ("不变量", "rc_不变量"),
                ("退净乙式", "合取五带边界解析"), ("复读行", "★遍次（件自身带）")):
    h = [i + 1 for i, l in enumerate(cl) if pat in l]
    P("   %-12s ‖ 05_cmp 行位＝%s ‖ 值＝%s" % (nm, ",".join(map(str, h)) or "★无★",
       (cl[h[0] - 1].strip()[:118] if h else "★查不到★")))
    if not h:
        miss.append("★窗内无该位＝%s★" % nm)
for a, b in (("A", "Ap"), ("A", "C"), ("A", "REF"), ("REF", "A")):
    pat = "%s↔%s：origin 相同" % (a, b)
    h = [i + 1 for i, l in enumerate(cl) if pat in l]
    if not h:
        miss.append("★跨态行缺失＝%s★" % pat)
        continue
    l = cl[h[0] - 1]
    raw = re.search(r"raw（同下标硬比）＝(\d+)", l)
    ali = re.search(r"aligned（世界对齐）＝(\d+)", l)
    P("   %s↔%s ‖ 行 %s ‖ raw＝%s ‖ aligned＝%s ‖ 不变量判定＝%s" % (
        a, b, h[0], raw.group(1) if raw else "★缺★", ali.group(1) if ali else "★缺★",
        "成立" if "不变量＝★成立★" in l else ("不适用" if "不适用" in l else "★不成立★")))
    if not (raw and ali):
        miss.append("★%s↔%s 的 raw／aligned 取不到★" % (a, b))
pc = rd(os.path.join(W, "run", "pub_C.txt")).split("\n")[0]
m = re.search(r"free=(\d+).*?lethal=(\d+).*?total=(\d+)", pc)
P("   靶锁（run/pub_C.txt 第 1 行）＝%s＋%s＝%s ‖ total＝%s ‖ 合＝%s" % (
    m.group(1) if m else "★缺★", m.group(2) if m else "★缺★",
    (int(m.group(1)) + int(m.group(2))) if m else "★缺★", m.group(3) if m else "★缺★",
    bool(m) and int(m.group(1)) + int(m.group(2)) == int(m.group(3))))
if not m:
    miss.append("★靶锁字段取不到★")
sw = hits(LOG, "SWAP 遍 进程 rc 回读")[0]
P("   SWAP 遍回读（志第 %s 行）＝件内★回读值★与★进程 rc★同枚；★r525 那句恒 0 自陈已更正、旧志不回改★" % sw)

P("\n=== 总判定 ===")
P("   ★红档枚数＝%d 枚★（★F4＝任何一处查不到即点名入册、★不以崩或空值冒充读数★）＝%s" % (len(miss), miss or "无"))
P("   ★本件退出码档＝%s★（0＝全部取到；5＝有红档＝★红的是我的式子或窗、逐枚点名★；9＝前置件不在位）" % (
    9 if any("前置件" in x for x in miss) else (5 if miss else 0)))
P("   ★本件只报不拦、不进任何在册件的 rc；未起节点、未起 master、无 sudo、未 kill★")
txt = "\n".join(O) + "\n"
assert "None" not in txt, "★G4/R534＝输出串里出现 None＝查不到被当成读数★（旧形＝件:34 返回 [None]→件:90 打印）"
assert '"' not in txt, "判读面不许 ASCII 双引号"
io.open(OUT, "w", encoding="utf-8").write(txt)
print(txt)
RC = 9 if any("前置件" in x for x in miss) else (5 if miss else 0)
sys.exit(RC)
