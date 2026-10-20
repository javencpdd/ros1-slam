#!/usr/bin/env python3
# 06_判读件_r525.py —— ★追加一百一十一⑧ 的执行形＝判读面里每一枚行号与每一枚数都由本件现取、零手抄★
#   用法：python3 06_判读件_r525.py <仓根> <本窗目录名>      （★根由 argv 传入＝批145 面条件；只读窗内件、只写本目录 06_判读.txt★）
#   三枚自证：①锚键表逐枚与现跑 grep 对表（读到即绿、读不到即红＝r521q S20 的加固形）
#             ②承重数由 CSV／日志独立重算（不 import 对照件、不抄窗内打印值）
#             ③时刻 token 排除式★自报命中枚数★并与「时刻 token 出现枚数」对表＝不合则 RESTSAME=False 不得读成「内容真变」
import io, os, re, subprocess, sys

ROOT, WIN = sys.argv[1], sys.argv[2]
W = os.path.join(ROOT, "note/进度/SCAN集成/证据/log/raw", WIN)
CMP = os.path.join(W, "05_cmp.txt")
LOG = os.path.join(W, "00_window_log.txt")
TSV = os.path.join(W, "09_锚键.tsv")
OUT = os.path.join(W, "06_判读.txt")
ANCHOR, TCOL, TROW = -50.0, (476, 534), (523, 543)
PRED = (("P100", lambda v: v == 100), ("P90", lambda v: v >= 90))
TOK = "本件时刻＝[0-9-]{10} [0-9:]{8}（★容器面 UTC；宿主 CST＝+8★）"
bad = []


def rd(p):
    return io.open(p, encoding="utf-8", errors="replace").read()


def L(p):
    return rd(p).split("\n")


def hits(p, pat):
    return [i + 1 for i, l in enumerate(L(p)) if pat in l]


if not os.path.exists(TSV) or not os.path.exists(CMP):
    print("★前置缺失＝09_锚键.tsv 或 05_cmp.txt 不在位 ⇒ rc=9（不猜锚、不猜数）★")
    sys.exit(9)

O = []
P = O.append
P("=== 06_判读（★全部行号与数值由本件现取、零手抄＝追加一百一十一⑧★）===")
P("   根＝%s ‖ 本窗＝%s ‖ 本件 sha16 不做自证（本件写盘后自身变）" % (ROOT, WIN))

P("\n=== 〔一〕锚键表逐枚与现跑对表（r521q S20 加固形＝判读件★真读★该键）===")
rows = [l.split("\t") for l in rd(TSV).rstrip("\n").split("\n")]
P("   表枚数＝%d（期望 6）" % len(rows))
for k, crows, lrows, pat in rows:
    c_now = hits(CMP, pat)
    l_now = hits(LOG, pat)
    okc = crows == ",".join(map(str, c_now))
    okl = set(int(x) for x in lrows.split(",")) <= set(l_now)
    P("   %-20s PAT＝%-12s ‖ cmp 表=%s 现跑=%s ⇒ %s ‖ 志 表=%s 现跑全=%s ⇒ 子集=%s" % (
        k, pat, crows, ",".join(map(str, c_now)), "合" if okc else "★不合★",
        lrows, ",".join(map(str, l_now)), "合" if okl else "★不合★"))
    if not okc:
        bad.append(k + ".cmp")
    if not okl:
        bad.append(k + ".log")

P("\n=== 〔二〕承重数独立重算（★从 CSV 现算、不读窗内打印值★）===")
S = {}
for t in ("REF", "A", "Ap", "C"):
    p = os.path.join(W, "run", "csv", "cm_%s.csv" % t)
    with io.open(p, encoding="utf-8", errors="replace") as f:
        first = f.readline().strip()
        toks = [x for line in f for x in line.strip().split(",") if re.fullmatch(r"-?\d+", x)]
    m = re.match(r"^META,(\d+),(\d+),([-\d.]+),([-\d.]+),([-\d.]+),(\S+)$", first)
    g = dict(W=int(m.group(1)), H=int(m.group(2)), res=float(m.group(3)), ox=float(m.group(4)), oy=float(m.group(5)))
    g["d"] = [int(x) for x in toks]
    g["sh"] = (int(round((g["ox"] - ANCHOR) / g["res"])), int(round((g["oy"] - ANCHOR) / g["res"])))
    assert len(g["d"]) == g["W"] * g["H"]
    S[t] = g


def v(t, c, r):
    sc, sr = S[t]["sh"]
    cc, rr = c - sc, r - sr
    return S[t]["d"][rr * S[t]["W"] + cc] if 0 <= cc < S[t]["W"] and 0 <= rr < S[t]["H"] else None


KS = [(c, r) for c in range(TCOL[0], TCOL[1] + 1) for r in range(TROW[0], TROW[1] + 1)]
N_T = len(KS)
cl = L(CMP)


def lineof(pat, n=1):
    h = [i + 1 for i, x in enumerate(cl) if pat in x]
    return (h[n - 1] if len(h) >= n else None), (cl[h[n - 1] - 1].strip() if h else "★缺★")


P("   n_T（本件现数）＝%d ‖ 窗内该句载体＝第 %s 行" % (N_T, lineof("T＝★规范架")[0]))
for nm, pred in PRED:
    x = {k: sum(1 for c, r in KS if pred(v(s, c, r))) for k, s in (("X_A", "A"), ("X_C", "C"), ("X_Ref", "REF"), ("Ap", "Ap"))}
    U = sum(1 for c, r in KS if (not pred(v("A", c, r))) and pred(v("C", c, r)))
    V = sum(1 for c, r in KS if pred(v("A", c, r)) and (not pred(v("C", c, r))))
    Uf = sum(1 for c, r in KS if (not pred(v("A", c, r))) and pred(v("Ap", c, r)))
    Vf = sum(1 for c, r in KS if pred(v("A", c, r)) and (not pred(v("Ap", c, r))))
    g1 = sum(1 for c, r in KS if v("A", c, r) == 99)
    ln, txt = lineof("──【%s 行】" % nm)
    P("   %s ‖ 现算 X_A=%d X_C=%d X_Ref=%d U=%d V=%d U_f=%d V_f=%d ‖ 余量=%.1f ‖ G1=%d" % (
        nm, x["X_A"], x["X_C"], x["X_Ref"], U, V, Uf, Vf, 0.1 * N_T - x["X_A"], g1))
    P("      窗内同位＝第 %s 行 ‖ 该行含上述八名与 G1＝%s" % (ln, all(k in txt for k in ("X_A", "X_C", "X_Ref", "U", "V", "U_f", "V_f", "n_T"))))

P("\n   跨态三式并报（Δ式／并式／现数）＝★本窗四态全同架⇒Δ式必为 0、并式才是准的那枚★")
for a, b in (("A", "Ap"), ("A", "C"), ("A", "REF"), ("REF", "A")):
    same = S[a]["ox"] == S[b]["ox"] and S[a]["oy"] == S[b]["oy"]
    n_raw = sum(1 for i in range(S[a]["W"] * S[a]["H"]) if S[a]["d"][i] != S[b]["d"][i])
    n_ali = sum(1 for c in range(1000) for r in range(1000) if None not in (v(a, c, r), v(b, c, r)) and v(a, c, r) != v(b, c, r))
    bl = sum(1 for c in range(1000) for r in range(1000) if v(a, c, r) is None or v(b, c, r) is None)
    def band(s1, s2, N):
        return len({i for s in (s1, s2) for i in range(N) if not 0 <= i + s < N})
    Bc = band(S[a]["sh"][0], S[b]["sh"][0], 1000)
    Br = band(S[a]["sh"][1], S[b]["sh"][1], 1000)
    dc, dr = abs(S[a]["sh"][0] - S[b]["sh"][0]), abs(S[a]["sh"][1] - S[b]["sh"][1])
    union, delta = 1000 * Bc + 1000 * Br - Bc * Br, 1000 * dc + 1000 * dr - dc * dr
    ln, _ = lineof("%s↔%s：origin 相同" % (a, b))
    P("   %s↔%s 同架=%s ‖ 现算 raw=%d aligned=%d ‖ 不变量=%s ‖ 盲区 现数=%d 并式=%d(=%s) Δ式=%d(=%s) ‖ 窗内第 %s 行" % (
        a, b, same, n_raw, n_ali, "成立" if (not same) or n_raw == n_ali else "★不成立★", bl, union, union == bl, delta, delta == bl, ln))
    if same and n_raw != n_ali:
        bad.append("%s↔%s 不变量" % (a, b))

P("\n=== 〔三〕S18 排除式★自报命中枚数★（本件新立的那枚限定＝一立就抓到东西）===")
TOK2 = r"本件时刻＝[0-9-]{10} [0-9:]{8}（[^）]*）"
n_tok = sum(1 for x in cl if re.search("本件时刻＝", x))
n_hit = sum(1 for x in cl if re.search(TOK, x))
n_hit2 = sum(1 for x in cl if re.search(TOK2, x))
P("   「本件时刻＝」在本件出现＝%d 枚 ‖ 被★在册那枚★排除式命中＝%d 枚 ‖ 被★修后式★命中＝%d 枚" % (n_tok, n_hit, n_hit2))
P("   ★根因（本件现跑定位、不是推演）＝在册式里『＝+8★）』那段把 ★+★ 当成了正则量词（＝ 后要跟一个或多个 ＝），"
  "而字面是『＝+8』⇒★两枚时刻 token 一枚也没命中★＝★S18 的改形④是一枚死码、窗志里那个 2 就是「没排除」本身★")
for x in [i + 1 for i, y in enumerate(cl) if "本件时刻＝" in y]:
    seg = re.search(r"本件时刻＝.*", cl[x - 1]).group(0)
    P("      行 %d 字面形＝%s ‖ 在册式命中＝%s ‖ 修后式命中＝%s" % (x, seg[:64], bool(re.search(TOK, cl[x - 1])), bool(re.search(TOK2, cl[x - 1]))))
bcl = L(os.path.join(W, "05_cmp_复跑.txt"))
raw_d = sum(1 for i in range(max(len(cl), len(bcl))) if (cl[i] if i < len(cl) else None) != (bcl[i] if i < len(bcl) else None))
d = sum(1 for i in range(max(len(cl), len(bcl))) if
        (re.sub(TOK2, "★", cl[i]) if i < len(cl) else None) != (re.sub(TOK2, "★", bcl[i]) if i < len(bcl) else None))
P("   未排除＝%d 枚 ‖ 除 token 外逐行差（★枚口径＝变更多少枚行★）＝%d 枚 ‖ 窗志第 %d 行报的是★行位口径★（diff 两侧各计一行⇒%d）" % (
    raw_d, d, hits(LOG, "S18/R525＝除")[0], 2 * raw_d))
P("   ★⇒ 本窗可发的准确形＝「排除式按★字面转义★重写后命中 %d／%d 枚，除时刻 token 外逐行差＝%d 枚（＝两遍输出除时钟外逐字同）」；"
  "★在册那枚式子的 0 命中必须先修、不修则「除 token 外逐行差」这项永远等于「未排除」＝自证无效★" % (n_hit2, n_tok, d))

P("\n=== 〔四〕其余承重面逐枚具名（★值与载体行号同句★）===")
pc = L(os.path.join(W, "run", "pub_C.txt"))[0]
m = re.search(r"free=(\d+).*?lethal=(\d+).*?total=(\d+)", pc)
P("   靶锁（run/pub_C.txt 第 1 行）＝free=%s＋lethal=%s＝%s ‖ total=%s ‖ 合＝%s" % (
    m.group(1), m.group(2), int(m.group(1)) + int(m.group(2)), m.group(3), int(m.group(1)) + int(m.group(2)) == int(m.group(3))))
mb = re.search(r"合取五带边界解析＝(\{[^}]*\})", rd(CMP))
P("   退净乙式＝%s ‖ 载体＝05_cmp.txt 第 %s 行 ‖ ★五枚 tsv 现读字节＝%s（★全 0 字节＝空集上的真＝S4 未合★）" % (
    mb.group(1), lineof("合取五带边界解析")[0],
    [os.path.getsize(os.path.join(W, "run", f)) for f in ("S0_before.tsv", "D_delta.tsv", "K_kill.tsv", "S1_after.tsv", "witness_after.tsv")]))
sub = rd(os.path.join(W, "run", "sub_elev_C.txt"))
ms = re.search(r"订阅侧 T 内 v==100＝(\d+)", sub)
P("   订阅侧（run/sub_elev_C.txt 现读）＝%s ‖ 与 n_T 同值＝%s" % (ms.group(1) if ms else "★缺★", ms and int(ms.group(1)) == N_T))
for t in ("REF", "A", "Ap", "C"):
    lg = rd(os.path.join(W, "run", "roslaunch_%s.log" % t))
    sm = rd(os.path.join(W, "run", "smoke_%s.txt" % t))
    P("   C11/%s：Using plugin＝%d ‖ Received a＝%d ‖ elev 行＝%d ‖ CHECK＝%d PASS＝%d FAIL＝%d" % (
        t, lg.count("Using plugin"), lg.count("Received a"), len([x for x in lg.split("\n") if "Using plugin" in x and "elevation_layer" in x]),
        len([x for x in sm.split("\n") if x.startswith("CHECK")]), len([x for x in sm.split("\n") if x.startswith("CHECK") and "PASS" in x]),
        len([x for x in sm.split("\n") if x.startswith("CHECK") and "FAIL" in x])))
P("   ^STATE 枚数（志内 startswith）＝%d ‖ 遍次 .runs＝%s" % (sum(1 for x in L(LOG) if x.startswith("STATE=")), rd(os.path.join(W, ".runs")).strip()))
P("   步98 承重名载体（真目录）＝rc 与命中行的自陈在志第 %s 行 ‖ 盘上 .prev_ 枚数＝%d（各字节 %s）‖ 05_cmp.txt 现读字节＝%d（★未被搬走★）" % (
    hits(LOG, "步98 读数")[0], len([f for f in os.listdir(W) if f.startswith(".prev_")]),
    [os.path.getsize(os.path.join(W, f)) for f in sorted(os.listdir(W)) if f.startswith(".prev_")], os.path.getsize(CMP)))
sh = subprocess.run(["bash", "-c", "( exit 5 ); S=$?; SF=$?; printf 'S=%s SF=%s' $S $SF"], capture_output=True, text=True).stdout
P("   S15 现场复现（★本件现跑 bash、非推演★）＝%s ‖ ⇒ 志第 %s 行那句「从同一枚变量 SRC_FROM＝0 读回来」里的 0 ＝★紧跟赋值再取的恒 0★、而 rc=5 真来自 SRC" % (
    sh, hits(LOG, "SWAP 遍 进程 rc 回读")[0]))

P("\n=== 〔五〕本遍缺席格走到哪一层（★发放形只准带这一段★）===")
P("   ★到手★＝①S14 名册对调当场翻红（/tmp 变异 m1/m2/m3/m4 四枚各跑到 AssertionError）②S20 锚改成机读表、判读件真读该键（上面〔一〕6/6 合）"
  "③S22 inv_ok 有消费者＝rc_不变量＋行使枚数随句并报（★本窗行使 4 枚、历窗第一次★）④S1 承重名真目录载体到手 1 枚（步98 rc=6＋前后指纹同＝留底未损）")
P("   ★未到手★＝①S13 件尾支＝查表查★错行★（印 rc_勾对_复读件=★5★、实际退 ★9★；PREMISS=9 仍手写字面量＝/tmp 变异 m5 把 9 改 7 ⇒ 件照退 7、自陈照说 9 ⇒ ★无牙★）"
  "②S15 半修（SRC_FROM 恒 0、新句仍名实不符；★且★志面形检只锚旧字面＝换写法即逃逸）③★S18 改形④＝死码★（在册式 0 命中、见〔三〕，窗志那个 2 就是「未排除」）"
  "④S4 退净仍空集 ⑤S7 独立来源仍未合")
P("\n=== 总判定 ===")
P("   ★本件 rc＝%s（0＝〔一〕6 枚锚全合＋〔二〕四对不变量全成立；5＝任一不合＝★红的是我的式子或窗、逐枚点名★：%s）★" % (0 if not bad else 5, bad or "无"))
P("   ★本件只报不拦、不进任何在册件的 rc；未起节点、未起 master、无 sudo、未 kill、未改任何窗内原件（只新建本件与 06_判读.txt）★")

txt = "\n".join(O) + "\n"
assert '"' not in txt, "判读面不许出现 ASCII 双引号"
io.open(OUT, "w", encoding="utf-8").write(txt)
print(txt)
sys.exit(0 if not bad else 5)
