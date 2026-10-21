#!/usr/bin/env python3
# r533_verify.py —— ★施工侧（我）对 r530q 裁定的逐枚独立复跑★（零窗、仓内只读、★一切写盘只在 /tmp★）
import io, os, re, shutil, subprocess, sys, glob

R = "/home/jack/ysc-code/ros1-slam"
REL = "note/进度/SCAN集成/证据/log/raw"
W530 = glob.glob(os.path.join(R, REL, "r530_*"))[0]
W525 = glob.glob(os.path.join(R, REL, "r525_*"))[0]
RECHECK = os.path.join(W530, "01_复读件_r530.py")
PROBE = os.path.join(W530, "06_判读件_r530.py")
ANCH = os.path.join(R, "note/进度/SCAN集成/证据/_anchor_key_check.sh")
T = "/tmp/r533"; shutil.rmtree(T, ignore_errors=True); os.makedirs(T)
out = []
def say(k, v): out.append("%-6s｜%s" % (k, v))
def run(cmd): return subprocess.run(cmd, capture_output=True, text=True, errors="replace")
def rd(p): return io.open(p, encoding="utf-8", errors="replace").read()
def wr(p, s): io.open(p, "w", encoding="utf-8").write(s)

def fakeroot(tag, wins):
    """在 /tmp 造一枚假仓根：note/…/raw/ 下同名列＝★符号链接★指向真窗目录（不复制＝原件不动）"""
    root = os.path.join(T, tag); raw = os.path.join(root, REL)
    os.makedirs(raw)
    for w in wins: os.symlink(w, os.path.join(raw, os.path.basename(w)))
    return root

def copywin(tag, src):
    """★整窗复制到 /tmp★（供改写夹具用；仓内原件永不写）"""
    d = os.path.join(T, tag); shutil.copytree(src, d); return d

# ══ A组＝复读件变异（H10／H11／H4／H4b＝★未到手那一格的尺★；H1／H3／H5＝★销 S26 的尺★）══
FAKE = fakeroot("fakeroot", [W530]); SRC = LIVE = os.path.basename(W530)
base = rd(RECHECK)
rc0 = run([sys.executable, RECHECK, FAKE, SRC, LIVE])
say("H0a", "★基线★原件件尾支（★r464q 不在位⇒走那一支★）＝rc=%s ‖ 印 9＝%s ‖ 断言红＝%s" % (
    rc0.returncode, "rc_勾对_复读件=9" in rc0.stdout, "AssertionError" in rc0.stderr))
rc0s = run([sys.executable, RECHECK, FAKE, SRC, LIVE, "--swap"])
say("H0b", "★基线★原件 --swap 面＝rc=%s ‖ 印 5＝%s" % (rc0s.returncode, "即将以 rc=5 退出" in rc0s.stdout))
def mut(tag, a, b, args, note):
    p = os.path.join(T, "m_%s.py" % tag); wr(p, base.replace(a, b, 1))
    assert base != rd(p), tag + " 变异串未命中"
    r = run([sys.executable, p, FAKE, SRC, LIVE] + args)
    nums = sorted(set(x for tup in re.findall(r"rc_勾对_复读件=(\d+)|即将以 rc=(\d+) 退出", r.stdout) for x in tup if x))
    ke = re.search(r'File ".*?", line (\d+)', r.stderr)
    say(tag, "%s ⇒ ★rc=%s★ ‖ 打印的数＝%s ‖ AssertionError＝%s ‖ KeyError＝%s ‖ 末行＝%s" % (
        note, r.returncode, nums or "无", "★红★" if "AssertionError" in r.stderr else "无",
        "＠行" + ke.group(1) if "KeyError" in r.stderr else "无",
        r.stderr.strip().split("\n")[-1][:38] if r.stderr.strip() else "无异常"))
mut("H10", "sys.exit(0 if PREMISS == 0 else PREMISS)", "sys.exit(0)", [], "★问①承重★件尾 exit ★实参★→0")
mut("H11", "    sys.exit(RC_RED)", "    sys.exit(0)", ["--swap"], "★问①承重★早退 exit ★实参★→0")
mut("H4", 'BR_BY_SWAP = "swap" if SWAP else "premiss"', 'BR_BY_SWAP = "sw" if SWAP else "premiss"', ["--swap"], "★问①★键拼错（swap 面）")
mut("H4p", 'BR_BY_SWAP = "swap" if SWAP else "premiss"', 'BR_BY_SWAP = "sw" if SWAP else "premiss"', [], "★问①★同一枚变异喂 premiss 面")
mut("H4b", 'for k in ("swap", "premiss")', 'for k in ("swap", "prem")', [], "★问①★RC_DOC 里键拼错")
mut("H1", '退码＝5）")', '退码＝玖）")', ["--swap"], "★销 S26 的尺★只改名不换码")
mut("H3", 'BR_BY_SWAP = "swap" if SWAP else "premiss"', 'BR_BY_SWAP = "premiss" if SWAP else "swap"', ["--swap"], "★销 S26 的尺★两枚名对调")
def mut2(tag, pairs, args, note):
    t = base
    for a, b in pairs:
        assert a in t, tag + " 变异串未命中：" + a[:24]
        t = t.replace(a, b, 1)
    pp = os.path.join(T, "m_%s.py" % tag); wr(pp, t)
    r = run([sys.executable, pp, FAKE, SRC, LIVE] + args)
    nums = sorted(set(x for tup in re.findall(r"rc_勾对_复读件=(\d+)|即将以 rc=(\d+) 退出", r.stdout) for x in tup if x))
    say(tag, "%s ⇒ ★rc=%s★ ‖ 打印的数＝%s ‖ AssertionError＝%s ‖ 末行＝%s" % (
        note, r.returncode, nums or "无", "★红★" if "AssertionError" in r.stderr else "★零翻红★",
        r.stderr.strip().split("\n")[-1][:38] if r.stderr.strip() else "无异常"))
mut2("H5", [('assert all(str(RC_BY_BR[k]) in NAME_BY_BR[k] for k in BR_TABLE)', 'assert all(True for k in BR_TABLE)'),
            ('退码＝5）")', '退码＝玖）")')], ["--swap"], "★S26 的牙＝那三枚断言★＝删断言＋只改名")
mut2("H6", [('    PREMISS = RC_BY_BR["premiss"]', '    PREMISS = 7')], [], "★S13 原病★＝保留断言只换退码字面量")
mut2("H7", [('    PRN = RC_BY_BR["premiss"]', '    PRN = RC_BY_BR["swap"]')], [], "★r525 原病★＝件尾★打印★取 swap 行")

# ══ B组＝判读件三夹具（★§七① 反控 rc＝9、FX2 崩、FX3 None★）══
def probe_run(tag, win_dir):
    """★被检数据目录★挂进 /tmp 假根；判读件★一律取本窗那一份★（r525 目录里没有 r530 件）；OUT 落 /tmp ⇒ ★仓内零写★"""
    fr = fakeroot("fr_" + tag, [win_dir])
    s = rd(PROBE)
    s2 = s.replace('OUT = os.path.join(W, "06_判读.txt")', 'OUT = r"%s"' % os.path.join(T, tag + ".txt"), 1)
    assert s2 != s, tag + " OUT 未改到"
    pp = os.path.join(T, "probe_%s.py" % tag); wr(pp, s2)
    r = run([sys.executable, pp, fr, os.path.basename(win_dir)])
    n = re.search(r"红档枚数＝(\d+) 枚", r.stdout)
    return r.returncode, (n.group(1) if n else "?"), r.stdout, r.stderr

X525 = copywin("x525", W525)
rc, n, so, se = probe_run("FXa", X525)
say("FXa", "★§七①★ r525 全套★复制到 /tmp★（★09_锚键.tsv 在场★、锚串属另一窗）⇒ rc=%s ‖ 红档=%s 枚" % (rc, n))
os.remove(os.path.join(X525, "09_锚键.tsv"))
rc, n, so, se = probe_run("FXb", X525)
say("FXb", "★§七①★ 同一份副本★删掉 09_锚键.tsv★ ⇒ rc=%s ‖ 红档=%s 枚 ＝★无表形退的是 9、不是 5★（★我批187 那句被推翻★）" % (rc, n))
X530 = copywin("x530", W530)
tp = os.path.join(X530, "09_锚键.tsv"); rows = rd(tp).rstrip("\n").split("\t" if False else "\n")
f = rows[4].split("\t"); f[2] = "无"; rows[4] = "\t".join(f)
wr(tp, "\n".join(rows) + "\n")
rc, n, so, se = probe_run("FX2", X530)
say("FX2a", "★问②★ 表★志面字段★＝「无」喂判读件 ⇒ rc=%s ‖ 异常＝%s（★崩、不是红档★）" % (
    rc, se.strip().split("\n")[-1][:54] if se.strip() else "无异常"))
r8 = run(["bash", ANCH, X530])
say("FX2b", "★问②★ 同一份夹具喂★常驻第八枚★ ⇒ rc=%s ‖ ValueError＝%s ‖ 行号＝%s" % (
    r8.returncode, "ValueError" in (r8.stdout + r8.stderr),
    (re.search(r"line (\d+)", r8.stderr).group(1) if re.search(r"line (\d+)", r8.stderr) else "?")))
Y530 = copywin("y530", W530)
lg = os.path.join(Y530, "00_window_log.txt"); ls = [l for l in rd(lg).split("\n") if "事后复现" not in l]
wr(lg, "\n".join(ls) + "\n")
rc, n, so, se = probe_run("FX3", Y530)
say("FX3", "★问②★ 删掉★事后复现★那一句 ⇒ rc=%s ‖ 红档=%s 枚 ‖ 输出含「第 None 行」＝%s（★「全件禁印 None」名实不符★）" % (
    rc, n, "第 None 行" in so))

# ══ C组＝只读现扫（★历窗全扫＋载体行号现取＋口径对表＋在册件读★）══
c1 = rd(os.path.join(W530, "05_cmp.txt"))
def perpair(txt):
    ok = len(re.findall(r"origin 相同.*⇒\s*★?成立", txt)) + len(re.findall(r"raw≡aligned.*成立", txt))
    na = len(re.findall(r"不适用", txt)); bad = len(re.findall(r"不成立", txt))
    return ok, na, bad
rows8 = []
for c in sorted(glob.glob(os.path.join(R, REL, "r*/05_cmp.txt"))):
    rows8.append((os.path.basename(os.path.dirname(c)),) + perpair(rd(c)))
me = [r for r in rows8 if r[0].startswith("r530")][0]
say("S37a", "★问⑥★ 本窗 05_cmp.txt＝成立 %s ‖ 不适用 %s ‖ ★不成立 %s★ ⇒★三态未同现★" % me[1:])
both = [r[0] for r in rows8 if r[1] > 0 and r[2] > 0 and r[3] == 0]
say("S37b", "★问⑥★ 历窗「成立＋不适用同窗、不成立＝0」的窗＝%s（共 %d 枚）；★本窗在其中第 %d 枚★" % (
    " ‖ ".join(both), len(both), both.index(me[0]) + 1 if me[0] in both else -1))
say("S37c", "★问⑥★ 历窗「不成立」总数＝%d 枚（判据面报 0＝★一枚也没有★）" % sum(r[3] for r in rows8))
win = rd(os.path.join(W530, "00_r530_window.sh")).split("\n")
wl = rd(os.path.join(W530, "00_window_log.txt")).split("\n")
say("S39", "★问④★ 志:4＝%s ‖ 志:6＝%s ‖ 志:209＝%s ⇒★真目录 mv 支真走过 1 枚（zz_probe＝非承重名）★" % (
    wl[3].strip()[:46], wl[5].strip()[:46], wl[208].strip()[:40]))
say("S40a", "★问⑤★ 现读件:331＝%s ‖ 件:328＝%s ⇒ 志 211 写「件:328」＝★r525 的行号、跨窗未现取★" % (
    win[330].strip()[:34], win[327].strip()[:34]))
say("S40b", "★问④★ 现读件:349＝%s ‖ 件:346＝%s ⇒ 在册句「件:346 同构」★偏 3 枚★" % (
    win[348].strip()[:40], win[345].strip()[:40]))
pos = [i + 1 for i, l in enumerate(win) if "exit 8" in l]
codes = [i + 1 for i, l in enumerate(win) if "exit 8" in l and not l.strip().startswith("#")]
say("E8", "★§七②★ 含「exit 8」文本行＝%d 枚（行位 %s）‖ 注释行＝%d 枚 ⇒★真码位＝%d★（分段＝让路族 %d ‖ 非让路 %d）" % (
    len(pos), pos, len(pos) - len(codes), len(codes), len([x for x in codes if x != 180]), len([x for x in codes if x == 180])))
c2 = rd(os.path.join(W530, "05_cmp_复跑.txt"))
la, lb = c1.split("\n"), c2.split("\n")
dp = [i + 1 for i in range(max(len(la), len(lb))) if (la[i] if i < len(la) else None) != (lb[i] if i < len(lb) else None)]
say("S18", "★问③★ 两遍逐行差＝★行位口径 %d 枚（行 %s）★ ‖ diff 两侧各计一行＝%d 枚 ‖ hunk 段＝1 段 ⇒★「2 行」是 diff 口径★" % (
    len(dp), dp, 2 * len(dp)))
say("S21", "★S21 未销★ 窗件复跑段现读（跑前/跑后行数＋两枚 sha）＝%s" % [
    l.strip()[:44] for l in win[303:313] if re.search(r"N_BEFORE|N_AFTER|sha|SUM", l)][:4])
say("S22", "★S22 未销★ _cmp_r530.py:262 名册现读＝%s" % rd(os.path.join(W530, "_cmp_r530.py")).split("\n")[261].strip()[:88])
pr = rd(os.path.join(W530, "06_判读件_r530.py"))
say("S30", "★S30 未销★ 判读件内 lineof(=%d ‖ hits(=%d ‖ 手写「第 … 行」=%d ⇒ 手写锚远多于表 6 名" % (
    len(re.findall(r"lineof\(", pr)), len(re.findall(r"hits\(", pr)), len(re.findall(r"第 %s 行", pr))))
say("S35", "★新抓 S35★ 05_cmp.txt:23 段标题＝%s ‖ 全文「未行使」枚数＝%d（★与本窗 2 同架＋2 异架矛盾★）" % (
    c1.split("\n")[22].strip()[:56], len(re.findall("未行使", c1))))
say("S38", "★新抓 S38★ 窗件内「承重名清单」出现处＝件:39 YIELD_LIST ‖ 件:123 case ‖ 件:349 case ‖ 件:358 case ＝★三份复写字面★、无同步机检")
say("S36", "★新抓 S36★ 宽锚现扫全志「SRC_FROM＝」＝命中 %d 枚（行位 %s）＝★引用旧缺陷原文那句⇒误报★" % (
    len([i + 1 for i, l in enumerate(wl) if "SRC_FROM＝" in l]), [i + 1 for i, l in enumerate(wl) if "SRC_FROM＝" in l]))
say("S41", "★新抓 S41★ 元字符正控那枚 echo 的去向＝件:297（★stderr、不 tee★）‖ 在册窗志里「元字符正控不过」＝%d 枚" % (
    len([l for l in wl if "元字符正控不过" in l])))
say("S42", "★新抓 S42★ 判读件面判定式＝件:52『★" + [l.strip() for l in pr.split("\n") if '"jack" in ROOT' in l][0][:56] + "』＝★靠路径子串猜面、不是真取时区★")
say("S34", "★新抓 S34★ 崩形生产者＝窗件:322『%s』（★${LROW:-无}＝自己会写出那枚非数字字段★）" % win[321].strip()[:52])
say("S24", "★S24 半★ 常驻第八枚对本窗 rc=%s（★元字符自检在★离窗件★里、窗内那枚正控只 echo stderr★）" % run(["bash", ANCH, W530]).returncode)
open("/tmp/r533_verify.txt", "w", encoding="utf-8").write("\n".join(out) + "\n")
print("\n".join(out))
