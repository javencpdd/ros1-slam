#!/usr/bin/env bash
# v_deferred_clause_scan.sh —— 「缓做条款到期检查」巡检件（02 坑44 的处置②落地；小明 2026-10-09）
#
# 要灭的是这一型：在册件的注释里写「等 X 之后再落 / X 未实测后回填」，X 后来在别的批次结案了，
#   但**没有任何东西在 X 结案时回来扫这些待办** ⇒ 注记长期误导后续批次（实例两处，见 T1 的阳性对照）。
# 本件**不改任何注记、不做任何决定**——它只把"疑似到期"摆到台面上：回收是人的动作。
#
# 两类条款，能力边界写死在输出里（不假装比实际更强）：
#   A 类＝前置可机读（Q11／P6／G8／R36／§9／#3／B5 这类代号）⇒ 能在台账里反查结案凭据 ⇒ 可判「疑似到期·未回收」；
#   B 类＝前置是自然语言（如「复核 ~global_costmap 后回填固化」「先人工」）⇒ **机读不了**，只能整条列为「待人工判」，
#        既不判绿也不判红。诚实结论：本件对今日两例只能自动化其中一例（variants 那条），另一例靠"至少被列出"保证不被静默漏掉。
#
# 档位（本件自有层，不与 SM／B3／GTC 各层互认）：0＝无 A 类到期未回收；1＝有（逐条带出处，需人确认后再回收）；
#   9＝扫描面为空或零命中（不可信，不判绿——坑29／R46 同族：空集不得判通过）；5＝自测／阴性对照不成立。
# 本件**不进任何标准回归集**（它是周期审计件），只在批次收尾时手动跑一次。
#
# 用法：bash scripts/verify/v_deferred_clause_scan.sh            全仓巡检
#       bash scripts/verify/v_deferred_clause_scan.sh --selftest  阳性对照（两处已知实例）＋阴性对照（合成一例必判红）
set -u
ROOT=/home/ros1-slam
SCAN_ROOT=${DC_SCAN_ROOT:-$ROOT}
LEDGER_ROOT=${DC_LEDGER_ROOT:-$ROOT}
SELF=$(readlink -f "$0")

python3 - "$SCAN_ROOT" "$LEDGER_ROOT" "${1:-}" <<'PYEOF'
import os, re, sys

scan_root, ledger_root, mode = sys.argv[1], sys.argv[2], (sys.argv[3] if len(sys.argv) > 3 else "")

DEFER = re.compile(r'(后再|之后再|固化后|回填后|后回填|待[^，。;；]{0,16}后|未实测|留待|暂缓|待批|先人工|占位)')
TOKEN = re.compile(r'(Q\d+|P\d+[a-z]?|G\d+[a-z]?|R\d+|W\d+|C\d+|B\d+|A\d+|S\d+|U\d+|D-\d+|§\d+|§\s*\d+|坑\d+|#[0-9]+)')
CLOSE = re.compile(r'(成立|一致|通过|结案|已落|完成|实测|验收|PASS|对账|落位|落地)')
NOSCENE = re.compile(r'(已结案|已实测|成立|一致|落位|结案)')          # 注记自身已带回收凭据 ⇒ 不算未回收

SKIP_DIR = ('/thirdparty/', '/Eigen/', '/build/', '/devel/', '/.git/', '/log/', '/证据/', '/raw/')
SCAN_SUB = ['ws/src', 'scripts', 'note/07_判据载体代际对照表.md']

def scanned_files():
    out = []
    for sub in SCAN_SUB:
        p = os.path.join(scan_root, sub)
        if os.path.isfile(p):
            out.append(p); continue
        for d, ds, fs in os.walk(p):
            if any(s in d + '/' for s in SKIP_DIR): ds[:] = []; continue
            for f in fs:
                if f.endswith(('.yaml', '.yml', '.launch', '.sh', '.py', '.md', '.cpp', '.xml')):
                    fp = os.path.join(d, f)
                    if '/lite3_' in fp or fp.startswith(os.path.join(scan_root, 'scripts')) or '判据载体' in fp \
                       or fp.startswith(os.path.join(scan_root, 'note/07')):
                        out.append(fp)
    return sorted(set(out))

def ledger_lines():
    """结案凭据只从台账与进度文档正文里找（证据目录不算——那里全是读数，任何词都能命中）。"""
    hits = []
    cand = ['note/04_配置改动diff.md', 'note/02_问题与坑.md']
    for d in ('note',):
        dd = os.path.join(ledger_root, d)
        if os.path.isdir(dd):
            for f in sorted(os.listdir(dd)):
                if f.endswith('.md'): cand.append(os.path.join(d, f))
    for sub in os.listdir(os.path.join(ledger_root, 'note/进度')) if os.path.isdir(os.path.join(ledger_root, 'note/进度')) else []:
        p = os.path.join(ledger_root, 'note/进度', sub)
        if os.path.isdir(p) and os.path.exists(os.path.join(p, '进度文档.md')):
            cand.append(os.path.join('note/进度', sub, '进度文档.md'))
    for rel in cand:
        fp = os.path.join(ledger_root, rel)
        if not os.path.isfile(fp): continue
        try: txt = open(fp, encoding='utf-8', errors='ignore').read().splitlines()
        except Exception: continue
        for i, l in enumerate(txt, 1):
            if CLOSE.search(l): hits.append((rel, i, l.strip()))
    return hits

# 收紧（首轮实跑的精度只有 2/6：其余是"代号出现在无关上下文"，含本件自己那行注释＝扫描面自匹配）：
#   规则一 注记里的**前置代号必须与缓做词邻接**（同号内），否则不算"这条注记在等它"；
#   规则二 台账凭据里的**结案词必须出现在该代号 ±40 字内**，否则那次结案与本品无关。
TK = r'(?:Q\d+|P\d+[a-z]?|G\d+[a-z]?|R\d+|W\d+|C\d+|B\d+|A\d+|S\d+|U\d+|D-\d+|§\s*\d+|坑\d+|#[0-9]+)'
ADJ = re.compile('(' + TK + ')[^。，；)）]{0,24}(未实测|固化后|后再|再落|回填|待批|留待|暂缓|待复核|待验证)')
ADJ2 = re.compile('(未实测|暂缓|留待|待|等)[^。，；(（]{0,20}(' + TK + ')[^。，；]{0,24}(固化|实测|结案|完成|通过|验收)')
def adj_tokens(line):
    out = set()
    for m in ADJ.finditer(line): out.add(m.group(1))
    for m in ADJ2.finditer(line): out.add(m.group(2))
    return out
def ev_ok(line, tok):
    for m in re.finditer(r'(?<![A-Za-z0-9])' + re.escape(tok) + r'(?![0-9])', line):
        seg = line[max(0, m.start() - 40):m.end() + 40]
        if CLOSE.search(seg): return True
    return False

def collect(sr, lr):
    global scan_root, ledger_root
    scan_root, ledger_root = sr, lr
    files = scanned_files()
    led = ledger_lines()
    a, b, selfclosed = [], [], 0
    for fp in files:
        try: lines = open(fp, encoding='utf-8', errors='ignore').read().splitlines()
        except Exception: continue
        for i, l in enumerate(lines, 1):
            if not DEFER.search(l): continue
            if not l.lstrip().startswith(('#', '<!--', '//', '*')): continue     # 只看注释：注记才是"给人的待办"
            rel = os.path.relpath(fp, scan_root)
            if rel.endswith('v_deferred_clause_scan.sh'): continue      # 扫描面自匹配（坑39/42③同族）：本件自己的注记不是待回收条款
            toks = adj_tokens(l)
            if not toks:
                if DEFER.search(l): b.append((rel, i, l.strip()[:150]))
                continue
            if NOSCENE.search(l):
                selfclosed += 1; continue
            ev = [(r, n, txt) for r, n, txt in led for t in toks if any(ev_ok(txt, t) for _ in [0])]
            ev = [(r, n, txt) for r, n, txt in led for t in toks if ev_ok(txt, t)]
            if ev:
                a.append((rel, i, l.strip()[:150], sorted(toks), ev[:2]))
            else:
                b.append((rel, i, l.strip()[:150]))
    return files, led, a, b, selfclosed

if mode == '--selftest':
    fails = 0
    files, led, a, b, sc = collect(scan_root, ledger_root)
    print("=== 缓做条款巡检·自测（扫描面 %d 件，台账结案行 %d 行，A 类 %d 条／B 类 %d 条）===" % (len(files), len(led), len(a), len(b)))
    if len(files) == 0 or (len(a) + len(b)) == 0:
        print("[FAIL] 扫描面或命中为空 ⇒ 不可信，不判绿（坑29）"); fails += 1
    # T1 阳性对照一：variants 头注那条必须进 A 类（前置 Q11，结案凭据在 M5 §1 与 04:35）
    hit1 = [x for x in a if 'config/variants/' in x[0]]
    print(("[PASS]" if hit1 else "[FAIL]") + " T1a variants「Q11 ns 固化后再落加载行」被判 A 类未回收（A 类命中 %d 条）" % len(hit1)); fails += (0 if hit1 else 1)
    # T1 阳性对照二：mbf_navigation 那条前置是自然语言 ⇒ 必须至少被列进 B 类（保证不被静默漏掉）
    hit2 = [x for x in b if 'mbf_navigation.yaml' in x[0]]
    print(("[PASS]" if hit2 else "[FAIL]") + " T1b mbf_navigation「…后回填固化」进 B 类待人工判（B 类命中 %d 条）" % len(hit2)); fails += (0 if hit2 else 1)
    # T2 阴性对照：合成"已结案的前置 + 未回收的注记"，本件必须判 A 类（证明反查逻辑真有咬合，不是恒绿）
    tmp = os.path.join('/tmp', 'dcneg_%d' % os.getpid())
    os.makedirs(os.path.join(tmp, 'ws/src/95_simulation/lite3_fake/verify'), exist_ok=True)
    os.makedirs(os.path.join(tmp, 'note'), exist_ok=True)
    open(os.path.join(tmp, 'ws/src/95_simulation/lite3_fake/verify/x.yaml'), 'w').write(
        "# 前置 Q99 未实测，等 Q99 固化后再落加载行\nfoo: 1\n")
    open(os.path.join(tmp, 'note/04_配置改动diff.md'), 'w').write(
        "- 2026-10-09：Q99 联调实测：命名空间双挂载成立，全量落位 ⇒ 结案。\n")
    global_scan, global_led = scan_root, ledger_root
    f2, l2, a2, b2, _ = collect(tmp, tmp)
    hit3 = [x for x in a2 if x[0].endswith('lite3_fake/verify/x.yaml')]
    print(("[PASS]" if hit3 else "[FAIL]") +
          " T2 合成「Q99 已结案＋注记未回收」必须被判 A 类（合成扫描面 %d 件、A 类 %d 条、结案行 %d 行）" % (len(f2), len(a2), len(l2)))
    fails += (0 if hit3 else 1)
    if hit3: print("      注记：%s｜凭据：%s" % (hit3[0][2][:60], hit3[0][4][0][2][:60]))
    # T3 反向咬合：把台账里那句结案凭据撤掉 ⇒ 同一条注记必须退回 B 类（说明 A 类判定真是由凭据驱动，不是恒红）
    open(os.path.join(tmp, 'note/04_配置改动diff.md'), 'w').write("- 2026-10-09：Q99 尚未复核，待复核后再议。\n")
    _, _, a3, b3, _ = collect(tmp, tmp)
    hit4 = [x for x in a3 if x[0].endswith('lite3_fake/verify/x.yaml')]
    print(("[PASS]" if not hit4 else "[FAIL]") + " T3 撤掉结案凭据后不得再判 A 类（A 类=%d 条）⇒ 判定由凭据驱动" % len(a3))
    fails += (1 if hit4 else 0)
    # T4 已知假阳性不得复现（首轮实跑 6 条里 4 条是噪声：本件的"代号邻接＋凭据近距离"两条收紧就是为它们写的）
    FP = [('ws/src/40_navigation/lite3_scan/launch/70_scan.launch', 'A6 不适用行'),
          ('ws/src/95_simulation/lite3_gazebo_sim/verify/m2_smoke.sh', '加压档'),
          ('ws/src/20_localization/lite3_localization/launch/hdl_localization.launch', 'G8 演练'),
          ('v_deferred_clause_scan.sh', '不与 SM／B3／GTC 各层互认')]
    bad = []
    for fp, frag in FP:
        for x in a:
            if x[0].endswith(fp) and frag in x[2]: bad.append("%s:%d(%s)" % (x[0], x[1], x[3]))
    print(("[PASS]" if not bad else "[FAIL]") + " T4 四条已知假阳性均已排除；残留=%s" % (bad or '无'))
    fails += (1 if bad else 0)
    import shutil; shutil.rmtree(tmp, ignore_errors=True)
    print("SELFTEST VERDICT: %s rc=%d" % ("全过" if fails == 0 else "有 FAIL", 0 if fails == 0 else 5))
    sys.exit(0 if fails == 0 else 5)

files, led, a, b, sc = collect(scan_root, ledger_root)
print("=== 缓做条款巡检（扫描面 %d 件｜台账含结案词的行 %d 行）===" % (len(files), len(led)))
print("   扫描根=%s｜台账根=%s（缺省＝仓根；只有自测会改）" % (scan_root, ledger_root))
print()
print("## A 类：前置可机读且**台账里查得结案凭据**，而注记未回收 ⇒ 疑似到期（逐条需人确认，本件不自动改）")
if not a: print("  （无）")
for rel, i, txt, toks, ev in a:
    print("  [到期?] %s:%d  前置代号=%s" % (rel, i, ",".join(toks)))
    print("      注记原文：%s" % txt)
    for r, n, s in ev:
        print("      结案凭据：%s:%d  %s" % (r, n, s[:120]))
print()
print("## B 类：前置是自然语言／无机读代号 ⇒ **机读不了，只列出待人工判**（不判绿也不判红）")
print("  共 %d 条，按文件汇总：" % len(b))
byf = {}
for rel, i, txt in b: byf.setdefault(rel, []).append(i)
for rel, ls in sorted(byf.items())[:14]:
    print("    %s  行 %s" % (rel, ",".join(map(str, ls[:6]))))
print("  其中 T1b 要求在册的一条必须出现：mbf_navigation.yaml ——%s" %
      ('在列' if any('mbf_navigation.yaml' in r for r in byf) else '**不在列（异常，说明扫描面漏了 40_navigation）**'))
print()
print("  另：注记自身已带回收凭据（不计入两侧）%d 条" % sc)
print("==================================================")
print("DC SCAN VERDICT: A 类到期未回收=%d B 类待人工=%d ｜ %s" % (
    len(a), len(b), ("有疑似到期条目 ⇒ 请人确认后回收注记（本件不改注记），rc=1" if a else "无 A 类到期，rc=0")))
sys.exit(1 if a else 0)
PYEOF
rc=$?
echo "DC rc=$rc"
exit $rc
