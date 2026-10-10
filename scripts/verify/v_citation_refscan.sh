#!/usr/bin/env bash
# v_citation_refscan.sh —— 「我正要改这个件，谁按行号引用它」反扫件（**候补闸第四件，未入门禁串**；小明 2026-10-11）
#
# 为何存在（02 坑61）：R111 我按裁定给两件 SCAN harness 中部各插 5 行，改动本身没错，
#   但我把它只当「件面变更」、没当「引用面变更」⇒ 顶走了在册 5 处指针，而且**没有任何东西变红**：
#   漂掉的指针语法合法、点开是另一行。事后是 VCC 抓到 1 条、手工扩扫才捞出其余 4 条 ⇒ 缺一个事前件。
#   现有正式串（SYN／GTC／ISO／DC／v_py_static／VAR）与候补三件（VAR／v_py_static／VCC）都不覆盖这件事。
#
# 判的是一件：被改件的**行号引用清单**——逐条给出「哪个文档的哪一行、引到第几行、那一行现在是什么」。
#   它**不自动判漂**（合不合意只有写引用的人知道），只保证「改之前你看见了自己会撞到谁」。
# 自标定（阳性对照在前，坑29／G05 C4a 姿势）：--selftest 造 1 条真引用（须命中）＋1 个无人引用的件（须零命中）；
#   任一对不上 ⇒ rc=9，普查结论不采信。
# 用法：bash scripts/verify/v_citation_refscan.sh <仓内相对路径> [...]        # 例：ws/src/.../v_scan_p1.sh
#       bash scripts/verify/v_citation_refscan.sh --selftest
#   rc：0 零命中（可以放心改）｜5 有行号引用（须逐条看，改完同批修引用形状或开移交单）｜9 前置缺失或自标定不过
# 性质：只读。零写盘（临时件在 /tmp，跑完即删）、零节点、无 sudo。是否入门禁串＝归 A15／#54／#16 那一族，不自落。
set -u
ROOT="${RS_ROOT:-/home/ros1-slam}"
cd "$ROOT" || { echo "前置缺失：仓根不在位"; exit 9; }
[ -d note ] || { echo "前置缺失：note/ 不在位"; exit 9; }
MODE="scan"
case "${1:-}" in
  --selftest) MODE="selftest" ;;
esac
if [ "$MODE" = "scan" ] && [ "$#" -eq 0 ]; then echo "用法错：未给被改件（空集不判绿）"; exit 9; fi

python3 - "$MODE" "$ROOT" "$@" <<'PYEOF'
# -*- coding: utf-8 -*-
import glob, io, os, re, subprocess, sys

MODE, ROOT = sys.argv[1], sys.argv[2]
TARGETS = sys.argv[3:]
CITE = re.compile(r'([A-Za-z0-9_\-./\u4e00-\u9fff]+\.(?:sh|py|md|ya?ml|launch|world|pcd|csv|txt|xacro|json|cpp|h|bak|log))[:：](\d+)')
DOCS = sorted(glob.glob("note/**/*.md", recursive=True))


def read(p):
    try:
        return io.open(p, encoding="utf-8", errors="replace").read().splitlines()
    except Exception:
        return []


def refscan(targets, docs):
    """返回 {被改件: [(引用文档, 文档行号, 引用的行号, 被引行现在内容, 引用句片段)]}"""
    hits = {}
    for t in targets:
        hits[t] = []
        if not os.path.exists(t):
            print("  [警告] 被改件不在盘上：%s（属前置缺失，不计入零命中）" % t)
            continue
        tlines = read(t)
        # 留底对撞：若给了 RS_BEFORE_DIR（内有 <basename>.before），同一条行的**旧内容**一并取出
        #   ⇒ 同一行号「旧≠今」就是被本次改动顶走的直接证据（不靠人猜片段说的是哪个构造）
        bdir = os.environ.get("RS_BEFORE_DIR", "")
        blines = []
        bpath = os.path.join(bdir, os.path.basename(t) + ".before") if bdir else ""
        if bpath and os.path.exists(bpath):
            blines = read(bpath)
        tb = os.path.basename(t)
        for d in docs:
            for i, ln in enumerate(read(d), 1):
                for m in CITE.finditer(ln):
                    name, num = m.group(1), int(m.group(2))
                    same = (name == t) or (name == tb) or t.endswith("/" + name) or name.endswith("/" + tb)
                    if not same:
                        continue
                    cur = tlines[num - 1].strip() if 0 < num <= len(tlines) else "（越界：该件现 %d 行）" % len(tlines)
                    snip = ln[max(0, m.start() - 28):m.end() + 40].replace("\n", " ")
                    if blines:
                        old = blines[num - 1].strip() if 0 < num <= len(blines) else "（留底越界：改前 %d 行）" % len(blines)
                        tag = "★漂(该行内容已变)" if old != cur else "同一行内容未变"
                        # 再看它是否只是整体位移：同一段内容在别处还在 ⇒ 给出新行号
                        moved = ""
                        if old and old != cur:
                            where = [j for j, x in enumerate(tlines, 1) if x.strip() == old]
                            if where:
                                moved = "；该内容现在第 %s 行" % where[:3]
                        hits[t].append((d, i, num, "%s ｜ %s%s｜改前=%s" % (cur[:60], tag, moved, old[:40]), snip))
                    else:
                        hits[t].append((d, i, num, cur[:70], snip))
    return hits


if MODE == "selftest":
    T = subprocess.run(["mktemp", "-d", "/tmp/rs_selftest.XXXXXX"], capture_output=True, text=True).stdout.strip()
    try:
        real = "scripts/verify/v_sh_py_syntax.sh"
        assert os.path.exists(real), "自测前置缺失"
        d1 = os.path.join(T, "doc_citing.md")
        io.open(d1, "w", encoding="utf-8").write("引用 %s:12 的某行\n" % real)
        orphan = os.path.join(T, "nobody_cites_me.sh")
        io.open(orphan, "w", encoding="utf-8").write("#!/usr/bin/env bash\necho x\n")
        # 自测**不往仓里写任何文件**（写进 note/ 万一没清干净就是污染工作树）⇒ 直接把临时文档当扫描面喂进去
        io.open(d1, "w", encoding="utf-8").write("引用 %s:12 的某行\n" % real)
        docs = [d1]
        pos = refscan([real], docs)[real]
        neg = refscan([orphan], docs)
        ok1 = any(h[2] == 12 for h in pos)
        ok2 = orphan in neg and len(neg[orphan]) == 0
        print("自标定：阳性（引 v_sh_py_syntax.sh:12 必须命中现值）= %s" % ("PASS" if ok1 else "FAIL"))
        print("        阴性（不存在的被改件必须零命中）= %s" % ("PASS" if ok2 else "FAIL"))
        if not (ok1 and ok2):
            print("REFSCAN SELFTEST: FAIL rc=9"); sys.exit(9)
        print("REFSCAN SELFTEST: PASS rc=0"); sys.exit(0)
    finally:
        subprocess.run(["rm", "-rf", T])
    sys.exit(0)

print("扫描面＝note/ 全部 .md %d 件；被改件 %d 个" % (len(DOCS), len(TARGETS)))
total = 0
for t, hs in sorted(refscan(TARGETS, DOCS).items()):
    print("\n### 被改件：%s（%d 处行号引用）" % (t, len(hs)))
    for d, i, num, cur, snip in hs:
        total += 1
        print("  - 引文：%s:%d" % (d, i))
        print("      指到该件第 %s 行，该行现在是：%s" % (num, cur))
        print("      原句片段：…%s…" % snip)
print("\n合计行号引用=%d" % total)
if total:
    print("REFSCAN VERDICT: rc=5 —— 有 %d 处按行号的引用会随本次改动漂移：改完必须逐条核，"
          "在册他人件与台账原文**不代改**（开移交单），自己的件按坑60 改成锚文本＋现行号" % total)
    sys.exit(5)
print("REFSCAN VERDICT: rc=0 —— 零行号引用，本次改动不会顶走任何在册指针")
sys.exit(0)
PYEOF
