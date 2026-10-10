#!/usr/bin/env bash
# w_drift_survey.sh —— 全仓单点引用的「版本漂移」普查件（只读；候选闸 VCC 的**上游扩展**，不改 VCC 判据）
#
# 为什么另起一件而不是改 VCC：VCC 判据（RED/AMBER/FIXUP/非单点形状）已入 02 坑60 的在册口径，
#   改它＝改判据，归请求单 A15/#54/#16 那一族，我不自落。本件只做两件 VCC 没做的事：
#   ① 把扫描面从缺省 20 份扩到 note/ 全仓 .md（分「活文档域／证据域」两格报）；
#   ② 对 VCC 判出的每条 RED/AMBER 回原文定分型——含**面外件**（/opt/ros 系统头，VCC 只索引仓根所以必然黄）、
#      **行号漂移**（锚在别处 ⇒ 用 git 历史证"写下时是否逐字正确"）、**键名错**（锚全文不现）。
#
# 自证（阳性对照在前）：先解析 VCC 自己的「[计数]」行，再核对逐条明细的条数；对不上 ⇒ rc=9，普查一律不采信。
# rc：0 每条 RED/AMBER 都定到型｜7 有「未定」条｜9 前置缺失或自证不过｜6 VCC 报 RED（透传信息，不改变本件判定）
set -u
ROOT="${W_ROOT:-/home/ros1-slam}"
SELF=$(readlink -f "$0")
cd "$ROOT" || exit 9
[ -r scripts/verify/v_citation_check.sh ] || { echo "前置缺失：VCC 件不在位"; exit 9; }

mapfile -t DOCS < <(find note -name '*.md' | sort)
[ "${#DOCS[@]}" -gt 0 ] || { echo "前置缺失：扫描面为空"; exit 9; }
DOCS_LIVE=(); DOCS_EV=()
for d in "${DOCS[@]}"; do
  case "$d" in
    */证据/*) DOCS_EV+=("$d") ;;
    *)        DOCS_LIVE+=("$d") ;;
  esac
done
echo "扫描面=全仓 ${#DOCS[@]} 件 ｜ 活文档域 ${#DOCS_LIVE[@]} 件（可整改）｜ 证据域 ${#DOCS_EV[@]} 件（原件冻结、只登勘误不回改）"

OUTDIR=$(dirname "$SELF")
VCC_OUT="${W_VCC_OUT:-$OUTDIR/vcc_full.out}"
VCC_MAX="${#DOCS[@]}" bash scripts/verify/v_citation_check.sh "${DOCS[@]}" > "$VCC_OUT" 2>&1
rc_vcc=$?
echo "VCC 全仓面 rc=$rc_vcc（0 干净／5 有 AMBER／6 有 RED）文档=${#DOCS[@]} 件；原件输出见 $VCC_OUT"
for dom in live ev; do
  case "$dom" in
    live) set -- "${DOCS_LIVE[@]}"; f="$OUTDIR/vcc_live.out" ;;
    ev)   set -- "${DOCS_EV[@]}";   f="$OUTDIR/vcc_evidence.out" ;;
  esac
  [ "$#" -gt 0 ] || { echo "域面为空：$dom 不生成"; continue; }
  VCC_MAX=0 bash scripts/verify/v_citation_check.sh "$@" > "$f" 2>&1
  echo "域面 $dom（$# 件）$(grep '^\[计数\]' "$f")"
done

python3 - "$SELF" "$ROOT" "$VCC_OUT" "${DOCS[@]}" <<'PYEOF'
# -*- coding: utf-8 -*-
import os, re, subprocess, sys

SELF, ROOT, VCC_OUT = sys.argv[1], sys.argv[2], sys.argv[3]
DOCS = sys.argv[4:]
CITE = re.compile(r'([A-Za-z0-9_\-./\u4e00-\u9fff]+\.(?:sh|py|md|ya?ml|launch|world|pcd|csv|txt|xacro|json|cpp|h|bak|log))[:：](\d+)')
KEY_RE = re.compile(r'键名「([^」]+)」不在该行')
SYSROOTS = ["/opt/ros"]

def sh(args):
    try:
        p = subprocess.run(args, cwd=ROOT, capture_output=True, text=True)
        return p.returncode, p.stdout, p.stderr
    except Exception as e:
        return 9, "", str(e)

# --- VCC 只索引仓根 ⇒ 系统头必然报"仓内根本没有这个文件名"。本件另建 /opt/ros 索引（只读）---
sys_index = {}
for sr in SYSROOTS:
    if not os.path.isdir(sr):
        continue
    for dirpath, dirnames, filenames in os.walk(sr):
        dirnames[:] = [d for d in dirnames if d not in ("share", "lib", "bin")]
        for fn in filenames:
            sys_index.setdefault(fn, []).append(os.path.join(dirpath, fn))

repo_index = {}
for dirpath, dirnames, filenames in os.walk(ROOT):
    dirnames[:] = [d for d in dirnames if d not in (".git", "__pycache__", "build", "devel")]
    for fn in filenames:
        repo_index.setdefault(fn, []).append(os.path.join(dirpath, fn))

def read_lines(path):
    try:
        with open(path, "r", encoding="utf-8", errors="replace") as f:
            return f.read().splitlines()
    except Exception:
        return []

def grep_line(lng, key):
    out = []
    for i, ln in enumerate(lng, 1):
        if key and key in ln:
            out.append(i)
    return out

def git_first_ok(rel, key, want_line):
    """在被引件的历史版本里找"锚恰好在被引那一行"的 commit——找到即证"写下时逐字正确"。"""
    _, out, _ = sh(["git", "log", "--format=%h", "--", rel])
    commits = out.split()[:40]
    for c in commits:
        _, body, _ = sh(["git", "show", "%s:%s" % (c, rel)])
        lines = body.splitlines()
        if want_line <= len(lines) and key and key in lines[want_line - 1]:
            return c, len(lines)
    return None, None


def hist_max_lines(rel):
    """该件历史上最多有多少行：区分"从没这么长(硬错)"与"曾这么长、后被改短(漂移)"。"""
    _, out, _ = sh(["git", "log", "--format=%h", "--", rel])
    commits = out.split()[:60]
    best = (0, None, len(commits))
    for c in commits:
        _, body, _ = sh(["git", "show", "%s:%s" % (c, rel)])
        n = len(body.splitlines())
        if n > best[0]:
            best = (n, c, len(commits))
    return best


def near_names(base):
    """给"仓内根本没有这个文件名"那条一个近名建议（在册坑52 的修法：写全名）。"""
    stem = os.path.splitext(base)[0]
    if len(stem) < 4:
        return []
    return sorted(k for k in repo_index if stem in k and k != base)[:5]


FIXUP = re.compile(r'写成|实为|勘误|应为|误为|改成|更正|原文不回改|已删|已移除|不在位|历史|当时|曾为')
doc_index = {}
for d in DOCS:
    doc_index.setdefault(os.path.basename(d), []).append(d)
doc_line_cache = {}


def locate_citation(doc_base, target, lineno):
    """把 VCC 明细里的 basename 还原成「哪个件的哪一行」，并判该句是否在册勘误句。
    注意：VCC 会把简写解析成仓内全路径（如 lvr2 CMakeLists.txt ⇒ ws/src/CMakeLists.txt），
    所以这里按 **basename:行号** 找原文，而不是按解析后的全路径找，否则原句找不到。"""
    base = os.path.basename(target)
    nrepo = len(repo_index.get(base, []))
    nsys = len(sys_index.get(base, []))
    hits = []
    for dp in doc_index.get(doc_base, []):
        if dp not in doc_line_cache:
            try:
                with open(dp, "r", encoding="utf-8", errors="replace") as f:
                    doc_line_cache[dp] = f.read().splitlines()
            except Exception:
                doc_line_cache[dp] = []
        for i, ln in enumerate(doc_line_cache[dp], 1):
            if "%s:%d" % (base, lineno) in ln:
                hits.append((dp, i, bool(FIXUP.search(ln))))
    return hits, (nrepo, nsys)

verdicts = []
red_items, amber_items = [], []
counts = {}
with open(VCC_OUT, "r", encoding="utf-8") as f:
    for ln in f.read().splitlines():
        if ln.startswith("[RED]"):
            red_items.append(ln)
        elif ln.startswith("[AMBER]"):
            amber_items.append(ln)
        elif ln.startswith("[计数]"):
            counts = dict(re.findall(r'(单点引用|RED|AMBER|OK|非单点形状|勘误句)=(\d+)', ln))

print("解析 VCC 明细：RED=%d AMBER=%d ｜ VCC 自报 RED=%s AMBER=%s 单点引用=%s"
      % (len(red_items), len(amber_items), counts.get("RED"), counts.get("AMBER"), counts.get("单点引用")))
if int(counts.get("RED", -1)) != len(red_items) or int(counts.get("AMBER", -1)) != len(amber_items):
    print("自证不过：明细条数与 VCC 自报计数不符 ⇒ 普查不采信")
    sys.exit(9)

def classify(kind, body):
    m = re.match(r'\[(RED|AMBER)\] (\S+) ⇒ (\S+):(\d+)(.*)$', body)
    if not m:
        return ("未定", "明细行形状不认识")
    _kind, doc, target, lineno_s, rest = m.groups()
    lineno = int(lineno_s)
    km = KEY_RE.search(rest)
    key = km.group(1) if km else ""
    outside = "仓内根本没有这个文件名" in rest
    candidates = []
    base = os.path.basename(target)
    if outside:
        candidates = sys_index.get(base, [])
        where = "系统域(/opt/ros)" if candidates else "系统域也没有"
    else:
        rel = target if os.path.isabs(target) else os.path.join(ROOT, target)
        if os.path.exists(rel):
            candidates = [rel]
            where = "仓内"
        else:
            candidates = repo_index.get(base, [])
            where = "仓内按 basename 找回" if candidates else "仓内找不到"
    if not candidates:
        nn = near_names(base)
        return ("面外件·无法定位" + ("（有近名候选⇒疑省写名）" if nn else "（盘上无同名⇒须给全名）"),
                "%s ｜ 近名候选 %s" % (where, nn if nn else "无"))
    path = candidates[0]
    real = os.path.realpath(path)
    islink = os.path.islink(path)
    lng = read_lines(path)
    if lineno > len(lng):
        hits = grep_line(lng, key) if key else []
        if islink and not real.startswith(ROOT):
            return ("越界(现 %d 行)｜**符号链接件**，实身在仓外 ⇒ git 史不可判（行号由安装版本限定）" % len(lng),
                    "链接目标=%s；锚行 %s" % (real, hits[:6] if hits else "全文无此键名"))
        relp = os.path.relpath(path, ROOT)
        mx, mxc, nver = hist_max_lines(relp)
        shape = ("符号链接件·" if islink else "") + "越界(现 %d 行｜该件历史最多 %d 行，查 %d 版)" % (len(lng), mx, nver)
        if hits:
            shape += "｜锚在别处⇒漂移候选"
        elif mx < lineno:
            shape += "｜历史从未达该行⇒硬错（指向不存在的行）"
        else:
            shape += "｜历史曾达（见 %s 版）⇒改短型漂移" % mxc
        return (shape, "%s ｜ 锚行 %s" % (where, hits[:6] if hits else "全文无此键名"))
    inline = lng[lineno - 1].strip()
    tag = "面外件(系统域)·" if outside else ("符号链接件·" if (islink and not real.startswith(ROOT)) else "")
    if not key:
        return (tag + "行在、未给键名(口径松)", "%s｜现该行=%s" % (path, inline[:110]))
    if key in inline:
        return (tag + "行号对(键名在该行)⇒VCC 黄因覆盖面", "%s｜现该行=%s" % (path, inline[:110]))
    hits = grep_line(lng, key)
    if not hits and "." in key:
        leaf = key.split(".")[-1]
        lhits = grep_line(lng, leaf)
        if lhits:
            return (tag + "口径松(在册写点路径、件内是嵌套键，叶名在 %s)" % lhits[:6],
                    "现该行=%s" % inline[:110])
    if hits:
        if not outside:
            relp = os.path.relpath(path, ROOT)
            c, n = git_first_ok(relp, key, lineno)
            if c:
                return ("漂移·已证(写下时正确)", "锚现在 %s；历史上 %s 版该行含锚" % (hits[:6], c))
        return (tag + "漂移候选(锚在别处，未证历史)", "现锚行 %s" % hits[:6])
    return (tag + "键名错(锚全文不现)", "现该行=%s" % inline[:110])

def cite_where(body):
    m = re.match(r'\[(?:RED|AMBER)\] (\S+) ⇒ (\S+):(\d+)', body)
    if not m:
        return ("本档位置=明细形状不认识", False)
    db, tgt, ln = m.groups()
    hits, nc = locate_citation(db, tgt, int(ln))
    if not hits:
        return ("本档位置=未找回（写法与解析后路径不同）", False)
    parts = ["%s:%d%s" % (p, i, "〔勘误句〕" if fx else "") for p, i, fx in hits[:3]]
    all_fx = all(fx for _, _, fx in hits)
    return ("本档位置=%s｜同名件数 仓内%d/系统域%d ⇒ %s"
            % (" ; ".join(parts), nc[0], nc[1],
               "件名多义(须给全名)" if (nc[0] + nc[1]) > 1 else "件名唯一")), all_fx


print("\n==== RED 逐条 ====")
for b in red_items:
    v, ev = classify("RED", b)
    wh, fx = cite_where(b)
    if fx and not v.startswith("面外件"):
        v = "在册勘误句内(非活错)·" + v
    verdicts.append(("RED", v))
    print("%s\n    型=%s ｜ 证据=%s ｜ %s" % (b[:150], v, ev, wh))

print("\n==== AMBER 逐条 ====")
for b in amber_items:
    v, ev = classify("AMBER", b)
    wh, fx = cite_where(b)
    if fx and not v.startswith("面外件"):
        v = "在册勘误句内(非活错)·" + v
    verdicts.append(("AMBER", v))
    print("%s\n    型=%s ｜ 证据=%s ｜ %s" % (b[:150], v, ev, wh))

from collections import Counter
cnt = Counter(v for _, v in verdicts)
print("\n==== 分型计数 ====")
for k in sorted(cnt):
    print("  %3d  %s" % (cnt[k], k))
und = sum(c for k, c in cnt.items() if k.startswith("未定"))
print("合计已定型=%d 条（RED %d ＋ AMBER %d）；其中「未定」=%d" % (len(verdicts), len(red_items), len(amber_items), und))
print("W_DRIFT_SURVEY VERDICT rc=%d" % (7 if und else 0))
sys.exit(7 if und else 0)
PYEOF
rc_py=$?
echo "W DRIFT SURVEY rc=$rc_py"
exit "$rc_py"
