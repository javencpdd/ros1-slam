#!/usr/bin/env bash
# v_sh_py_syntax.sh —— 「改脚本后必做两检」的可执行件（小美 M2R4 路由项 R5；R46 精度修订，小明 2026-10-08）
# 为何存在：05:3x 那次编辑把 stageB 内嵌 python 的两行并作一行，bash -n 全过、脚本却根本跑不了。
#   教训 = 只查 shell 层不够；heredoc 里的 python 必须各自过一次语法解析。
# R46 修的三处（小美 M2R8 实测；属门禁件精度修订，不改任何门值）：
#   （一）无参数即打 PY SYNTAX OK 并 exit 0 ＝空集真命题 ⇒ 被检件数必须 ≥1，否则判 FAIL 并 exit 5。
#   （二）旧正则只认「python3 - <<PYEOF」紧邻形态，实测漏 4 段：v_tf_edge_conflict.sh（- 与 << 之间有重定向）、
#         sim_nav_smoke.sh／v_scan_p1.sh／v_scan_p2.sh（定界符为 PY，P1/P2 前还有命令替换）
#         ⇒ 改为「行内含 python 调用 ＋ 行内任意位置的 heredoc 重定向」，定界符按捕获组取（不再写死 PYEOF），
#           且未闭合的 heredoc 判 FAIL（旧式会一路扫到文件尾当成一段，等于把语法错吞掉）。
#   （三）旧式「找不到段即绿」使零段不可指认 ⇒ 现打印被扫行数与段数合计，零段成为一条可核对的断言。
# 用法：bash scripts/verify/v_sh_py_syntax.sh <脚本文件> [...]
#       bash scripts/verify/v_sh_py_syntax.sh --selftest    # 自标定：必须能真 FAIL 一次（G05 C4a）
# rc：0 全过；1 有被检件语法错；5 用法错（未给被检件）或自标定不过。
set -u

if [ "${1:-}" = "--selftest" ]; then
  T=$(mktemp -d /tmp/syn_selftest.XXXXXX)
  python3 - "$T" <<'MKFIX'
import os, sys
T = sys.argv[1]
open(os.path.join(T, "bad.sh"), "w").write("#!/usr/bin/env bash\npython3 - <<'PYEOF'\nx = (1 +\nPYEOF\n")
open(os.path.join(T, "good_redirect.sh"), "w").write('#!/usr/bin/env bash\npython3 - "$1" > /tmp/syn_o.txt <<\'PY\'\nimport sys\nprint(sys.argv[1])\nPY\n')
open(os.path.join(T, "noseg.sh"), "w").write("#!/usr/bin/env bash\necho no python here\n")
MKFIX
  r1=0; bash "$0" "$T/bad.sh" >/dev/null 2>&1 || r1=$?
  r2=0; bash "$0" "$T/good_redirect.sh" >/dev/null 2>&1 || r2=$?
  r3=0; bash "$0" "$T/noseg.sh" >/dev/null 2>&1 || r3=$?
  r4=0; bash "$0" >/dev/null 2>&1 || r4=$?
  rm -rf "$T"
  ok=1
  [ "$r1" != "0" ] || { echo "[FAIL] 自标定：坏段未被识破（rc=$r1，应非 0）"; ok=0; }
  [ "$r2" = "0" ]  || { echo "[FAIL] 自标定：带重定向的合法段被误伤（rc=$r2）"; ok=0; }
  [ "$r3" = "0" ]  || { echo "[FAIL] 自标定：无内嵌段的合法件被误伤（rc=$r3）"; ok=0; }
  [ "$r4" = "5" ]  || { echo "[FAIL] 自标定：未给被检件应 exit 5，实得 $r4"; ok=0; }
  if [ "$ok" = "1" ]; then
    echo "[PASS] 自标定四项：坏段真红／重定向段不误伤／零段不误伤／空参判废（R46 一）"
    echo "SYNTAX SELFTEST: PASS"; exit 0
  fi
  echo "SYNTAX SELFTEST: FAIL"; exit 5
fi

if [ "$#" -lt 1 ]; then
  echo "[FAIL] 未给被检件 ⇒ 空集不得判绿（R46 一；与坑29「先阳性对照再零命中」同族）"
  echo "PY SYNTAX FAIL（被检件数=0）"
  exit 5
fi

FAIL=0; NF=$#; NS=0
for F in "$@"; do
  [ -f "$F" ] || { echo "[FAIL] 不存在: $F"; FAIL=1; continue; }
  if head -1 "$F" | grep -qi "python" || [ "${F##*.}" = "py" ]; then
    if python3 -c 'import ast,sys; ast.parse(open(sys.argv[1],encoding="utf-8").read())' "$F" 2>/dev/null; then
      echo "[OK] python ast $F"
    else
      echo "[FAIL] python ast $F"; FAIL=1
    fi
    continue
  fi
  if bash -n "$F" 2>/dev/null; then echo "[OK] bash -n   $F"; else echo "[FAIL] bash -n   $F"; FAIL=1; fi
  out=$(python3 - "$F" <<'SYNPY'
import io, ast, re, sys
p = sys.argv[1]
lines = io.open(p, encoding='utf-8', errors='replace').read().split('\n')
hd = re.compile(r"<<-?\s*(['\"]?)([A-Za-z_][A-Za-z0-9_]*)\1")
py = re.compile(r"(^|[^A-Za-z0-9_])python[23]?([^A-Za-z0-9_]|$)")
bad = 0
segs = 0
i = 0
while i < len(lines):
    l = lines[i]
    if l.lstrip().startswith('#'):
        i += 1
        continue
    m = hd.search(l)
    if not m:
        i += 1
        continue
    delim = m.group(2)
    j = i + 1
    while j < len(lines) and lines[j].strip() != delim:
        j += 1
    if py.search(l):
        if j >= len(lines):
            bad += 1
            print('[FAIL] heredoc 未闭合 L%d 定界符=%s %s' % (i + 1, delim, p))
        else:
            segs += 1
            try:
                ast.parse('\n'.join(lines[i + 1:j]))
                print('[OK] python 段 L%d-L%d 定界符=%s %s' % (i + 2, j, delim, p))
            except SyntaxError as e:
                bad += 1
                print('[FAIL] python 段 L%d-L%d 定界符=%s %s -> 段内行 %s: %s' % (i + 2, j, delim, p, e.lineno, e.msg))
    i = j + 1 if j < len(lines) else len(lines)
if segs == 0 and bad == 0:
    print('[OK] 该件无内嵌 python 段（已按 python 调用＋heredoc 双条件扫全 %d 行，非默认绿）' % len(lines))
print('SEGS=%d' % segs)
raise SystemExit(1 if bad else 0)
SYNPY
)
  rc=$?
  printf '%s\n' "$out" | grep -E '^\[(OK|FAIL|SUM)\]' | sed 's/^/  /'
  c=$(printf '%s\n' "$out" | sed -n 's/^SEGS=//p' | tail -1)
  case "$c" in ''|*[!0-9]*) c=0 ;; esac
  NS=$((NS + c))
  [ "$rc" = "0" ] || FAIL=1
done
if [ "$FAIL" = "0" ]; then echo "PY SYNTAX OK（被检件数=$NF，内嵌 python 段合计=$NS）"; else echo "PY SYNTAX FAIL（被检件数=$NF，内嵌 python 段合计=$NS）"; fi
exit "$FAIL"
