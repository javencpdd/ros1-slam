#!/usr/bin/env bash
# v_sh_py_syntax.sh —— 「改脚本后必做两检」的可执行件（小美 M2R4 路由项 R5，2026-10-08）
# 为何存在：05:3x 那次编辑把 stageB 内嵌 python 的两行并作一行，bash -n 全过、脚本却根本跑不了。
#   教训 = 只查 shell 层不够；heredoc 里的 python 必须各自过一次语法解析。
# 用法：bash scripts/verify/v_sh_py_syntax.sh <脚本文件> [...]
set -u
FAIL=0
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
  python3 - "$F" <<'PYEOF' || FAIL=1
import io, ast, re, sys
p = sys.argv[1]
lines = io.open(p, encoding='utf-8').read().split('\n')
starts = [i for i, l in enumerate(lines) if re.match(r"\s*python3? - <<'?PYEOF'?", l)]
if not starts:
    print('[OK] 无内嵌 python 段 %s' % p); raise SystemExit(0)
bad = 0
for si in starts:
    j = si + 1
    while j < len(lines) and lines[j].strip() != 'PYEOF':
        j += 1
    seg = '\n'.join(lines[si+1:j])
    try:
        ast.parse(seg)
        print('[OK] python 段 L%d-L%d %s' % (si+2, j, p))
    except SyntaxError as e:
        bad += 1
        print('[FAIL] python 段 L%d-L%d %s -> 段内行 %s: %s' % (si+2, j, p, e.lineno, e.msg))
if bad: print('[FAIL] %d 段语法错 %s' % (bad, p))
else:   print('[OK] %d 段全过 %s' % (len(starts), p))
raise SystemExit(1 if bad else 0)
PYEOF
done
[ "$FAIL" = "0" ] && echo "PY SYNTAX OK" || echo "PY SYNTAX FAIL"
exit "$FAIL"
