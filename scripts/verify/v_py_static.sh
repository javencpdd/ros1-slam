#!/usr/bin/env bash
# v_py_static.sh —— Python 静态三查：语法编译 → pyflakes（未定义名/未用导入）→ 格式串占位符 vs 字面元组实参
#
# 为什么存在（坑46／坑48 的合并处置）：R75 连烧四条活体链才查出四个**静态就能查出**的错——
#   ①`tf.TransformListener(queue_size=...)`/`buffer_size=...`：API 签名靠猜（introspect 一次即可）；
#   ②`len(axis(m,"x") or [])`：ndarray 真值判断必抛 ValueError；
#   ③模块级函数闭包 main 的局部量 `buf` ⇒ NameError；④同样方式引用 `pose` ⇒ NameError。
#   py_compile 只能查语法（以上四条语法全对）；pyflakes 当场查出 ③④，且实测 0 误报在本仓自研件上。
# 性质：**候补闸件**——本件不进在册门禁串（SYN/GTC/ISO/DC 四件一字不改），等请求单 #54 由小美追认后再议入门。
#
# 用法：bash scripts/verify/v_py_static.sh <文件.py> [...]
#       bash scripts/verify/v_py_static.sh --selftest     # 自标定：植入的坏件必须被抓（G05 C4a：先阳性对照再零命中）
# rc：0 全过；1 有被检件不过；5 用法错（未给件）或自标定不过
# 注：非 .py 入参一律 SKIP 不判红（实测我把 .sh 喂进来时 pyflakes 报 invalid syntax，那是喂错件不是件有错）
set -u
SELF=/tmp/v_py_static_selftest
if [ "${1:-}" = "--selftest" ]; then
  rm -rf "$SELF"; mkdir -p "$SELF"
  printf 'import os\n\ndef f():\n    return os.sep\n' > "$SELF/good.py"
  printf 'import sys\n\ndef f():\n    return buf["raw"]      # NameError 族：模块级函数引用 main 的局部量（R75 第三崩同款）\n' > "$SELF/closure.py"
  printf 'import sys\n\ndef f(vals):\n    print("%%s %%s" %% (vals,))      # 占位符 2 个、字面元组 1 个\n' > "$SELF/fmt.py"
  printf 'def f(:\n' > "$SELF/bad_syntax.py"
  r=0
  "$0" "$SELF/good.py" >/dev/null 2>&1; g=$?
  "$0" "$SELF/closure.py" >/dev/null 2>&1; c=$?
  "$0" "$SELF/fmt.py" >/dev/null 2>&1; m=$?
  "$0" "$SELF/bad_syntax.py" >/dev/null 2>&1; b=$?
  "$0" >/dev/null 2>&1; e=$?
  [ "$g" = 0 ] || { echo "自标定 FAIL：good.py 应过，实得 rc=$g"; r=1; }
  [ "$c" = 1 ] || { echo "自标定 FAIL：closure.py（未定义名）应 rc=1，实得 $c"; r=1; }
  [ "$m" = 1 ] || { echo "自标定 FAIL：fmt.py（占位符不符）应 rc=1，实得 $m"; r=1; }
  [ "$b" = 1 ] || { echo "自标定 FAIL：bad_syntax.py 应 rc=1，实得 $b"; r=1; }
  [ "$e" = 5 ] || { echo "自标定 FAIL：未给件应 rc=5（空集不得判绿，R46 一），实得 $e"; r=1; }
  rm -rf "$SELF"
  [ "$r" = 0 ] && echo "V_PY_STATIC SELFTEST: PASS（4 例＋空集共 5 项，含 3 例必须真红）"
  exit $r
fi

if [ "$#" -lt 1 ]; then
  echo "[FAIL] 未给被检件 ⇒ 空集不得判绿（R46 一同款规矩）"
  exit 5
fi

FAIL=0
SCANNER=$(mktemp /tmp/ast_fmt_XXXX.py)
cat > "$SCANNER" <<'PYEOF'
import ast, io, re, sys
PAT = re.compile(r"%[-+ #0]*\d*(?:\.\d+)?[sdfiExXgGocrR%]")
bad = 0
for p in sys.argv[1:]:
    tree = ast.parse(io.open(p, encoding="utf-8").read())
    for node in ast.walk(tree):
        if isinstance(node, ast.BinOp) and isinstance(node.op, ast.Mod) and isinstance(node.left, ast.Str):
            fmt = node.left.s
            ph = len([m for m in PAT.findall(fmt) if m != "%%"])
            rhs = node.right
            if not isinstance(rhs, ast.Tuple):
                continue          # 只有**字面元组**才可判实参数；变量/推导式一律不判（防假阳，实测 6 处假阳即由此而来）
            if any(isinstance(e, ast.Starred) for e in rhs.elts):
                continue
            if ph != len(rhs.elts):
                print("  [FAIL] %s:%d 占位符=%d 字面元组实参=%d | %s" % (p, node.lineno, ph, len(rhs.elts), fmt[:52]))
                bad = 1

# 第四查：同文件内**函数调用实参个数**（本仓 R76 第六次烧链就是 measure() 少传一个位置参数，
#         py_compile／pyflakes 都查不出——它们不看调用点。只判"本文件内定义、纯字面位置实参、且无 *args"的调用）
import inspect
defs = {}
for node in ast.walk(tree):
    if isinstance(node, (ast.FunctionDef,)):
        a = node.args
        if a.vararg is None and a.kwarg is None and not a.kwonlyargs:
            req = len(a.args) - len(a.defaults)
            opt = len(a.defaults)
            skip_self = 1 if a.args and a.args[0].arg in ("self", "cls") else 0
            defs[node.name] = (req - skip_self, opt - skip_self, node.lineno)
for node in ast.walk(tree):
    if isinstance(node, ast.Call) and isinstance(node.func, ast.Name) and node.func.id in defs:
        if any(isinstance(x, ast.Starred) for x in node.args) or node.keywords:
            continue
        req, opt, dl = defs[node.func.id]
        if not (req <= len(node.args) <= req + opt):
            print("  [FAIL] %s:%d 调用 %s() 给了 %d 个位置实参，定义(:%d)要求 %d–%d 个"
                  % (p, node.lineno, node.func.id, len(node.args), dl, req, req + opt))
            bad = 1
sys.exit(bad)
PYEOF
for f in "$@"; do
  case "$f" in
    *.py) ;;
    *) echo "[SKIP] 非 .py（shell 件归 v_sh_py_syntax.sh 管）：$f"; continue ;;
  esac
  if [ ! -f "$f" ]; then echo "[FAIL] 文件不存在 $f"; FAIL=1; continue; fi
  python3 -m py_compile "$f" 2>/dev/null || { echo "[FAIL] 语法编译 $f"; FAIL=1; continue; }
  echo "[OK]  py_compile $f"
  all=$(python3 -m pyflakes "$f" 2>&1)
  # 只把**未定义名**判红：pyflakes 2.1.1 的 CLI 没有 --select（实测报 no such option），且未用导入属风格问题、
  # 拿来判红会把老件与三方副本全打红（假阳）——先例＝ISO 项[5] 的精度修订（只收真 include）。
  out=$(printf '%s\n' "$all" | grep "undefined name" || true)
  if [ -n "$out" ]; then echo "  [FAIL] pyflakes $f"; echo "$out" | sed 's/^/      /'; FAIL=1; else echo "[OK]  pyflakes $f（未定义名 0）"; fi
  python3 "$SCANNER" "$f" || FAIL=1
done
rm -f "$SCANNER"
find "$(dirname "$1")" -name '__pycache__' -type d -newermt '-2 minutes' -exec rm -rf {} + 2>/dev/null
[ "$FAIL" = 0 ] && echo "PY STATIC OK（被检件数=$#）" || echo "PY STATIC FAIL（被检件数=$#）"
exit $FAIL
