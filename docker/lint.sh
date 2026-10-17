#!/usr/bin/env bash
# =============================================================================
# docker/lint.sh —— 构建前的静态自检（把"跑 20 分钟才在 shell 语法上炸"挡在门外）
#
#   bash /home/jack/ysc-code/ros1-slam/docker/lint.sh
#
# 查四件事：
#   1. Dockerfile 每个 RUN 块的单引号是否配平（2026-10-17 实测踩坑：
#      `RUN bash -lc '...'` 漏了收尾单引号，构建 0.5 秒就
#      "syntax error: unexpected end of file"，但错误信息完全不指向 Dockerfile 行号）
#   2. 每个 RUN 的 shell 命令能否通过 `bash -n`
#   3. docker-compose.yml 能否被 compose 解析
#   4. 本项目自带 shell 脚本能否通过 `bash -n`
# =============================================================================
set -eo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(dirname "$HERE")"
FAIL=0

ok()   { echo "  [OK]   $*"; }
bad()  { echo "  [FAIL] $*"; FAIL=1; }

# ---- 1+2: 解析 Dockerfile，逐 RUN 块做引号配平 + bash -n ----
echo "== 1) Dockerfile RUN 块检查 =="
"${PYTHON:-python3}" - "$ROOT/docker/Dockerfile" <<'PYEOF'
import re, subprocess, sys, tempfile, os

path = sys.argv[1]
raw = open(path, encoding="utf-8").read()

# 去掉注释行，合并续行（Dockerfile 的 \ 续行规则）
lines, buf = [], ""
for ln in raw.split("\n"):
    s = ln.rstrip()
    if s.endswith("\\"):
        buf += s[:-1] + " "
        continue
    buf += s
    lines.append(buf)
    buf = ""
if buf:
    lines.append(buf)

runs = []
for ln in lines:
    s = ln.strip()
    if not s or s.startswith("#"):
        continue
    m = re.match(r"^RUN\s+(.*)$", s, re.S)
    if m:
        runs.append(m.group(1))

if not runs:
    print("  [FAIL] 没找到任何 RUN 指令")
    sys.exit(1)

bad = 0
for i, cmd in enumerate(runs, 1):
    # 引号配平：整块里单引号必须是偶数个
    n = cmd.count("'")
    if n % 2 != 0:
        print("  [FAIL] RUN #%d 单引号不配平（%d 个）——典型原因是 bash -lc '...' 漏了收尾引号" % (i, n))
        print("         开头: %s" % cmd[:90])
        bad += 1
        continue
    # bash -n 语法检查（shell 形式）
    with tempfile.NamedTemporaryFile("w", suffix=".sh", delete=False) as f:
        f.write("#!/usr/bin/env bash\n" + cmd + "\n")
        tmp = f.name
    try:
        r = subprocess.run(["bash", "-n", tmp], capture_output=True, text=True)
        if r.returncode != 0:
            print("  [FAIL] RUN #%d 语法错误: %s" % (i, r.stderr.strip().split("\n")[0]))
            print("         内容: %s" % cmd[:120])
            bad += 1
        else:
            print("  [OK]   RUN #%d (%d 字符)" % (i, len(cmd)))
    finally:
        os.unlink(tmp)

sys.exit(1 if bad else 0)
PYEOF
[ $? -eq 0 ] || FAIL=1

# ---- 3: compose 解析 ----
echo "== 2) docker-compose.yml 解析 =="
if command -v docker >/dev/null 2>&1; then
    if ( cd "$HERE" && timeout 60 docker compose config -q ); then
        ok "compose config -q 通过"
    else
        bad "compose config -q 失败"
    fi
else
    echo "  [SKIP] 无 docker CLI"
fi

# ---- 4: 自带脚本语法 ----
echo "== 3) 自带 shell 脚本 bash -n =="
for f in "$HERE"/*.sh; do
    [ -f "$f" ] || continue
    if bash -n "$f" 2>/dev/null; then
        ok "$(basename "$f")"
    else
        bad "$(basename "$f") 语法错误"
    fi
done

echo
if [ "$FAIL" -eq 0 ]; then
    echo "lint 全部通过 —— 可以构建了"
else
    echo "lint 有 FAIL —— 先修再构建（别让错误在 20 分钟后才暴露）"
fi
exit "$FAIL"
