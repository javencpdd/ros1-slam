#!/usr/bin/env bash
# 把 ros1-slam 跨 Agent 协作黑板部署到远端 103 主机
#
# 用法：
#   bash install.sh <user@host> <目标项目根目录>
#   bash install.sh ysc@192.168.1.103 /home/ros1-slam
#
# 只会新增文件；已存在的 AGENTS.md 会先备份，已存在的 .ai/state.json 会跳过（保留历史任务）。

set -euo pipefail

HOST="${1:-ysc@192.168.1.103}"
DEST="${2:-/home/ros1-slam}"
SRC="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "==> 源目录   : $SRC"
echo "==> 目标主机 : $HOST"
echo "==> 项目根   : $DEST"

# ---------- 0. 连通性检查 ----------
if ! ssh -o BatchMode=yes -o ConnectTimeout=8 "$HOST" 'echo -n ""' 2>/dev/null; then
  echo "ERROR: 无法免密 ssh 到 $HOST"
  echo "请先在 ~/.ssh/config 配好密钥，或用 ssh-copy-id："
  echo "  ssh-copy-id $HOST"
  exit 1
fi
echo "==> SSH 连通 OK"

# ---------- 1. 确认目标目录存在 ----------
ssh "$HOST" "test -d '$DEST' || { echo 'ERROR: $DEST 不存在'; exit 1; }"
echo "==> 目标目录存在"

# ---------- 2. 备份 + 放置 AGENTS.md ----------
TS="$(date +%Y%m%d-%H%M%S)"
ssh "$HOST" "if [ -f '$DEST/AGENTS.md' ]; then cp -p '$DEST/AGENTS.md' '$DEST/AGENTS.md.bak-$TS'; echo '==> 已备份旧 AGENTS.md -> AGENTS.md.bak-$TS'; fi"

scp -q "$SRC/AGENTS.md" "$HOST:$DEST/AGENTS.md"
echo "==> AGENTS.md 已就位"

# ---------- 3. .ai/ 目录 ----------
ssh "$HOST" "mkdir -p '$DEST/.ai/tasks'"

if ssh "$HOST" "test -f '$DEST/.ai/state.json'"; then
  echo "==> 检测到已有 .ai/state.json —— 保留历史任务，只更新 bridge.py 与模板"
  scp -q "$SRC/.ai/bridge.py" "$HOST:$DEST/.ai/bridge.py"
  if [ -f "$SRC/.ai/knowledge.md" ]; then
    ssh "$HOST" "test -f '$DEST/.ai/knowledge.md' || true"
    scp -q "$SRC/.ai/knowledge.md" "$HOST:$DEST/.ai/knowledge.md.tpl" || true
  fi
else
  # 全新部署：整目录拷
  scp -q -r "$SRC/.ai/." "$HOST:$DEST/.ai/"
  echo "==> .ai/ 已就位（全新）"
fi

# ---------- 4. qoder rules ----------
ssh "$HOST" "mkdir -p '$DEST/.qoder/rules'"
scp -q "$SRC/.qoder/rules/ros1-slam-collab.md" "$HOST:$DEST/.qoder/rules/"
echo "==> .qoder/rules/ 已就位"

# ---------- 5. 初始化 / 刷新状态 ----------
ssh "$HOST" "cd '$DEST' && python3 .ai/bridge.py init 2>/dev/null || python3 .ai/bridge.py init --force"

# ---------- 6. 验证 ----------
echo
echo "==> 远端自检："
ssh "$HOST" "cd '$DEST' && python3 .ai/bridge.py check && echo '---' && python3 .ai/bridge.py json"

cat <<'TIP'

==========================================================
 部署完成。
==========================================================

接下来在两个工具里各说一句就完成了设定：

  【zcode】读取 AGENTS.md，执行 cd /home/ros1-slam && python3 .ai/bridge.py resume --as zcode
  【qoder】读取 AGENTS.md，执行 cd /home/ros1-slam && python3 .ai/bridge.py resume --as qoder

可选：在远端 ~/.zcode/AGENTS.md（用户全局）追加一句，让 zcode 每次自动指路：
  进入 /home/ros1-slam 工作区时，先跑 python3 .ai/bridge.py resume --as zcode 并按指示读文件。
TIP
