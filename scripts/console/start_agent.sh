#!/usr/bin/env bash
# Host agent for the Lite3 console stack lifecycle (P3). Binds 127.0.0.1.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PIDFILE=/tmp/v8-console-agent.pid
LOG=/tmp/v8-console-agent.log
PORT="${LITE3_AGENT_PORT:-8780}"
BIND="${LITE3_AGENT_BIND:-127.0.0.1}"
ACTION="${1:---start}"

running() {
  [[ -f "$PIDFILE" ]] || return 1
  local pid; pid=$(cat "$PIDFILE")
  kill -0 "$pid" 2>/dev/null || { rm -f "$PIDFILE"; return 1; }
}

case "$ACTION" in
  --start|"")
    command -v docker >/dev/null || { echo 'docker 不可用'; exit 1; }
    if running; then echo "agent 已在运行 PID $(cat "$PIDFILE")"; exit 0; fi
    setsid nohup python3 "$ROOT/scripts/console/agent.py" --bind "$BIND" --port "$PORT" \
      > "$LOG" 2>&1 < /dev/null &
    echo $! > "$PIDFILE"
    sleep 1
    if running; then
      echo "agent 已启动：PID $(cat "$PIDFILE") ｜ http://$BIND:$PORT ｜ 日志 $LOG"
    else
      echo 'agent 启动失败'; tail -20 "$LOG" || true; rm -f "$PIDFILE"; exit 1
    fi
    ;;
  --stop)
    if running; then pid=$(cat "$PIDFILE"); kill "$pid" 2>/dev/null || true; rm -f "$PIDFILE"; echo "agent 已停止 PID $pid"; else echo 'agent 未运行'; fi
    ;;
  --status)
    if running; then echo "agent 运行中 PID $(cat "$PIDFILE")"; else echo 'agent 未运行'; fi
    ;;
  *) echo "用法: $0 [--start|--stop|--status]"; exit 2 ;;
esac
