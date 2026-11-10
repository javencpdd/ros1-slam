#!/usr/bin/env bash
# Lite3 AI 策略 MuJoCo 演示：宿主单机启动器
# 资产默认在仓外 ~/.cache/lite3-policy-staging-20261109，可用环境变量覆盖。
set -euo pipefail

ROOT="${LITE3_DEMO_ROOT:-$HOME/.cache/lite3-policy-staging-20261109}"
PY="${LITE3_DEMO_PY:-$HOME/.cache/mujoco-demo-venv/bin/python}"
export DISPLAY="${LITE3_DEMO_DISPLAY:-${DISPLAY:-:1}}"
ENTRY="$ROOT/multiskill_demo.py"
PIDFILE="$ROOT/demo.pid"
LOG="$ROOT/demo.log"
ACTION="${1:---start}"

usage() {
  cat <<EOF
用法: bash $0 [--check|--start|--stop|--status]
  --check   只检查 Python / 资产 / 显示，不启动
  --start   后台启动演示（默认）
  --stop    停止本启动器启动的实例
  --status  显示运行状态
环境变量: LITE3_DEMO_ROOT  LITE3_DEMO_PY  LITE3_DEMO_DISPLAY
EOF
}

check() {
  local fail=0
  [[ -x "$PY" ]] || { echo "缺少 venv python: $PY"; fail=1; }
  [[ -f "$ENTRY" ]] || { echo "缺少入口: $ENTRY"; fail=1; }
  local d
  for d in policies mjcf csv; do
    [[ -d "$ROOT/$d" ]] || { echo "缺少资产目录: $ROOT/$d"; fail=1; }
  done
  [[ -f "$ROOT/mjcf/Lite3.xml" ]] || { echo "缺少模型: $ROOT/mjcf/Lite3.xml"; fail=1; }
  if command -v xdpyinfo >/dev/null 2>&1; then
    DISPLAY="$DISPLAY" xdpyinfo >/dev/null 2>&1 || echo "警告：DISPLAY=$DISPLAY 不可达（MuJoCo 窗口需要宿主 X）"
  fi
  "$PY" -c 'import mujoco, onnxruntime, tkinter' 2>/dev/null \
    || { echo "venv 缺少 mujoco / onnxruntime / tkinter"; fail=1; }
  local count
  count=$(find "$ROOT/policies" -maxdepth 1 -name '*.onnx' 2>/dev/null | wc -l)
  echo "检查：策略 $count 个 ONNX ｜ 入口 $ENTRY ｜ DISPLAY=$DISPLAY"
  return "$fail"
}

running() {
  [[ -f "$PIDFILE" ]] || return 1
  local pid
  pid=$(cat "$PIDFILE")
  kill -0 "$pid" 2>/dev/null || { rm -f "$PIDFILE"; return 1; }
}

demo_pids() {
  local pid exe want
  want=$(readlink -f "$PY" 2>/dev/null || echo "$PY")
  for pid in $(pgrep -f "[m]ultiskill_demo.py" 2>/dev/null || true); do
    exe=$(readlink -f "/proc/$pid/exe" 2>/dev/null || true)
    [[ "$exe" == "$want" ]] && echo "$pid"
  done
}

start() {
  check || { echo "检查未通过，未启动"; exit 1; }
  if running; then
    echo "已在运行（PID $(cat "$PIDFILE")），日志 $LOG"
    exit 0
  fi
  local existing
  existing=$(demo_pids | head -1)
  if [[ -n "$existing" ]]; then
    echo "已有实例在运行（非本启动器启动）：PID $existing"
    echo "先停止它，或直接用现有窗口。"
    exit 1
  fi
  cd "$ROOT"
  DISPLAY="$DISPLAY" setsid nohup "$PY" "$ENTRY" > "$LOG" 2>&1 < /dev/null &
  echo $! > "$PIDFILE"
  sleep 2
  local pid
  pid=$(cat "$PIDFILE")
  if kill -0 "$pid" 2>/dev/null; then
    echo "已启动：PID $pid ｜ 日志 $LOG"
    echo "停止：bash $0 --stop"
    echo "键盘（3D 窗口或面板，含小键盘）：8前 2后 4左 6右 7左转 9右转 5/0停（±0.1/步）"
  else
    echo "启动失败，日志末尾："
    tail -20 "$LOG" || true
    rm -f "$PIDFILE"
    exit 1
  fi
}

stop() {
  if ! running; then
    echo "未运行（没有有效 $PIDFILE）"
    return 0
  fi
  local pid
  pid=$(cat "$PIDFILE")
  kill "$pid" 2>/dev/null || true
  local i
  for i in $(seq 1 20); do
    kill -0 "$pid" 2>/dev/null || break
    sleep 0.2
  done
  kill -9 "$pid" 2>/dev/null || true
  rm -f "$PIDFILE"
  echo "已停止 PID $pid"
}

case "$ACTION" in
  --check) check ;;
  --start|"") start ;;
  --stop) stop ;;
  --status) if running; then echo "运行中（PID $(cat "$PIDFILE")）"; else echo "未运行"; fi ;;
  -h|--help) usage ;;
  *) usage; exit 2 ;;
esac
