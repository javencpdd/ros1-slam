#!/usr/bin/env bash
# m_start.sh — 采图三窗一键启动（tmux 自动建窗；已在其一窗内运行时用 bash m_start.sh）
# 超参:
#   SESSION=p3b          tmux 会话名
#   VIZ=1                1=自动建 viz 窗（默认，/map_snapshot+Lite3 模型）0=不建
# 用法: bash m_start.sh   （在 tmux 内任一窗执行；无 tmux 时直接前台跑 roscore）
source "$(dirname "$0")/m_env.sh"
SESSION=${SESSION:-p3b}
echo "[m_start] 检查体检状态（不强制，建议先跑 m_check.sh）"
if ! in_tmux=$( [ -n "${TMUX:-}" ] && echo yes || echo no ); then in_tmux=no; fi
if [ "$in_tmux" = "yes" ]; then
  # 已在 tmux：逐窗创建（已存在的窗跳过——防重复跑建重复 roscore）
  # 建窗即切焦属正常；remain-on-exit 逐窗设置（窗内进程秒退时保留窗口与报错，勿 exec）
  mk_win() {  # mk_win 窗名 窗内命令（死窗自动重建，活窗跳过）
    local wname="$1" wcmd="$2"
    local dead
    dead=$(tmux list-windows -t "$SESSION" -F '#{window_name} #{pane_dead}' 2>/dev/null | awk -v w="$wname" '$1==w && $2==1' | wc -l)
    if [ "$dead" -gt 0 ]; then
      echo "[m_start] 窗 $wname 为死窗（上次残留）→ 重建"
      tmux kill-window -t "$SESSION:$wname" 2>/dev/null
    elif tmux list-windows -t "$SESSION" -F "#W" 2>/dev/null | grep -qx "$wname"; then
      echo "[m_start] 窗 $wname 已存在且存活，跳过"
      return 0
    fi
    tmux new-window -t "$SESSION" -n "$wname" "bash -c '$wcmd; exec bash'"
    tmux set-option -w -t "$SESSION:$wname" remain-on-exit on
  }
  mk_win core "source /home/ros1-slam/scripts/mapping/m_run_core.sh"
  sleep 2
  mk_win drv  "source /home/ros1-slam/scripts/mapping/m_run_drv.sh"
  sleep 3
  mk_win slam "source /home/ros1-slam/scripts/mapping/m_run_slam.sh"
  if [ "${VIZ:-1}" = "1" ]; then
    sleep 2
    mk_win viz  "source /home/ros1-slam/scripts/mapping/m_viz.sh"
  fi
  echo "[m_start] 窗组就绪（core/drv/slam${VIZ:+/viz}）——切窗 Ctrl-b 数字"
  echo "[m_start] 顺序：drv 窗确认 livox 启动 → m_verify 验频率 → slam 窗 /Odometry → 开走"
  echo "[m_start] 停机勿直接杀窗：任一窗跑 m_stop.sh（SLAM 先停等落盘）"
else
  echo "[m_start] 不在 tmux 内——请先: sudo -u ros1-slam tmux new-session -A -s $SESSION"
  exit 1
fi
