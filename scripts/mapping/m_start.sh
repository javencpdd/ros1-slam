#!/usr/bin/env bash
# m_start.sh — 采图三窗一键启动（tmux 自动建窗；已在其一窗内运行时用 bash m_start.sh）
# 超参:
#   SESSION=p3b          tmux 会话名
#   MAP_SAVE=1           1=落盘(默认) 0=试跑不落盘（faster_lio pcd_save_en 覆盖需改 yaml，此开关仅提示）
# 用法: bash m_start.sh   （在 tmux 内任一窗执行；无 tmux 时直接前台跑 roscore）
source "$(dirname "$0")/m_env.sh"
SESSION=${SESSION:-p3b}
echo "[m_start] 检查体检状态（不强制，建议先跑 m_check.sh）"
if ! in_tmux=$( [ -n "${TMUX:-}" ] && echo yes || echo no ); then in_tmux=no; fi
if [ "$in_tmux" = "yes" ]; then
  # 已在 tmux：创建独立窗（不开新会话）
  tmux new-window -t "$SESSION" -n core "bash -lc 'source /home/ros1-slam/scripts/mapping/m_run_core.sh; exec bash'"
  sleep 2
  tmux new-window -t "$SESSION" -n drv  "bash -lc 'source /home/ros1-slam/scripts/mapping/m_run_drv.sh; exec bash'"
  sleep 3
  tmux new-window -t "$SESSION" -n slam "bash -lc 'source /home/ros1-slam/scripts/mapping/m_run_slam.sh; exec bash'"
  echo "[m_start] 三窗已建（core/drv/slam）——切窗 Ctrl-b 数字；先验 /rslidar_points≥9Hz 与 /livox/imu≈200Hz 再看 slam 窗 /Odometry"
else
  echo "[m_start] 不在 tmux 内——请先: sudo -u ros1-slam tmux new-session -A -s $SESSION"
  exit 1
fi
