#!/usr/bin/env bash
# as_ros1slam.sh —— 以 ros1-slam 身份执行单条命令（root 调用；ysc 经 sudo 代劳）
# 用法: sudo bash /home/ros1-slam/scripts/as_ros1slam.sh '<命令>'
#
# ★ 非交互 bash 不加载 .bashrc 尾部环境段（Ubuntu .bashrc 顶部对非交互 shell 直接 return），
#   本脚本不依赖 .bashrc。需要 ROS 环境时在命令串里显式 source：
#   sudo bash /home/ros1-slam/scripts/as_ros1slam.sh 'source /opt/ros/noetic/setup.bash && <命令>'
set -eu
if [ "$(id -u)" -ne 0 ]; then
  echo "[as_ros1slam] ERR: 必须 root 运行（sudo bash $0 '<命令>'）" >&2; exit 1
fi
if [ "$#" -lt 1 ]; then
  echo "[as_ros1slam] 用法: sudo bash $0 '<命令>'" >&2; exit 1
fi
export HOME=/home/ros1-slam
exec sudo -u ros1-slam env HOME=/home/ros1-slam USER=ros1-slam LOGNAME=ros1-slam /bin/bash -c "$*"
