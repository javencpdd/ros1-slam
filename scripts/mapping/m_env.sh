#!/usr/bin/env bash
# m_env.sh — 采图脚本族公共环境（被其他 m_*.sh source，不单独运行）
# 功能：ROS1 noetic 预置（防 foxy 混装）+ 沙箱 env.sh + 采图域变量
export ROS_MASTER_URI_DEFAULT=http://192.168.1.103:11312
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION 2>/dev/null || true
source /opt/ros/noetic/setup.bash
source /home/ros1-slam/scripts/env.sh
export ROS_LOG_DIR=/home/ros1-slam/logs
