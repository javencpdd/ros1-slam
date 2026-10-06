#!/usr/bin/env bash
# m_pose.sh — 当前位姿单帧（回起点记坐标用）
# 超参: N=帧数（默认 1）
source "$(dirname "$0")/m_env.sh"
rostopic echo /Odometry/pose/pose/position -n "${N:-1}" 2>&1 | grep -E "^[xyz]:" || echo "（/Odometry 无数据——SLAM 未起）"
