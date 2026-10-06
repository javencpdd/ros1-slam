#!/usr/bin/env bash
# m_verify.sh — 联锁验证（驱动起来后、SLAM 起来前跑；也可采图中复核）
# 超参: ODOM=1 额外验 /Odometry（SLAM 起后）
source "$(dirname "$0")/m_env.sh"
ODOM=${ODOM:-0}
echo "[m_verify] /rslidar_points（期望 ≥9Hz，5s 采样）"
timeout 7 rostopic hz /rslidar_points 2>&1 | grep -m1 "average rate" || echo "  ✗ 无数据"
echo "[m_verify] /livox/imu（期望 ≈200Hz）"
timeout 7 rostopic hz /livox/imu 2>&1 | grep -m1 "average rate" || echo "  ✗ 无数据"
if [ "$ODOM" = "1" ]; then
  echo "[m_verify] /Odometry（期望 ~10Hz）"
  timeout 7 rostopic hz /Odometry 2>&1 | grep -m1 "average rate" || echo "  ✗ 无数据（SLAM 未起或未收敛）"
fi
