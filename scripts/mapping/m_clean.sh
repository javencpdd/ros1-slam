#!/usr/bin/env bash
# m_clean.sh — 采图残留清场（上一场结束后/开工前跑）
# 用法: bash m_clean.sh   （ros1-slam 身份或 ysc 均可——只杀 ROS1 采图相关）
pkill -f "roscore|rosmaster|livox_ros_driver2|faster_lio|run_mapping_online|map_snapshot_pub|robot_state_publisher|joint_state_publisher" 2>/dev/null
sleep 2
LEFT=$(ps aux | grep -E "roscore|rosmaster|livox|run_mapping|map_snapshot" | grep -v grep | wc -l)
if [ "$LEFT" -eq 0 ]; then echo "[m_clean] 清场完成 ✓"; else echo "[m_clean] ⚠ 仍有 $LEFT 个残留："; ps aux | grep -E "roscore|livox|run_mapping" | grep -v grep; fi
