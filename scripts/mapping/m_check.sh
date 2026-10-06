#!/usr/bin/env bash
# m_check.sh — 采图开工前环境体检（11 项，包装 premap_check.sh）
# 用法: bash m_check.sh [雷达IP]   退出码: 0=全绿 1=阻塞 2=警告
sudo -n bash /home/ros1-slam/scripts/verify/premap_check.sh "${1:-192.168.1.201}" 2>/dev/null \
  || bash /home/ros1-slam/scripts/verify/premap_check.sh "${1:-192.168.1.201}"
