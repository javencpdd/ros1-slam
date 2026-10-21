#!/bin/bash
tag="$1"; out="$2"
{ echo "# 见证 $tag（$(date '+%F %T')）"
  echo "## 被试域（容器 netns）进程计数"
  ps -eo comm= | grep -cE '^(roslaunch|rosmaster|rosout|gzserver|gzclient|rosbag|costmap_2d_node)$' | sed 's/^/五名计数=/'
  ps -eo comm= | grep -E '^(roslaunch|rosmaster|rosout|gzserver|gzclient|rosbag|costmap_2d_node)$' | sort | uniq -c | sed 's/^/  明细 /'
  echo "## 被试域端口（TCP/UDP 分查）"
  echo "TCP:"; ss -lntp 2>/dev/null | grep -E ':(11311|11312|11313)\b' || echo "  0 条"
  echo "UDP:"; ss -lnup 2>/dev/null | grep -E ':(11311|11312|11313)\b' || echo "  0 条"
} > "$out" 2>&1
