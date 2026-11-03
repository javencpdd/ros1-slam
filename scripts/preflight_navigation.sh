#!/bin/bash
# preflight_navigation.sh —— mode_navigation 开工前置检查（只读）
# 用法: bash scripts/preflight_navigation.sh <Q8确认串>
# 退出码: 0＝全绿可起；9＝有阻塞项（逐条打印）
set -u
ROOT="${ROS1_SLAM_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
CONFIRM="${1:-}"
rc=0
say() { printf '  %-6s %s\n' "$1" "$2"; }

echo "== mode_navigation 前置检查（ROOT=$ROOT）=="

# 1) Q8 确认串（操作人必须显式给出，避免默认发车）
if [ -n "$CONFIRM" ] && [ "$CONFIRM" = "Q8-CONFIRMED" ]; then
  say PASS "Q8 确认串在位"
else
  say FAIL "缺 Q8 确认串（起导航须传 Q8-CONFIRMED，现场已确认急停/看护/站立）"; rc=9
fi

# 2) foxy 侧 UDP 桥必须停（43897 让给 qnx2ros）
if systemctl is-active --quiet transfer_ros2; then
  say FAIL "transfer_ros2 仍在跑（会占 43897；先 sudo systemctl stop transfer_ros2）"; rc=9
else
  say PASS "transfer_ros2 已停"
fi

# 3) 43897 必须空闲
if ss -uln 2>/dev/null | grep -q ':43897 '; then
  say FAIL "43897 已被占用"; rc=9
else
  say PASS "43897 空闲"
fi

# 4) 不得有残留 ros2qnx（唯一底盘桥只由本 mode 起）
if pgrep -f 'ros2qnx' >/dev/null 2>&1; then
  say FAIL "已有 ros2qnx 在跑（双写风险）"; rc=9
else
  say PASS "无残留 ros2qnx"
fi

# 5) master 在位
if timeout 5 rosnode list >/dev/null 2>&1; then
  say PASS "ROS master 在位（$(echo "${ROS_MASTER_URI:-http://192.168.1.103:11312}")）"
else
  say FAIL "ROS master 不在位（先 roscore -p 11312）"; rc=9
fi

# 6) 安全件在位（提示性）
for f in "$ROOT/ws/src/60_safety/lite3_safety/scripts/safe_cmd_vel_mux.py" \
         "$ROOT/ws/src/90_platform/v8_platform/framework/ultrasonic_guard_node.py"; do
  [ -f "$f" ] && say PASS "件在位 $(basename "$f")" || { say FAIL "缺件 $f"; rc=9; }
done

echo "== 结论：rc=$rc（0=可起；9=阻塞）=="
exit "$rc"
