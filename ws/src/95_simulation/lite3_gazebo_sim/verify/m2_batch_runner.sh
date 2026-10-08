#!/usr/bin/env bash
# m2_batch_runner.sh —— M2 批次级驱动（小美 M2R6 路由项 R20 三 的落地件，小明 2026-10-08）
# 为何存在：press_late_G5 只在单跑内判定 ⇒「本批是否真的至少做过一次加压档」不可机判（她 R9续 的残余）。
#   本件把该断言上收到批次层：批内统计「G5_DELAY_SIM>0 且 gate_press_late=PASS」的跑数，<1 即批次判不符。
# 用法：bash m2_batch_runner.sh <标签1:延时1> <标签2:延时2> ...   （延时空=不开档）
#   例：bash m2_batch_runner.sh b1: b2:62 b3:99999
set +u
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION
source /opt/ros/noetic/setup.bash
source /home/ros1-slam/ws/devel/setup.bash
set -u
export GAZEBO_PLUGIN_PATH=/opt/ros/noetic/lib:/home/ros1-slam/ws/devel/lib
cd /home/ros1-slam/ws/src/95_simulation/lite3_gazebo_sim
B=${BATCH_OUT:-/tmp/m2_batch}
mkdir -p "$B"
snap(){ echo "### foxy $1 $(date '+%T')"; ps -o pid,user,lstart,etime,comm --no-headers \
        -p 115711,115792,115795,115797,115799,180237 2>/dev/null | tee "$B/foxy_$1.txt" | wc -l; }
declare -a LINES
PRESS_OK=0; NPASS=0; NFAIL=0
for spec in "$@"; do
  tag=${spec%%:*}; dly=${spec#*:}
  echo "##### $tag G5_DELAY_SIM=[$dly] $(date '+%T') #####"
  snap "start_$tag"
  TF_SOURCE="" G5_DELAY_SIM="$dly" bash verify/m2_smoke.sh stageB > "$B/$tag.log" 2>&1
  rc=$?
  snap "end_$tag"
  pl=$(grep -oE "gate_press_late=[A-Z]+ branch=[a-zA-Z+]+[^\n]*" "$B/$tag.log" | head -1)
  echo "  rc=$rc | ${pl:-press 未参与（未开档）} | $(grep -oE 'teb_state=[0-9]+ dt=[0-9.]+s dev_odom=[0-9.]+' "$B/$tag.log" | head -1)"
  grep -E "^\[exitclean\]|^\[PASS\]|^\[FAIL\]" "$B/$tag.log" | sed 's/^/    /' | tail -12
  [ "$rc" = "0" ] && NPASS=$((NPASS+1)) || NFAIL=$((NFAIL+1))
  if [ -n "$dly" ] && [ "$dly" != "0" ] && echo "$pl" | grep -q "=PASS"; then PRESS_OK=$((PRESS_OK+1)); fi
  sleep 4
done
echo "##### 批次级断言（R20 三）#####"
[ "$PRESS_OK" -ge 1 ] && echo "  [PASS] 批内含 ≥1 次加压档达标跑（实测 $PRESS_OK）" || echo "  [FAIL] 批内无加压档达标跑 ⇒ 批次判不符（R20 三）"
echo "  跑况：PASS=$NPASS FAIL=$NFAIL / $# 跑"
echo "  foxy 成对快照件数=$(ls "$B" | grep -c foxy_)"
[ "$PRESS_OK" -ge 1 ] || exit 6
