#!/usr/bin/env bash
# m2r6_reg.sh —— M2R6（小美 R10/R14/R16 落地后的验证批）
# 三跑：fresh6n 常规（验 R10 逐跑退净打印）、fresh7n 加压达标（G5_DELAY_SIM=62 ⇒ press_late_G5=PASS）、
#       fresh8n 加压不达标阴性对照（G5_DELAY_SIM=25 ⇒ 期望 press_late_G5=FAIL、gate=FAIL、rc<>0）——R14 要求的新门真实 FAIL
set +u
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION
source /opt/ros/noetic/setup.bash
source /home/ros1-slam/ws/devel/setup.bash
export GAZEBO_PLUGIN_PATH=/opt/ros/noetic/lib:/home/ros1-slam/ws/devel/lib
cd /home/ros1-slam/ws/src/95_simulation/lite3_gazebo_sim
P=/tmp/m2r6_out; mkdir -p $P
snap_foxy(){ echo "### foxy $1 $(date '+%T')"; ps -o pid,user,lstart,etime,comm --no-headers -p 115711,115792,115795,115797,115799,180237 2>/dev/null | tee "$P/foxy_$1.txt" | wc -l | sed 's/^/  计数=/'; }
run(){ local tag=$1 dly=$2 exp=$3
  echo "##### $tag G5_DELAY_SIM=[$dly] 期望=$exp $(date '+%T') #####"
  snap_foxy start_$tag
  TF_SOURCE="" G5_DELAY_SIM="$dly" bash verify/m2_smoke.sh stageB > "$P/$tag.log" 2>&1
  local rc=$?
  snap_foxy end_$tag
  echo "  rc=$rc"
  grep -E "SMOKE (gate|SIMNOW|G5_DELAY|E3 tf|E3 base|teb_state)" "$P/$tag.log" | sed 's/^/    /'
  grep -E "^\[exitclean\]" "$P/$tag.log" | sed 's/^/    /'
  echo "  判定行: $(grep -cE '^\[PASS\]' "$P/$tag.log") PASS / $(grep -cE '^\[FAIL\]' "$P/$tag.log") FAIL"
  if [ "$exp" = "PASS" ] && [ $rc -eq 0 ]; then echo "  VERDICT=符合期望"; elif [ "$exp" = "FAIL" ] && [ $rc -ne 0 ]; then echo "  VERDICT=符合期望（阴性对照如预期失败 ⇒ 新门 press_late_G5 有阻断力，R14 义务履行）"; else echo "  VERDICT=★不符期望"; fi
  sleep 4
}
run fresh6n "" PASS
run fresh7n "62" PASS
run fresh8n "25" FAIL
echo "### 终态退净 $(date '+%T')"; pgrep -af 'roslaunch|rosmaster|gzserver|move_base_flex' | head -5; snap_foxy final
echo "R6_DONE $(date '+%T')"
