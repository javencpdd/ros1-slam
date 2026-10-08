#!/usr/bin/env bash
# m2r5_regB.sh —— M2R5 批次二：V6 前后对照（探针字段已按盘上 srv 定义改正）+ 新 5 连续跑（fresh2n-fresh5n，fresh3n 走加压档）
set +u
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION
source /opt/ros/noetic/setup.bash
source /home/ros1-slam/ws/devel/setup.bash
export GAZEBO_PLUGIN_PATH=/opt/ros/noetic/lib:/home/ros1-slam/ws/devel/lib
export ROS_MASTER_URI=http://127.0.0.1:11313 GAZEBO_MASTER_URI=http://127.0.0.1:11345
cd /home/ros1-slam/ws/src/95_simulation/lite3_gazebo_sim
P=/tmp/m2r5_out
SM=scripts/m2_v6_probe.py
mkdir -p $P
teardown() { rosnode kill -a >/dev/null 2>&1; sleep 3; pkill -f gzserver >/dev/null 2>&1; pkill -f rosmaster >/dev/null 2>&1; sleep 4; echo "  teardown 残留:[$(pgrep -f 'roslaunch|rosmaster|gzserver|move_base_flex' | tr '\n' ' ')]"; }
snap_foxy() { echo "### foxy lstart $1 $(date '+%T')"; ps -o pid,user,lstart,etime,comm --no-headers -p 115711,115792,115795,115797,115799,180237 2>/dev/null | tee "$P/foxy_$1.txt" | wc -l | sed 's/^/  计数=/'; }
ports() { echo "  TCP 11312=$(ss -lnt 2>/dev/null | grep -c ':11312') 11313=$(ss -lnt 2>/dev/null | grep -c ':11313')"; echo "  UDP 11312=$(ss -lnu 2>/dev/null | grep -c ':11312') 11313=$(ss -lnu 2>/dev/null | grep -c ':11313')"; }

probe() { local tag=$1 pd=$2
  echo "##### V6r2 $tag  payload_dir=$pd  $(date '+%T') #####"
  snap_foxy "start_v6r2_$tag"
  roslaunch lite3_gazebo_sim m2_gazebo.launch payload_dir:=$pd > "$P/v6r2_${tag}_gazebo.log" 2>&1 &
  local LP=$! t=0
  while [ $t -lt 300 ]; do rostopic list 2>/dev/null | grep -qx /gazebo/model_states && break; sleep 5; t=$((t+5)); done
  echo "  model_states 等待=${t}s"; sleep 25
  for i in 1 2 3; do
    python3 "$SM" "$tag" > "$P/v6r2_$tag.txt" 2>&1
    grep -q "tag=$tag samples=" "$P/v6r2_$tag.txt" && { echo "  probe 第 $i 次取到"; break; }
    echo "  probe 第 $i 次空采，再等 20s"; sleep 20
  done
  grep -E "^V6" "$P/v6r2_$tag.txt" | sed 's/^/    /'
  kill $LP >/dev/null 2>&1; sleep 3; teardown
}

runb() { local tag=$1 dly=$2
  echo "##### $tag  G5_DELAY_SIM=[$dly]  $(date '+%T') #####"
  snap_foxy "start_$tag"
  TF_SOURCE="" G5_DELAY_SIM="$dly" bash verify/m2_smoke.sh stageB > "$P/$tag.log" 2>&1
  local rc=$?
  snap_foxy "end_$tag"
  echo "  rc=$rc"
  grep -E "SMOKE (gate|SIMNOW|G5_DELAY|E3|teb_state)" "$P/$tag.log" | sed 's/^/    /'
  echo "  $(grep -cE '^\[PASS\]' "$P/$tag.log") PASS / $(grep -cE '^\[FAIL\]' "$P/$tag.log") FAIL   TF_OLD_DATA=$(grep -c TF_OLD_DATA "$P/$tag.log")"
  ports; sleep 4
}

probe pre  /tmp/m2r5_pre
probe post /home/ros1-slam/ws/src/95_simulation/lite3_gazebo_sim
runb fresh2n ""
runb fresh3n "62"
runb fresh4n ""
runb fresh5n ""
echo "##### 退净核查 $(date '+%T') #####"; echo "残留:[$(pgrep -f 'roslaunch|rosmaster|gzserver|move_base_flex' | tr '\n' ' ')]"; ports
snap_foxy final
echo "REGB_DONE $(date '+%T')"
