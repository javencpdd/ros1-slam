#!/usr/bin/env bash
# m2r5_regA.sh —— M2R5 批次一：V6 结构等价对照（改前/改后各一台）+ stageA 物理回归 + 新 5 连首跑
set +u
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION
source /opt/ros/noetic/setup.bash
source /home/ros1-slam/ws/devel/setup.bash
export GAZEBO_PLUGIN_PATH=/opt/ros/noetic/lib:/home/ros1-slam/ws/devel/lib
export ROS_MASTER_URI=http://127.0.0.1:11313 GAZEBO_MASTER_URI=http://127.0.0.1:11345
cd /home/ros1-slam/ws/src/95_simulation/lite3_gazebo_sim
P=/tmp/m2r5_out; mkdir -p $P
SM=/home/ros1-slam/ws/src/95_simulation/lite3_gazebo_sim/scripts/m2_v6_probe.py
PROBE_DIR=/tmp/m2r5_pre; mkdir -p $PROBE_DIR/urdf
cp -p /home/ros1-slam/note/进度/SCAN集成/证据/m2r5_baseline/lite3_planar.urdf.xacro.before_M2R5.bak \
      $PROBE_DIR/urdf/lite3_planar.urdf.xacro
ln -sfn /home/ros1-slam/ws/src/95_simulation/lite3_gazebo_sim/worlds $PROBE_DIR/worlds

teardown() {
  rosnode kill -a >/dev/null 2>&1; sleep 3
  pkill -f gzserver >/dev/null 2>&1; pkill -f rosmaster >/dev/null 2>&1; pkill -f rosserver >/dev/null 2>&1
  sleep 4
  echo "  teardown 残留:[$(pgrep -f 'roslaunch|rosmaster|gzserver|move_base_flex' | tr '\n' ' ')]"
}
snap_foxy() { echo "### foxy lstart $1 $(date '+%T')"; ps -o pid,user,lstart,etime,comm --no-headers \
  -p 115711,115792,115795,115797,115799,180237 2>/dev/null | tee "$P/foxy_$1.txt"; echo "  计数=$(wc -l < "$P/foxy_$1.txt")"; }
ports() { echo "  TCP 11312=$(ss -lnt 2>/dev/null | grep -c ':11312') 11313=$(ss -lnt 2>/dev/null | grep -c ':11313')"; \
          echo "  UDP 11312=$(ss -lnu 2>/dev/null | grep -c ':11312') 11313=$(ss -lnu 2>/dev/null | grep -c ':11313')"; }

probe() { # $1=pre|post  $2=payload_dir
  local tag=$1 pd=$2
  echo "##### V6 $tag  payload_dir=$pd  $(date '+%T') #####"
  snap_foxy "start_v6_$tag"
  roslaunch lite3_gazebo_sim m2_gazebo.launch payload_dir:=$pd > "$P/v6_$tag_gazebo.log" 2>&1 &
  local LP=$!
  local t=0 got=0
  while [ $t -lt 300 ]; do
    if rostopic list 2>/dev/null | grep -qx /gazebo/model_states; then got=1; break; fi
    sleep 5; t=$((t+5))
  done
  echo "  model_states 出现=$got 等待=${t}s"
  [ $got -eq 1 ] || sleep 60
  for i in 1 2 3 4 5; do
    python3 "$SM" "$tag" > "$P/v6_$tag.txt" 2>&1
    if grep -q "tag=$tag samples=" "$P/v6_$tag.txt"; then echo "  probe 第 $i 次取到"; break; fi
    echo "  probe 第 $i 次空采，再等 20s"; sleep 20
  done
  grep -E "^V6" "$P/v6_$tag.txt" | sed 's/^/    /'
  kill $LP >/dev/null 2>&1; sleep 3; teardown
}

probe pre  /tmp/m2r5_pre
probe post /home/ros1-slam/ws/src/95_simulation/lite3_gazebo_sim

echo "##### stageA 物理回归（root 变更属被测件变更，A0-A4 必须重取） $(date '+%T') #####"
snap_foxy start_stageA
bash verify/m2_smoke.sh stageA > "$P/stageA.log" 2>&1; echo "  stageA rc=$?"
snap_foxy end_stageA
grep -oE "drift=[0-9.]+|rate_pts=[0-9.]+|rate_imu=[0-9.]+|rtf_motion=[0-9.]+|fly_ratio=[0-9.]+|\[[PASS|FAIL]\] A" "$P/stageA.log" | head -12 | tr '\n' ' '; echo
echo "  $(grep -cE '^\[PASS\]' "$P/stageA.log") PASS / $(grep -cE '^\[FAIL\]' "$P/stageA.log") FAIL"; teardown

echo "##### 新 5 连首跑 fresh1n（G5c1 已升阻断门） $(date '+%T') #####"
snap_foxy start_fresh1n
TF_SOURCE="" bash verify/m2_smoke.sh stageB > "$P/fresh1n.log" 2>&1; RC=$?
snap_foxy end_fresh1n
echo "  rc=$RC"
grep -E "SMOKE (gate|G5_DELAY|SIMNOW|DIAG|E3|teb_state)" "$P/fresh1n.log" | sed 's/^/    /'
echo "  $(grep -cE '^\[PASS\]' "$P/fresh1n.log") PASS / $(grep -cE '^\[FAIL\]' "$P/fresh1n.log") FAIL"; ports
echo "REGA_DONE rc=$RC $(date '+%T')"
