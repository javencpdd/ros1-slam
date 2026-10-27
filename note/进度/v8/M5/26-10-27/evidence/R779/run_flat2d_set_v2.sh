#!/bin/bash
# v2 定标子集：24 manifest（SC01/SC02 各 4 组×3 次），静态栈＋真 mux
set +u
source /opt/ros/noetic/setup.bash
source /opt/ros1-slam/ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11319 ROS_IP=127.0.0.1 HOME=/work ROS_HOME=/work/ros ROS_LOG_DIR=/work/logs
FLAT=/repo/note/计划书v8/calibration/flat2d
OUT=/work/flat2d_v2_runs
mkdir -p $OUT
[ -e /work/flat2d_v2.lock ] && { echo LOCKED; exit 1; }
touch /work/flat2d_v2.lock

bash /work/flat2d_stop2.sh
rosparam delete /move_base/global_costmap >/dev/null 2>&1
rosparam delete /move_base/local_costmap >/dev/null 2>&1
timeout 8 rosservice call /gazebo/set_model_state '{model_state: {model_name: lite3_planar, pose: {position: {x: 0.0, y: 0.0, z: 0.15}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}, reference_frame: world}}' >/dev/null 2>&1
rosparam set use_sim_time true
printf 'output_topic: /cmd_vel\noutput_rate_hz: 20\nhold_topic: /lite3/supervisor/hold\nsources:\n  - {name: follow, topic: /follow/cmd_vel, priority: 1, timeout_s: 0.5}\n' > /work/mux_flat2d.yaml
rosparam set /mux/mux_config /work/mux_flat2d.yaml
python3 /repo/ws/src/60_safety/lite3_safety/scripts/safe_cmd_vel_mux.py __name:=mux > /work/v2_mux.log 2>&1 &
python3 $FLAT/scan_relay.py > /work/v2_relay.log 2>&1 &
roslaunch $FLAT/flat2d_stack.launch > /work/v2_stack.log 2>&1 &
sleep 14

for m in $(ls /repo/note/计划书v8/calibration/flat2d/runs_v2/*.json | grep -v index.sha256 | sort); do
  tag=$(basename $m .json)
  [ -f $OUT/${tag}.json ] && { echo "SKIP $tag"; continue; }
  rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID '{}' >/dev/null 2>&1
  timeout 8 rosservice call /gazebo/set_model_state '{model_state: {model_name: lite3_planar, pose: {position: {x: 0.0, y: 0.0, z: 0.15}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}, reference_frame: world}}' >/dev/null 2>&1
  timeout 8 rosservice call /move_base/clear_costmaps >/dev/null 2>&1
  sleep 1
  timeout 180 python3 $FLAT/driver.py "$m" "$OUT/${tag}.json" > "$OUT/${tag}.log" 2>&1
  echo "V2 ${tag} rc=$? $(tail -1 $OUT/${tag}.log | cut -c1-120)"
  sleep 1
done
bash /work/flat2d_stop2.sh
rm -f /work/flat2d_v2.lock
echo V2_ALL_DONE
