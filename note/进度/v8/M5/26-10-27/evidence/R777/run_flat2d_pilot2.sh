#!/bin/bash
# flat2d v2 pilot：真栈（map_server+move_base+真 mux+感知层）+ 仓内 driver，候选目标定标
set +u
source /opt/ros/noetic/setup.bash
source /opt/ros1-slam/ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11319 ROS_IP=127.0.0.1 HOME=/work ROS_HOME=/work/ros ROS_LOG_DIR=/work/logs
FLAT=/repo/note/计划书v8/calibration/flat2d
OUT=/work/flat2d_pilot2
mkdir -p $OUT

bash /work/flat2d_stop2.sh
timeout 8 rosservice call /gazebo/set_model_state '{model_state: {model_name: lite3_planar, pose: {position: {x: 0.0, y: 0.0, z: 0.15}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}, reference_frame: world}}' >/dev/null 2>&1

printf 'output_topic: /cmd_vel\noutput_rate_hz: 20\nhold_topic: /lite3/supervisor/hold\nsources:\n  - {name: follow, topic: /follow/cmd_vel, priority: 1, timeout_s: 0.5}\n' > /work/mux_flat2d.yaml
rosparam set /mux/mux_config /work/mux_flat2d.yaml
python3 /repo/ws/src/60_safety/lite3_safety/scripts/safe_cmd_vel_mux.py __name:=mux > /work/flat2d_mux.log 2>&1 &
python3 $FLAT/scan_relay.py > /work/flat2d_relay.log 2>&1 &
rosparam set use_sim_time true
roslaunch $FLAT/flat2d_stack.launch > /work/flat2d_stack2.log 2>&1 &
sleep 14

python3 - "$OUT" <<'PY'
import json, pathlib, sys
out = pathlib.Path(sys.argv[1])
goals = [('SC01', 0, (3.0, 0.0)), ('SC02', 0, (5.0, 2.2)), ('SC02', 1, (5.2, 2.6)),
         ('SC02', 2, (6.2, 2.2)), ('SC02', 3, (5.0, -2.6)), ('SC02', 4, (4.0, -1.8)),
         ('SC02', 5, (6.2, -2.2))]
for sc, g, goal in goals:
    run_id = 'pilot2_%s_g%d' % (sc, g)
    man = {'run_id': run_id, 'run_class': 'pilot', 'calibration_only': False,
           'scenario_id': sc, 'seed': g, 'repetition': 0, 'goal': list(goal),
           'start': [0.0, 0.0], 'criteria': {'goal_position_m': 0.30, 'min_clearance_m': 0.20, 'deadline_s': 90},
           'inject': None}
    (out / (run_id + '.json')).write_text(json.dumps(man))
PY

for m in $OUT/pilot2_*.json; do
  tag=$(basename $m .json)
  rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID '{}' >/dev/null 2>&1
  timeout 8 rosservice call /gazebo/set_model_state '{model_state: {model_name: lite3_planar, pose: {position: {x: 0.0, y: 0.0, z: 0.15}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}, reference_frame: world}}' >/dev/null 2>&1
  timeout 8 rosservice call /move_base/clear_costmaps >/dev/null 2>&1
  timeout 300 python3 $FLAT/driver.py "$m" "$OUT/${tag}_res.json" > "$OUT/${tag}.log" 2>&1
  echo "PILOT ${tag} rc=$? $(tail -1 $OUT/${tag}.log | cut -c1-220)"
  sleep 2
done
echo PILOT_ALL_DONE
