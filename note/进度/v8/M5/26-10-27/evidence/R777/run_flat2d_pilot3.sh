#!/bin/bash
set +u
source /opt/ros/noetic/setup.bash
source /opt/ros1-slam/ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11319 ROS_IP=127.0.0.1 HOME=/work ROS_HOME=/work/ros ROS_LOG_DIR=/work/logs
FLAT=/repo/note/计划书v8/calibration/flat2d
OUT=/work/flat2d_pilot3
mkdir -p $OUT
python3 - "$OUT" <<'PY'
import json, pathlib, sys
out = pathlib.Path(sys.argv[1])
for i, goal in enumerate([(2.0, 3.0), (4.0, 2.6), (6.4, 0.2)]):
    man = {'run_id': 'pilot3_g%d' % i, 'run_class': 'pilot', 'calibration_only': False,
           'scenario_id': 'SC02', 'seed': i, 'repetition': 0, 'goal': list(goal),
           'start': [0.0, 0.0],
           'criteria': {'goal_position_m': 0.30, 'min_clearance_m': 0.20, 'deadline_s': 90},
           'inject': None}
    (out / ('pilot3_g%d.json' % i)).write_text(json.dumps(man))
PY
for m in $OUT/pilot3_g*.json; do
  tag=$(basename $m .json)
  rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID '{}' >/dev/null 2>&1
  timeout 8 rosservice call /gazebo/set_model_state '{model_state: {model_name: lite3_planar, pose: {position: {x: 0.0, y: 0.0, z: 0.15}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}, reference_frame: world}}' >/dev/null 2>&1
  timeout 8 rosservice call /move_base/clear_costmaps >/dev/null 2>&1
  timeout 200 python3 $FLAT/driver.py "$m" "$OUT/${tag}_res.json" > "$OUT/${tag}.log" 2>&1
  echo "P3 ${tag} rc=$? $(tail -1 $OUT/${tag}.log | cut -c1-200)"
  sleep 2
done
echo PILOT3_DONE
