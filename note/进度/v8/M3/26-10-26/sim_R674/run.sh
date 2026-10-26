source /opt/ros/noetic/setup.bash
mkdir -p /work/logs /work/ros
export ROS_HOME=/work/ros ROS_LOG_DIR=/work/logs HOME=/work
export ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=127.0.0.1
export PYTHONUNBUFFERED=1
export PYTHONPATH=/work/planner:/work/contracts:$PYTHONPATH
roscore -p 11311 >/tmp/rc.log 2>&1 &
sleep 6
rosparam set /v8_support_planner/map_json /work/map.json
rosparam set /v8_support_planner/start "[0.0, 0.0, 0.0]"
rosparam set /v8_support_planner/goal "[1.0, 0.0, 0.35]"
rosparam set /v8_support_planner/budget 2000
rosparam set /v8_support_planner/path_topic /support_route
rosparam set /v8_support_planner/status_topic /support_route_status
rosparam set /v8_support_planner/enable_getpath true
rosparam set /v8_support_planner/keep_alive_s 40
python3 /work/planner/ros_node.py __name:=v8_support_planner >/tmp/node.log 2>&1 &
sleep 3
python3 /work/check.py; cat /work/check.out
echo "--- node log ---"; tail -12 /tmp/node.log
