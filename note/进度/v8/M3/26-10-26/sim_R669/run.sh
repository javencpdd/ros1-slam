source /opt/ros/noetic/setup.bash
mkdir -p /work/logs /work/ros
export ROS_HOME=/work/ros ROS_LOG_DIR=/work/logs HOME=/work
export ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=127.0.0.1
export GAZEBO_MASTER_URI=http://127.0.0.1:11345 GAZEBO_MODEL_DATABASE_URI=
export PYTHONPATH=/work/planner:/work/contracts:$PYTHONPATH
roscore -p 11311 >/tmp/rc.log 2>&1 &
sleep 6
gzserver --verbose -s libgazebo_ros_api_plugin.so /work/w.world >/tmp/gz.log 2>&1 &
sleep 8
rosparam set /use_sim_time true
sleep 2
rosparam set /v8_support_planner/map_json /work/map.json
rosparam set /v8_support_planner/start "[0.0, 0.0, 0.0]"
rosparam set /v8_support_planner/goal "[1.0, 0.0, 0.35]"
rosparam set /v8_support_planner/budget 2000
rosparam set /v8_support_planner/path_topic /support_route
rosparam set /v8_support_planner/status_topic /support_route_status
python3 /work/check.py > /work/check.out 2>/tmp/check.err &
CPID=$!
sleep 2
python3 /work/planner/ros_node.py __name:=v8_support_planner >/tmp/node.log 2>&1 || echo "NODE_RC=$?" > /work/node.rc
wait $CPID || true
cat /work/check.out; echo "--- gz ---"; tail -3 /tmp/gz.log; echo "--- check.err ---"; tail -3 /tmp/check.err
