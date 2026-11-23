set -u
source /opt/ros/noetic/setup.bash; source /opt/ros1-slam/ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11313
EV=/tmp/v8map
rosnode kill -a >/dev/null 2>&1; sleep 2
for pid in $(ps -eo pid,args | awk '/gzserver/ && /worlds\// {print $1}'); do kill -TERM $pid 2>/dev/null; done
sleep 2
setsid roslaunch lite3_gazebo_sim m1_gazebo.launch > $EV/gz10.log 2>&1 < /dev/null &
for i in $(seq 1 40); do sleep 2; timeout 2 rostopic hz /rslidar_points 2>/dev/null | grep -q "average rate" && break; done
setsid rosrun topic_tools relay /livox/imu /imu/data > $EV/imu2.log 2>&1 < /dev/null &
sleep 2
L=$(rospack find lite3_localization)/launch/hdl_m1_tracking_smoke.launch
setsid roslaunch $L map_pcd:=$EV/run11.pcd > $EV/hdl2.log 2>&1 < /dev/null &
sleep 18
echo "--- 节点 ---"; rosnode list | grep -E "hdl|nodelet" | tr '\n' ' '; echo
echo "--- 初始位姿下发 ---"
timeout 3 rostopic pub -1 /initialpose geometry_msgs/PoseWithCovarianceStamped "{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}" >/dev/null 2>&1
sleep 12
echo "--- /hdl_odom 是否有数据 ---"; timeout 4 rostopic hz /hdl_odom 2>&1 | grep -E "average rate|no new" | head -1
echo "--- 位姿 ---"; timeout 2 rostopic echo -n1 /hdl_odom 2>/dev/null | grep -E "^ +x:|^ +y:" | head -2
echo "--- /status ---"; timeout 2 rostopic echo -n1 /status 2>/dev/null | head -2 | cut -c1-120
echo "--- 崩溃 ---"; grep -icE "SIGSEGV|exit code -11|died" $EV/hdl2.log
