set -u
source /opt/ros/noetic/setup.bash; source /opt/ros1-slam/ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11313
EV=/tmp/v8map
rosnode kill -a >/dev/null 2>&1; sleep 2
for pid in $(ps -eo pid,args | awk '/gzserver/ && /worlds\// {print $1}'); do kill -TERM $pid 2>/dev/null; done
sleep 2
setsid roslaunch lite3_gazebo_sim m1_gazebo.launch > $EV/gz12.log 2>&1 < /dev/null &
for i in $(seq 1 40); do sleep 2; timeout 2 rostopic hz /rslidar_points 2>/dev/null | grep -q "average rate" && break; done
# 话题桥接：rslidar_points -> velodyne_points（hdl 硬编码）
setsid rosrun topic_tools relay /rslidar_points /velodyne_points > $EV/relay_pc.log 2>&1 < /dev/null &
# IMU 桥接（launch 里 use_imu=false，备用）
setsid rosrun topic_tools relay /livox/imu /gpsimu_driver/imu_data > $EV/relay_imu.log 2>&1 < /dev/null &
sleep 3
L=$(rospack find lite3_localization)/launch/hdl_m1_tracking_smoke.launch
setsid roslaunch $L map_pcd:=$EV/run11.pcd > $EV/hdl4.log 2>&1 < /dev/null &
sleep 25
echo "--- 订阅核对 ---"; rostopic info /velodyne_points | sed -n "/Subscribers/,+2p"
echo "--- /globalmap ---"; timeout 3 rostopic hz /globalmap 2>&1 | grep -E "average rate|no new" | head -1
echo "--- 发初始位姿 ---"
timeout 3 rostopic pub -1 /initialpose geometry_msgs/PoseWithCovarianceStamped "{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}" >/dev/null 2>&1
sleep 10
echo "--- /hdl_odom ---"; timeout 5 rostopic hz /hdl_odom 2>&1 | grep -E "average rate|no new" | head -1
timeout 3 rostopic echo -n1 /hdl_odom 2>/dev/null | grep -E "^ +x:|^ +y:" | head -2
