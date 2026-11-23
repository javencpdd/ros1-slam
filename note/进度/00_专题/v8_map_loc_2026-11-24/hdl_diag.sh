set -u
source /opt/ros/noetic/setup.bash; source /opt/ros1-slam/ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11313
EV=/tmp/v8map
rosnode kill -a >/dev/null 2>&1; sleep 2
for pid in $(ps -eo pid,args | awk '/gzserver/ && /worlds\// {print $1}'); do kill -TERM $pid 2>/dev/null; done
sleep 2
setsid roslaunch lite3_gazebo_sim m1_gazebo.launch > $EV/gz11.log 2>&1 < /dev/null &
for i in $(seq 1 40); do sleep 2; timeout 2 rostopic hz /rslidar_points 2>/dev/null | grep -q "average rate" && break; done
setsid rosrun topic_tools relay /livox/imu /imu/data > $EV/imu3.log 2>&1 < /dev/null &
sleep 2
L=$(rospack find lite3_localization)/launch/hdl_m1_tracking_smoke.launch
setsid roslaunch $L map_pcd:=$EV/run11.pcd > $EV/hdl3.log 2>&1 < /dev/null &
sleep 20
echo "=== hdl nodelet 订阅 ==="; rosnode info /hdl_localization_nodelet 2>/dev/null | sed -n "/Subscriptions/,/Services/p" | head -10
echo "=== map server 发布 ==="; rosnode info /globalmap_server_nodelet 2>/dev/null | sed -n "/Publications/,/Subscriptions/p" | head -6
echo "=== /rslidar_points 订阅者 ==="; rostopic info /rslidar_points | sed -n "/Subscribers/,+3p"
echo "=== /map 是否有数据 ==="; timeout 3 rostopic hz /map 2>&1 | grep -E "average rate|no new" | head -1
echo "=== nodelet 日志 ==="; ls -t /opt/ros1-slam/logs/latest/*hdl* 2>/dev/null | head -3
echo "=== 日志里的错误 ==="; grep -hiE "error|fail|exit|core|abort" /opt/ros1-slam/logs/latest/*hdl*.log 2>/dev/null | tail -6 | cut -c1-150
