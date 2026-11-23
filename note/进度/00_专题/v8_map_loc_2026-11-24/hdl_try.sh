source /opt/ros/noetic/setup.bash; source /opt/ros1-slam/ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11313
EV=/tmp/v8map
rosnode kill -a >/dev/null 2>&1; sleep 2
for pid in $(ps -eo pid,args | awk '/gzserver/ && /worlds\// {print $1}'); do kill -TERM $pid 2>/dev/null; done
sleep 2
setsid roslaunch lite3_gazebo_sim m1_gazebo.launch > $EV/gz8.log 2>&1 < /dev/null &
for i in $(seq 1 40); do sleep 2; timeout 2 rostopic hz /rslidar_points 2>/dev/null | grep -q "average rate" && break; done
# IMU 名字对齐：hdl 订阅 /imu/data（仿真给 /livox/imu）
setsid rosrun topic_tools relay /livox/imu /imu/data > $EV/imu_relay.log 2>&1 < /dev/null &
sleep 2
L=$(rospack find lite3_localization)/launch/hdl_m1_tracking_smoke.launch
setsid roslaunch $L map_pcd:=$EV/out.pcd > $EV/hdl.log 2>&1 < /dev/null &
sleep 40
echo "--- 节点数 ---"; rosnode list | grep -cE "hdl|nodelet"
echo "--- hdl 进程 ---"; ps -eo pid,args | grep -c "[h]dl_localization"
echo "--- 崩溃关键字 ---"; grep -iE "SIGSEGV|exit code -11|process has died|segmentation" $EV/hdl.log | tail -3 | cut -c1-140
echo "--- 位姿输出 ---"; timeout 3 rostopic echo -n1 /hdl_odom 2>/dev/null | grep -E "^ +x:|^ +y:" | head -2
