source /opt/ros/noetic/setup.bash; source /opt/ros1-slam/ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11313
EV=/tmp/v8map; S=/home/ros1-slam/data/maps/lite3.pcd
rosnode kill -a >/dev/null 2>&1; sleep 2
for pid in $(ps -eo pid,args | awk '/gzserver/ && /worlds\// {print $1}'); do kill -TERM $pid 2>/dev/null; done
sleep 2
setsid roslaunch lite3_gazebo_sim m1_gazebo.launch > $EV/gz4.log 2>&1 < /dev/null &
for i in $(seq 1 40); do sleep 2; timeout 2 rostopic hz /rslidar_points 2>/dev/null | grep -q "average rate" && break; done
rosparam load $(rospack find lite3_slam)/config/faster_lio_outdoor.yaml /
rosparam set use_sim_time true
rosparam set pcd_save/pcd_save_en true
rosparam set pcd_save/interval -1
setsid rosrun faster_lio run_mapping_online __name:=faster_lio_online > $EV/lio3.log 2>&1 < /dev/null &
sleep 10
timeout 16 rostopic pub -r 10 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0}, angular: {z: 0.6}}" >/dev/null 2>&1
timeout 26 rostopic pub -r 10 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.4}, angular: {z: 0.0}}" >/dev/null 2>&1
timeout 14 rostopic pub -r 10 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0}, angular: {z: -0.6}}" >/dev/null 2>&1
rosnode kill /faster_lio_online >/dev/null 2>&1; sleep 6
cp -f $S $EV/lite3_flat_full.pcd
n=$(python3 -c "
f=open(\"$EV/lite3_flat_full.pcd\",\"rb\")
for line in f:
    if line.startswith(b\"POINTS\"): print(line.split()[1].decode()); break")
echo "PCD=$(ls -lh $EV/lite3_flat_full.pcd | awk '{print $5}') points=$n"
