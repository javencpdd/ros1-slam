source /opt/ros/noetic/setup.bash; source /opt/ros1-slam/ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11313
EV=/tmp/v8map; S=/home/ros1-slam/data/maps/lite3.pcd
cp -f $EV/sample_backup.pcd $S; echo "样张已还原 sha=$(sha256sum $S | awk '{print substr($1,1,12)}')"
sha0=$(sha256sum $S | awk '{print $1}'); rm -f $EV/out.pcd
rosnode kill -a >/dev/null 2>&1; sleep 2
for pid in $(ps -eo pid,args | awk '/gzserver/ && /worlds\// {print $1}'); do kill -TERM $pid 2>/dev/null; done
sleep 2
setsid roslaunch lite3_gazebo_sim m1_gazebo.launch > $EV/gz7.log 2>&1 < /dev/null &
for i in $(seq 1 40); do sleep 2; timeout 2 rostopic hz /rslidar_points 2>/dev/null | grep -q "average rate" && break; done
# 关键：先写私有参数，再起节点
rosparam load $(rospack find lite3_slam)/config/faster_lio_outdoor.yaml /
rosparam set use_sim_time true
rosparam set /faster_lio_online/pcd_save/pcd_save_en true
rosparam set /faster_lio_online/pcd_save/interval -1
rosparam set /faster_lio_online/pcd_save/path $EV/out.pcd
echo "pre_set=$(rosparam get /faster_lio_online/pcd_save/path)"
setsid rosrun faster_lio run_mapping_online __name:=faster_lio_online > $EV/lio6.log 2>&1 < /dev/null &
sleep 10
timeout 12 rostopic pub -r 10 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0}, angular: {z: 0.6}}" >/dev/null 2>&1
timeout 26 rostopic pub -r 10 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.45}, angular: {z: 0.0}}" >/dev/null 2>&1
timeout 12 rostopic pub -r 10 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0}, angular: {z: 1.2}}" >/dev/null 2>&1
timeout 26 rostopic pub -r 10 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.45}, angular: {z: 0.0}}" >/dev/null 2>&1
rosnode kill /faster_lio_online >/dev/null 2>&1; sleep 10
echo "out=$(ls -lh $EV/out.pcd 2>/dev/null | awk '{print $5}')"
sha1=$(sha256sum $S | awk '{print $1}'); [ "$sha0" = "$sha1" ] && echo "样张未变 ✓" || echo "样张被改动 ✗"
grep -oE "saved to [^ ]+" $EV/lio6.log | tail -1
