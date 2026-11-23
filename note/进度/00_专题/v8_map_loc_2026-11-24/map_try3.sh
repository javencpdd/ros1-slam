source /opt/ros/noetic/setup.bash; source /opt/ros1-slam/ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11313
EV=/tmp/v8map; mkdir -p $EV; rm -f $EV/*.pcd
echo "cloud=$(timeout 4 rostopic hz /rslidar_points 2>&1 | grep -c 'average rate')"
rosparam load $(rospack find lite3_slam)/config/faster_lio_outdoor.yaml /
rosparam set use_sim_time true
rosparam set pcd_save/pcd_save_en true
rosparam set pcd_save/path /tmp/v8map/lite3_flat.pcd
rosparam set pcd_save/interval 10
setsid rosrun faster_lio run_mapping_online __name:=faster_lio_online > $EV/lio.log 2>&1 < /dev/null &
sleep 12
echo "lio_nodes=$(rosnode list | grep -c faster_lio)"
echo "--- odom ---"; timeout 3 rostopic echo -n1 /Odometry 2>/dev/null | grep -E "x:|y:" | head -2
echo "--- 驱动 40s（原地转+前进）---"
timeout 16 rostopic pub -r 10 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0}, angular: {z: 0.5}}" >/dev/null 2>&1
timeout 16 rostopic pub -r 10 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.35}, angular: {z: 0.0}}" >/dev/null 2>&1
timeout 10 rostopic pub -r 10 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0}, angular: {z: -0.5}}" >/dev/null 2>&1
rosnode kill /faster_lio_online >/dev/null 2>&1; sleep 5
echo "--- PCD ---"; ls -lh $EV/*.pcd 2>&1 | awk '{print $5, $9}'
grep -iE "save|pcd" $EV/lio.log | tail -3 | cut -c1-120
