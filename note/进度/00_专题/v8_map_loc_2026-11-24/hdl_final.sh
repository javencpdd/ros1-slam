set -u
source /opt/ros/noetic/setup.bash; source /opt/ros1-slam/ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11313
EV=/tmp/v8map
rosnode kill -a >/dev/null 2>&1; sleep 2
for pid in $(ps -eo pid,args | awk '/gzserver/ && /worlds\// {print $1}'); do kill -TERM $pid 2>/dev/null; done
sleep 2
setsid roslaunch lite3_gazebo_sim m1_gazebo.launch > $EV/gz15.log 2>&1 < /dev/null &
for i in $(seq 1 40); do sleep 2; timeout 2 rostopic hz /rslidar_points 2>/dev/null | grep -q "average rate" && break; done
# 去零点适配器（替代裸 relay）
setsid python3 /opt/ros1-slam/scripts/verify/cloud_denull.py --in /rslidar_points --out /velodyne_points > $EV/denull.log 2>&1 < /dev/null &
sleep 4
echo "denull_kept=$(grep -o 'kept [0-9]*/[0-9]*' $EV/denull.log | tail -1)"
setsid rosrun nodelet nodelet manager __name:=mgr > $EV/mgr4.log 2>&1 < /dev/null &
sleep 4
rosparam set /gms/globalmap_pcd $EV/run11.pcd
rosparam set /gms/convert_utm_to_local false
rosparam set /gms/downsample_resolution 0.1
setsid rosrun nodelet nodelet load hdl_localization/GlobalmapServerNodelet mgr __name:=gms > $EV/gms5.log 2>&1 < /dev/null &
sleep 12
echo "globalmap=$(rostopic list | grep -cx /globalmap)"
rosparam set /hdl/odom_child_frame_id base_link
rosparam set /hdl/use_imu false
rosparam set /hdl/use_global_localization false
rosparam set /hdl/specify_init_pose true
rosparam set /hdl/init_pos_x 0.0
rosparam set /hdl/init_pos_y 0.0
rosparam set /hdl/init_pos_z 0.0
rosparam set /hdl/init_ori_w 1.0
rosparam set /hdl/reg_method NDT_OMP
rosparam set /hdl/ndt_resolution 1.0
rosparam set /hdl/ndt_neighbor_search_method DIRECT1
rosparam set /hdl/ndt_neighbor_search_radius 3.0
rosparam set /hdl/downsample_resolution 0.3
rosparam set /hdl/cool_time_duration 1.0
setsid rosrun nodelet nodelet load hdl_localization/HdlLocalizationNodelet mgr __name:=hdl /odom:=/hdl_odom > $EV/hdl6.log 2>&1 < /dev/null &
sleep 18
echo "--- manager 是否存活 ---"; rosnode list | grep -cx /mgr
echo "--- /hdl_odom ---"; timeout 6 rostopic hz /hdl_odom 2>&1 | grep -E "average rate|no new" | head -1
timeout 3 rostopic echo -n1 /hdl_odom 2>/dev/null | grep -E "^ +x:|^ +y:" | head -2
echo "--- mgr 日志 ---"; tail -4 $EV/mgr4.log | cut -c1-140
