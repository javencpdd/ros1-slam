set -u
source /opt/ros/noetic/setup.bash; source /opt/ros1-slam/ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11313
EV=/tmp/v8map
rosnode kill -a >/dev/null 2>&1; sleep 2
for pid in $(ps -eo pid,args | awk '/gzserver/ && /worlds\// {print $1}'); do kill -TERM $pid 2>/dev/null; done
sleep 2
setsid roslaunch lite3_gazebo_sim m1_gazebo.launch > $EV/gz13.log 2>&1 < /dev/null &
for i in $(seq 1 40); do sleep 2; timeout 2 rostopic hz /rslidar_points 2>/dev/null | grep -q "average rate" && break; done
setsid rosrun topic_tools relay /rslidar_points /velodyne_points > $EV/rp.log 2>&1 < /dev/null &
sleep 2
setsid rosrun nodelet nodelet manager __name:=mgr > $EV/mgr3.log 2>&1 < /dev/null &
sleep 4
rosparam set /gms/globalmap_pcd $EV/run11.pcd
rosparam set /gms/convert_utm_to_local false
rosparam set /gms/downsample_resolution 0.1
setsid rosrun nodelet nodelet load hdl_localization/GlobalmapServerNodelet mgr __name:=gms > $EV/gms4.log 2>&1 < /dev/null &
sleep 12
echo "globalmap=$(rostopic list | grep -cx /globalmap)"
# hdl 自身参数（私有 ns = /hdl）
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
setsid rosrun nodelet nodelet load hdl_localization/HdlLocalizationNodelet mgr __name:=hdl /odom:=/hdl_odom > $EV/hdl5.log 2>&1 < /dev/null &
sleep 15
echo "--- 订阅 ---"; rostopic info /velodyne_points | sed -n "/Subscribers/,+2p"
echo "--- /hdl_odom ---"; timeout 5 rostopic hz /hdl_odom 2>&1 | grep -E "average rate|no new" | head -1
timeout 3 rostopic echo -n1 /hdl_odom 2>/dev/null | grep -E "^ +x:|^ +y:" | head -2
echo "--- /status ---"; timeout 3 rostopic echo -n1 /status 2>/dev/null | head -2 | cut -c1-100
