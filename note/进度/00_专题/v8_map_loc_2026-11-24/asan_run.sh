set -u
source /opt/ros/noetic/setup.bash; source /opt/ros1-slam/ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11313
EV=/tmp/v8map
LIB=/opt/ros1-slam/ws/devel/lib/libhdl_localization_nodelet.so
[ -f $LIB.orig ] || cp -f $LIB $LIB.orig
cp -f /tmp/asan/libasan_hdl.so $LIB
echo "已换 ASAN 库: $(ls -lh $LIB | awk '{print $5}')"
ASAN=$(ls /usr/lib/x86_64-linux-gnu/libasan.so.* 2>/dev/null | head -1); echo "asan=$ASAN"
rosnode kill -a >/dev/null 2>&1; sleep 2
for pid in $(ps -eo pid,args | awk '/gzserver/ && /worlds\// {print $1}'); do kill -TERM $pid 2>/dev/null; done
sleep 2
setsid roslaunch lite3_gazebo_sim m1_gazebo.launch > $EV/gz16.log 2>&1 < /dev/null &
for i in $(seq 1 40); do sleep 2; timeout 2 rostopic hz /rslidar_points 2>/dev/null | grep -q "average rate" && break; done
setsid rosrun topic_tools relay /rslidar_points /velodyne_points > $EV/rp2.log 2>&1 < /dev/null &
sleep 2
setsid env LD_PRELOAD=$ASAN ASAN_OPTIONS=detect_leaks=0:abort_on_error=0:log_path=/tmp/v8map/asan \
  rosrun nodelet nodelet manager __name:=mgr > $EV/mgr_asan.log 2>&1 < /dev/null &
sleep 5
rosparam set /gms/globalmap_pcd $EV/run11.pcd
rosparam set /gms/convert_utm_to_local false
rosparam set /gms/downsample_resolution 0.1
setsid rosrun nodelet nodelet load hdl_localization/GlobalmapServerNodelet mgr __name:=gms > $EV/gms_asan.log 2>&1 < /dev/null &
sleep 12
rosparam set /hdl/odom_child_frame_id base_link
rosparam set /hdl/use_imu false
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
setsid rosrun nodelet nodelet load hdl_localization/HdlLocalizationNodelet mgr __name:=hdl /odom:=/hdl_odom > $EV/hdl_asan.log 2>&1 < /dev/null &
sleep 20
echo "--- mgr 存活 ---"; rosnode list | grep -cx /mgr
echo "--- asan 报告文件 ---"; ls -t /tmp/v8map/asan* 2>/dev/null | head -2
