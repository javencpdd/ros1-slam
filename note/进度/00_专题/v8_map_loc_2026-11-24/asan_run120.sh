set -u
source /opt/ros/noetic/setup.bash; source /opt/ros1-slam/ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11313
EV=/tmp/v8map; LIB=/opt/ros1-slam/ws/devel/lib/libhdl_localization_nodelet.so
cp -f $LIB $LIB.normal; cp -f /tmp/asan/libasan_hdl.so $LIB
echo "lib_now=$(ls -lh $LIB | awk '{print $5}')"
ASAN=/usr/lib/x86_64-linux-gnu/libasan.so.4
rosparam set use_sim_time true
rosnode kill -a >/dev/null 2>&1; sleep 2
for pid in $(ps -eo pid,args | awk '/gzserver/ && /worlds\// {print $1}'); do kill -TERM $pid 2>/dev/null; done
sleep 2
setsid roslaunch lite3_gazebo_sim m1_gazebo.launch > $EV/gz17.log 2>&1 < /dev/null &
for i in $(seq 1 40); do sleep 2; timeout 2 rostopic hz /rslidar_points 2>/dev/null | grep -q "average rate" && break; done
setsid rosrun topic_tools relay /rslidar_points /velodyne_points > $EV/rp3.log 2>&1 < /dev/null &
sleep 2
setsid env LD_PRELOAD=$ASAN ASAN_OPTIONS=detect_leaks=0:abort_on_error=0:log_path=$EV/asan120 \
  rosrun nodelet nodelet manager __name:=mgr > $EV/mgr120.log 2>&1 < /dev/null &
sleep 5
echo "asan_loaded=$(grep -c libasan /proc/$(pgrep -f 'nodelet manager' | head -1)/maps 2>/dev/null)"
rosparam set /gms/globalmap_pcd $EV/run11.pcd
rosparam set /gms/convert_utm_to_local false
rosparam set /gms/downsample_resolution 0.1
setsid rosrun nodelet nodelet load hdl_localization/GlobalmapServerNodelet mgr __name:=gms > $EV/gms120.log 2>&1 < /dev/null &
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
rosparam set /hdl/cool_time_duration 1.0
setsid rosrun nodelet nodelet load hdl_localization/HdlLocalizationNodelet mgr __name:=hdl /odom:=/hdl_odom > $EV/hdl120.log 2>&1 < /dev/null &
sleep 120
echo "=== 120s 后 ==="
echo "mgr_alive=$(rosnode list | grep -cx /mgr)"
echo "callbacks=$(grep -ch 'scan: raw' /opt/ros1-slam/logs/latest/*.log 2>/dev/null | paste -sd+ | bc 2>/dev/null)"
grep -rh 'scan: raw' /opt/ros1-slam/logs/latest/*.log 2>/dev/null | wc -l | sed 's/^/scan_lines=/'
grep -rh 'align done' /opt/ros1-slam/logs/latest/*.log 2>/dev/null | wc -l | sed 's/^/align_lines=/'
echo "topics: $(rostopic list | grep -E '^/(hdl_)?odom$' | tr '\n' ' ')"
echo "hdl_odom_pubs=$(rostopic info /hdl_odom 2>/dev/null | sed -n '/Publishers/,+1p' | grep -c '\*')"
timeout 5 rostopic hz /hdl_odom 2>&1 | grep -E 'average rate' | head -1
echo "asan_files=$(ls $EV/asan120* 2>/dev/null | wc -l)"
