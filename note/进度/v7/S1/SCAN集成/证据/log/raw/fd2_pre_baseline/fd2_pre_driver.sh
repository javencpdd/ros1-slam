#!/usr/bin/env bash
set +u; unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION
source /opt/ros/noetic/setup.bash; source /home/ros1-slam/ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11313 GAZEBO_MASTER_URI=http://127.0.0.1:11345
export GAZEBO_PLUGIN_PATH=/opt/ros/noetic/lib:/home/ros1-slam/ws/devel/lib
cd /home/ros1-slam/ws/src/95_simulation/lite3_gazebo_sim
O=/tmp/fd2; mkdir -p $O
rosnode kill -a >/dev/null 2>&1; pkill -f "rosmaster --core -p 11313" >/dev/null 2>&1; sleep 2
roslaunch lite3_bringup mode_sim_teb.launch > $O/teb_chain.log 2>&1 &
T=0; until [ $T -ge 300 ]; do rostopic list 2>/dev/null | grep -qx /Odometry && break; sleep 3; T=$((T+3)); done
echo "### 窗一：stageB 拓扑（无 faster_lio）$([ $T -lt 300 ] && echo "/Odometry 在位 ${T}s" || echo "超时")"
bash verify/v_pose_source.sh legacy_b 8 2>&1 | tee $O/legacy_b.txt
echo "### 窗二：加 faster_lio（stageA 拓扑）$(date '+%T')"
roslaunch lite3_gazebo_sim m2_mapping.launch > $O/map_chain.log 2>&1 &
T=0; until [ $T -ge 360 ]; do rostopic list 2>/dev/null | grep -qx /cloud_registered && break; sleep 3; T=$((T+3)); done
echo "  /cloud_registered $([ $T -lt 360 ] && echo "在位 ${T}s" || echo 超时)"; sleep 25
bash verify/v_pose_source.sh pre 10 2>&1 | tee $O/pre.txt
echo "### 随窗取证：rostopic info 原样落件"
for t in /Odometry /sim/odom_gt; do echo "--- $t ---"; timeout 15 rostopic info $t 2>&1; done > $O/topics_info.txt 2>&1
sed -n '1,14p' $O/topics_info.txt
rosnode kill -a >/dev/null 2>&1; sleep 3; pkill -f gzserver >/dev/null 2>&1; pkill -f run_mapping_online >/dev/null 2>&1
pkill -f "roslaunch.*mode_sim" >/dev/null 2>&1; pkill -f "rosmaster --core -p 11313" >/dev/null 2>&1; sleep 4
echo "### FD2 退净：残留[$(pgrep -af 'roslaunch|rosmaster|gzserver|run_mapping_online|move_base_flex' | grep -v pgrep | wc -l)] 端口11313=$(ss -lnt 2>/dev/null | grep -c ':11313')"
echo "FD2_DONE $(date '+%T')"
