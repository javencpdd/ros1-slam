#!/usr/bin/env bash
# SLAM 栈干净重启（2026-11-18 起草于 ~/.cache，2026-11-20 收进仓）
# 依赖件同在仓内：scripts/sim/{map_camera_init_tf.py,slam_view.rviz}
# 目的：清掉 11 小时运行留下的僵尸 TF 发布者（robot_state_publisher 等），
#      重建一棵干净的 TF 树，并补上 map→camera_init（让点云与机器人同系）。
set -e
cd /home/jack/ysc-code/ros1-slam
C="docker compose -f docker/docker-compose.yml --env-file docker/.env"

echo "[1/7] 清场：停掉 Gazebo/faster_lio/rviz/TF 桥（保留 roscore）"
$C exec -T -u 0 sim-viz bash -lc 'pkill -f "[g]zserver"; pkill -f "[g]zclient"; pkill -f "[r]un_mapping_online"; pkill -f "[r]oslaunch"; pkill -f "[r]obot_state_publisher"; pkill -f "[s]tatic_transform"; pkill -f "[m]ap_camera_init"; pkill -x rviz; sleep 4; echo "  剩余: gz=$(pgrep -c "[g]z" || echo 0) lio=$(pgrep -cf "[r]un_mapping_online" || echo 0)"'

echo "[2/7] 放入 TF 桥脚本与 rviz 配置"
$C cp scripts/sim/map_camera_init_tf.py sim-viz:/tmp/map_camera_init_tf.py
$C cp scripts/sim/slam_view.rviz sim-viz:/tmp/slam_view.rviz
$C exec -T -u 0 sim-viz bash -lc 'chmod 0755 /tmp/map_camera_init_tf.py; chmod 0644 /tmp/slam_view.rviz'

echo "[3/7] 起 m1 平地场景"
$C exec -T sim-viz bash -lc 'setsid nohup bash -c "source /opt/ros/noetic/setup.bash; source /opt/ros1-slam/ws/devel/setup.bash; export GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models:/opt/ros1-slam/sim_ws/src:/opt/ros1-slam/sim_ws/src/lite3_gazebo_sim/worlds; export GAZEBO_RESOURCE_PATH=/usr/share/gazebo-11:/opt/ros1-slam/sim_ws/src; export DISPLAY=:1; roslaunch lite3_gazebo_sim m1_gazebo.launch gui:=true > /opt/ros1-slam/logs/slam_gazebo.log 2>&1" >/dev/null 2>&1 &'
sleep 45

echo "[4/7] 起 faster_lio（重新初始化，清掉静止漂移）"
$C exec -T sim-viz bash -lc 'setsid nohup bash -c "source /opt/ros/noetic/setup.bash; source /opt/ros1-slam/ws/devel/setup.bash; roslaunch lite3_gazebo_sim m1_mapping.launch > /opt/ros1-slam/logs/slam_mapping.log 2>&1" >/dev/null 2>&1 &'
sleep 30

echo "[5/7] 补 map→camera_init 静态 TF"
$C exec -T sim-viz bash -lc 'setsid nohup bash -c "source /opt/ros/noetic/setup.bash; python3 /tmp/map_camera_init_tf.py > /opt/ros1-slam/logs/tf_bridge.log 2>&1" >/dev/null 2>&1 &'
sleep 6

echo "[6/7] 起 rviz"
$C exec -T sim-viz bash -lc 'setsid nohup bash -c "source /opt/ros/noetic/setup.bash; export DISPLAY=:1; rviz -d /tmp/slam_view.rviz > /opt/ros1-slam/logs/slam_rviz.log 2>&1" >/dev/null 2>&1 &'
sleep 14

echo "[7/7] 验证 TF 链"
$C exec -T sim-viz bash -lc '
source /opt/ros/noetic/setup.bash
echo "  --- map→base_link（机器人）---"
timeout 8 rosrun tf tf_echo map base_link 2>&1 | head -3 | sed "s/^/    /"
echo "  --- map→camera_init（点云）---"
timeout 6 rosrun tf tf_echo map camera_init 2>&1 | head -2 | sed "s/^/    /"
echo "  --- 进程 ---"
printf "    gz=%s lio=%s rviz=%s tf=%s\n" "$(pgrep -c "[g]zserver")" "$(pgrep -cf "[r]un_mapping_online")" "$(pgrep -c "[r]viz")" "$(pgrep -cf "[m]ap_camera_init")"
'
