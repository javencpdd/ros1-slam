#!/usr/bin/env bash
# r419_M3归因窗/win_driver.sh —— R419 窗「增采步」驱动（2026-10-19；x86 侧施工）
# 目的＝M3/M4 悬而未决的一项：**体素话题的实际名**（静态不可读，只能活体）。
# 做法＝复用格③ 段（x2_g6）已落盘的 params_x2_g6.yaml，另起一个同名参数的节点（ns=x2_g6t），
#        在窗口内 `rostopic list` 抓全量话题、再单独抓 voxel 命中与 plugins 回读。
# ★只增采、不改探针件（probe_x2_fourcells.sh 一字未动）、不改在册 yaml。
set -u
D="/opt/ros1-slam/note/进度/SCAN集成/证据/log/raw/r419_M3归因窗"
R="$D/run"
export ROS_MASTER_URI=http://127.0.0.1:11313 PYTHONNOUSERSITE=1
set +u; source /opt/ros/noetic/setup.bash; source /opt/ros1-slam/ws/devel/setup.bash; set -u

PIDS=""
cleanup() {
  for p in $PIDS; do kill "$p" 2>/dev/null; done
  sleep 3
  for q in $(ss -lntp 2>/dev/null | grep -o ':11313.*pid=[0-9]*' | grep -o 'pid=[0-9]*' | cut -d= -f2 | sort -u); do kill "$q" 2>/dev/null; done
}
trap cleanup EXIT

roscore -p 11313 > "$R/roscore_topics.log" 2>&1 & PIDS="$PIDS $!"
for i in $(seq 1 30); do rosnode list >/dev/null 2>&1 && break; sleep 1; done

rosparam load "$R/params_x2_g6.yaml" "/x2_g6t/costmap" || { echo "LOAD_FAIL"; exit 5; }
rosrun tf static_transform_publisher 0 0 0 0 0 0 map base_link 10 > "$R/tf_base_t.log" 2>&1 & PIDS="$PIDS $!"
rosrun tf static_transform_publisher 0 0 0.3 0 0 0 base_link rslidar 10 > "$R/tf_r_t.log" 2>&1 & PIDS="$PIDS $!"
X2_CLOUD_TAG=x2_g6t python3 "/opt/ros1-slam/note/进度/SCAN集成/设计/scripts/x2_pub_cloud.py" > "$R/cloud_x2_g6t.log" 2>&1 & PIDS="$PIDS $!"
rosrun costmap_2d costmap_2d_node __name:=x2_g6t > "$R/node_x2_g6t.log" 2>&1 & PIDS="$PIDS $!"
sleep 22

rostopic list 2>/dev/null | sort > "$R/topics_x2_g6t.txt"
grep -i "voxel" "$R/topics_x2_g6t.txt" > "$R/voxel_topics.txt" 2>/dev/null
rosparam get "/x2_g6t/costmap/plugins" > "$R/plugins_readback.txt" 2>&1
for t in $(cat "$R/voxel_topics.txt"); do
  echo "=== $t ===" >> "$R/voxel_topic_info.txt"
  timeout 15 rostopic info "$t" >> "$R/voxel_topic_info.txt" 2>&1
done
echo "TOPICS=$(wc -l < "$R/topics_x2_g6t.txt") VOXEL=$(wc -l < "$R/voxel_topics.txt")"
exit 0
