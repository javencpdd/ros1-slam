#!/usr/bin/env bash
# r419_M3归因窗/win_driver_v2.sh —— R419 窗「增采步」第二版（2026-10-19）
# v1 失败留名＝node_x2_g6t.log 0 字节、话题表无 /x2_g6t/costmap/costmap ⇒ 节点没挂上 master。
# v2 增加：①起前只清 11313 上 cmdline 含 rosmaster/roscore 的 holder（★不杀他人进程）
#          ②逐步自检并写进 selfcheck.txt（master 活着／节点在 rosnode list 里／话题里有没有 costmap）
#          ③节点给足 30s，再抓 topics。
set -u
D="/opt/ros1-slam/note/进度/SCAN集成/证据/log/raw/r419_M3归因窗"
R="$D/run"
export ROS_MASTER_URI=http://127.0.0.1:11313 PYTHONNOUSERSITE=1
set +u; source /opt/ros/noetic/setup.bash; source /opt/ros1-slam/ws/devel/setup.bash; set -u

SC="$R/selfcheck.txt"; : > "$SC"
say() { echo "$*" | tee -a "$SC"; }

for q in $(ss -lntp 2>/dev/null | grep -o ':11313.*pid=[0-9]*' | grep -o 'pid=[0-9]*' | cut -d= -f2 | sort -u); do
  cl=$(tr '\0' ' ' < /proc/$q/cmdline 2>/dev/null)
  case "$cl" in *rosmaster*|*roscore*) say "清残留 holder pid=$q cmd=$cl"; kill -9 "$q" 2>/dev/null;; *) say "★跳过非 master holder pid=$q cmd=$cl";; esac
done
sleep 2

PIDS=""
cleanup() { for p in $PIDS; do kill "$p" 2>/dev/null; done; sleep 3; }
trap cleanup EXIT

roscore -p 11313 > "$R/roscore_topics2.log" 2>&1 & PIDS="$PIDS $!"
ok=0; for i in $(seq 1 30); do rosnode list 2>/dev/null | grep -q rosout && { ok=1; break; }; sleep 1; done
say "master_ready=$ok rosnodes=$(rosnode list 2>/dev/null | tr '\n' ' ')"

rosparam load "$R/params_x2_g6.yaml" "/x2_g6t/costmap" && say "load_ok plugins=$(rosparam get /x2_g6t/costmap/plugins 2>&1 | grep -c 'name:')" || say "load_FAIL"
rosrun tf static_transform_publisher 0 0 0 0 0 0 map base_link 10 > "$R/tf_base_t2.log" 2>&1 & PIDS="$PIDS $!"
rosrun tf static_transform_publisher 0 0 0.3 0 0 0 base_link rslidar 10 > "$R/tf_r_t2.log" 2>&1 & PIDS="$PIDS $!"
X2_CLOUD_TAG=x2_g6t python3 "/opt/ros1-slam/note/进度/SCAN集成/设计/scripts/x2_pub_cloud.py" > "$R/cloud_x2_g6t2.log" 2>&1 & PIDS="$PIDS $!"
rosrun costmap_2d costmap_2d_node __name:=x2_g6t > "$R/node_x2_g6t2.log" 2>&1 & PIDS="$PIDS $!"
sleep 30

say "nodes_after=$(rosnode list 2>/dev/null | tr '\n' ' ')"
say "node_log_bytes=$(stat -c%s "$R/node_x2_g6t2.log" 2>/dev/null)"
rostopic list 2>/dev/null | sort > "$R/topics_v2.txt"
say "topics=$(wc -l < "$R/topics_v2.txt") costmap_topic=$(grep -c 'x2_g6t/costmap/costmap$' "$R/topics_v2.txt")"
grep -i "voxel" "$R/topics_v2.txt" > "$R/voxel_topics_v2.txt" 2>/dev/null
say "voxel=$(wc -l < "$R/voxel_topics_v2.txt")"
for t in $(cat "$R/voxel_topics_v2.txt"); do
  { echo "=== $t ==="; timeout 15 rostopic info "$t"; } >> "$R/voxel_topic_info_v2.txt" 2>&1
done
grep -aiE "plugin|voxel|error|fail" "$R/node_x2_g6t2.log" > "$R/node_kw.txt" 2>/dev/null
say "node_kw_lines=$(wc -l < "$R/node_kw.txt")"
exit 0
