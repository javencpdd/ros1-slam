#!/usr/bin/env bash
# r414_B4_T4对齐窗 驱动（x86 联调旁证窗；甲选＝V6-3 定向覆盖）
# 内容＝一帧 /LIO/clouds_lidar 证「同一码路」对齐（v5.4 T4 行定义＋v6 §B.4）
set +u
. /opt/ros/noetic/setup.bash
. /opt/ros1-slam/ws/devel/setup.bash
export PYTHONNOUSERSITE=1
export ROS_MASTER_URI=http://127.0.0.1:11313
EV=/opt/ros1-slam/note/进度/SCAN集成/证据/log/raw/r414_B4_T4对齐窗
mkdir -p "$EV"
LOG=$EV/window_run.log
say(){ echo "[$(date '+%F %T')] $*" | tee -a "$LOG"; }

witness(){ # $1=标签
  {
    echo "# 见证基准 $1（$(date '+%F %T')；分域两路：被试域＝容器 netns 判、装置域＝宿主机只并报）"
    echo "被试域 pgrep：gzserver=$(pgrep -x gzserver|wc -l) gzclient=$(pgrep -x gzclient|wc -l) roslaunch=$(pgrep -x roslaunch|wc -l) roscore=$(pgrep -x roscore|wc -l) rosmaster=$(pgrep -x rosmaster|wc -l)"
    echo "被试域 ss：TCP_1131=$(ss -lnt 2>/dev/null|grep -cE ':1131[1-9]') UDP_1131=$(ss -lnu 2>/dev/null|grep -cE ':1131[1-9]') TCP_1134=$(ss -lnt 2>/dev/null|grep -cE ':1134[0-9]') UDP_1134=$(ss -lnu 2>/dev/null|grep -cE ':1134[0-9]')"
    echo "装置域（宿主，只并报）：TCP_1131=$(ss -lnt 2>/dev/null|grep -cE ':1131[1-9]') UDP_1131=$(ss -lnu 2>/dev/null|grep -cE ':1131[1-9]')"
  } > "$EV/witness_$1.txt"
}
wait_topic(){ local t=$1 n=$2 i; for i in $(seq 1 "$n"); do rostopic info "$t" >/dev/null 2>&1 && return 0; sleep 1; done; return 1; }

witness before
bash /opt/ros1-slam/scripts/verify/v_window_preflight_x86_variant.sh > "$EV/preflight_variant.txt" 2>&1
say "预检变体 rc=$? （0＝放行）"
tail -1 "$EV/preflight_variant.txt" | tee -a "$LOG"

say "起链：m2_gazebo → m2_mapping → lio_relay_sim"
roslaunch lite3_gazebo_sim m2_gazebo.launch > "$EV/gazebo.log" 2>&1 &
GZ=$!
wait_topic /clock 180        || { say "FAIL /clock 未出"; }
wait_topic /rslidar_points 300 || { say "FAIL /rslidar_points 未出"; }
wait_topic /livox/imu 180    || { say "FAIL /livox/imu 未出"; }
say "sim 三话题就位"
roslaunch lite3_gazebo_sim m2_mapping.launch > "$EV/mapping.log" 2>&1 &
MP=$!
wait_topic /cloud_registered_body 300 || { say "FAIL /cloud_registered_body 未出"; }
say "faster_lio 就位"
roslaunch lite3_scan lio_relay_sim.launch > "$EV/relay.log" 2>&1 &
RL=$!
if wait_topic /LIO/clouds_lidar 180; then
  say "/LIO/clouds_lidar 在位 ⇒ 采一帧"
  timeout 90 rostopic echo -n1 /LIO/clouds_lidar > "$EV/frame_LIO_clouds_lidar.txt" 2>&1
  echo "frame_rc=$?" >> "$EV/frame_LIO_clouds_lidar.txt"
  rostopic info /LIO/clouds_lidar > "$EV/topic_info.txt" 2>&1
  say "帧采完（字节=$(wc -c < "$EV/frame_LIO_clouds_lidar.txt")）"
else
  say "★/LIO/clouds_lidar 未出 ⇒ 记 N3（前件不成立）、不判被试"
  echo "N3: /LIO/clouds_lidar 未在 180s 内出现" > "$EV/frame_LIO_clouds_lidar.txt"
fi
# 同一码路证据：faster_lio 可执行件 sha（x86 侧）
{
  echo "# 同一码路证据（x86 侧，2026-10-19）"
  find /opt/ros1-slam/ws/devel -name 'run_mapping_online' -type f 2>/dev/null | head -2 | xargs -r sha256sum
  find /opt/ros1-slam/ws/devel -name '*faster_lio*' -name '*.so' 2>/dev/null | head -4 | xargs -r sha256sum
} > "$EV/same_codepath_shas.txt"
say "teardown（只收本件 PID）"
kill $RL $MP $GZ 2>/dev/null
sleep 8
for p in $(pgrep -f 'm2_gazebo|m2_mapping|lio_relay_sim' 2>/dev/null); do kill "$p" 2>/dev/null; done
sleep 5
witness after
say "窗结束（起前／退后见证见 witness_*.txt）"
