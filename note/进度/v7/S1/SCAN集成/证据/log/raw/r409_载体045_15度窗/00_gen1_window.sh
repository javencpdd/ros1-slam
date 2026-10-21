#!/usr/bin/env bash
# 00_tf_window.sh —— 第 0 步活体 TF 实测窗驱动（x86 侧、容器内跑）
# 窗＝r409_载体045_15度窗/w1_tf_live｜窗型＝联调·旁证（★不入分子、★不开判据窗、★不录 bag 故不称三样存证齐）
# ★纪律：预检变体 rc≠0 ⇒ 不起窗；★只 kill 本窗自己起的 PID；★在册件一字不改；跑完清 __pycache__ 且与计数同条命令（坑67②）
set -u
cd /opt/ros1-slam
H=/opt/ros1-slam/note/进度/SCAN集成/证据/log/raw/r409_载体045_15度窗
W="$H/${WNAME:-w3_gen1_live}"
mkdir -p "$W"
set +u; . /opt/ros/noetic/setup.bash; . ws/devel/setup.bash; set -u
export ROS_MASTER_URI=http://127.0.0.1:11313 GAZEBO_MASTER_URI=http://localhost:11355
export PYTHONNOUSERSITE=1 PYTHONUNBUFFERED=1
say(){ echo "[$(date '+%F %T')] $*" | tee -a "$H/00_window_log.txt"; }
T0=$(date +%s)
ROSCORE_PID=""; LAUNCH_PID=""

say "== 步0 预检变体当刻（在册预检本体一字未动，走 x86 变体）=="
bash scripts/verify/v_window_preflight_x86_variant.sh > "$H/01_preflight.txt" 2>&1
PRC=$?
say "预检变体 rc=$PRC"
if [ "$PRC" != 0 ]; then say "★预检未过 ⇒ 按纪律不起窗、当场拒窗"; exit 7; fi

say "== 步1 起本窗 roscore（隔离域 11313／GAZEBO 11355）=="
nohup roscore -p 11313 > "$H/02_roscore.log" 2>&1 &
ROSCORE_PID=$!
say "  roscore PID=$ROSCORE_PID"
for i in $(seq 1 30); do rosnode list >/dev/null 2>&1 && break; sleep 1; done
rosnode list > /dev/null 2>&1 && say "  master 在位（等待 ${i}s）" || { say "★master 30s 未起 ⇒ 前置缺失、不起launch"; kill -INT $ROSCORE_PID 2>/dev/null; exit 8; }

say "== 步2 起 m2_gazebo.launch（gui:=false；★在册 launch 一字未改）=="
nohup roslaunch lite3_gazebo_sim m2_gazebo.launch gui:=false > "$H/03_roslaunch.log" 2>&1 &
LAUNCH_PID=$!
say "  roslaunch PID=$LAUNCH_PID"
for i in $(seq 1 75); do rosnode list 2>/dev/null | grep -q /robot_state_publisher && break; sleep 1; done
say "  等待 robot_state_publisher 用时 ${i}s"
rosnode list > "$W/09_rosnode.txt" 2>&1
rostopic list > "$W/10_rostopic.txt" 2>&1
say "  节点数=$(grep -c '^/' "$W/09_rosnode.txt") ‖ /tf 在位=$(grep -c '^/tf$' "$W/10_rostopic.txt") ‖ /gazebo/model_states 在位=$(grep -c '/gazebo/model_states' "$W/10_rostopic.txt")"

say "== 步3 活体实测（只读探针、12s 采样窗）=="
python3 "$H/_probe_gen1.py" > "$W/04_gen1_live.txt" 2>&1
TFRC=$?
say "  探针 rc=$TFRC（★该 rc 是探针自己的读数成功与否、★不是任何判据）"
cat "$W/04_gen1_live.txt" | tee -a "$H/00_window_log.txt"

say "== 步4 活体侧 robot_description 展开值旁证（★rosparam 不是 rostopic）=="
{
  timeout 20 rosparam get /robot_description 2>/dev/null | grep -A3 'base_to_rslidar' | head -8 || echo "    （取不到）"
} > "$H/05_urdf_live.txt" 2>&1
cat "$H/05_urdf_live.txt" | tee -a "$H/00_window_log.txt"

say "== 步5 退窗（★只 kill 本窗自己起的两个 PID：SIGINT 优雅退）=="
[ -n "$LAUNCH_PID" ] && kill -INT $LAUNCH_PID 2>/dev/null
for i in $(seq 1 30); do pgrep -x gzserver >/dev/null 2>&1 || break; sleep 1; done
[ -n "$ROSCORE_PID" ] && kill -INT $ROSCORE_PID 2>/dev/null
sleep 3
{
  echo "# 退净形状（容器 $(date '+%F %T')）"
  echo "  pgrep gzserver=$(pgrep -x gzserver|wc -l) gzclient=$(pgrep -x gzclient|wc -l) roslaunch=$(pgrep -x roslaunch|wc -l) roscore=$(pgrep -x roscore|wc -l) rosmaster=$(pgrep -x rosmaster|wc -l)"
  echo "  TCP 1131x=$(ss -lnt|grep -cE ':1131[1-9]') UDP 1131x=$(ss -lnu|grep -cE ':1131[1-9]') TCP 11345/11355=$(ss -lnt|grep -cE ':1134[5]|:1135[5]')"
} > "$H/06_exitclean.txt" 2>&1
cat "$H/06_exitclean.txt" | tee -a "$H/00_window_log.txt"

say "== 步6 清盘（坑67②＝清理与计数同条命令、路径写全集）=="
find /opt/ros1-slam/note /opt/ros1-slam/ws/src /opt/ros1-slam/scripts /opt/ros1-slam/tooling -name __pycache__ -type d -prune -exec rm -rf {} + 2>/dev/null
echo "  pyc=$(find /opt/ros1-slam/note /opt/ros1-slam/ws/src /opt/ros1-slam/scripts /opt/ros1-slam/tooling -name '*.pyc' | wc -l) ws_src=$(find /opt/ros1-slam/ws/src -type f|wc -l) tooling=$(find /opt/ros1-slam/tooling -type f|wc -l) note=$(find /opt/ros1-slam/note -type f|wc -l)" | tee -a "$H/00_window_log.txt"

say "== 窗结束：预检 rc=$PRC｜探针 rc=$TFRC｜时长 $(( $(date +%s) - T0 ))s（★上限 10 分钟）｜★档名＝联调·旁证、★不入分子 =="
exit 0
