#!/usr/bin/env bash
# r145_cpu_hz/driver.sh —— C13＝乙 换形状后的**开销与点流**实测（诊断件，非判据采数；小明 2026-10-11）
#
# 为什么要这一窗：q 稿 §2 案乙自己写了风险条——「`isfinite`／盒判定放在 20k 点×10Hz 的循环里，需实测单帧开销
#   （量级估计可忽略，但**这条我不猜**）」。小张复核又加一条：网格给的负载 ×1.48 只作用在体素化**之前**，
#   进 iSAM 的量基本不变 ⇒ CPU 结论**必须活体量**，不能拿离线网格数顶。
#
# 边界（写死）：①不起 SCAN／不投 goal（本窗只测预处理与建图侧的点流与 CPU）
#   ②不改任何配置：吃的就是现役 `faster_lio_outdoor.yaml`（含本批新落的四个 self_mask_* 键）
#   ③loadavg≤4 才起窗，超即 rc=7 停手留证（在册 O-5），**无 FORCE 口子**
#   ④世界件与样张两条红线照旧逐窗自证（sha 起前＝退后）
set -u
RUN=${HZ_RUN:-/home/ros1-slam/note/进度/SCAN集成/证据/log/raw/r145_cpu_hz}
mkdir -p "$RUN"; LOG=$RUN/hz.log
say() { echo "[$(date '+%T')] $*" | tee -a "$LOG"; }
WORLD=/home/ros1-slam/ws/src/95_simulation/lite3_gazebo_sim/worlds/m1_world.world
PCD=/home/ros1-slam/data/maps/lite3.pcd

L1=$(cut -d' ' -f1 /proc/loadavg)
if python3 -c "import sys;sys.exit(0 if float('$L1')<=4.0 else 1)"; then :; else
  say "起窗门槛不过：loadavg=$L1＞4 ⇒ 停手不跑（不绕行）"
  echo "开销自证结论 = INVALID|未起窗（loadavg=$L1＞4）" | tee -a "$LOG"; echo "HZ VERDICT rc=7" | tee -a "$LOG"; exit 7
fi
say "起窗门槛：loadavg=$L1 ≤ 4"

set +u
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION
source /opt/ros/noetic/setup.bash
[ -f /home/ros1-slam/ws/devel/setup.bash ] && source /home/ros1-slam/ws/devel/setup.bash
set -u
export ROS_MASTER_URI=http://127.0.0.1:11313
export GAZEBO_MASTER_URI=http://127.0.0.1:11345
export GAZEBO_PLUGIN_PATH=/opt/ros/noetic/lib:/home/ros1-slam/ws/devel/lib
sha256sum "$WORLD" > "$RUN/world_before.sha"; sha256sum "$PCD" > "$RUN/pcd_before.sha"

cleanup() {
  say "-- teardown --"
  rosnode kill -a >/dev/null 2>&1; sleep 2
  pkill -f 'roslaunch.*m2_gazebo' 2>/dev/null; pkill -f 'roslaunch.*m2_mapping' 2>/dev/null
  pkill -f gzserver 2>/dev/null; pkill -f rosmaster 2>/dev/null; sleep 2
  say "残留：gzserver=$(pgrep -c gzserver 2>/dev/null) mapping=$(pgrep -fc run_mapping_online 2>/dev/null) 11313 监听=$(ss -ltn 2>/dev/null | grep -c ':11313')"
}
trap cleanup EXIT

say "== [0] roscore =="
roscore > "$RUN/roscore.log" 2>&1 & sleep 5
say "== [1] m2_gazebo =="
roslaunch lite3_gazebo_sim m2_gazebo.launch gui:=false > "$RUN/gazebo.log" 2>&1 &
for i in $(seq 1 120); do rostopic list 2>/dev/null | grep -q /clock && break; sleep 2; done
say "== [2] m2_mapping（faster_lio 位姿源＝被试面）=="
roslaunch lite3_gazebo_sim m2_mapping.launch > "$RUN/mapping.log" 2>&1 &
for i in $(seq 1 90); do rostopic list 2>/dev/null | grep -q /cloud_registered && break; sleep 2; done
rostopic list 2>/dev/null | grep -E '/cloud_registered|/rslidar_points' > "$RUN/topics.txt"
say "话题在位：$(tr '\n' ' ' < "$RUN/topics.txt")"
box_wait() {  # 最多等 60 秒取盒生效回执；取不到就照实报"本窗未取到"，不猜
  local i
  for i in $(seq 1 30); do
    if grep -qm1 "C13-乙" "$RUN/mapping.log" 2>/dev/null; then
      grep -m1 -o "C13-乙.*" "$RUN/mapping.log" > "$RUN/box_receipt.txt"; return 0
    fi
    sleep 2
  done
  return 1
}
PID=$(pgrep -f run_mapping_online | head -1)
say "被试节点 PID=$PID"
[ -z "$PID" ] && { echo "开销自证结论 = INVALID|被试节点未起" | tee -a "$LOG"; echo "HZ VERDICT rc=9"; exit 9; }

say "== [3] 点流频率（入 /rslidar_points ＋出 /cloud_registered，各 20s）=="
for T in /rslidar_points /cloud_registered; do
  timeout 34 rostopic hz -w 3 "$T" > "$RUN/hz_$(basename $T).txt" 2>&1
  say "  $T ⇒ $(grep -E 'average rate' "$RUN/hz_$(basename $T).txt" | tail -1)"
done
say "== [4] 节点开销（ps 累计均值 ＋ top 瞬时值两路，10s 内取）=="
ps -o pid,pcpu,pmem,rss,etime,args -p "$PID" > "$RUN/ps_node.txt" 2>&1
top -bn1 -p "$PID" 2>/dev/null | tail -2 > "$RUN/top_node.txt"
say "  ps  $(tail -1 "$RUN/ps_node.txt" | cut -c1-120)"
say "  top $(tail -1 "$RUN/top_node.txt" | cut -c1-120)"
nproc > "$RUN/nproc.txt"; say "  核数=$(cat "$RUN/nproc.txt")（pcpu 是多核累加，换算须除它）"
say "== [5] 点数吞吐（帧均值点数＝出/入点数，与离线网格 ×1.48 口径对表）=="
timeout 12 rostopic echo -n3 /cloud_registered 2>/dev/null | grep -m3 -E "^width|^height" > "$RUN/cloud_dims.txt" || true
say "  $(tr '\n' ' ' < "$RUN/cloud_dims.txt" | cut -c1-150)"

say "== [5b] 盒生效回执（在采样之后取，最多再等 60s；不沿用 P4 窗那一行）=="
if box_wait; then say "  $(cut -c1-150 "$RUN/box_receipt.txt")"; else say "  ★本窗未取到盒生效回执（最多等 60s，mapping.log 里 0 命中）⇒ 不许推断成"生效了但没打印"，也不许推断成"没生效"，只记没取到"; : > "$RUN/box_receipt.txt"; fi

say "== [6] 红线与还原自证 =="
WA=$(sha256sum "$WORLD" | cut -d' ' -f1); WBk=$(cut -d' ' -f1 "$RUN/world_before.sha")
say "  世界件本体 sha 起前＝退后：$([ "$WA" = "$WBk" ] && echo 是 || echo "否!!")"
PA=$(sha256sum "$PCD" | cut -d' ' -f1); PB=$(cut -d' ' -f1 "$RUN/pcd_before.sha")
say "  样张 sha 起前＝退后：$([ "$PA" = "$PB" ] && echo 是 || echo "否!!")"
HZIN=$(grep -E 'average rate' "$RUN/hz__rslidar_points.txt" 2>/dev/null | tail -1 | awk '{print $3}')
HZOUT=$(grep -E 'average rate' "$RUN/hz_cloud_registered.txt" 2>/dev/null | tail -1 | awk '{print $3}')
CPU=$(tail -1 "$RUN/ps_node.txt" | awk '{print $2}')
if [ -n "$HZOUT" ] && [ -n "$CPU" ]; then
  # 注：Linux `ps %CPU` 与 `top %CPU` 对单进程都是**以一个核为 100%**（多线程可超 100），不除核数——
  #     我上一版在此处除了 nproc，把 27.2% 写成"单核当量≈4.53%"，那是**我自己算错**，已改回不除（R146 登记）。
  MISS=""
  [ -z "$HZIN" ] && MISS="$MISS 入流hz未取到"
  [ -z "$HZOUT" ] && MISS="$MISS 出流hz未取到"
  if [ -n "$MISS" ]; then
    echo "开销自证结论 = PARTIAL|出流 hz=${HZOUT:-未取到} 入流 hz=${HZIN:-未取到} 节点 pcpu=${CPU}%（ps 与 top 两路，**以单核为 100%，不除核数**）$MISS ⇒ 有一路没取到，不写全过；盒回执=$([ -s "$RUN/box_receipt.txt" ] && echo 在位 || echo 本窗未取到)" | tee -a "$LOG"
    echo "HZ VERDICT rc=9" | tee -a "$LOG"; exit 9
  fi
  echo "开销自证结论 = OK|入流 hz=${HZIN} 出流 hz=${HZOUT} 节点 pcpu=${CPU}%（ps 与 top 两路，以单核为 100%，不除核数）｜盒回执=$([ -s "$RUN/box_receipt.txt" ] && echo 在位 || echo 本窗未取到)" | tee -a "$LOG"
  echo "HZ VERDICT rc=0" | tee -a "$LOG"; exit 0
fi
echo "开销自证结论 = INVALID|读数没取全（in=$HZIN out=$HZOUT cpu=$CPU）" | tee -a "$LOG"
echo "HZ VERDICT rc=9" | tee -a "$LOG"; exit 9
