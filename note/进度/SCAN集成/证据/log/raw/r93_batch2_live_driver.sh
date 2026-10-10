#!/usr/bin/env bash
# r93_batch2_live_driver.sh —— 批2 条1/条4 的 SCAN 窗活体取数驱动（小明 2026-10-10，#80）
# 命令形状照在册 v_scan_p1.sh:58-98（起链三档＋wait_topic）与 cleanup（:44-58）抄，不自创口径；
# 本驱动**不作任何判定**：只起链、只读 TF、只停手（STP 的 rc=7 是 fd 稿 :212 的在册停手条）。
# 与 P1 的区别写死：本窗不发 goal、不跑 mbf、不读占据图质量——只为拿一份 SCAN 窗的 base_link 父集合读数。
set -u
set +u   # source 段必须关 -u：在册 driver 首跑当场死于 "ROS_DISTRO: unbound variable"（v_scan_p1.sh:11 的实测教训）
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION
source /opt/ros/noetic/setup.bash
[ -f /home/ros1-slam/ws/devel/setup.bash ] && source /home/ros1-slam/ws/devel/setup.bash
set -u
ROOT=/home/ros1-slam
BASE49="$ROOT/note/进度/SCAN集成/证据/scan_src_baseline_49.sha256"
OUT="$ROOT/note/进度/SCAN集成/证据/log/raw/r93_batch2_live_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$OUT" || exit 9
LOG="$OUT/driver.log"
PIDS=()
now() { date '+%F %T'; }
note() { echo "$@" | tee -a "$LOG"; }
say() { note ""; note "===== $* $(now) ====="; }
wait_topic() { local t=$1 x=$2 i=0; while [ $i -lt $x ]; do rostopic list 2>/dev/null | grep -qx "$t" && return 0; sleep 1; i=$((i+1)); done; return 1; }

cleanup() {
  say "teardown"
  ps -o pid,user,lstart,etime,comm --no-headers -p 115711,115792,115795,115797,115799,180237 > "$OUT/foxy_end.txt" 2>/dev/null
  note "foxy 六 PID 窗末（teardown 前）命中=$(wc -l < "$OUT/foxy_end.txt")（与起前 $(wc -l < "$OUT/foxy_start.txt") 成对比照；retake_driver.sh:205-207 形态②＝两侧皆空 ⇒ 只作空洞登记，不指控扰动）"
  rosnode kill -a >/dev/null 2>&1; sleep 2
  for p in "${PIDS[@]:-}"; do kill "$p" 2>/dev/null; done
  pkill -f 'roslaunch.*m2_gazebo' 2>/dev/null; pkill -f 'roslaunch.*m2_mapping' 2>/dev/null
  pkill -f 'roslaunch.*70_scan' 2>/dev/null; pkill -f 'roslaunch.*lio_relay' 2>/dev/null
  pkill -f gzserver 2>/dev/null; pkill -f sim_adapter 2>/dev/null
  pkill -f run_mapping_online 2>/dev/null; pkill -f scan_planner_node 2>/dev/null
  pkill -f closed_loop_controller 2>/dev/null; pkill -f 'topic_tools.*relay' 2>/dev/null
  pkill -f static_transform_publisher 2>/dev/null
  sleep 3; kill "${ROSCORE_PID:-0}" 2>/dev/null; sleep 2
  note "残留进程: $(pgrep -f 'roscore|rosmaster|gzserver|roslaunch|run_mapping_online|scan_planner_node|closed_loop_controller|static_transform_publisher' | tr '\n' ' ')(空=净)"
  note "端口监听（须全 0）: 11311=$(ss -lnt 2>/dev/null | grep -c ':11311') 11312=$(ss -lnt 2>/dev/null | grep -c ':11312') 11313=$(ss -lnt 2>/dev/null | grep -c ':11313') 11345=$(ss -lnt 2>/dev/null | grep -c ':11345')"
  sha256sum "$ROOT/ws/src/95_simulation/lite3_gazebo_sim/worlds/m1_world.world" "$ROOT/data/maps/lite3.pcd" \
    > "$OUT/sha_protected_after.txt" 2>&1
  note "受保护件跑后 sha（完整路径核，坑52）: $(cut -d' ' -f1 "$OUT/sha_protected_after.txt" | cut -c1-16 | tr '\n' ' ')"
  ( cd "$ROOT/ws/src/30_3rd" && sha256sum -c "$BASE49" > "$OUT/baseline_after.txt" 2>&1 )
  RC_B=$?
  CNT_B=$(find "$ROOT/ws/src/30_3rd" -type f | wc -l)
  note "30_3rd 基线跑后：sha256sum -c rc=${RC_B}（0＝内容全等）find 计数=${CNT_B}（须=49）⇒ 上游件仍只读未动"
}
trap cleanup EXIT

say "0) 前置（工序闸，不判被测件）"
L=$(cut -d' ' -f1 /proc/loadavg)
note "loadavg=${L}（在册工序闸 4，出处 retake_driver.sh:44 LOAD_MAX=\${LOAD_MAX:-4}）"
awk -v l="$L" 'BEGIN{exit !(l>4.0)}' && { note "[停手] 负载超闸 ⇒ 不用坏窗，rc=9 INVALID"; echo "DRIVER VERDICT rc=9 INVALID(负载)" | tee -a "$LOG"; exit 9; }
note "沙箱 ROS 残留(起前): $(pgrep -f 'roscore|roslaunch|gzserver|rosmaster' | grep -v $$ | tr '\n' ' ')(空=净)"
note "端口(起前): 11311=$(ss -lnt 2>/dev/null | grep -c ':11311') 11312=$(ss -lnt 2>/dev/null | grep -c ':11312') 11313=$(ss -lnt 2>/dev/null | grep -c ':11313') 11345=$(ss -lnt 2>/dev/null | grep -c ':11345')"
note "ros2qnx 存在性(须=0): $(pgrep -fa ros2qnx | wc -l)"
ps -o pid,user,lstart,etime,comm --no-headers -p 115711,115792,115795,115797,115799,180237 > "$OUT/foxy_start.txt" 2>/dev/null
NF0=$(wc -l < "$OUT/foxy_start.txt")
note "foxy 六 PID 起前命中=${NF0}（0＝本 boot 名单全不在位 ⇒ 照 retake_driver.sh:205-207 形态②：「零触碰」空洞成立，**不得引用为红线达成**（O-7），本窗不作扰动指控）"
sha256sum "$ROOT/ws/src/95_simulation/lite3_gazebo_sim/worlds/m1_world.world" "$ROOT/data/maps/lite3.pcd" > "$OUT/sha_protected_before.txt" 2>&1
( cd "$ROOT/ws/src/30_3rd" && sha256sum -c "$BASE49" > "$OUT/baseline_before.txt" 2>&1 )
RC_B0=$?
note "30_3rd 基线起前：sha256sum -c rc=${RC_B0}（0＝内容全等）find 计数=$(find "$ROOT/ws/src/30_3rd" -type f | wc -l)（须=49）"

export ROS_MASTER_URI=http://localhost:11313
export ROS_LOG_DIR="$OUT/roslog"; mkdir -p "$ROS_LOG_DIR"
say "1) roscore 11313"
roscore -p 11313 > "$OUT/roscore.log" 2>&1 & ROSCORE_PID=$!
for i in $(seq 1 30); do rostopic list >/dev/null 2>&1 && break; sleep 1; done
note "master 就绪：节点数=$(rosnode list 2>/dev/null | wc -l)（应为 1=/rosout）"

say "2) m2_gazebo.launch（物理世界；gui:=false）"
roslaunch lite3_gazebo_sim m2_gazebo.launch gui:=false > "$OUT/gazebo.log" 2>&1 & PIDS+=($!)
wait_topic /clock 300 || { note "FAIL 前置：/clock 未出（跑不动⇒不判被测面）"; echo "DRIVER VERDICT rc=9 INVALID(/clock)" | tee -a "$LOG"; exit 9; }
wait_topic /rslidar_points 480 || { note "FAIL 前置：/rslidar_points 未出"; echo "DRIVER VERDICT rc=9 INVALID(/rslidar_points)" | tee -a "$LOG"; exit 9; }
wait_topic /livox/imu 300 || { note "FAIL 前置：/livox/imu 未出"; echo "DRIVER VERDICT rc=9 INVALID(/livox/imu)" | tee -a "$LOG"; exit 9; }
note "世界层三话题在位"

say "3) m2_mapping.launch（faster_lio；pcd_save 缺省 false ⇒ 样张保护）"
roslaunch lite3_gazebo_sim m2_mapping.launch > "$OUT/mapping.log" 2>&1 & PIDS+=($!)
wait_topic /Odometry 360 || { note "FAIL 前置：/Odometry 未出"; echo "DRIVER VERDICT rc=9 INVALID(/Odometry)" | tee -a "$LOG"; exit 9; }
wait_topic /cloud_registered_body 360 || { note "FAIL 前置：/cloud_registered_body 未出"; echo "DRIVER VERDICT rc=9 INVALID(cloud)" | tee -a "$LOG"; exit 9; }
note "faster_lio 双输出在位"

say "4) 70_scan.launch navi_mode:=1（在册接线：include 指壳包 lio_relay_sim.launch）"
roslaunch lite3_scan 70_scan.launch navi_mode:=1 > "$OUT/scan.log" 2>&1 & PIDS+=($!)
wait_topic /LIO/clouds_lidar 240 || { note "FAIL 前置：/LIO/clouds_lidar 未出（relay 链断）"; echo "DRIVER VERDICT rc=9 INVALID(clouds_lidar)" | tee -a "$LOG"; exit 9; }
wait_topic /grid_map/occupancy 240 || { note "FAIL 前置：/grid_map/occupancy 未出"; echo "DRIVER VERDICT rc=9 INVALID(occupancy)" | tee -a "$LOG"; exit 9; }
note "SCAN 输入链与占据图话题在位 ⇒ 本窗确是 SCAN 窗（这是坑55 要的那一列）"

say "5) 实照相：STP 取 SCAN 窗的 base_link 父集合（期望 恰 ['odom'] 且 /tf_static 零条以 base_link 为子）"
bash "$ROOT/ws/src/40_navigation/lite3_scan/verify/v_scan_tf_parent_probe.sh" 实照相 > "$OUT/stp_实照相.txt" 2>&1
RC_SHI=$?
note "STP 实照相 rc=${RC_SHI}"; cat "$OUT/stp_实照相.txt" | tee -a "$LOG"

say "6) 真链级阳性对照（只读起上游基线件，不动它：期望双父 ⇒ 证探针在真链里也抓得到）"
# 包名教训（本轮实测翻出）：目录是 ws/src/30_3rd/plan_manage，但该件 package.xml 的 <name> 是 **scan_planner**
#   ⇒ `roslaunch plan_manage lio_relay.launch` 当场 RLException（首版就是这么死的）。正解按包名调用。
roslaunch scan_planner lio_relay.launch > "$OUT/upstream_relay.log" 2>&1 & PIDS+=($!)
NUP=0
for i in $(seq 1 20); do
  if rosnode list 2>/dev/null | grep -qx "/body_to_base_link"; then NUP=1; break; fi
  sleep 1
done
if [ "$NUP" != "1" ]; then
  note "[对照相未起跑] 等 20s 仍无 /body_to_base_link 节点 ⇒ 这一步失败属**前置缺失**，不是被测面结论"
  note "roslaunch 输出: $(head -3 "$OUT/upstream_relay.log" | tr '\n' ' ')"
  echo "DRIVER VERDICT rc=9 INVALID(对照相未起跑；实照相 rc=${RC_SHI} 的读数仍在档，但**不得引用为批2 到手**——没有对照的零命中不算数，坑29)" | tee -a "$LOG"
  exit 9
fi
note "对照相已起跑：/body_to_base_link 在位（第 ${i} 秒）"
bash "$ROOT/ws/src/40_navigation/lite3_scan/verify/v_scan_tf_parent_probe.sh" 真链阳性对照 > "$OUT/stp_真链阳性对照.txt" 2>&1
RC_DUZ=$?
note "STP 真链阳性对照 rc=${RC_DUZ}（**期望=7**；若这里也是单父 ⇒ 对照失败，实照相那条读数不能算被验过）"
cat "$OUT/stp_真链阳性对照.txt" | tee -a "$LOG"
pkill -f 'roslaunch.*lio_relay' 2>/dev/null; pkill -f 'topic_tools.*relay' 2>/dev/null
pkill -f 'static_transform_publisher.*base_link' 2>/dev/null; sleep 4
note "对照相关节点残留=$(rosnode list 2>/dev/null | grep -cE 'body_to_base_link|lio_odom')（摘干净才谈实照相未被污染）"

say "7) 判定归位（本驱动不设判据：只把两格读数与在册期望并列陈述）"
note "实照相 rc=${RC_SHI}｜真链阳性对照 rc=${RC_DUZ}（期望 7）"
note "批2 条1/条4 到手判据：实照相 rc=0 **且** 真链阳性对照 rc=7 ⇒ 两条都取到且探针被验过"
if [ "$RC_SHI" = "0" ] && [ "$RC_DUZ" = "7" ]; then
  echo "DRIVER VERDICT rc=0 取数成立（实照相单父＋真链对照双父）" | tee -a "$LOG"; exit 0
elif [ "$RC_SHI" = "7" ]; then
  echo "DRIVER VERDICT rc=7 停手：实照相出现双父 ⇒ FD-G 在 F2 之后仍发作（TF 结构回归，照 fd :212 上报，不静默继续）" | tee -a "$LOG"; exit 7
else
  echo "DRIVER VERDICT rc=9 INVALID：取数未成立（实照相 rc=${RC_SHI}／对照 rc=${RC_DUZ}）⇒ 属跑不动或对照不成立，**不判被测面**" | tee -a "$LOG"; exit 9
fi
