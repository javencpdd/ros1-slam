#!/usr/bin/env bash
# v_scan_p1.sh —— SCAN 集成 A7 P1 感知入判据① harness（主理人直接执行，2026-10-08）
# 复用 m2_smoke.sh 已验证骨架：wait_topic（D-M2-P4 启动窗门控）+ trap EXIT 退净。
# 链：m2_gazebo(物理世界) → m2_mapping(faster_lio, pcd_save=false) → 70_scan.launch(SCAN 独立进程)
# 判据①：/grid_map/occupancy 在 30s 窗内出帧且收敛，占据包围盒落在 LIO 世界系累积云界内。
# 隔离域：ROS_MASTER_URI 11313 + GAZEBO_MASTER_URI 11345；生产 11312 零监听；foxy 零接触。
set -u
export ROS_MASTER_URI=http://127.0.0.1:11313
export GAZEBO_MASTER_URI=http://127.0.0.1:11345
# 注：catkin 的 profile.d/1.ros_distro.sh 在 ROS_DISTRO 未设时引用 $ROS_DISTRO，与 set -u 冲突
#     （实测首跑 driver 当场死于 "ROS_DISTRO: unbound variable"）⇒ source 段临时关 -u，完成后恢复。
set +u
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION
source /opt/ros/noetic/setup.bash
[ -f /home/ros1-slam/ws/devel/setup.bash ] && source /home/ros1-slam/ws/devel/setup.bash
set -u
[ "${ROS_DISTRO:-}" = "noetic" ] || { echo "FATAL: ROS_DISTRO=$ROS_DISTRO 非 noetic（foxy 混装），结果作废"; exit 2; }
# Mid360 Gazebo 插件搜索路径（对齐已验收的 m2_smoke.sh 口径；缺此行则插件不加载、/rslidar_points 永不出）
export GAZEBO_PLUGIN_PATH=/opt/ros/noetic/lib:/home/ros1-slam/ws/devel/lib

RUN=/tmp/scan_p1_run
LOG=$RUN/harness.log
mkdir -p "$RUN"
WIN=${WIN:-30}
SAMPLES=()
declare -a PIDS=()
P1_PASS=0
P1_FAILMSG=""

say() { echo "[$(date '+%T')] $*" | tee -a "$LOG"; }
note() { echo "$*" | tee -a "$LOG"; }

wait_topic() { # $1=topic $2=timeout_s —— D-M2-P4：rosnode list 门控 + 短 echo 重试，避开启动窗长 echo 挂起
  local t=$1 deadline=$((SECONDS + ${2:-300}))
  while [ $SECONDS -lt $deadline ]; do
    if rostopic list 2>/dev/null | grep -qx "$t"; then
      if timeout 8 rostopic echo -n1 "$t" >/dev/null 2>&1; then return 0; fi
    fi
    sleep 2
  done
  return 1
}

cleanup() {
  say "-- teardown --"
  rosnode kill -a >/dev/null 2>&1; sleep 2
  for p in "${PIDS[@]:-}"; do kill "$p" 2>/dev/null; done
  pkill -f 'roslaunch.*m2_gazebo' 2>/dev/null; pkill -f 'roslaunch.*m2_mapping' 2>/dev/null
  pkill -f 'roslaunch.*70_scan' 2>/dev/null
  pkill -f gzserver 2>/dev/null; pkill -f sim_adapter 2>/dev/null
  pkill -f run_mapping_online 2>/dev/null; pkill -f scan_planner_node 2>/dev/null
  pkill -f closed_loop_controller 2>/dev/null; pkill -f 'topic_tools.*relay' 2>/dev/null
  sleep 3
  kill "${ROSCORE_PID:-0}" 2>/dev/null; sleep 2
  note "残留进程: $(pgrep -f 'roscore|rosmaster|gzserver|roslaunch|run_mapping_online|scan_planner_node|closed_loop_controller' | tr '\n' ' ')(空=净)"
  note "11313 监听: $(ss -lntp 2>/dev/null | grep -c ':11313')  11312 监听: $(ss -lntp 2>/dev/null | grep -c ':11312')"
}
trap cleanup EXIT

# ---------- 0) P0.5 双查 + 基线哈希 ----------
say "== [0] P0.5 双查 =="
note "foxy 生产六 PID（须全部在位、启动时刻不变）:"
ps -o pid,user,lstart,etime,comm --no-headers -p 115711,115792,115795,115797,115799,180237 2>/dev/null | tee -a "$LOG"
note "沙箱 ROS 残留(起前): $(pgrep -f 'roscore|roslaunch|gzserver|rosmaster' | grep -v $$ | tr '\n' ' ')(空=净)"
note "11312 生产 master 监听(起前): $(ss -lntp 2>/dev/null | grep -c ':11312') 须=0"
note "UDP 43897 归属(起前): $(ss -ulnp 2>/dev/null | awk '/43897/' | tr -s ' ')"
sha256sum /home/ros1-slam/data/maps/lite3.pcd | tee "$RUN/pcd_before.sha"
note "ros2qnx 存在性: $(pgrep -fa ros2qnx | wc -l) 须=0"

# ---------- 1) roscore ----------
say "== [1] roscore 11313 =="
roscore -p 11313 > "$RUN/roscore.log" 2>&1 &
ROSCORE_PID=$!
for _ in $(seq 1 30); do rostopic list >/dev/null 2>&1 && break; sleep 1; done
note "master 就绪: $(rosnode list 2>/dev/null | wc -l) 节点（应为 1=/rosout）"

# ---------- 2) 物理世界 ----------
say "== [2] m2_gazebo.launch =="
roslaunch lite3_gazebo_sim m2_gazebo.launch gui:=false > "$RUN/gazebo.log" 2>&1 &
PIDS+=($!)
wait_topic /clock 300 || { P1_FAILMSG="/clock 未出"; say "FAIL: $P1_FAILMSG"; tail -20 "$RUN/gazebo.log" | tee -a "$LOG"; exit 1; }
wait_topic /rslidar_points 480 || { P1_FAILMSG="/rslidar_points 未出"; say "FAIL: $P1_FAILMSG"; exit 1; }
wait_topic /livox/imu 300 || { P1_FAILMSG="/livox/imu 未出"; say "FAIL: $P1_FAILMSG"; exit 1; }
note "世界层三话题在位（/clock /rslidar_points /livox/imu）"

# ---------- 3) faster_lio ----------
say "== [3] m2_mapping.launch (pcd_save:=false 默认，样张保护) =="
roslaunch lite3_gazebo_sim m2_mapping.launch > "$RUN/mapping.log" 2>&1 &
PIDS+=($!)
wait_topic /Odometry 360 || { P1_FAILMSG="/Odometry 未出"; say "FAIL: $P1_FAILMSG"; tail -20 "$RUN/mapping.log" | tee -a "$LOG"; exit 1; }
wait_topic /cloud_registered_body 360 || { P1_FAILMSG="/cloud_registered_body 未出（lio_relay 输入缺失）"; say "FAIL: $P1_FAILMSG"; exit 1; }
note "faster_lio 双输出在位：/Odometry + /cloud_registered_body"

# ---------- 4) SCAN 独立进程 ----------
say "== [4] 70_scan.launch（SCAN 独立进程 + lio_relay） =="
roslaunch lite3_scan 70_scan.launch navi_mode:=1 > "$RUN/scan.log" 2>&1 &
PIDS+=($!)
wait_topic /LIO/clouds_lidar 240 || { P1_FAILMSG="/LIO/clouds_lidar 未出（relay 链断）"; say "FAIL: $P1_FAILMSG"; tail -25 "$RUN/scan.log" | tee -a "$LOG"; exit 1; }
wait_topic /grid_map/occupancy 240 || { P1_FAILMSG="/grid_map/occupancy 未出（grid_map 未建图）"; say "FAIL: $P1_FAILMSG"; tail -25 "$RUN/scan.log" | tee -a "$LOG"; exit 1; }
note "SCAN 输入链与占据图话题均在位"

# ---------- 5) 互斥与红线活体核查 ----------
say "== [5] F9 互斥 + /cmd_vel 写者 =="
note "F9 互斥（节点级判定，不用 pgrep -f —— 实测 'move_base|teb' 正则会误报）: move_base 节点数=$(rosnode list 2>/dev/null | grep -cE 'move_base|/teb')  须=0"
note "/cmd_vel publisher: $(rosnode info /safe_cmd_vel_mux 2>/dev/null | grep -c 'cmd_vel')（本链无 M6/mux，应无订阅侧）"
note "cmd_vel 话题在位: $(rostopic list | grep -cx '/cmd_vel')  /scan_planner/cmd_vel 在位: $(rostopic list | grep -cx '/scan_planner/cmd_vel')"
note "SCAN 节点清单:"; rosnode list 2>/dev/null | tee -a "$LOG"

# ---------- 6) 判据① 采样 ----------
say "== [6] 判据① 采样 ${WIN}s =="
python3 /home/ros1-slam/ws/src/40_navigation/lite3_scan/verify/p1_occ_sample.py \
  /grid_map/occupancy "$WIN" /cloud_registered > "$RUN/occ.json" 2>"$RUN/occ.err"
note "occ.json:"; cat "$RUN/occ.json" | tee -a "$LOG"
note "sampler stderr:"; tail -5 "$RUN/occ.err" | tee -a "$LOG"

# ---------- 7) 判定 ----------
say "== [7] 判定 =="
J=$(python3 - "$RUN/occ.json" <<'PY'
import json,sys
try:
    d=json.load(open(sys.argv[1]))
except Exception as e:
    print("FAIL:①occ.json 不可读/为空 (%s)" % type(e).__name__); sys.exit(0)
f,l=d.get("first"),d.get("last")
checks={}
checks["①帧数≥3"]= d.get("frames",0)>=3
if f and l:
    checks["①末帧点数>0"]= l["n"]>0
    checks["①末帧≥首帧(建图不减)"]= l["n"]>=f["n"]
    checks["①frame_id=world"]= l.get("frame_id")=="world"
    sp=lambda s:(s["x"][1]-s["x"][0])*(s["y"][1]-s["y"][0]) if "x" in s and "y" in s else 0
    checks["①包围盒非退化(>1m²)"]= sp(l)>1.0
    checks["①包围盒落在 LIO 累积云界内"]= bool(d.get("bbox_within_ref"))
else:
    checks["①有采样"]=False
print("; ".join(("PASS" if v else "FAIL")+":"+k for k,v in checks.items()))
PY
)
RJ=$?
note "判定原文: $J"
NPASS=$(printf '%s' "$J" | grep -o 'PASS:' | wc -l)
if [ "$RJ" -ne 0 ] || [ -z "$J" ]; then
  P1_PASS=0; P1_FAILMSG="判定脚本无输出或异常（RJ=$RJ）—— fail-closed 判 FAIL"
elif printf '%s' "$J" | grep -q 'FAIL'; then
  P1_PASS=0; P1_FAILMSG="$J"
elif [ "$NPASS" -lt 6 ]; then
  P1_PASS=0; P1_FAILMSG="PASS 项仅 $NPASS/6，疑采样缺字段 —— fail-closed 判 FAIL"
else
  P1_PASS=1
fi

# ---------- 8) 样张零污染复核 ----------
say "== [8] 样张与红线终检 =="
sha256sum /home/ros1-slam/data/maps/lite3.pcd > "$RUN/pcd_after.sha"
if diff -q "$RUN/pcd_before.sha" <(cut -d' ' -f1-2 "$RUN/pcd_after.sha") >/dev/null 2>&1 || diff -q "$RUN/pcd_before.sha" "$RUN/pcd_after.sha" >/dev/null 2>&1; then
  note "样张 sha256 起前=退后 OK"
else
  note "★样张 sha256 变化！前=$(cat $RUN/pcd_before.sha) 后=$(cat $RUN/pcd_after.sha)"; P1_PASS=0; P1_FAILMSG="样张污染"
fi
note "foxy 六 PID 终检:"; ps -o pid,user,lstart,comm --no-headers -p 115711,115792,115795,115797,115799,180237 2>/dev/null | tee -a "$LOG"

echo "==================================================" | tee -a "$LOG"
if [ "$P1_PASS" = "1" ]; then echo "P1 判据① 结论 = PASS"; else echo "P1 判据① 结论 = FAIL —— $P1_FAILMSG"; fi | tee -a "$LOG"
RC=1; [ "$P1_PASS" = "1" ] && RC=0
echo "P1_EXIT=$RC LOGDIR=$RUN" | tee -a "$LOG"
exit "$RC"
