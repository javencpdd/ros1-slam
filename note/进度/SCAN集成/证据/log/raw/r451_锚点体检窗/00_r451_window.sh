#!/usr/bin/env bash
# 00_r451_window.sh —— ★窗号 R451★（先报后开）＝★锚点体检窗★：按 L1 裁定（r448q，11 条机检硬条件）里★承重的三条★先给这把闸"验牙"
#   C6＝同参数面★连跑两遍★（A／A2）⇒ 量出★噪声地板★（哪些读数天生会漂）
#   C7＝★验牙跑★ C＝在冒烟件跑到中途把 ★/move_base_flex★ 杀掉 ⇒ 后续 CHECK★必须变红★；若★仍全 PASS＝这把闸没有牙 ⇒ 段 B 任何"逐行同值"读数一律不采信★
#   C11＝★挂载活性锁★（每跑都记：节点日志字节／`Using plugin` 行数／`Received a` 行数）
# ★在册件一字不改★（冒烟件／launch／yaml 均未碰；跑前后 sha 自证）；★本窗不★加 elevation 层（那是 R452 段 B）
# 姿势＝预检不过不起窗｜★名集按实际枚数标签（坑76 追加例）★｜★端口 TCP／UDP 分查（同上）★｜退净按 05 追加四十五 落地补记｜清理与计数同条命令
set -u
R=/opt/ros1-slam/note/进度/SCAN集成/证据/log/raw/r451_锚点体检窗
RUN="$R/run"; mkdir -p "$RUN"
set +u; . /opt/ros/noetic/setup.bash; . /opt/ros1-slam/ws/devel/setup.bash; set -u
export ROS_MASTER_URI=http://127.0.0.1:11313 PYTHONNOUSERSITE=1 PYTHONUNBUFFERED=1
export ROS_STDOUT_STDERR=1 ROSCONSOLE_STDOUT_LINE_BUFFERED=1
NAMEPAT='^(roslaunch|rosmaster|rosout|gzserver|gzclient|rosbag|costmap_2d_node|move_base_flex|pcl_render_node|sim_adapter|rostopic|health_monitor|map_server|mockamap)$'
NSZ=14
LOGD="${ROS_LOG_DIR:-$HOME/.ros/log}"
say(){ echo "[$(date '+%F %T')] $*" | tee -a "$R/00_window_log.txt"; }
snap(){ local f="$1"; : > "$f"
  for pid in $(ps -eo pid=,comm= | awk -v re="$NAMEPAT" '$2 ~ re {print $1}'); do
    L=$(ps -o lstart= -p "$pid" 2>/dev/null) || continue
    C=$(ps -o comm= -p "$pid" 2>/dev/null) || continue
    A=$(tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null)
    printf '%s\t%s\t%s\t%s\n' "$pid" "$L" "$C" "$A" >> "$f"
  done; wc -l < "$f"; }
ports(){ TCP=$( { ss -lnt; } 2>/dev/null | grep -cE ":(11311|11312|11313)\b"); UDP=$( { ss -lnu; } 2>/dev/null | grep -cE ":(11311|11312|11313)\b"); }
T0=$(date +%s); PIDS=""
cleanup(){ for p in $PIDS; do kill "$p" 2>/dev/null; done; }
on_exit(){ local rc=$?
  if [ -n "$PIDS" ]; then say "★EXIT（rc=$rc）＝没走到步9 ⇒ 就地收本窗 PID 集★"; for p in $PIDS; do kill -INT "$p" 2>/dev/null; done; sleep 2; cleanup; PIDS=""; fi
}
trap on_exit EXIT
SM=ws/src/70_bringup/lite3_bringup/verify/sim_nav_smoke.sh

say "== 步0 预检变体当刻（在册预检件一字未动）=="
bash /opt/ros1-slam/scripts/verify/v_window_preflight_x86_variant.sh > "$R/01_preflight.txt" 2>&1
PRC=$?; say "  预检 rc=$PRC"
[ "$PRC" != 0 ] && { say "★预检未过 ⇒ 不起窗"; exit 7; }
say "  ★本窗名集＝$NSZ 名（★标签写实际枚数、★不再叫"五名"）：$NAMEPAT"
SM_SHA0=$(sha256sum "$SM" | cut -c1-16); say "  ★在册冒烟件跑前 sha 前 16=$SM_SHA0 行数=$(wc -l < "$SM")★"

say "== 步0b 起前快照 S0 =="
BN=$(snap "$RUN/S0_before.tsv"); say "  ★基线（起前 $NSZ 名计数）＝$BN★（★批104 后应为 0★）"
sed 's/^/    S0｜/' "$RUN/S0_before.tsv" | tee -a "$R/00_window_log.txt"
roscore -p 11313 > "$RUN/roscore.log" 2>&1 & PIDS="$PIDS $!"; WPID=$!
ok=0; for i in $(seq 1 30); do rosnode list 2>/dev/null | grep -q rosout && { ok=1; break; }; sleep 1; done
say "== 步1 master_ready=$ok（等待 ${i}s）‖ WPID=$WPID =="
[ "$ok" != 1 ] && { say "★master 未起"; exit 8; }
rosrun tf static_transform_publisher 0 0 0 0 0 0 map base_link 10 > "$RUN/tf_base.log" 2>&1 & PIDS="$PIDS $!"
UUID=$(basename "$(readlink -f "$LOGD/latest" 2>/dev/null)" 2>/dev/null)
RL="$LOGD/$UUID/roslaunch-$(hostname)-$WPID.log"; [ -f "$RL" ] || RL=$(ls -1t "$LOGD/$UUID"/roslaunch-*.log 2>/dev/null | head -1)
N=$(grep -oE 'process\[rosout-1\]: started with pid \[[0-9]+\]' "$RL" 2>/dev/null | grep -oE '[0-9]+' | tail -1)
say "== 步1b uuid 自校验＝$([ -n "$UUID" ] && [ -f "$RL" ] && echo OK || echo FAIL) ‖ 本窗 rosout pid N=$N =="
{ echo "UUID=$UUID"; echo "WPID=$WPID"; echo "ROSOUT_PID=$N"; echo "ROSLAUNCH_LOG=$RL"; } > "$RUN/OWNERSHIP.txt"

# run_smoke <标签> <★是否验牙：yes＝中途杀 /move_base_flex★>
run_smoke(){
  local tag="$1" killmid="$2"; local out="$RUN/smoke_${tag}.txt"; local t0 t; t0=$(date +%s)
  say "== 跑 $tag（★在册冒烟件一字不改★；★超时上限 300s★）=="
  if [ "$killmid" = "yes" ]; then
    timeout 300 bash "$SM" > "$out" 2>&1 & local sp=$!
    sleep 28
    say "  ★验牙动作（第 28s）：rosnode kill /move_base_flex★"
    timeout 8 rosnode kill /move_base_flex > "$RUN/kill_mid.txt" 2>&1; say "  kill 返回 rc=$?（$(head -1 "$RUN/kill_mid.txt" | cut -c1-70)）"
    wait $sp; local rc=$?
  else
    timeout 300 bash "$SM" > "$out" 2>&1; local rc=$?
  fi
  t=$(( $(date +%s) )); say "  $tag：冒烟件 rc=$rc ‖ CHECK 行数=$(grep -c '^CHECK|' "$out") ‖ PASS=$(grep -c '^CHECK|[^|]*|PASS' "$out") ‖ FAIL=$(grep -c '^CHECK|[^|]*|FAIL' "$out")"
  cp -f /tmp/sim_nav_smoke_launch.log "$RUN/roslaunch_${tag}.log" 2>/dev/null
  say "  ★C11 挂载活性锁（$tag）＝launch 日志字节=$(stat -c%s "$RUN/roslaunch_${tag}.log" 2>/dev/null) ‖ Using plugin 行=$(grep -c 'Using plugin' "$RUN/roslaunch_${tag}.log" 2>/dev/null) ‖ Received a 行=$(grep -c 'Received a' "$RUN/roslaunch_${tag}.log" 2>/dev/null) ‖ 占据格读数=$(grep -oE '占据格=[0-9]+' "$RUN/roslaunch_${tag}.log" 2>/dev/null | tail -1)★"
  say "  $tag 用时≈$(( t - t0 ))s（★单跑不得超 300s、★整窗不放宽 10 分钟上限★）"
}
run_smoke A no
run_smoke A2 no
run_smoke C yes

say "== 步9 退净（★SIGTERM wrapper → 轮询 rosout 消失 → 差集双过滤 → 五数★）=="
for p in $PIDS; do kill "$p" 2>/dev/null; done; PIDS=""
for k in $(seq 1 12); do [ -z "$N" ] && break; kill -0 "$N" 2>/dev/null || { say "  ★第 ${k}s：本窗 rosout pid=$N 已消失★"; break; }; sleep 1; done
AN=$(snap "$RUN/S1_after.tsv")
awk -F'\t' 'NR==FNR{s[$1"|"$2]=1;next} !($1"|"$2 in s)' "$RUN/S0_before.tsv" "$RUN/S1_after.tsv" > "$RUN/D_delta.tsv"
DN=$(wc -l < "$RUN/D_delta.tsv"); : > "$RUN/K_kill.tsv"
while IFS=$'\t' read -r pid lstart comm args; do
  [ -n "${pid:-}" ] || continue
  if [ -n "${UUID:-}" ] && printf '%s' "$args" | grep -q -- "__log:=$LOGD/$UUID/"; then kill -INT "$pid" 2>/dev/null; echo -e "$pid\t$lstart\t$comm\t$args" >> "$RUN/K_kill.tsv"; say "  K：pid=$pid comm=$comm（★带本窗 uuid★）"; else say "  ★跳过（非本窗 uuid）pid=$pid comm=$comm★"; fi
done < "$RUN/D_delta.tsv"
KN=$(wc -l < "$RUN/K_kill.tsv"); sleep 3
DPK=$(awk -F'\t' 'NR==FNR{k[$1"|"$2]=1;next} !($1"|"$2 in k)' "$RUN/K_kill.tsv" "$RUN/D_delta.tsv" | wc -l)
ports; CN=$(snap "$RUN/witness_after.tsv")
say "  ★判定行五数＝基线=$BN｜D=$DN｜K=$KN｜D∖K=$DPK｜端口 TCP=$TCP UDP=$UDP★（现读 $NSZ 名计数=$CN）"
[ "$CN" -le "$BN" ] && [ "$TCP" = "0" ] && [ "$UDP" = "0" ] && [ "$DN" -eq "$KN" ] && say "  ★退净达标（乙式四合取项全成立）★" || say "  ★退净未达标／D∖K≠∅＝INVALID★"
sed 's/^/    S1｜/' "$RUN/S1_after.tsv" | tee -a "$R/00_window_log.txt"
say "== 步10 三跑对照（★C6 噪声地板 ＋ C7 验牙★）=="
python3 -u "$R/_cmp_runs.py" "$RUN/smoke_A.txt" "$RUN/smoke_A2.txt" "$RUN/smoke_C.txt" > "$R/05_three_runs.txt" 2>&1
say "  对照件 rc=$?"; sed 's/^/    ｜/' "$R/05_three_runs.txt" | tee -a "$R/00_window_log.txt"
say "== 步11 在册件未改自证 ＋ 日志落盘 =="
SM_SHA1=$(sha256sum "$SM" | cut -c1-16)
say "  冒烟件跑后 sha 前 16=$SM_SHA1 ‖ 跑前=$SM_SHA0 ⇒ ★同值＝$([ "$SM_SHA0" = "$SM_SHA1" ] && echo True || echo False)★、行数=$(wc -l < "$SM")"
say "  roscore.log 字节=$(stat -c%s "$RUN/roscore.log") ‖ killing 行=$(grep -c 'killing on exit' "$RUN/roscore.log")"
say "== 步99 清盘与计数同条命令 =="
find /opt/ros1-slam -name '__pycache__' -type d -prune -exec rm -rf {} + 2>/dev/null
say "  pyc=$(find /opt/ros1-slam -name '*.pyc' | wc -l) ws_src=$(find /opt/ros1-slam/ws/src -type f | wc -l) tooling=$(find /opt/ros1-slam/tooling -type f | wc -l) note=$(find /opt/ros1-slam/note -type f | wc -l) ‖ 时长 $(( $(date +%s) - T0 ))s"
