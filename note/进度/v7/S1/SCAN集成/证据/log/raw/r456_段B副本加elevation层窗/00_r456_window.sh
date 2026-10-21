#!/usr/bin/env bash
# 00_r456_window.sh —— ★窗号 R456★（先报后开）＝★§8.3 段 B＝"改前／改后逐行同值"这条锚点形式的首次真用★
#   两跑＝REG＝★在册形★（在册冒烟件，一字未改） ‖ CP＝★副本形★（本窗留底目录里的四件副本，只多一枚 elevation StaticLayer）
#   比对＝★只比（CHECK 名，判定）二元组★（C6：数字读数天生会漂、不拿来比）
#   C11 挂载活性锁＝每跑记 launch 日志字节／Using plugin 行数／Received a 行数；★副本形应 5→6 且 Received a 1→2★
#   ★两侧互校（05 追加四十六 姿势①）★＝参数面 rosparam get 的读回值 ↔ 产出侧实际名：
#        CP 跑＝期望 use_maximum=True 且 combination_method=1；REG 跑＝期望 elevation_layer 那两把键"Parameter is not set"
#   ★在册四件（冒烟件／mode／mbf／global_costmap yaml）跑前跑后 sha＋行数自证＝一字未改★
# 姿势＝预检不过不起窗｜名集写实际枚数｜端口 TCP／UDP 分查｜退净按乙式｜★local 一行内不引用同行刚声明的变量★｜清理与计数同条命令
set -u
R=/opt/ros1-slam/note/进度/SCAN集成/证据/log/raw/r456_段B副本加elevation层窗
RUN="$R/run"
mkdir -p "$RUN"
set +u; . /opt/ros/noetic/setup.bash; . /opt/ros1-slam/ws/devel/setup.bash; set -u
export ROS_MASTER_URI=http://127.0.0.1:11313 PYTHONNOUSERSITE=1 PYTHONUNBUFFERED=1
export ROS_STDOUT_STDERR=1 ROSCONSOLE_STDOUT_LINE_BUFFERED=1
NAMEPAT='^(roslaunch|rosmaster|rosout|gzserver|gzclient|rosbag|costmap_2d_node|move_base_flex|pcl_render_node|sim_adapter|rostopic|health_monitor|map_server|mockamap)$'
NSZ=14
LOGD="${ROS_LOG_DIR:-$HOME/.ros/log}"
T0=$(date +%s)
PIDS=""
BN=0
UUID=""
say(){ echo "[$(date '+%F %T')] $*" | tee -a "$R/00_window_log.txt"; }
snap(){
  local f="$1"
  : > "$f"
  local pid L C A
  for pid in $(ps -eo pid=,comm= | awk -v re="$NAMEPAT" '$2 ~ re {print $1}'); do
    L=$(ps -o lstart= -p "$pid" 2>/dev/null) || continue
    C=$(ps -o comm= -p "$pid" 2>/dev/null) || continue
    A=$(tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null)
    printf '%s\t%s\t%s\t%s\n' "$pid" "$L" "$C" "$A" >> "$f"
  done
  wc -l < "$f"
}
cls(){ local v="$1"; case "$v" in *"not set"*) echo NONE;; true|True) echo True;; false|False) echo False;; *) echo "$v";; esac; }
ports(){
  TCP=$( { ss -lnt; } 2>/dev/null | grep -cE ":(11311|11312|11313)\b")
  UDP=$( { ss -lnu; } 2>/dev/null | grep -cE ":(11311|11312|11313)\b")
}
cleanup(){ local p; for p in $PIDS; do kill "$p" 2>/dev/null; done; }
on_exit(){
  local rc=$?
  if [ -n "$PIDS" ]; then
    say "★EXIT（rc=$rc）＝没走到步9 ⇒ 就地收本窗 PID 集★"
    local p; for p in $PIDS; do kill -INT "$p" 2>/dev/null; done
    sleep 2; cleanup; PIDS=""
  fi
}
trap on_exit EXIT

SM_REG=/opt/ros1-slam/ws/src/70_bringup/lite3_bringup/verify/sim_nav_smoke.sh
SM_CP="$R/sim_nav_smoke_r456.sh"
Y_REG=/opt/ros1-slam/ws/src/40_navigation/lite3_navigation/config/global_costmap_params.yaml
M_REG=/opt/ros1-slam/ws/src/70_bringup/lite3_bringup/launch/mode_nav_sim.launch
B_REG=/opt/ros1-slam/ws/src/40_navigation/lite3_navigation/launch/mbf_navigation.launch
KEYS=/move_base_flex/global_costmap/elevation_layer

sha4(){
  local tag="$1"
  { echo "sha 前 16｜smoke=$(sha256sum "$SM_REG" | cut -c1-16) mode=$(sha256sum "$M_REG" | cut -c1-16) mbf=$(sha256sum "$B_REG" | cut -c1-16) yaml=$(sha256sum "$Y_REG" | cut -c1-16)"
    echo "行数｜smoke=$(wc -l < "$SM_REG") mode=$(wc -l < "$M_REG") mbf=$(wc -l < "$B_REG") yaml=$(wc -l < "$Y_REG")"; } > "$RUN/sha4_${tag}.txt"
  cat "$RUN/sha4_${tag}.txt"
}

say "== 步0 预检变体当刻（在册预检件一字未动）=="
bash /opt/ros1-slam/scripts/verify/v_window_preflight_x86_variant.sh > "$R/01_preflight.txt" 2>&1
PRC=$?
say "  预检 rc=$PRC"
[ "$PRC" != 0 ] && { say "★预检未过 ⇒ 不起窗"; exit 7; }
say "  ★本窗名集＝$NSZ 名（写实际枚数）"
say "== 步0a 在册四件跑前自证 =="
sha4 before > /dev/null
say "  $(head -1 "$RUN/sha4_before.txt") ‖ $(tail -1 "$RUN/sha4_before.txt")"
say "  ★副本四件 sha＝$(for x in "$SM_CP" "$R/mode_nav_sim_r456.launch" "$R/mbf_navigation_r456.launch" "$R/global_costmap_params_r456.yaml"; do printf '%s ' "$(sha256sum "$x" | cut -c1-8)"; done)★"

say "== 步0b 起前快照 S0 =="
BN=$(snap "$RUN/S0_before.tsv")
say "  ★基线（起前 $NSZ 名计数）＝$BN★"
sed 's/^/    S0｜/' "$RUN/S0_before.tsv" | tee -a "$R/00_window_log.txt"
roscore -p 11313 > "$RUN/roscore.log" 2>&1 &
PIDS="$PIDS $!"
WPID=$!
ok=0; i=0
for i in $(seq 1 30); do rosnode list 2>/dev/null | grep -q rosout && { ok=1; break; }; sleep 1; done
say "== 步1 master_ready=$ok（等待 ${i}s）‖ WPID=$WPID =="
[ "$ok" != 1 ] && { say "★master 未起"; exit 8; }
rosrun tf static_transform_publisher 0 0 0 0 0 0 map base_link 10 > "$RUN/tf_base.log" 2>&1 &
PIDS="$PIDS $!"
UUID=$(basename "$(readlink -f "$LOGD/latest" 2>/dev/null)" 2>/dev/null)
RL="$LOGD/$UUID/roslaunch-$(hostname)-$WPID.log"
[ -f "$RL" ] || RL=$(ls -1t "$LOGD/$UUID"/roslaunch-*.log 2>/dev/null | head -1)
N=$(grep -oE 'process\[rosout-1\]: started with pid \[[0-9]+\]' "$RL" 2>/dev/null | grep -oE '[0-9]+' | tail -1)
say "== 步1b uuid 自校验＝$([ -n "$UUID" ] && [ -f "$RL" ] && echo OK || echo FAIL) ‖ 本窗 rosout pid N=$N =="
{ echo "UUID=$UUID"; echo "WPID=$WPID"; echo "ROSOUT_PID=$N"; echo "ROSLAUNCH_LOG=$RL"; } > "$RUN/OWNERSHIP.txt"

# run_one <标签> <冒烟件路径> <期望 use_maximum> <期望 combination_method>
run_one(){
  local tag="$1" sm="$2" want_um="$3" want_cm="$4"
  local out="$RUN/smoke_${tag}.txt"
  local t0 rc sp el f sampled=0
  t0=$(date +%s)
  say "== 跑 $tag（★$sm★｜超时 300s）=="
  timeout 300 bash "$sm" > "$out" 2>&1 &
  sp=$!
  : > "$RUN/param_${tag}.txt"
  while kill -0 "$sp" 2>/dev/null; do
    el=$(( $(date +%s) - t0 ))
    f=$(grep -c '^CHECK|' "$out" 2>/dev/null); [ -n "$f" ] || f=0
    printf '%s\t%s\t%s\t%s\n' "$tag" "$el" "$f" "$(grep '^CHECK|' "$out" 2>/dev/null | tail -1 | cut -d'|' -f2)" >> "$RUN/timeline.tsv"
    if [ "$sampled" = "0" ] && [ "$f" -ge 5 ]; then
      sampled=1
      { echo "[$tag @${el}s 已印 $f 条]"
        echo "use_maximum=$(timeout 5 rosparam get "$KEYS/use_maximum" 2>&1 | head -1)"
        echo "combination_method=$(timeout 5 rosparam get "$KEYS/combination_method" 2>&1 | head -1)"
        echo "map_topic=$(timeout 5 rosparam get "$KEYS/map_topic" 2>&1 | head -1)"
        echo "plugins_global=$(timeout 5 rosparam get /move_base_flex/global_costmap/plugins 2>&1 | tr '\n' ' ' | cut -c1-300)"
        echo "名册含 mbf=$(timeout 3 rosnode list 2>/dev/null | grep -c '^/move_base_flex$')"
      } > "$RUN/param_${tag}.txt"
      say "  ★参数面互校（第 ${el}s）★：$(tr '\n' ' ' < "$RUN/param_${tag}.txt")"
      local got_um got_cm
      got_um=$(grep -m1 '^use_maximum=' "$RUN/param_${tag}.txt" | cut -d= -f2-)
      got_cm=$(grep -m1 '^combination_method=' "$RUN/param_${tag}.txt" | cut -d= -f2-)
      say "  ★同式互校（★读回值先归一：含 "not set" 记作 NONE★）＝use_maximum 读回=$(cls "$got_um")／期望=$(cls "$want_um") ⇒ 同＝$([ "$(cls "$got_um")" = "$(cls "$want_um")" ] && echo True || echo False)★ ‖ combination_method 读回=$(cls "$got_cm")／期望=$(cls "$want_cm") ⇒ 同＝$([ "$(cls "$got_cm")" = "$(cls "$want_cm")" ] && echo True || echo False)★"
    fi
    sleep 1
  done
  wait $sp; rc=$?
  say "  $tag：冒烟件 rc=$rc ‖ CHECK 行数=$(grep -c '^CHECK|' "$out") ‖ PASS=$(grep -c '^CHECK|[^|]*|PASS' "$out") ‖ FAIL=$(grep -c '^CHECK|[^|]*|FAIL' "$out")"
  cp -f /tmp/sim_nav_smoke_launch.log "$RUN/roslaunch_${tag}.log" 2>/dev/null
  say "  ★C11（$tag）＝launch 日志字节=$(stat -c%s "$RUN/roslaunch_${tag}.log" 2>/dev/null) ‖ Using plugin 行=$(grep -c 'Using plugin' "$RUN/roslaunch_${tag}.log" 2>/dev/null) ‖ Received a 行=$(grep -c 'Received a' "$RUN/roslaunch_${tag}.log" 2>/dev/null) ‖ 层名清单=$(grep -oE 'Using plugin [A-Za-z_:]+' "$RUN/roslaunch_${tag}.log" 2>/dev/null | tr '\n' ',' )★"
  say "  $tag 用时≈$(( $(date +%s) - t0 ))s（★单跑不得超 300s★）"
}

run_one REG "$SM_REG" "Parameter is not set." "Parameter is not set."
run_one CP "$SM_CP" "True" "1"

say "== 步9 退净（★SIGTERM wrapper → 轮询 rosout 消失 → 差集双过滤 → 五数★）=="
for p in $PIDS; do kill "$p" 2>/dev/null; done
PIDS=""
N=$(grep -o '^ROSOUT_PID=[0-9]*' "$RUN/OWNERSHIP.txt" | cut -d= -f2)
for k in $(seq 1 12); do [ -z "$N" ] && break; kill -0 "$N" 2>/dev/null || { say "  ★第 ${k}s：本窗 rosout pid=$N 已消失★"; break; }; sleep 1; done
AN=$(snap "$RUN/S1_after.tsv")
awk -F'\t' 'NR==FNR{s[$1"|"$2]=1;next} !($1"|"$2 in s)' "$RUN/S0_before.tsv" "$RUN/S1_after.tsv" > "$RUN/D_delta.tsv"
DN=$(wc -l < "$RUN/D_delta.tsv")
: > "$RUN/K_kill.tsv"
while IFS=$'\t' read -r pid lstart comm args; do
  [ -n "${pid:-}" ] || continue
  if [ -n "${UUID:-}" ] && printf '%s' "$args" | grep -q -- "__log:=$LOGD/$UUID/"; then
    kill -INT "$pid" 2>/dev/null
    printf '%s\t%s\t%s\t%s\n' "$pid" "$lstart" "$comm" "$args" >> "$RUN/K_kill.tsv"
    say "  K：pid=$pid comm=$comm（★带本窗 uuid★）"
  else
    say "  ★跳过（非本窗 uuid）pid=$pid comm=$comm★"
  fi
done < "$RUN/D_delta.tsv"
KN=$(wc -l < "$RUN/K_kill.tsv")
sleep 3
DPK=$(awk -F'\t' 'NR==FNR{k[$1"|"$2]=1;next} !($1"|"$2 in k)' "$RUN/K_kill.tsv" "$RUN/D_delta.tsv" | wc -l)
ports
CN=$(snap "$RUN/witness_after.tsv")
say "  ★判定行五数＝基线=$BN｜D=$DN｜K=$KN｜D∖K=$DPK｜端口 TCP=$TCP UDP=$UDP★（现读 $NSZ 名计数=$CN）"
if [ "$CN" -le "$BN" ] && [ "$TCP" = "0" ] && [ "$UDP" = "0" ] && [ "$DN" -eq "$KN" ]; then
  say "  ★退净达标（乙式四合取项全成立）★"
else
  say "  ★退净未达标／D∖K≠∅＝INVALID★"
fi
sed 's/^/    S1｜/' "$RUN/S1_after.tsv" | tee -a "$R/00_window_log.txt"

say "== 步10 段 B 对照（★在册形 vs 副本形：只比（CHECK 名，判定）二元组★）=="
python3 -u "$R/_cmp_r456.py" "$RUN/smoke_REG.txt" "$RUN/smoke_CP.txt" "$RUN/roslaunch_REG.log" "$RUN/roslaunch_CP.log" > "$R/05_ab_cmp.txt" 2>&1
say "  对照件 rc=$?"
sed 's/^/    ｜/' "$R/05_ab_cmp.txt" | tee -a "$R/00_window_log.txt"

say "== 步11 在册四件跑后自证 =="
sha4 after > /dev/null
say "  跑前＝$(head -1 "$RUN/sha4_before.txt")"
say "  跑后＝$(head -1 "$RUN/sha4_after.txt")"
say "  ★四件逐字同值＝$([ "$(cat "$RUN/sha4_before.txt")" = "$(cat "$RUN/sha4_after.txt")" ] && echo True || echo False)★"
say "  roscore.log 字节=$(stat -c%s "$RUN/roscore.log") ‖ killing 行=$(grep -c 'killing on exit' "$RUN/roscore.log")"

say "== 步99 清盘与计数同条命令 =="
find /opt/ros1-slam -name '__pycache__' -type d -prune -exec rm -rf {} + 2>/dev/null
say "  pyc=$(find /opt/ros1-slam -name '*.pyc' | wc -l) ws_src=$(find /opt/ros1-slam/ws/src -type f | wc -l) tooling=$(find /opt/ros1-slam/tooling -type f | wc -l) note=$(find /opt/ros1-slam/note -type f | wc -l) ‖ 时长 $(( $(date +%s) - T0 ))s"
