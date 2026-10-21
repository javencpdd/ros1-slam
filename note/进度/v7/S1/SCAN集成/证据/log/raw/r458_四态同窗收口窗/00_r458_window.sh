#!/usr/bin/env bash
# 00_r458_window.sh —— ★窗号 R458★（先报后开）＝★四态同窗收口窗★＝按 r456q 裁定（签部分）补两处缺：
#   缺①C1 的在册机检形＝★同一扇窗、同一 boot、同一 load1 带内连跑 A／A′／B／C 四态★，每态在窗志打一行
#        `STATE=<态> master=<URI> seed=<n> traj=<路径> load1=<x> precheck=<rc>` ⇒ 机检 `grep -c '^STATE='` ＝ 4
#   缺②C7 的原式靶形＝★C 态不再是"打断进程"，而是给第二枚层喂一张刻意抬高代价的图★（发布器自报行 unknown=／lethal= 逐字留底）
#   态义＝A／A′＝在册形两遍（同窗噪声地板）；B＝副本形（加一枚与 static 同源、MAX 语义的层）；C＝B 的层改吃 /elev_map（起点半径 0.40m 内涂致命）
#   ★对照两条栏并列★＝栏一（名，判定）向量异行数 ＋ ★栏二 C4 同因同码（共同 FAIL 行逐字比 outcome／message／连带句）★
#        （r456q 点名：上一棒只报栏一＝对真实行为变化是盲的）
#   ★A↔B 参数面 diff 的白名单逐行验由生成器当场跑、结果落 03_delta.txt（★不合字面的行数如实报、不改判、不假装通过★）
#   ★在册四件一字不改★（跑前跑后 sha＋行数自证）
# 姿势＝预检不过不起窗｜名集写实际枚数｜端口 TCP／UDP 分查｜退净按乙式｜local 一行内不引用同行刚声明的变量｜清理与计数同条命令
set -u
R=/opt/ros1-slam/note/进度/SCAN集成/证据/log/raw/r458_四态同窗收口窗
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
PUBPID=""
say(){ echo "[$(date '+%F %T')] $*" | tee -a "$R/00_window_log.txt"; }
state(){ echo "STATE=$1 master=$ROS_MASTER_URI seed=$SEED traj=$TRAJ load1=$(cut -d' ' -f1 /proc/loadavg) precheck=$PRC" | tee -a "$R/00_window_log.txt"; }
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
SM_CP="$R/sim_nav_smoke_r458.sh"
Y_REG=/opt/ros1-slam/ws/src/40_navigation/lite3_navigation/config/global_costmap_params.yaml
M_REG=/opt/ros1-slam/ws/src/70_bringup/lite3_bringup/launch/mode_nav_sim.launch
B_REG=/opt/ros1-slam/ws/src/40_navigation/lite3_navigation/launch/mbf_navigation.launch
Y_B="$R/global_costmap_params_B_r458.yaml"
Y_C="$R/global_costmap_params_C_r458.yaml"
KEYS=/move_base_flex/global_costmap

sha4(){
  local tag="$1"
  { echo "sha 前 16｜smoke=$(sha256sum "$SM_REG" | cut -c1-16) mode=$(sha256sum "$M_REG" | cut -c1-16) mbf=$(sha256sum "$B_REG" | cut -c1-16) yaml=$(sha256sum "$Y_REG" | cut -c1-16)"
    echo "行数｜smoke=$(wc -l < "$SM_REG") mode=$(wc -l < "$M_REG") mbf=$(wc -l < "$B_REG") yaml=$(wc -l < "$Y_REG")"; } > "$RUN/sha4_${tag}.txt"
}

say "== 步0 预检变体当刻（在册预检件一字未动）=="
bash /opt/ros1-slam/scripts/verify/v_window_preflight_x86_variant.sh > "$R/01_preflight.txt" 2>&1
PRC=$?
say "  预检 rc=$PRC"
[ "$PRC" != 0 ] && { say "★预检未过 ⇒ 不起窗"; exit 7; }
SEED=7
TRAJ=$(rospack find lite3_drivers 2>/dev/null)/config/traj_maze.yaml
say "  ★本窗名集＝$NSZ 名（写实际枚数）｜STATE 行常量＝seed=$SEED traj=$TRAJ master=$ROS_MASTER_URI★"
[ -f "$TRAJ" ] && say "  traj 在位＝True" || say "  ★traj 不在位＝False ⇒ STATE 行不可信、不起窗★" 
[ -f "$TRAJ" ] || exit 9
say "== 步0a 副本生成＋白名单逐行验＋在册四件跑前自证 =="
python3 "$R/02_make_copies_r458.py" /opt/ros1-slam >> "$R/00_window_log.txt" 2>&1
GRC=$?
DL=$(wc -l < "$R/03_delta.txt")
BAD=$(grep -c '★不合字面行数=[^0]' "$R/03_delta.txt")
say "  生成器 rc=$GRC ‖ 03_delta.txt 现读行数=$DL ‖ ★不合白名单字面的组数＝$BAD★（★逐组明细在 03_delta.txt、本窗不改判★）"
sha4 before
say "  $(head -1 "$RUN/sha4_before.txt") ‖ $(tail -1 "$RUN/sha4_before.txt")"
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

# run_state <态> <冒烟件> <yaml 或 REG> <带发布器 yes|no>
run_state(){
  local tag="$1" sm="$2" yml="$3" withpub="$4"
  local out="$RUN/smoke_${tag}.txt" t0 rc sp el f sampled=0
  if [ "$yml" = "REG" ]; then unset R458_YAML || true; else export R458_YAML="$yml"; fi
  state "$tag"
  local pw=0
  t0=$(date +%s)
  say "== 跑态 $tag（★$sm｜R458_YAML=${R458_YAML:-（在册形・未设）}★｜超时 300s）=="
  timeout 300 bash "$sm" > "$out" 2>&1 &
  sp=$!
  if [ "$withpub" = "yes" ]; then
    python3 "$R/pub_elev_probe_r458.py" 170 > "$RUN/pub_${tag}.txt" 2>&1 &
    PUBPID=$!
    PIDS="$PIDS $PUBPID"
    say "  发布器随栈起动（pid=$PUBPID；★它自己等 /map ≤60s、拿到后按当刻 base_link 涂致命再 latched 发★）"
  fi
  while kill -0 "$sp" 2>/dev/null; do
    el=$(( $(date +%s) - t0 ))
    f=$(grep -c '^CHECK|' "$out" 2>/dev/null); [ -n "$f" ] || f=0
    printf '%s\t%s\t%s\t%s\n' "$tag" "$el" "$f" "$(grep '^CHECK|' "$out" 2>/dev/null | tail -1 | cut -d'|' -f2)" >> "$RUN/timeline.tsv"
    if [ "$withpub" = "yes" ] && [ "$pw" = "0" ] && grep -q 'SELFREPORT topic' "$RUN/pub_${tag}.txt" 2>/dev/null; then
      pw=1
      say "  ★发布器自报★＝$(grep -m1 'SELFREPORT topic' "$RUN/pub_${tag}.txt" | cut -c1-190)"
    fi
    if [ "$sampled" = "0" ] && [ "$f" -ge 5 ]; then
      sampled=1
      rosparam dump "$RUN/dump_${tag}.yaml" "$KEYS" >/dev/null 2>&1
      { echo "tag=$tag el=${el}s 已印=$f"
        echo "plugins_get=$(timeout 5 rosparam get "$KEYS/plugins" 2>&1 | tr '\n' ' ' | cut -c1-260)"
        echo "use_maximum=$(timeout 5 rosparam get "$KEYS/elevation_layer/use_maximum" 2>&1 | head -1)"
        echo "combination_method=$(timeout 5 rosparam get "$KEYS/elevation_layer/combination_method" 2>&1 | head -1)"
        echo "map_topic=$(timeout 5 rosparam get "$KEYS/elevation_layer/map_topic" 2>&1 | head -1)"
        echo "dump_lines=$(wc -l < "$RUN/dump_${tag}.yaml" 2>/dev/null)"
      } > "$RUN/param_${tag}.txt"
      say "  ★参数面读数（第 ${el}s）★＝$(tr '\n' '|' < "$RUN/param_${tag}.txt" | cut -c1-300)"
    fi
    sleep 1
  done
  wait $sp; rc=$?
  say "  $tag：冒烟件 rc=$rc ‖ CHECK 行数=$(grep -c '^CHECK|' "$out") ‖ PASS=$(grep -c '^CHECK|[^|]*|PASS' "$out") ‖ FAIL=$(grep -c '^CHECK|[^|]*|FAIL' "$out") ‖ 用时≈$(( $(date +%s) - t0 ))s"
  cp -f /tmp/sim_nav_smoke_launch.log "$RUN/roslaunch_${tag}.log" 2>/dev/null
  say "  ★C11（$tag）＝launch 日志字节=$(stat -c%s "$RUN/roslaunch_${tag}.log" 2>/dev/null) ‖ Using plugin 行=$(grep -c 'Using plugin' "$RUN/roslaunch_${tag}.log" 2>/dev/null) ‖ Received a 行=$(grep -c 'Received a' "$RUN/roslaunch_${tag}.log" 2>/dev/null) ‖ 含 elevation_layer 的 Using plugin 行=$(grep -c 'Using plugin "elevation_layer"' "$RUN/roslaunch_${tag}.log" 2>/dev/null)★"
  if [ "$withpub" = "yes" ]; then
    kill -INT "$PUBPID" 2>/dev/null
    for q in 1 2 3 4 5 6; do kill -0 "$PUBPID" 2>/dev/null || break; sleep 1; done
    say "  发布器 SIGINT 后第 ${q}s 已退＝$([ "$(kill -0 "$PUBPID" 2>/dev/null; echo $?)" != 0 ] && echo True || echo False) ‖ 尾行=$(tail -1 "$RUN/pub_${tag}.txt" | cut -c1-90)"
    PUBPID=""
  fi
}

run_state A "$SM_REG" REG no
run_state Ap "$SM_REG" REG no
run_state B "$SM_CP" "$Y_B" no
run_state C "$SM_CP" "$Y_C" yes

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

say "== 步9b C1 的在册机检式现跑 =="
say "  ★grep -c '^STATE=' 00_window_log.txt＝$(grep -c '^STATE=' "$R/00_window_log.txt")★（★在册要求＝4★）"
grep '^STATE=' "$R/00_window_log.txt" | sed 's/^/    STATE｜/' | tee -a "$R/00_window_log.txt"
say "  四态 boot 同一条式子＝同一 WPID=$WPID ‖ 同一 uuid=$UUID ‖ 同一 seed=$SEED ‖ 同一 traj=$TRAJ ‖ 同一 master=$ROS_MASTER_URI"

say "== 步10 四态对照（★栏一（名，判定）＋栏二 C4 同因同码 两条并列★）=="
python3 -u "$R/_cmp_r458.py" "$RUN/smoke_A.txt" "$RUN/smoke_Ap.txt" "$RUN/smoke_B.txt" "$RUN/smoke_C.txt" "$RUN/dump_A.yaml" "$RUN/dump_B.yaml" "$RUN/dump_C.yaml" > "$R/05_four_states.txt" 2>&1
say "  对照件 rc=$? ‖ 行数=$(wc -l < "$R/05_four_states.txt")（★非空自证★）"
sed 's/^/    ｜/' "$R/05_four_states.txt" | tee -a "$R/00_window_log.txt"

say "== 步11 在册四件跑后自证 =="
sha4 after
say "  跑前＝$(head -1 "$RUN/sha4_before.txt")"
say "  跑后＝$(head -1 "$RUN/sha4_after.txt")"
say "  ★四件逐字同值＝$([ "$(cat "$RUN/sha4_before.txt")" = "$(cat "$RUN/sha4_after.txt")" ] && echo True || echo False)★"
say "  roscore.log 字节=$(stat -c%s "$RUN/roscore.log") ‖ killing 行=$(grep -c 'killing on exit' "$RUN/roscore.log")"

say "== 步99 清盘与计数同条命令 =="
find /opt/ros1-slam -name '__pycache__' -type d -prune -exec rm -rf {} + 2>/dev/null
say "  pyc=$(find /opt/ros1-slam -name '*.pyc' | wc -l) ws_src=$(find /opt/ros1-slam/ws/src -type f | wc -l) tooling=$(find /opt/ros1-slam/tooling -type f | wc -l) note=$(find /opt/ros1-slam/note -type f | wc -l) ‖ 时长 $(( $(date +%s) - T0 ))s"
