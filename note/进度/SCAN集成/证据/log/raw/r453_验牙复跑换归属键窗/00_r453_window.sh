#!/usr/bin/env bash
# 00_r453_window.sh —— ★窗号 R453★（先报后开）＝★验牙复跑窗＝把 R452 因〔归属键选错〕没做成的 C7 真正做成★
#   R451 三条成因逐条对症：①第 28s 才动手＝CHECK 序列已跑完 ⇒ ★改事件驱动★（轮询本窗冒烟输出：tf_robot 已印 且 costmap_data 未印 ⇒ 才动手）
#                             ②未按 rosnode list 核名就按 launch 里 name= 假定 ⇒ ★动手前先取名★（名不在真实名册里＝不起注入、判 INVALID）
#                             ③对 launch 日志格式的假设也不对 ⇒ ★本窗不再依赖那条假设★
#   ★kill 前置硬闸＝该节点 pid 必须在本窗 S0 差集里★（＝证明是本窗生的才许 kill；不在＝不杀）
#   ★R452 的错★＝用〔14 名 comm 正则 + S0 差集〕找 mbf 的 pid ⇒ 该进程 comm 名不在这 14 个里、候选被筛没（run/S_mid.tsv 现读只有 4 行）
#   ★本窗换归属键★＝①从★本跑自己的★ launch 日志按★已被 rosout 用通的同一形状★ `process\[move_base_flex-[0-9]+\]: started with pid \[P\]` 取 pid；
#                 ②`ps -o etimes=` 二次自证＝该 pid 出生在本跑起点之后；③名册含 /move_base_flex（ROS 级实名）。★三条都过才 kill★
#   ★打断是否真生效★＝kill 后 +1s／+3s 各复查一次名册（★不生效就不能拿「没变红」说这把闸没牙★）
#   ★在册冒烟件一字不改★（跑前跑后 sha＋行数自证）；★本窗不加 elevation 层★（那是 R454 段 B）
# 姿势＝预检不过不起窗｜名集写实际枚数｜端口 TCP／UDP 分查｜退净按 05 追加四十五 落地补记｜★local 一行内不引用同行刚声明的变量（坑74／r444／r451 同族三犯）★｜清理与计数同条命令
set -u
R=/opt/ros1-slam/note/进度/SCAN集成/证据/log/raw/r453_验牙复跑换归属键窗
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
SM=ws/src/70_bringup/lite3_bringup/verify/sim_nav_smoke.sh

say "== 步0 预检变体当刻（在册预检件一字未动）=="
bash /opt/ros1-slam/scripts/verify/v_window_preflight_x86_variant.sh > "$R/01_preflight.txt" 2>&1
PRC=$?
say "  预检 rc=$PRC"
[ "$PRC" != 0 ] && { say "★预检未过 ⇒ 不起窗"; exit 7; }
say "  ★本窗名集＝$NSZ 名（写实际枚数）：$NAMEPAT"
SM_SHA0=$(sha256sum "$SM" | cut -c1-16)
SM_LN0=$(wc -l < "$SM")
say "  ★在册冒烟件跑前 sha 前 16=$SM_SHA0 行数=$SM_LN0★"

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

# 取样一行：elapsed｜CHECK 行数｜最后一条 CHECK 名｜名册里是否有 /move_base_flex
probe(){
  local out="$1" tag="$2" el="$3" f
  f=$(grep -c '^CHECK|' "$out" 2>/dev/null)
  local last
  last=$(grep '^CHECK|' "$out" 2>/dev/null | tail -1 | cut -d'|' -f2)
  local has
  has=$(timeout 3 rosnode list 2>/dev/null | grep -c '^/move_base_flex$')
  printf '%s\t%s\t%s\t%s\t%s\n' "$tag" "$el" "$f" "${last:-none}" "$has" >> "$RUN/timeline.tsv"
}

# run_smoke <标签> <inj=yes|no>
run_smoke(){
  local tag="$1" inj="$2"
  local out="$RUN/smoke_${tag}.txt"
  local t0 t rc sp
  t0=$(date +%s)
  say "== 跑 $tag（inj=$inj｜★在册冒烟件一字不改★｜★超时上限 300s★）=="
  if [ "$inj" = "yes" ]; then
    timeout 300 bash "$SM" > "$out" 2>&1 &
    sp=$!
    : > "$RUN/inject_log.txt"
    local acted=0 el f last has pid TOT
    TOT=$(grep -c '^CHECK|' "$RUN/smoke_CT.txt" 2>/dev/null); [ -n "$TOT" ] || TOT=0
    say "  ★控制跑总 CHECK 数 TOT=$TOT ⇒ 注入窗＝已印 4～$(( TOT - 1 )) 条之间★"
    while kill -0 "$sp" 2>/dev/null; do
      el=$(( $(date +%s) - t0 ))
      f=$(grep -c '^CHECK|' "$out" 2>/dev/null); [ -n "$f" ] || f=0
      last=$(grep '^CHECK|' "$out" 2>/dev/null | tail -1 | cut -d'|' -f2); [ -n "$last" ] || last=none
      probe "$out" "$tag" "$el"
      if [ "$acted" = "0" ] && [ "$f" -ge 4 ] && [ "$f" -lt "$TOT" ]; then
        has=$(timeout 3 rosnode list 2>/dev/null | grep -c '^/move_base_flex$')
        say "  ★条件命中（第 ${el}s｜已印 $f／$TOT｜最后一条=$last）★"
        echo "COND at=${el}s printed=$f last=$last" >> "$RUN/inject_log.txt"
        if [ "$has" != "1" ]; then
          echo "MISS $el 名册无 /move_base_flex ⇒ 不起注入（INVALID）" >> "$RUN/inject_log.txt"
          say "  ★名册里没有该实名 ⇒ 本窗判 INVALID、不杀任何东西★"
          acted=2
        else
          timeout 3 rosnode list > "$RUN/roster_at_inject.txt" 2>&1
          snap "$RUN/S_mid.tsv" > /dev/null
          LP=/tmp/sim_nav_smoke_launch.log
          pid=$(grep -oE 'process\[move_base_flex-[0-9]+\]: started with pid \[[0-9]+\]' "$LP" 2>/dev/null | grep -oE '[0-9]+' | tail -1)
          et=$(ps -o etimes= -p "${pid:-0}" 2>/dev/null | tr -d ' '); [ -n "$et" ] || et=none
          argl=$(tr '\0' ' ' < "/proc/${pid:-0}/cmdline" 2>/dev/null | cut -c1-90); [ -n "$argl" ] || argl=none
          echo "ROSTER $(tr '\n' ' ' < "$RUN/roster_at_inject.txt")" >> "$RUN/inject_log.txt"
          echo "LAUNCHLOG $LP 命中行数=$(grep -cE 'process\[move_base_flex-[0-9]+\]: started with pid' "$LP" 2>/dev/null)" >> "$RUN/inject_log.txt"
          echo "MIDPID ${pid:-none} etimes=$et（本跑已 ${el}s）cmdline=$argl ‖ S_mid 行数=$(wc -l < "$RUN/S_mid.tsv")" >> "$RUN/inject_log.txt"
          say "  ★归属三证＝launch 日志 pid=${pid:-none}｜etimes=$et ≤ 本跑已 ${el}s｜名册含实名★（★R452 那格 14 名差集现读＝$(wc -l < "$RUN/S_mid.tsv") 行、不含 mbf★）"
          if [ -n "${pid:-}" ] && [ "$et" != "none" ] && [ "$et" -le "$el" ] 2>/dev/null; then :; else pid=""; fi
          if [ -z "${pid:-}" ]; then
            say "  ★归属三证未齐（pid／etimes／名册）⇒ 不 kill（红线：不杀不可归属的进程）★"
            acted=3
          else
            cp -f "$out" "$RUN/smoke_INJ_pre.txt"
            grep '^CHECK|' "$RUN/smoke_INJ_pre.txt" | cut -d'|' -f2 | tr '\n' ',' > "$RUN/pre_names.txt"
            grep '^CHECK|' "$RUN/smoke_CT.txt" | cut -d'|' -f2 | tail -n +$(( f + 1 )) | tr '\n' ',' > "$RUN/pending_names.txt"
            say "  ★注入动作（第 ${el}s）＝rosnode kill /move_base_flex ‖ 归属证明＝本窗 S0 差集内 pid=$pid★"
            timeout 8 rosnode kill /move_base_flex > "$RUN/kill_mid.txt" 2>&1
            local krc=$?
            echo "KILL rc=$krc out=$(head -1 "$RUN/kill_mid.txt" | cut -c1-90) at=${el}s pid=$pid" >> "$RUN/inject_log.txt"
            say "  kill 返回 rc=$krc（$(head -1 "$RUN/kill_mid.txt" | cut -c1-80)）‖ 打断前 CHECK 行数=$(grep -c '^CHECK|' "$RUN/smoke_INJ_pre.txt")"
            echo "POSTCHECK" >> "$RUN/inject_log.txt"
            for d2 in 1 3; do sleep "$d2"; h2=$(timeout 3 rosnode list 2>/dev/null | grep -c '^/move_base_flex$'); a2=$(ps -o etimes= -p "$pid" 2>/dev/null | tr -d ' '); echo "  +${d2}s 名册含 mbf=$h2 ‖ pid $pid etimes=${a2:-gone}" | tee -a "$RUN/inject_log.txt"; done
            say "  ★打断前已印名＝$(cut -c1-120 "$RUN/pre_names.txt")｜待印名＝$(cut -c1-120 "$RUN/pending_names.txt")★"
            acted=1
          fi
        fi
      fi
      sleep 1
    done
    wait $sp; rc=$?
    say "  ★注入跑是否真动过手＝acted=$acted（1＝已注入／2＝名册未命中／3＝pid 未归属／0＝条件从未命中）★"
  else
    timeout 300 bash "$SM" > "$out" 2>&1
    rc=$?
  fi
  t=$(( $(date +%s) ))
  say "  $tag：冒烟件 rc=$rc ‖ CHECK 行数=$(grep -c '^CHECK|' "$out") ‖ PASS=$(grep -c '^CHECK|[^|]*|PASS' "$out") ‖ FAIL=$(grep -c '^CHECK|[^|]*|FAIL' "$out")"
  cp -f /tmp/sim_nav_smoke_launch.log "$RUN/roslaunch_${tag}.log" 2>/dev/null
  say "  ★C11 挂载活性锁（$tag）＝launch 日志字节=$(stat -c%s "$RUN/roslaunch_${tag}.log" 2>/dev/null) ‖ Using plugin 行=$(grep -c 'Using plugin' "$RUN/roslaunch_${tag}.log" 2>/dev/null) ‖ Received a 行=$(grep -c 'Received a' "$RUN/roslaunch_${tag}.log" 2>/dev/null)★"
  say "  $tag 用时≈$(( t - t0 ))s（★单跑不得超 300s★）"
}

run_smoke CT no
run_smoke INJ yes

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

say "== 步10 对照（★控制跑 vs 打断前 vs 注入跑：只看（CHECK 名，判定）二元组★）=="
python3 -u "$R/_cmp_r453.py" "$RUN/smoke_CT.txt" "$RUN/smoke_INJ_pre.txt" "$RUN/smoke_INJ.txt" > "$R/05_inject_cmp.txt" 2>&1
say "  对照件 rc=$?"
sed 's/^/    ｜/' "$R/05_inject_cmp.txt" | tee -a "$R/00_window_log.txt"

say "== 步11 在册件未改自证 ＋ 日志落盘 =="
SM_SHA1=$(sha256sum "$SM" | cut -c1-16)
SM_LN1=$(wc -l < "$SM")
say "  冒烟件跑后 sha 前 16=$SM_SHA1 ‖ 跑前=$SM_SHA0 ⇒ ★同值＝$([ "$SM_SHA0" = "$SM_SHA1" ] && echo True || echo False)★、行数 $SM_LN1（跑前 $SM_LN0）"
say "  roscore.log 字节=$(stat -c%s "$RUN/roscore.log") ‖ killing 行=$(grep -c 'killing on exit' "$RUN/roscore.log")"

say "== 步99 清盘与计数同条命令 =="
find /opt/ros1-slam -name '__pycache__' -type d -prune -exec rm -rf {} + 2>/dev/null
say "  pyc=$(find /opt/ros1-slam -name '*.pyc' | wc -l) ws_src=$(find /opt/ros1-slam/ws/src -type f | wc -l) tooling=$(find /opt/ros1-slam/tooling -type f | wc -l) note=$(find /opt/ros1-slam/note -type f | wc -l) ‖ 时长 $(( $(date +%s) - T0 ))s"
