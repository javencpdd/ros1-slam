#!/usr/bin/env bash
# 00_r448_window.sh —— ★窗号 R448★（先报后开）＝§8 预检 3★本体★的第一段＝★在册 13 CHECK 冒烟件能不能在这台镜像跑起来★（起量探针）
#   为什么只跑这一段＝冒烟件★自己会 setsid roslaunch mode_nav_sim.launch 并自带退净★ ⇒ 它一跑＝整条导航栈＋mockamap maze2d＋规划链全起；
#   ★包齐不齐在宿主侧无法验（本版 roslaunch 无 --deps/--params、我的 XML 解析件处理 $(find) 失败）⇒ 只有窗内现跑才有答案★
#   本窗★不★加 elevation_layer（★那是段 B、另窗★）；★在册件一字不改★（★不碰 launch／不碰 yaml／不碰冒烟件本体★）
#   纪律＝预检不过不起窗｜★单段上限 300s（超即 timeout 掐掉、照实记"起量未成"）★｜退净按 05 追加四十五 落地补记（SIGTERM wrapper＋差集双过滤＋判定行五数）｜清理与计数同条命令
set -u
R=/opt/ros1-slam/note/进度/SCAN集成/证据/log/raw/r448_预检3本体起量探针窗
RUN="$R/run"; mkdir -p "$RUN"
set +u; . /opt/ros/noetic/setup.bash; . /opt/ros1-slam/ws/devel/setup.bash; set -u
export ROS_MASTER_URI=http://127.0.0.1:11313 PYTHONNOUSERSITE=1 PYTHONUNBUFFERED=1
export ROS_STDOUT_STDERR=1 ROSCONSOLE_STDOUT_LINE_BUFFERED=1
NAMESET='^(roslaunch|rosmaster|rosout|gzserver|gzclient|rosbag|costmap_2d_node|move_base_flex|pcl_render_node|sim_adapter|rostopic|health_monitor|map_server|mockamap)$'
LOGD="${ROS_LOG_DIR:-$HOME/.ros/log}"
say(){ echo "[$(date '+%F %T')] $*" | tee -a "$R/00_window_log.txt"; }
snap(){ local f="$1"; : > "$f"
  for pid in $(ps -eo pid=,comm= | awk -v re="$NAMESET" '$2 ~ re {print $1}'); do
    L=$(ps -o lstart= -p "$pid" 2>/dev/null) || continue
    C=$(ps -o comm= -p "$pid" 2>/dev/null) || continue
    A=$(tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null)
    printf '%s\t%s\t%s\t%s\n' "$pid" "$L" "$C" "$A" >> "$f"
  done; wc -l < "$f"; }
T0=$(date +%s); PIDS=""
cleanup(){ for p in $PIDS; do kill "$p" 2>/dev/null; done; }
on_exit(){ local rc=$?
  if [ -n "$PIDS" ]; then say "★EXIT（rc=$rc）＝没走到步9 ⇒ 就地收本窗 PID 集★"; for p in $PIDS; do kill -INT "$p" 2>/dev/null; done; sleep 2; cleanup; PIDS=""; fi
}
trap on_exit EXIT

say "== 步0 预检变体当刻（在册预检件一字未动）=="
bash /opt/ros1-slam/scripts/verify/v_window_preflight_x86_variant.sh > "$R/01_preflight.txt" 2>&1
PRC=$?; say "  预检 rc=$PRC"
[ "$PRC" != 0 ] && { say "★预检未过 ⇒ 不起窗"; exit 7; }
say "== 步0b 起前快照 S0（★名集已扩到导航栈成员★）=="
BN=$(snap "$RUN/S0_before.tsv"); say "  基线=$BN"
sed 's/^/    S0｜/' "$RUN/S0_before.tsv" | tee -a "$R/00_window_log.txt"
roscore -p 11313 > "$RUN/roscore.log" 2>&1 & PIDS="$PIDS $!"; WPID=$!
ok=0; for i in $(seq 1 30); do rosnode list 2>/dev/null | grep -q rosout && { ok=1; break; }; sleep 1; done
say "== 步1 master_ready=$ok（等待 ${i}s）‖ WPID=$WPID =="
[ "$ok" != 1 ] && { say "★master 未起"; exit 8; }
UUID=$(basename "$(readlink -f "$LOGD/latest" 2>/dev/null)" 2>/dev/null)
RL="$LOGD/$UUID/roslaunch-$(hostname)-$WPID.log"; [ -f "$RL" ] || RL=$(ls -1t "$LOGD/$UUID"/roslaunch-*.log 2>/dev/null | head -1)
N=$(grep -oE 'process\[rosout-1\]: started with pid \[[0-9]+\]' "$RL" 2>/dev/null | grep -oE '[0-9]+' | tail -1)
say "== 步1b uuid 自校验＝$([ -n "$UUID" ] && [ -f "$RL" ] && echo OK || echo FAIL) ‖ UUID=$UUID ‖ 本窗 rosout pid N=$N =="
{ echo "UUID=$UUID"; echo "WPID=$WPID"; echo "ROSOUT_PID=$N"; echo "ROSLAUNCH_LOG=$RL"; } > "$RUN/OWNERSHIP.txt"
rosrun tf static_transform_publisher 0 0 0 0 0 0 map base_link 10 > "$RUN/tf_base.log" 2>&1 & PIDS="$PIDS $!"

say "== 步2 跑★在册★冒烟件（★一字未改、★上限 300s、★只报不判★）=="
SM=ws/src/70_bringup/lite3_bringup/verify/sim_nav_smoke.sh
say "  冒烟件 sha 前 16=$(sha256sum "$SM" | cut -c1-16) 行数=$(wc -l < "$SM")（★跑前后必须同值⇒证明未改★）"
timeout 300 bash "$SM" > "$RUN/smoke_stdout.txt" 2>&1; SC=$?
say "  ★冒烟件 rc=$SC★（0＝全 PASS｜1＝有 FAIL｜124＝被 300s 上限掐掉＝★起量未成★）‖ 时长 $(( $(date +%s) - T0 ))s"
say "  ★CHECK 行数=$(grep -c '^CHECK|' "$RUN/smoke_stdout.txt")‖ 其中 PASS=$(grep -c '^CHECK|[^|]*|PASS' "$RUN/smoke_stdout.txt")‖ FAIL=$(grep -c '^CHECK|[^|]*|FAIL' "$RUN/smoke_stdout.txt")★"
cp -f /tmp/sim_nav_smoke_launch.log "$RUN/roslaunch_smoke.log" 2>/dev/null && say "  冒烟件自带 launch 全量日志已随窗留底＝字节=$(stat -c%s "$RUN/roslaunch_smoke.log")" || say "  ★无 /tmp/sim_nav_smoke_launch.log★"
say "  ★起不来时最要紧的三行（★launch 日志尾）★："; tail -3 "$RUN/roslaunch_smoke.log" 2>/dev/null | sed 's/^/    ｜/' | tee -a "$R/00_window_log.txt"
say "  冒烟件收尾（★末 8 行★）："; tail -8 "$RUN/smoke_stdout.txt" | sed 's/^/    ｜/' | tee -a "$R/00_window_log.txt"
sha256sum "$SM" | cut -c1-16 | sed 's/^/  冒烟件跑后 sha 前 16=/' | tee -a "$R/00_window_log.txt"

say "== 步9 退净（★SIGTERM wrapper → 轮询 rosout 消失 → 差集双过滤 → 五数★）=="
for p in $PIDS; do kill "$p" 2>/dev/null; say "  SIGTERM 本窗 PID pid=$p"; done
PIDS=""
for k in $(seq 1 12); do
  [ -z "$N" ] && break
  kill -0 "$N" 2>/dev/null || { say "  ★第 ${k}s：本窗 rosout pid=$N 已消失★"; break; }
  sleep 1
done
AN=$(snap "$RUN/S1_after.tsv")
awk -F'\t' 'NR==FNR{s[$1"|"$2]=1;next} !($1"|"$2 in s)' "$RUN/S0_before.tsv" "$RUN/S1_after.tsv" > "$RUN/D_delta.tsv"
DN=$(wc -l < "$RUN/D_delta.tsv"); : > "$RUN/K_kill.tsv"
while IFS=$'\t' read -r pid lstart comm args; do
  [ -n "${pid:-}" ] || continue
  case " $PIDS " in *" $pid "*) kill -INT "$pid" 2>/dev/null; echo "$pid	$lstart	$comm	$args" >> "$RUN/K_kill.tsv"; say "  K：pid=$pid（∈本窗 PID 集）";;
    *) if [ -n "${UUID:-}" ] && printf '%s' "$args" | grep -q -- "__log:=$LOGD/$UUID/"; then kill -INT "$pid" 2>/dev/null; echo "$pid	$lstart	$comm	$args" >> "$RUN/K_kill.tsv"; say "  K：pid=$pid comm=$comm（★带本窗 uuid★）"; else say "  ★跳过（非本窗 uuid／不在 PID 集）pid=$pid comm=$comm★"; fi;;
  esac
done < "$RUN/D_delta.tsv"
KN=$(wc -l < "$RUN/K_kill.tsv"); sleep 3
DPK=$(awk -F'\t' 'NR==FNR{k[$1"|"$2]=1;next} !($1"|"$2 in k)' "$RUN/K_kill.tsv" "$RUN/D_delta.tsv" | wc -l)
P=$( { ss -lnt 2>/dev/null; ss -lntu 2>/dev/null; } | grep -cE ":(11311|11312|11313)\b")
CN=$(snap "$RUN/witness_after.tsv")
say "  ★判定行五数＝基线=$BN｜D=$DN｜K=$KN｜D∖K=$DPK｜端口=$P★（现读计数=$CN）"
[ "$CN" -le "$BN" ] && [ "$P" = "0" ] && [ "$DN" -eq "$KN" ] && say "  ★退净达标（乙式）★" || say "  ★退净未达标／D∖K≠∅＝INVALID★（★照实入册★）"
sed 's/^/    S1｜/' "$RUN/S1_after.tsv" | tee -a "$R/00_window_log.txt"
say "== 步11 日志落盘 =="
say "  roscore.log 字节=$(stat -c%s "$RUN/roscore.log") ‖ killing 行=$(grep -c 'killing on exit' "$RUN/roscore.log")"
say "== 步99 清盘与计数同条命令 =="
find /opt/ros1-slam -name '__pycache__' -type d -prune -exec rm -rf {} + 2>/dev/null
say "  pyc=$(find /opt/ros1-slam -name '*.pyc' | wc -l) ws_src=$(find /opt/ros1-slam/ws/src -type f | wc -l) tooling=$(find /opt/ros1-slam/tooling -type f | wc -l) note=$(find /opt/ros1-slam/note -type f | wc -l) ‖ 时长 $(( $(date +%s) - T0 ))s"
