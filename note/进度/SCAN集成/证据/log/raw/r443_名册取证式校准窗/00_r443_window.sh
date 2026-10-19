#!/usr/bin/env bash
# 00_r443_window.sh —— ★窗号 R443★（先报后开）＝★名册这条取证式到底能不能用★（★R442 自曝的假阴性机器＋R442 的 07 件"节点日志全 0 字节"两条并成一窗校准★）
#   两态只差一个参数＝c1 `first_map_only: false`（★持续订★） ‖ c2 `first_map_only: true`（★收首帧即退订＝候选解释★）；每段★取样前后各读一次★ `rostopic info /gmap_*`
#   同窗把节点输出通道打开＝export ROS_STDOUT_STDERR=1 ＋ ROSCONSOLE_STDOUT_LINE_BUFFERED=1 ⇒ 若仍 0 字节 ⇒ 该形不对、照实记回坑72 追加例
#   三分支判死＝(a) c1 名册出现订阅者而 c2 不出现 ⇒ 退订解释成立、姿势改"为真时须在首帧前读"；(b) 两态皆 None ⇒ 名册在本链路不可用、★从姿势里删掉★；(c) 节点日志仍 0 字节 ⇒ export 形不对
# 纪律＝预检不过不起窗｜在册件一字不改（探针／两枚参数面新建在本留底目录内）｜退净按 05 追加四十五 落地补记（先 SIGTERM、双过滤、判定行五数）｜清理与计数同条命令
set -u
R=/opt/ros1-slam/note/进度/SCAN集成/证据/log/raw/r443_名册取证式校准窗
RUN="$R/run"
set +u; . /opt/ros/noetic/setup.bash; . /opt/ros1-slam/ws/devel/setup.bash; set -u
export ROS_MASTER_URI=http://127.0.0.1:11313 PYTHONNOUSERSITE=1 PYTHONUNBUFFERED=1
export ROS_STDOUT_STDERR=1 ROSCONSOLE_STDOUT_LINE_BUFFERED=1
NAMESET='^(roslaunch|rosmaster|rosout|gzserver|gzclient|rosbag|costmap_2d_node)$'
LOGD="${ROS_LOG_DIR:-$HOME/.ros/log}"
say(){ echo "[$(date '+%F %T')] $*" | tee -a "$R/00_window_log.txt"; }
snap(){ local f="$1"; : > "$f"
  for pid in $(ps -eo pid=,comm= | awk -v re="$NAMESET" '$2 ~ re {print $1}'); do
    L=$(ps -o lstart= -p "$pid" 2>/dev/null) || continue
    C=$(ps -o comm= -p "$pid" 2>/dev/null) || continue
    A=$(tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null)
    printf '%s\t%s\t%s\t%s\n' "$pid" "$L" "$C" "$A" >> "$f"
  done; wc -l < "$f"; }
roster(){ local tag="$1" topic="$2" f="$3"; { echo "# [$tag] rostopic info $topic（$(date '+%T')）"; rostopic info "$topic" 2>&1; } > "$f"
  echo "$(sed -n '/Subscribers/,$p' "$f" | grep -c '^ \*')"; }
T0=$(date +%s); PIDS=""; seg_pids=""
cleanup_seg(){ for p in $seg_pids; do kill "$p" 2>/dev/null; done; seg_pids=""; }

say "== 步0 预检变体当刻（在册预检件一字未动）=="
bash /opt/ros1-slam/scripts/verify/v_window_preflight_x86_variant.sh > "$R/01_preflight.txt" 2>&1
PRC=$?; say "  预检 rc=$PRC"
[ "$PRC" != 0 ] && { say "★预检未过 ⇒ 不起窗"; exit 7; }
say "  ★本窗打开的输出通道＝ROS_STDOUT_STDERR=${ROS_STDOUT_STDERR} ROSCONSOLE_STDOUT_LINE_BUFFERED=${ROSCONSOLE_STDOUT_LINE_BUFFERED}★"

say "== 步0b 起前快照 S0 =="
BN=$(snap "$RUN/S0_before.tsv"); say "  名集=[$NAMESET] ‖ ★基线＝$BN★"

roscore -p 11313 > "$RUN/roscore.log" 2>&1 & PIDS="$PIDS $!"; WPID=$!
ok=0; for i in $(seq 1 30); do rosnode list 2>/dev/null | grep -q rosout && { ok=1; break; }; sleep 1; done
say "== 步1 master_ready=$ok（等待 ${i}s）‖ WPID=$WPID =="
[ "$ok" != 1 ] && { say "★master 未起"; exit 8; }
rosrun tf static_transform_publisher 0 0 0 0 0 0 map base_link 10 > "$RUN/tf_base.log" 2>&1 & PIDS="$PIDS $!"
UUID=$(basename "$(readlink -f "$LOGD/latest" 2>/dev/null)" 2>/dev/null)
RL="$LOGD/$UUID/roslaunch-$(hostname)-$WPID.log"; [ -f "$RL" ] || RL=$(ls -1t "$LOGD/$UUID"/roslaunch-*.log 2>/dev/null | head -1)
if [ -n "$UUID" ] && [ -f "$RL" ]; then
  N=$(grep -oE 'process\[rosout-1\]: started with pid \[[0-9]+\]' "$RL" | grep -oE '[0-9]+' | tail -1)
  say "== 步1b uuid 自校验＝OK ‖ UUID=$UUID ‖ 本窗 rosout pid N=$N =="
  { echo "UUID=$UUID"; echo "WPID=$WPID"; echo "ROSOUT_PID=$N"; echo "ROSLAUNCH_LOG=$RL"; } > "$RUN/OWNERSHIP.txt"
else
  N=""; say "== 步1b ★uuid 自校验失败 ⇒ 按裁定判 INVALID、不 kill 任何不在 $PIDS 里的进程★ =="
fi

# run_seg <段> <参数面> <topic后缀>
run_seg(){
  local NS="$1" SEGF="$2" TOP="$3"
  say "== 段 $NS（params_$SEGF.yaml，first_map_only 见参数面）=="
  python3 -u "$R/pub_grid_probe2.py" "$TOP" "3state" > "$RUN/pub_$TOP.log" 2>&1 & seg_pids="$seg_pids $!"
  sleep 3
  rosparam load "$RUN/params_$SEGF.yaml" "/$NS/costmap" || { say "  ★load_FAIL"; cleanup_seg; return 9; }
  say "  plugins 枚数回读＝$(rosparam get /$NS/costmap/plugins 2>&1 | grep -c 'name:') ‖ first_map_only 回读＝$(rosparam get /$NS/costmap/static_a/first_map_only 2>&1)"
  rosrun costmap_2d costmap_2d_node __name:=$NS > "$RUN/node_$NS.log" 2>&1 & seg_pids="$seg_pids $!"
  local rdy=0 j
  for j in $(seq 1 40); do rostopic info "/$NS/costmap/costmap" >/dev/null 2>&1 && { rdy=1; break; }; sleep 1; done
  say "  costmap 就绪＝$rdy（等待 ${j}s）"
  [ "$rdy" != 1 ] && { say "  ★未就绪 ⇒ N3"; cleanup_seg; return 8; }
  local PRE=$(roster "取样前" "/gmap_$TOP" "$RUN/roster_${NS}_before.txt")
  timeout 45 rostopic echo -n 1 "/$NS/costmap/costmap" > "$RUN/cost_$NS.txt" 2>&1
  local POST=$(roster "取样后" "/gmap_$TOP" "$RUN/roster_${NS}_after.txt")
  say "  ★名册两读＝取样前订阅者 $PRE 枚 ‖ 取样后 $POST 枚★（★主题＝/gmap_$TOP★）"
  say "  取样 data_行数=$(grep -c 'data:' "$RUN/cost_$NS.txt") 字节=$(stat -c%s "$RUN/cost_$NS.txt") ‖ 非零格数现算＝见 05 件"
  say "  ★节点输出通道＝node_$NS.log 字节=$(stat -c%s "$RUN/node_$NS.log") ‖ 总行数=$(wc -l < "$RUN/node_$NS.log") ‖ 含 layer 行数=$(grep -ic 'layer' "$RUN/node_$NS.log") ‖ 含 map 行数=$(grep -ic 'map' "$RUN/node_$NS.log")★"
  cleanup_seg; sleep 3
  return 0
}
run_seg c1 c1 k1 ; A=$?
run_seg c2 c2 k2 ; B=$?
say "== 段 rc＝c1=$A c2=$B ‖ 时长 $(( $(date +%s) - T0 ))s =="

say "== 步9 退净（★先 SIGTERM → 轮询本窗 rosout 消失 → 差集双过滤 → 判定行五数★）=="
for p in $PIDS; do kill "$p" 2>/dev/null; say "  SIGTERM 本窗 PID pid=$p（★禁 -9★）"; done
gone=0; for k in $(seq 1 10); do
  [ -z "$N" ] && break
  if ! kill -0 "$N" 2>/dev/null; then gone=1; say "  ★第 ${k}s：本窗 rosout pid=$N 已消失★"; break; fi
  sleep 1
done
[ "$gone" != 1 ] && [ -n "$N" ] && say "  ★轮询 10s 未消失 pid=$N★"
AN=$(snap "$RUN/S1_after.tsv")
awk -F'\t' 'NR==FNR{s[$1"|"$2]=1;next} !($1"|"$2 in s)' "$RUN/S0_before.tsv" "$RUN/S1_after.tsv" > "$RUN/D_delta.tsv"
DN=$(wc -l < "$RUN/D_delta.tsv"); : > "$RUN/K_kill.tsv"
while IFS=$'\t' read -r pid lstart comm args; do
  [ -n "${pid:-}" ] || continue
  case " $PIDS $seg_pids " in *" $pid "*) kill -TERM "$pid" 2>/dev/null; printf '%s\t%s\t%s\t%s\n' "$pid" "$lstart" "$comm" "$args" >> "$RUN/K_kill.tsv"; say "  K：pid=$pid（∈本窗 PID 集）";;
    *) if [ -n "${UUID:-}" ] && printf '%s' "$args" | grep -q -- "__log:=$LOGD/$UUID/"; then kill -TERM "$pid" 2>/dev/null; printf '%s\t%s\t%s\t%s\n' "$pid" "$lstart" "$comm" "$args" >> "$RUN/K_kill.tsv"; say "  K：pid=$pid comm=$comm（★带本窗 uuid★）"; else say "  ★跳过（非本窗 uuid／不在 PID 集）pid=$pid comm=$comm★"; fi;;
  esac
done < "$RUN/D_delta.tsv"
KN=$(wc -l < "$RUN/K_kill.tsv"); sleep 2
DPK=$(awk -F'\t' 'NR==FNR{k[$1"|"$2]=1;next} !($1"|"$2 in k)' "$RUN/K_kill.tsv" "$RUN/D_delta.tsv" | wc -l)
P=$( { ss -lnt 2>/dev/null; ss -lntu 2>/dev/null; } | grep -cE ":(11311|11312|11313)\b")
CN=$(snap "$RUN/witness_after.tsv")
say "  ★判定行五数＝基线=$BN｜D=$DN｜K=$KN｜D∖K=$DPK｜端口=$P★（现读计数=$CN）"
if [ "$CN" -le "$BN" ] && [ "$P" = "0" ] && [ "$DN" -eq "$KN" ]; then say "  ★退净达标（乙式四合取项全成立）★"; else say "  ★退净未达标／或 D∖K≠∅＝INVALID★"; fi
say "== 步10 两态 cost 面同异（★顺带自校验：两态只差 first_map_only、输入同＝应逐格等★）=="
python3 -u "$R/_cmp_two.py" "$RUN/cost_c1.txt" "$RUN/cost_c2.txt" > "$R/05_two_face.txt" 2>&1
say "  对比件 rc=$?"; sed 's/^/    ｜/' "$R/05_two_face.txt" | tee -a "$R/00_window_log.txt"
say "== 步11 roscore／节点日志落盘否 =="
say "  roscore.log 字节=$(stat -c%s "$RUN/roscore.log") ‖ killing 行=$(grep -c 'killing on exit' "$RUN/roscore.log") ‖ node_c1 字节=$(stat -c%s "$RUN/node_c1.log") ‖ node_c2 字节=$(stat -c%s "$RUN/node_c2.log")"
say "== 步99 清盘与计数同条命令 =="
find /opt/ros1-slam -name '__pycache__' -type d -prune -exec rm -rf {} + 2>/dev/null
say "  pyc=$(find /opt/ros1-slam -name '*.pyc' | wc -l) ws_src=$(find /opt/ros1-slam/ws/src -type f | wc -l) tooling=$(find /opt/ros1-slam/tooling -type f | wc -l) note=$(find /opt/ros1-slam/note -type f | wc -l) ‖ 时长 $(( $(date +%s) - T0 ))s"
