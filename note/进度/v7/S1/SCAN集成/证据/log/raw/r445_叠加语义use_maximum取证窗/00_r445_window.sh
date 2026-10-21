#!/usr/bin/env bash
# 00_r445_window.sh —— ★窗号 R445★（先报后开）＝★叠加语义取证★：两枚 StaticLayer 同栈时，第二枚能不能★把第一枚的致命格抹成自由★？（＝设计件 §5-A 档那条承重假设，★也是 R441／R442 留下的最后一条边界"combination／use_maximum 通路未测"★）
#   形＝同一条链三段，★唯一变量＝elevation_layer 的 use_maximum★，第二枚输入＝★整张全自由 0★（＝对第一枚那 9 个致命格施加"降格压力"）：
#     u1＝单实例（三态图，基准） ‖ u2＝双实例 use_maximum:★true★（★MAX 期望＝致命格保住★） ‖ u3＝双实例 use_maximum:★false★（★若致命被抹＝该层能降格★）
#   ★两条待判假设（★不预设、★只测★）＝(甲) MAX 只升不降 ⇒ u2 非零仍 9；(乙) 非 MAX 走覆盖 ⇒ u3 非零变 0（★致命被抹光★）——★u3 若真把 9 个致命格降成 0，就是"加 elevation 层会抹掉致命障碍"的直接证据★
#   ★本窗只用已校准的姿势★＝段前★逐层两侧同式互校★（每枚层的 map_topic 读回来与实际发布话题逐字比、不等即不起节点段）＋★export ROS_STDOUT_STDERR=1★（主证＝节点日志 `Received a … map` 行）＋★发布器 SIGINT★＋★乙式退净五数★
# 纪律＝预检不过不起窗｜在册件一字不改（探针＝R444 那份复制后新增一个 mode；三枚参数面新建在本留底目录内）｜清理与计数同条命令
set -u
R=/opt/ros1-slam/note/进度/SCAN集成/证据/log/raw/r445_叠加语义use_maximum取证窗
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
cleanup_seg(){ local p
  for p in $seg_pids; do kill -INT "$p" 2>/dev/null; done   # ★rospy 在 SIGINT 上做关停钩子（R443：SIGTERM 无效）★
  sleep 1
  for p in $seg_pids; do kill "$p" 2>/dev/null; done
  for p in $seg_pids; do kill -9 "$p" 2>/dev/null; done      # ★-9 只用在★本窗亲手起、★在 PID 集里★的对象
  seg_pids=""; }
# ★中断也要收干净★（★本批第一次尝试就是被一行变量错误打断在步1b 之后、把 rosmaster＋rosout 留在了容器里★）
on_exit(){ local rc=$?
  if [ -n "$seg_pids" ] || [ -n "$PIDS" ]; then
    say "★EXIT（rc=$rc）＝脚本没走到步9 ⇒ 就地收本窗 PID 集（INT→TERM）★"
    cleanup_seg
    for p in $PIDS; do kill -INT "$p" 2>/dev/null; done; sleep 2
    for p in $PIDS; do kill -TERM "$p" 2>/dev/null; done
    PIDS=""
  fi
}
trap on_exit EXIT

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

# run_seg <段名> <参数面> <层=…,mode=…,ns=… …（空格分隔）>
run_seg(){
  local NS="$1" SEGF="$2"; shift 2
  say "== 段 $NS（params_$SEGF.yaml）=="
  local spec L M T GOT USEM BAD=0
  for spec in "$@"; do
    L=$(printf '%s' "$spec" | sed -n 's/.*layer=\([^,]*\).*/\1/p')
    M=$(printf '%s' "$spec" | sed -n 's/.*mode=\([^,]*\).*/\1/p')
    T=$(printf '%s' "$spec" | sed -n 's/.*ns=\([^,]*\).*/\1/p')
    python3 -u "$R/pub_grid_probe3.py" "$T" "$M" > "$RUN/pub_$T.log" 2>&1 & seg_pids="$seg_pids $!"
    say "  起发布器 topic=/gmap_$T layer=$L mode=$M"
  done
  sleep 3
  rosparam load "$RUN/params_$SEGF.yaml" "/$NS/costmap" || { say "  ★load_FAIL"; cleanup_seg; return 9; }
  for spec in "$@"; do
    L=$(printf '%s' "$spec" | sed -n 's/.*layer=\([^,]*\).*/\1/p')
    T=$(printf '%s' "$spec" | sed -n 's/.*ns=\([^,]*\).*/\1/p')
    GOT=$(rosparam get "/$NS/costmap/$L/map_topic" 2>&1 | tr -d '" ')
    USEM=$(rosparam get "/$NS/costmap/$L/use_maximum" 2>&1 | tr -d ' ')
    [ "$GOT" = "/gmap_$T" ] || BAD=1
    say "  ★逐层互校＝$L ‖ 参数面 map_topic=[$GOT] ‖ 发布器实际=[/gmap_$T] ⇒ 同＝$([ "$GOT" = "/gmap_$T" ] && echo True || echo False) ‖ use_maximum 回读=[$USEM]★"
  done
  say "  plugins 枚数回读＝$(rosparam get /$NS/costmap/plugins 2>&1 | grep -c 'name:')"
  if [ "$BAD" != 0 ]; then say "  ★互校有不等 ⇒ 不起节点、该段判 INVALID（★R443 的病根在这一步被闸住★）"; cleanup_seg; return 9; fi
  rosrun costmap_2d costmap_2d_node __name:=$NS > "$RUN/node_$NS.log" 2>&1 & seg_pids="$seg_pids $!"
  local rdy=0 j
  for j in $(seq 1 40); do rostopic info "/$NS/costmap/costmap" >/dev/null 2>&1 && { rdy=1; break; }; sleep 1; done
  say "  costmap 就绪＝$rdy（等待 ${j}s）"
  [ "$rdy" != 1 ] && { say "  ★未就绪 ⇒ N3"; cleanup_seg; return 8; }
  timeout 45 rostopic echo -n 1 "/$NS/costmap/costmap" > "$RUN/cost_$NS.txt" 2>&1
  say "  取样 data_行数=$(grep -c 'data:' "$RUN/cost_$NS.txt") 字节=$(stat -c%s "$RUN/cost_$NS.txt")"
  say "  ★节点日志＝字节=$(stat -c%s "$RUN/node_$NS.log") ‖ 行数=$(wc -l < "$RUN/node_$NS.log") ‖ Using plugin 行=$(grep -c 'Using plugin' "$RUN/node_$NS.log") ‖ Received 行=$(grep -c 'Received a' "$RUN/node_$NS.log")★"
  cp "$RUN/node_$NS.log" "$R/node_${NS}_log_excerpt.txt"
  cleanup_seg; sleep 3
  return 0
}
run_seg u1 u1 "layer=static_a,mode=3state,ns=u1"                                        ; A=$?
run_seg u2 u2 "layer=static_a,mode=3state,ns=u2" "layer=elevation_layer,mode=allfree,ns=u2e" ; B=$?
run_seg u3 u3 "layer=static_a,mode=3state,ns=u3" "layer=elevation_layer,mode=allfree,ns=u3e" ; C=$?
say "== 段 rc＝u1=$A u2=$B u3=$C ‖ 时长 $(( $(date +%s) - T0 ))s =="

say "== 步9 退净（★先 SIGTERM → 轮询本窗 rosout 消失 → 差集双过滤 → 判定行五数★）=="
for p in $PIDS; do kill "$p" 2>/dev/null; say "  SIGTERM 本窗 PID pid=$p（★禁 -9★）"; done
PIDS=""; say "  ★本窗 PID 集已交出（此后 EXIT 钩子不再重复收）★"
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
say "== 步10 三段叠加语义对比（★唯一变量＝elevation_layer 的 use_maximum；第二枚输入＝整张全自由 0★）=="
python3 -u "$R/_cmp_three_u.py" "$RUN/cost_u1.txt" "$RUN/cost_u2.txt" "$RUN/cost_u3.txt" > "$R/05_three_face_u.txt" 2>&1
say "  对比件 rc=$?"; sed 's/^/    ｜/' "$R/05_three_face_u.txt" | tee -a "$R/00_window_log.txt"
say "== 步11 roscore／节点日志落盘否 =="
say "  roscore.log 字节=$(stat -c%s "$RUN/roscore.log") ‖ killing 行=$(grep -c 'killing on exit' "$RUN/roscore.log") ‖ node_u1 字节=$(stat -c%s "$RUN/node_u1.log") ‖ node_u2 字节=$(stat -c%s "$RUN/node_u2.log") ‖ node_u3 字节=$(stat -c%s "$RUN/node_u3.log")"
say "== 步99 清盘与计数同条命令 =="
find /opt/ros1-slam -name '__pycache__' -type d -prune -exec rm -rf {} + 2>/dev/null
say "  pyc=$(find /opt/ros1-slam -name '*.pyc' | wc -l) ws_src=$(find /opt/ros1-slam/ws/src -type f | wc -l) tooling=$(find /opt/ros1-slam/tooling -type f | wc -l) note=$(find /opt/ros1-slam/note -type f | wc -l) ‖ 时长 $(( $(date +%s) - T0 ))s"
