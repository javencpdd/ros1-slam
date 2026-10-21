#!/usr/bin/env bash
# 00_r442_window.sh —— ★窗号 R442★（先报后开）＝R441 弱形的★决定性补测★：第二枚 StaticLayer 到底吃没吃到图（阳性对照）
#   三段＝w1 单实例（三态图）‖ w2 双实例（static_a 三态＋elevation_layer ★全未知★＝复现 R441 v2b）‖ w3 双实例（elevation_layer ★全致命 100★＝阳性对照）
#   判读＝w3 非零数远大于 w1/w2（≈1600）⇒ 该层真在合成 ⇒ R441 的"零变化"落在解释(甲)＝未知被当自由；w3 与 w1/w2 逐格等 ⇒ 该层空转 ⇒ R441★降为弱形★
#   ★同窗带第二条★＝把 05 追加四十五 落地补记里"先 SIGTERM → 认 rosout pid → 轮询消失 → 双过滤补救 → 判定行五数"★首次实跑★
# 纪律＝预检不过不起窗｜在册件一字不改（探针／参数面／对比件全新建在本留底目录内）｜只 kill 本窗 PID ∪ 本窗 uuid 差集｜清理与计数同条命令
set -u
R=/opt/ros1-slam/note/进度/SCAN集成/证据/log/raw/r442_第二实例阳性对照窗
RUN="$R/run"
set +u; . /opt/ros/noetic/setup.bash; . /opt/ros1-slam/ws/devel/setup.bash; set -u
export ROS_MASTER_URI=http://127.0.0.1:11313 PYTHONNOUSERSITE=1 PYTHONUNBUFFERED=1
NAMESET='^(roslaunch|rosmaster|rosout|gzserver|gzclient|rosbag|costmap_2d_node)$'
LOGD="${ROS_LOG_DIR:-$HOME/.ros/log}"
say(){ echo "[$(date '+%F %T')] $*" | tee -a "$R/00_window_log.txt"; }
# ★快照＝(pid, lstart) 双键＋comm＋args（args 里带 __log:=<uuid>＝归属令牌）★
snap(){ local f="$1"; : > "$f"
  for pid in $(ps -eo pid=,comm= | awk -v re="$NAMESET" '$2 ~ re {print $1}'); do
    L=$(ps -o lstart= -p "$pid" 2>/dev/null) || continue
    C=$(ps -o comm= -p "$pid" 2>/dev/null) || continue
    A=$(tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null)
    printf '%s\t%s\t%s\t%s\n' "$pid" "$L" "$C" "$A" >> "$f"
  done; wc -l < "$f"; }
T0=$(date +%s); PIDS=""; seg_pids=""
cleanup_seg(){ for p in $seg_pids; do kill "$p" 2>/dev/null; done; seg_pids=""; }

say "== 步0 预检变体当刻（在册预检件一字未动）=="
bash /opt/ros1-slam/scripts/verify/v_window_preflight_x86_variant.sh > "$R/01_preflight.txt" 2>&1
PRC=$?; say "  预检 rc=$PRC"
[ "$PRC" != 0 ] && { say "★预检未过 ⇒ 不起窗"; exit 7; }

say "== 步0b 起前快照 S0（★名集写进读数＝坑76★）=="
BN=$(snap "$RUN/S0_before.tsv"); say "  名集=[$NAMESET] ‖ ★基线＝退前计数＝$BN★"; sed 's/^/    S0｜/' "$RUN/S0_before.tsv" | tee -a "$R/00_window_log.txt"

say "== 步1 起 roscore（★记 wrapper pid＝WPID★）=="
roscore -p 11313 > "$RUN/roscore.log" 2>&1 & PIDS="$PIDS $!"; WPID=$!
ok=0; for i in $(seq 1 30); do rosnode list 2>/dev/null | grep -q rosout && { ok=1; break; }; sleep 1; done
say "  master_ready=$ok（等待 ${i}s）‖ WPID=$WPID"
[ "$ok" != 1 ] && { say "★master 未起"; exit 8; }
rosrun tf static_transform_publisher 0 0 0 0 0 0 map base_link 10 > "$RUN/tf_base.log" 2>&1 & PIDS="$PIDS $!"

say "== 步1b 归属令牌 UUID＋本窗 rosout pid N（★裁定认可的认亲法★）=="
UUID=$(basename "$(readlink -f "$LOGD/latest" 2>/dev/null)" 2>/dev/null)
RL="$LOGD/$UUID/roslaunch-$(hostname)-$WPID.log"
[ -f "$RL" ] || RL=$(ls -1t "$LOGD/$UUID"/roslaunch-*.log 2>/dev/null | head -1)
if [ -n "$UUID" ] && [ -f "$RL" ]; then
  N=$(grep -oE 'process\[rosout-1\]: started with pid \[[0-9]+\]' "$RL" | grep -oE '[0-9]+' | tail -1)
  say "  uuid 自校验＝OK ‖ UUID=$UUID ‖ 日志件=$(basename "$RL") ‖ 本窗 rosout pid N=$N"
  { echo "UUID=$UUID"; echo "WPID=$WPID"; echo "ROSOUT_PID=$N"; echo "ROSLAUNCH_LOG=$RL"; echo "时刻=$(date '+%F %T')"; } > "$RUN/OWNERSHIP.txt"
else
  N=""; say "  ★uuid 自校验失败 ⇒ 按裁定该窗判 INVALID、★不 kill 任何不在 $PIDS 里的进程★"; echo "UUID=自校验失败" > "$RUN/OWNERSHIP.txt"
fi

# run_seg <段名> <参数面> <图发布列表 mode:topic后缀 ...>
run_seg(){
  local NS="$1" SEGF="$2"; shift 2
  say "== 段 $NS（参数面＝params_$SEGF.yaml）=="
  for spec in "$@"; do
    local mode="${spec%%:*}" topic="${spec##*:}"
    python3 -u "$R/pub_grid_probe2.py" "$topic" "$mode" > "$RUN/pub_${topic}.log" 2>&1 & seg_pids="$seg_pids $!"
    say "  起图发布器 topic=/gmap_$topic mode=$mode"
  done
  sleep 3
  rosparam load "$RUN/params_$SEGF.yaml" "/$NS/costmap" || { say "  ★load_FAIL"; cleanup_seg; return 9; }
  say "  plugins 枚数回读＝$(rosparam get /$NS/costmap/plugins 2>&1 | grep -c 'name:')"
  rosrun costmap_2d costmap_2d_node __name:=$NS > "$RUN/node_$NS.log" 2>&1 & seg_pids="$seg_pids $!"
  local rdy=0 j
  for j in $(seq 1 40); do rostopic info "/$NS/costmap/costmap" >/dev/null 2>&1 && { rdy=1; break; }; sleep 1; done
  say "  costmap 就绪＝$rdy（按 rostopic info 成功返回判；等待 ${j}s）"
  [ "$rdy" != 1 ] && { say "  ★未就绪 ⇒ 该段记 N3"; cleanup_seg; return 8; }
  for spec in "$@"; do
    local topic="${spec##*:}"
    { echo "# rostopic info /gmap_$topic（★订阅者名册＝该层真订到图的在带内证据★）"; rostopic info "/gmap_$topic" 2>&1; } > "$RUN/roster_${NS}_${topic}.txt"
    say "  名册 /gmap_$topic ⇒ ★订阅者枚数=$(sed -n '/Subscribers/,$p' "$RUN/roster_${NS}_${topic}.txt" | grep -c '^ \*')★（$(sed -n '/Subscribers/,$p' "$RUN/roster_${NS}_${topic}.txt" | tail -n +2 | tr -d '\n*' | cut -c1-90)）"
  done
  { echo "# rostopic info 逐字"; rostopic info "/$NS/costmap/costmap" 2>&1; } > "$RUN/topics_$NS.txt"
  rostopic list 2>/dev/null | sort > "$RUN/alltopics_$NS.txt"
  timeout 45 rostopic echo -n 1 "/$NS/costmap/costmap" > "$RUN/cost_$NS.txt" 2>&1
  say "  取样完成 data_行数=$(grep -c 'data:' "$RUN/cost_$NS.txt") 字节=$(stat -c%s "$RUN/cost_$NS.txt" 2>/dev/null)"
  cleanup_seg; sleep 3
  return 0
}
run_seg w1 w1 "3state:w1"            ; A=$?
run_seg w2 w2 "3state:w2" "allunknown:el2" ; B=$?
run_seg w3 w3 "3state:w3" "alllethal:el3"  ; C=$?
say "== 段 rc 汇总＝w1=$A w2=$B w3=$C｜时长 $(( $(date +%s) - T0 ))s =="

say "== 步9 退净（★先礼后兵：SIGTERM wrapper → 轮询 rosout 消失 → 双过滤补救 → 判定行五数★）=="
for p in $PIDS; do kill "$p" 2>/dev/null; say "  SIGTERM 本窗 PID pid=$p（★禁 -9★）"; done
gone=0; for k in $(seq 1 10); do
  [ -z "$N" ] && { say "  N 未认到 ⇒ 不轮询（INVALID 分支）"; break; }
  if ! kill -0 "$N" 2>/dev/null; then gone=1; say "  ★第 ${k}s：本窗 rosout pid=$N 已消失★"; break; fi
  sleep 1
done
[ "$gone" != 1 ] && [ -n "$N" ] && say "  ★轮询 10s 未消失 pid=$N（★照实入册、★不强杀外来者）★"
AN=$(snap "$RUN/S1_after.tsv"); say "  退后快照 S1 计数=$AN"; sed 's/^/    S1｜/' "$RUN/S1_after.tsv" | tee -a "$R/00_window_log.txt"
awk -F'\t' 'NR==FNR{s[$1"|"$2]=1;next} !($1"|"$2 in s)' "$RUN/S0_before.tsv" "$RUN/S1_after.tsv" > "$RUN/D_delta.tsv"
DN=$(wc -l < "$RUN/D_delta.tsv")
: > "$RUN/K_kill.tsv"
while IFS=$'\t' read -r pid lstart comm args; do
  [ -n "${pid:-}" ] || continue
  case " $PIDS $seg_pids " in *" $pid "*) kill -TERM "$pid" 2>/dev/null; printf '%s\t%s\t%s\t%s\n' "$pid" "$lstart" "$comm" "$args" >> "$RUN/K_kill.tsv"; say "  K：pid=$pid（★∈本窗 PID 集★）SIGTERM"; continue;; esac
  if [ -n "${UUID:-}" ] && printf '%s' "$args" | grep -q -- "__log:=$LOGD/$UUID/"; then
    kill -TERM "$pid" 2>/dev/null; printf '%s\t%s\t%s\t%s\n' "$pid" "$lstart" "$comm" "$args" >> "$RUN/K_kill.tsv"; say "  K：pid=$pid comm=$comm（★带本窗 uuid＝本窗派生★）SIGTERM"
  else
    say "  ★跳过（非本窗 uuid／不在 PID 集）pid=$pid comm=$comm args=$(printf '%s' "$args" | cut -c1-90)"
  fi
done < "$RUN/D_delta.tsv"
KN=$(wc -l < "$RUN/K_kill.tsv")
sleep 2
DPK=$(awk -F'\t' 'NR==FNR{k[$1"|"$2]=1;next} !($1"|"$2 in k)' "$RUN/K_kill.tsv" "$RUN/D_delta.tsv" | wc -l)
for k in 1 2 3; do
  P=$(ss -lntp 2>/dev/null | grep -oE ":(11311|11312|11313)\b" | wc -l)
  U=$(ss -lnup 2>/dev/null | grep -oE ":(11311|11312|11313)\b" | wc -l)
  CN=$(snap "$RUN/witness_after${k}.tsv")
  say "  第${k}采五数＝基线=$BN｜D=$DN｜K=$KN｜D∖K=$DPK｜端口 TCP=$P UDP=$U（现读计数=$CN）"
  if [ "$CN" -le "$BN" ] && [ "$P" = "0" ] && [ "$U" = "0" ] && [ "$DN" -eq "$KN" ]; then say "  ★退净达标（乙式四合取项全成立）★"; break; fi
  [ "$DPK" != "0" ] && say "  ★D∖K≠∅ ⇒ 按裁定判 INVALID（★一个都不许多杀★）★"
  sleep 2
done
say "== 步10 三面 cost 对比（★决定性★）=="
python3 -u "$R/_cmp_three.py" "$RUN/cost_w1.txt" "$RUN/cost_w2.txt" "$RUN/cost_w3.txt" > "$R/05_three_face_cmp.txt" 2>&1
say "  对比件 rc=$?（★读数见 05 件★）"; tail -6 "$R/05_three_face_cmp.txt" | sed 's/^/    ｜/' | tee -a "$R/00_window_log.txt"
say "== 步11 roscore 日志在位否（★本窗要拿到的那句级联证据★）=="
say "  roscore.log 字节=$(stat -c%s "$RUN/roscore.log") ‖ killing 行数=$(grep -c 'killing on exit' "$RUN/roscore.log")"; grep -n "killing on exit" "$RUN/roscore.log" | sed 's/^/    ｜/' | tee -a "$R/00_window_log.txt"
say "== 步99 清盘与计数同条命令 =="
find /opt/ros1-slam -name '__pycache__' -type d -prune -exec rm -rf {} + 2>/dev/null
say "  pyc=$(find /opt/ros1-slam -name '*.pyc' | wc -l) ws_src=$(find /opt/ros1-slam/ws/src -type f | wc -l) tooling=$(find /opt/ros1-slam/tooling -type f | wc -l) note=$(find /opt/ros1-slam/note -type f | wc -l) ‖ 时长 $(( $(date +%s) - T0 ))s"
