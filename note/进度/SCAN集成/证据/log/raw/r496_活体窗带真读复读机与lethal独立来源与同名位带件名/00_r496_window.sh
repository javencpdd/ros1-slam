#!/usr/bin/env bash
# 00_r496_window.sh —— ★窗号 R496★（先报后开）＝★单块矩形靶活体窗★＝首次按 R465q 立的「C7-B 内容栏『致命』谓词钉」执行
#   被试体具名＝A 规范格点 列 476..534 × 行 523..543 ⇒ n_T=1239（★留底 P100 下 X_A=118、0.1n=123.9、余量 5.9 格＝11 枚合规块里最大者★）
#   三态＝A（flow 同形副本＝在册形语义等价基线）／Ap（同参数双跑＝每窗重量级 D_floor）／C（该层吃 /elev_map、喂★世界绝对矩形★）
#   机检＝grep -c '^STATE=' 应为 4（REF／A／Ap／C 同窗同 boot）；归属数同遍现取并带窗志行号
#   ★批146＝把 R494 否证到的四处形在活体上一次做对★＝①★两行复读都真 open 被引件★（`01_复读件_r496.py` → `07_复读.txt`，★留底不得是手抄常量★、并随读数打被引件 sha 与行数）
#   ②★lethal= 取两个独立来源★（图层计数 ‖ 发布数组逐格重数）＋★补齐在册模板缺的 robot=／band=★（TF 现查、查不到＝C 态 INVALID）
#   ③★同名机检位一律带件名★＝对照件那枚改名 rc_勾对_对照件 并声明不具 L-E 效力（真取外部物那枚在变体预检件里）
#   ④★机检式自带面条件★＝对照件与复读件的仓根都由 argv 传入（★不绑容器字面★）
#   ★旧四枚形仍在＝缺①活体复读栏／缺②lethal=／缺③勾对行＋rc_勾对／缺④申报行＋rc_申报★
#   ★对照件另修三枚已证伪的坏形＝对称差不除 2／rc_机检 真消费五枚合取项／不变量补盲区枚数★
#   在册四件一字不改、跑前跑后 sha＋行数自证；数字比较只用在册容差；退净按乙式；单跑 300s
# 姿势＝预检不过不起窗｜local 一行内不引用同行刚声明的变量｜清理与计数同条命令｜发布器用 SIGINT 不用 SIGTERM
set -u
R=/opt/ros1-slam/note/进度/SCAN集成/证据/log/raw/r496_活体窗带真读复读机与lethal独立来源与同名位带件名
RUN="$R/run"
DUMP="$RUN/dump"
CSV="$RUN/csv"
mkdir -p "$RUN" "$DUMP" "$CSV"
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
# ★重跑不串接★（批146 实犯：二次跑把两遍的 STATE 行叠成 8、申报行叠成 2 ⇒ 两枚机检位为 procedural 红）
if [ -f "$R/00_window_log.txt" ]; then mv "$R/00_window_log.txt" "$R/00_window_log_$(date +%H%M%S)_上一次跑.txt"; fi
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

SM_CP="$R/sim_nav_smoke_r496.sh"
Y_REG=/opt/ros1-slam/ws/src/40_navigation/lite3_navigation/config/global_costmap_params.yaml
M_REG=/opt/ros1-slam/ws/src/70_bringup/lite3_bringup/launch/mode_nav_sim.launch
B_REG=/opt/ros1-slam/ws/src/40_navigation/lite3_navigation/launch/mbf_navigation.launch
S_REG=/opt/ros1-slam/ws/src/70_bringup/lite3_bringup/verify/sim_nav_smoke.sh
Y_A="$R/global_costmap_params_A_r496.yaml"
Y_C="$R/global_costmap_params_C_r496.yaml"
KEYS=/move_base_flex/global_costmap

sha4(){
  local tag="$1"
  { echo "sha 前 16｜smoke=$(sha256sum "$S_REG" | cut -c1-16) mode=$(sha256sum "$M_REG" | cut -c1-16) mbf=$(sha256sum "$B_REG" | cut -c1-16) yaml=$(sha256sum "$Y_REG" | cut -c1-16)"
    echo "行数｜smoke=$(wc -l < "$S_REG") mode=$(wc -l < "$M_REG") mbf=$(wc -l < "$B_REG") yaml=$(wc -l < "$Y_REG")"; } > "$RUN/sha4_${tag}.txt"
}

say "== 步0 预检变体当刻（在册预检件一字未动）=="
bash /opt/ros1-slam/scripts/verify/v_window_preflight_x86_variant.sh > "$R/01_preflight.txt" 2>&1
PRC=$?
say "  预检 rc=$PRC"
[ "$PRC" != 0 ] && { say "★预检未过 ⇒ 不起窗"; exit 7; }
SEED=7
TRAJ=$(rospack find lite3_drivers 2>/dev/null)/config/traj_maze.yaml
[ -f "$TRAJ" ] || { say "★traj 不在位 ⇒ 不起窗（STATE 行不可信）★"; exit 9; }
say "  ★名集＝$NSZ 名｜STATE 常量＝seed=$SEED traj=$TRAJ master=$ROS_MASTER_URI★"
say "== 步0a 副本生成＋白名单逐行验＋在册四件跑前自证 =="
python3 "$R/02_make_copies_r496.py" /opt/ros1-slam >> "$R/00_window_log.txt" 2>&1
GRC=$?
BADALL=$(grep -o '★不合字面行数=[0-9]*★' "$R/03_delta.txt" | grep -o '[0-9]*' | awk '{t+=$1} END{print t+0}')
HITS=$(grep -o '★命中行数=[0-9]*★' "$R/03_delta.txt" | grep -o '[0-9]*' | awk '{t+=$1} END{print t+0}')
say "  生成器 rc=$GRC ‖ ★全链命中行数合计=$HITS、不合白名单行数合计=$BADALL★（明细在 03_delta.txt）"
[ "$GRC" = 0 ] || { say "★生成器失败 ⇒ 不起窗"; exit 9; }
sha4 before
say "  $(head -1 "$RUN/sha4_before.txt") ‖ $(tail -1 "$RUN/sha4_before.txt")"
say "== 步0b 起前快照 S0 =="
BN=$(snap "$RUN/S0_before.tsv")
say "  ★基线（起前 $NSZ 名计数）＝$BN★"
roscore -p 11313 > "$RUN/roscore.log" 2>&1 &
PIDS="$PIDS $!"
WPID=$!
ok=0; i=0
for i in $(seq 1 30); do rosnode list 2>/dev/null | grep -q rosout && { ok=1; break; }; sleep 1; done
say "== 步1 master_ready=$ok（等待 ${i}s）‖ ★本遍 WPID=$WPID（同遍现取）★=="
[ "$ok" != 1 ] && { say "★master 未起"; exit 8; }
rosrun tf static_transform_publisher 0 0 0 0 0 0 map base_link 10 > "$RUN/tf_base.log" 2>&1 &
PIDS="$PIDS $!"
UUID=$(basename "$(readlink -f "$LOGD/latest" 2>/dev/null)" 2>/dev/null)
RL="$LOGD/$UUID/roslaunch-$(hostname)-$WPID.log"
[ -f "$RL" ] || RL=$(ls -1t "$LOGD/$UUID"/roslaunch-*.log 2>/dev/null | head -1)
N=$(grep -oE 'process\[rosout-1\]: started with pid \[[0-9]+\]' "$RL" 2>/dev/null | grep -oE '[0-9]+' | tail -1)
say "== 步1b uuid 自校验＝$([ -n "$UUID" ] && [ -f "$RL" ] && echo OK || echo FAIL) ‖ 本遍 rosout pid=$N ‖ ★本遍 uuid=$UUID★=="
{ echo "UUID=$UUID"; echo "WPID=$WPID"; echo "ROSOUT_PID=$N"; echo "ROSLAUNCH_LOG=$RL"; } > "$RUN/OWNERSHIP.txt"

# run_state <态> <yaml 或 REG> <带靶 yes|no>；REF 态＝★在册形原样跑（不设 R496_YAML、用在册冒烟件与在册 yaml）★
run_state(){
  local tag="$1" yml="$2" withpub="$3"
  local out="$RUN/smoke_${tag}.txt" t0 rc sp el f s1=0 s2=0 sm="$SM_CP"
  if [ "$yml" = "REG" ]; then unset R496_YAML || true; sm="$S_REG"; else export R496_YAML="$yml"; fi
  printf 'STATE=%s master=%s seed=%s traj=%s load1=%s precheck=%s wpid=%s uuid=%s\n' "$tag" "$ROS_MASTER_URI" "$SEED" "$TRAJ" "$(cut -d' ' -f1 /proc/loadavg)" "$PRC" "$WPID" "$UUID" | tee -a "$R/00_window_log.txt"
  t0=$(date +%s)
  say "== 跑态 $tag（R496_YAML=${R496_YAML:-（★在册形・未设★）} ‖ 冒烟件＝${sm##*/} ‖ 超时 300s）=="
  timeout 300 bash "$sm" > "$out" 2>&1 &
  sp=$!
  if [ "$withpub" = "yes" ]; then
    python3 "$R/pub_rect_probe_r496.py" 170 > "$RUN/pub_${tag}.txt" 2>&1 &
    PUBPID=$!
    PIDS="$PIDS $PUBPID"
    say "  ★靶发布器随栈起动（pid=$PUBPID）★"
  fi
  while kill -0 "$sp" 2>/dev/null; do
    el=$(( $(date +%s) - t0 ))
    f=$(grep -c '^CHECK|' "$out" 2>/dev/null); [ -n "$f" ] || f=0
    printf '%s\t%s\t%s\t%s\n' "$tag" "$el" "$f" "$(grep '^CHECK|' "$out" 2>/dev/null | tail -1 | cut -d'|' -f2)" >> "$RUN/timeline.tsv"
    if [ "$s1" = "0" ] && [ "$f" -ge 5 ]; then
      s1=1
      rosparam dump "$DUMP/dump_${tag}_T1.yaml" "$KEYS" >/dev/null 2>&1
      timeout 90 python3 "$R/sub_costmap_dump_r496.py" "$CSV/cm_${tag}.csv" 80 > "$RUN/sample_${tag}.txt" 2>&1
      say "  ★T1 采样（第 ${el}s，已印 $f）★＝$(head -1 "$RUN/sample_${tag}.txt" | cut -c1-230)"
    fi
    if [ "$s2" = "0" ] && [ "$f" -ge 10 ]; then
      s2=1
      rosparam dump "$DUMP/dump_${tag}_T2.yaml" "$KEYS" >/dev/null 2>&1
      say "  ★T2 采样（第 ${el}s，已印 $f）＝自 T1 后二次 dump★"
    fi
    sleep 1
  done
  wait $sp; rc=$?
  say "  $tag：rc=$rc ‖ CHECK 行数=$(grep -c '^CHECK|' "$out") ‖ PASS=$(grep -c '^CHECK|[^|]*|PASS' "$out") ‖ FAIL=$(grep -c '^CHECK|[^|]*|FAIL' "$out") ‖ 用时≈$(( $(date +%s) - t0 ))s ‖ T1＝$([ -s "$DUMP/dump_${tag}_T1.yaml" ] && echo 在 || echo ★缺★) T2＝$([ -s "$DUMP/dump_${tag}_T2.yaml" ] && echo 在 || echo ★缺★) csv＝$([ -s "$CSV/cm_${tag}.csv" ] && echo 在 || echo ★缺★)"
  cp -f /tmp/sim_nav_smoke_launch.log "$RUN/roslaunch_${tag}.log" 2>/dev/null
  say "  ★C11（$tag）＝Using plugin 行=$(grep -c 'Using plugin' "$RUN/roslaunch_${tag}.log" 2>/dev/null) ‖ Received a 行=$(grep -c 'Received a' "$RUN/roslaunch_${tag}.log" 2>/dev/null) ‖ 含 elevation_layer 行=$(grep -c 'Using plugin "elevation_layer"' "$RUN/roslaunch_${tag}.log" 2>/dev/null)★"
  if [ "$withpub" = "yes" ]; then
    kill -INT "$PUBPID" 2>/dev/null
    local q=0
    for q in 1 2 3 4 5 6; do kill -0 "$PUBPID" 2>/dev/null || break; sleep 1; done
    say "  靶发布器 SIGINT 后第 ${q}s 已退＝$([ "$(kill -0 "$PUBPID" 2>/dev/null; echo $?)" != 0 ] && echo True || echo False)"
    PUBPID=""
  fi
}

say "== 步1c ★申报行=P100 ‖ 留底 X_A[P100]=118 ‖ 0.1n_T=123.9 ‖ 留底出处=note/进度/SCAN集成/证据/log/raw/r472_收口窗重跑规范架与出处随行与勾对表/05_cmp.txt:25（余量同件第 27 行、G1 同件第 66 行） ‖ 跨行不合成＝形 L-A1 ‖ G1 所报对象＝逐态 T 内 v==99 枚数（式引 r468q 第 77 行）★；★另一行 P90 同栏打印留底、不据此开窗也不据此判过★=="
run_state REF REG no
run_state A "$Y_A" no
run_state Ap "$Y_A" no
run_state C "$Y_C" yes

say "== 步9 退净 =="
for p in $PIDS; do kill "$p" 2>/dev/null; done
PIDS=""
N=$(grep -o '^ROSOUT_PID=[0-9]*' "$RUN/OWNERSHIP.txt" | cut -d= -f2)
for k in $(seq 1 12); do [ -z "$N" ] && break; kill -0 "$N" 2>/dev/null || { say "  ★第 ${k}s：本遍 rosout pid=$N 已消失★"; break; }; sleep 1; done
AN=$(snap "$RUN/S1_after.tsv")
awk -F'\t' 'NR==FNR{s[$1"|"$2]=1;next} !($1"|"$2 in s)' "$RUN/S0_before.tsv" "$RUN/S1_after.tsv" > "$RUN/D_delta.tsv"
DN=$(wc -l < "$RUN/D_delta.tsv")
: > "$RUN/K_kill.tsv"
while IFS=$'\t' read -r pid lstart comm args; do
  [ -n "${pid:-}" ] || continue
  if [ -n "${UUID:-}" ] && printf '%s' "$args" | grep -q -- "__log:=$LOGD/$UUID/"; then
    kill -INT "$pid" 2>/dev/null
    printf '%s\t%s\t%s\t%s\n' "$pid" "$lstart" "$comm" "$args" >> "$RUN/K_kill.tsv"
    say "  K：pid=$pid comm=$comm（★带本遍 uuid★）"
  else
    say "  ★跳过（非本遍 uuid）pid=$pid comm=$comm★"
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

say "== 步9b STATE 机检＋行号同报 =="
SAYN=$(grep -c '^STATE=' "$R/00_window_log.txt")
say "  ★grep -c '^STATE='＝$SAYN★（★本窗★四跑⇒应为 4★＝REF／A／Ap／C；★R458 那条＝四态同窗的旧达成、与本窗无关★）"
grep -n '^STATE=' "$R/00_window_log.txt" | sed 's/^/    行号｜/' | tee -a "$R/00_window_log.txt"

say "== 步10 对照件（八栏×两谓词）=="
python3 -u "$R/_cmp_r496.py" "$RUN/smoke_REF.txt" "$RUN/smoke_A.txt" "$RUN/smoke_Ap.txt" "$RUN/smoke_C.txt" "$DUMP" "$CSV" "$RUN" "$R/00_window_log.txt" /opt/ros1-slam > "$R/05_cmp.txt" 2>&1
CRC=$?
say "  对照件 rc=$CRC ‖ 输出行数=$(wc -l < "$R/05_cmp.txt")（★非空自证★）"
sed 's/^/    ｜/' "$R/05_cmp.txt" | tee -a "$R/00_window_log.txt"

say "== 步10b ★真读复读件（P100 与 P90 两行都复、留底侧真 open 被引件）★=="
python3 -u "$R/01_复读件_r496.py" /opt/ros1-slam "r472_收口窗重跑规范架与出处随行与勾对表" "$(basename "$R")" > "$R/07_复读.txt" 2>&1
FRC=$?
say "  复读件 rc=$FRC ‖ 输出行数=$(wc -l < "$R/07_复读.txt")（★非空自证★）"
grep -E 'rc_复读＝|rc_勾对_复读件|同值＝|真对称差|sha 前 16' "$R/07_复读.txt" | sed 's/^/    ｜/' | tee -a "$R/00_window_log.txt" | head -12
say "== 步11 在册四件跑后自证 =="
sha4 after
say "  跑前＝$(head -1 "$RUN/sha4_before.txt")"
say "  跑后＝$(head -1 "$RUN/sha4_after.txt")"
say "  ★四件逐字同值＝$([ "$(cat "$RUN/sha4_before.txt")" = "$(cat "$RUN/sha4_after.txt")" ] && echo True || echo False)★"
say "  roscore.log 字节=$(stat -c%s "$RUN/roscore.log") ‖ killing 行=$(grep -c 'killing on exit' "$RUN/roscore.log")"

say "== 步99 清盘与计数同条命令 =="
find /opt/ros1-slam -name '__pycache__' -type d -prune -exec rm -rf {} + 2>/dev/null
say "  pyc=$(find /opt/ros1-slam -name '*.pyc' | wc -l) ws_src=$(find /opt/ros1-slam/ws/src -type f | wc -l) tooling=$(find /opt/ros1-slam/tooling -type f | wc -l) note=$(find /opt/ros1-slam/note -type f | wc -l) ‖ 时长 $(( $(date +%s) - T0 ))s"
