#!/usr/bin/env bash
# 00_r521_window.sh —— ★窗号 R521★（先报后开）＝★单块矩形靶活体窗★＝首次按 R465q 立的「C7-B 内容栏『致命』谓词钉」执行
#   ★批164＝R521 落 r505q 给的其余四处件内改形★＝①合取五的唯一标记改成只由被检动作产生的 ASCII 记号（窗脚本退净行行尾打 TIDY5=OK、对照件的标签字面里不再出现它）②丁的边界写进代码字面（SWAP 负例⇒rc=5｜正测缺被引件⇒rc=9 前置缺失档）③清单出处件不在位那支并进退出码（9）④机检位报 0 时同报离窗复跑值＝步10a 拿同一份冻存件第二遍独立重跑对照件、并打两枚 rc 与逐行差枚数
#   被试体具名＝A 规范格点 列 476..534 × 行 523..543 ⇒ n_T=1239（★留底 P100 下 X_A=118、0.1n=123.9、余量 5.9 格＝11 枚合规块里最大者★）
#   三态＝A（flow 同形副本＝在册形语义等价基线）／Ap（同参数双跑＝每窗重量级 D_floor）／C（该层吃 /elev_map、喂★世界绝对矩形★）
#   机检＝grep -c '^STATE=' 应为 4（REF／A／Ap／C 同窗同 boot）；归属数同遍现取并带窗志行号
#   ★批146＝把 R494 否证到的四处形在活体上一次做对★＝①★两行复读都真 open 被引件★（`01_复读件_r521.py` → `07_复读.txt`，★留底不得是手抄常量★、并随读数打被引件 sha 与行数）
#   ②★lethal= 取两个独立来源★（图层计数 ‖ 发布数组逐格重数）＋★补齐在册模板缺的 robot=／band=★（TF 现查、查不到＝C 态 INVALID）
#   ③★同名机检位一律带件名★＝对照件那枚改名 rc_勾对_对照件 并声明不具 L-E 效力（真取外部物那枚在变体预检件里）
#   ④★机检式自带面条件★＝对照件与复读件的仓根都由 argv 传入（★不绑容器字面★）
#   ★旧四枚形仍在＝缺①活体复读栏／缺②lethal=／缺③勾对行＋rc_勾对／缺④申报行＋rc_申报★
#   ★对照件另修三枚已证伪的坏形＝对称差不除 2／rc_机检 真消费五枚合取项／不变量补盲区枚数★
#   在册四件一字不改、跑前跑后 sha＋行数自证；数字比较只用在册容差；退净按乙式；单跑 300s
# 姿势＝预检不过不起窗｜local 一行内不引用同行刚声明的变量｜清理与计数同条命令｜发布器用 SIGINT 不用 SIGTERM
set -u
R=/opt/ros1-slam/note/进度/SCAN集成/证据/log/raw/r521_活体窗按r517q缺席清单落六枚件内修_两支红档名由一枚码查表导出与独立来源从句改整枚data与盲区枚数带遍历域与复跑判据改除时刻行外逐行同与判读锚由件内现取与真目录撞名让路载体
RUN="$R/run"
DUMP="$RUN/dump"
CSV="$RUN/csv"
# ★甲＝让路先于任何建目录★（在册 W498 件这里有一句 mkdir、早于第 37—39 行让路 ⇒ 首遍必自造 run 可让）
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
RUNS=0
if [ -f "$R/.runs" ]; then RUNS=$(cat "$R/.runs"); fi
STAMP=$(date +%Y%m%d-%H%M%S.$(date +%N | cut -c1-6))   # ★丙＝戳到微秒（在册只到秒⇒同秒连跑两遍会静默覆盖让路件）★
# ★R521 改形③＝让路抽成一枚函数★（r511q 延二：R511 那版自证是★把判定式镜像重演一遍＋CLRC 硬赋值★、`$?` 在件里 0 枚、真让路那一圈没走过）
#   ⇒ 现在★同一段码★跑两圈：第一圈让开真目录、第二圈在 /tmp 的 scratch 上用同一枚 STAMP 撞名，★两圈的 rc 都从 $? 读回来★。
YIELD_LIST="00_window_log.txt 01_preflight.txt 03_delta.txt 05_cmp.txt 07_复读.txt run"
YIELDED=""
yield_prev(){
  local DDIR="$1"
  local ST="$2"
  local LEFT=""
  local f
  for f in $YIELD_LIST; do
    if [ -e "$DDIR/$f" ]; then
      if [ -e "$DDIR/.prev_${ST}_$f" ]; then echo "★撞名即停＝.prev_${ST}_$f 已在位、不静默覆盖★" >&2; return 6; fi
      mv "$DDIR/$f" "$DDIR/.prev_${ST}_$f"; LEFT="$LEFT $f"
    fi
  done
  YIELDED="$LEFT"
  return 0
}
SKIPPED=""
yield_prev "$R" "$STAMP"; YRC=$?
if [ "$YRC" != "0" ]; then echo "★让路撞名即停＝rc=$YRC⇒本窗不起、不静默覆盖★" >&2; exit 6; fi
SKIPPED="$YIELDED"
# ★自证（同一份函数、scratch、读 $?）★：先造一枚"上一遍遗留"⇒第一圈应 rc=0（真让路那一圈被走过）；同秒再跑一圈⇒应 rc=6（会停）
SCR=$(mktemp -d /tmp/r521_yield_XXXXXX)
mkdir -p "$SCR/run"
yield_prev "$SCR" "$STAMP"; Y1=$?
Y1L="$YIELDED"
# ★B1（r514q 缺席清单 B1）＝r514 那版第二圈进函数时源文件已被第一圈改名搬走 ⇒ 判定式根本不进入 ⇒ rc=0 是「没走到分支」不是「分支放行」
#   ⇒ 本批★第二圈前把源文件造回位★（touch）⇒ 撞名态真出现；★第二圈的 rc 紧跟调用现取★、★stderr 用命令替换捕获★（分支 echo 走 >&2、say 只 tee stdout ⇒ 不捕获就进不了志）
touch "$SCR/run"
Y2ERR=$(yield_prev "$SCR" "$STAMP" 2>&1 >/dev/null); Y2=$?
Y2HITS=$(printf '%s\n' "$Y2ERR" | grep -c '撞名即停')
rm -rf "$SCR"
mkdir -p "$RUN" "$DUMP" "$CSV"
BUILT="run/dump/csv"   # ★乙＝〈本遍自建〉另算一枚、不混进〈上一遍遗留〉★
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

SM_CP="$R/sim_nav_smoke_r521.sh"
Y_REG=/opt/ros1-slam/ws/src/40_navigation/lite3_navigation/config/global_costmap_params.yaml
M_REG=/opt/ros1-slam/ws/src/70_bringup/lite3_bringup/launch/mode_nav_sim.launch
B_REG=/opt/ros1-slam/ws/src/40_navigation/lite3_navigation/launch/mbf_navigation.launch
S_REG=/opt/ros1-slam/ws/src/70_bringup/lite3_bringup/verify/sim_nav_smoke.sh
Y_A="$R/global_costmap_params_A_r521.yaml"
Y_C="$R/global_costmap_params_C_r521.yaml"
KEYS=/move_base_flex/global_costmap

sha4(){
  local tag="$1"
  { echo "sha 前 16｜smoke=$(sha256sum "$S_REG" | cut -c1-16) mode=$(sha256sum "$M_REG" | cut -c1-16) mbf=$(sha256sum "$B_REG" | cut -c1-16) yaml=$(sha256sum "$Y_REG" | cut -c1-16)"
    echo "行数｜smoke=$(wc -l < "$S_REG") mode=$(wc -l < "$M_REG") mbf=$(wc -l < "$B_REG") yaml=$(wc -l < "$Y_REG")"; } > "$RUN/sha4_${tag}.txt"
}

say "  ★改形③＋B1 自证（★同一份函数真跑两圈、两圈的 rc 都紧跟调用现取★）＝scratch 第一圈 rc=$Y1（让开＝$Y1L） ‖ ★第二圈前已把源文件造回位★ ⇒ 同秒第二圈 rc=$Y2（★期望 6＝会停、不是静默覆盖★） ‖ ★分支命中枚数＝$Y2HITS（对 stderr 捕获串 grep '撞名即停'）★ ‖ ★stderr 原文（亲手搬进窗志）＝$Y2ERR★ ‖ ★该分支字面本走 stderr、不经搬运进不了窗志 ⇒ 志里数不到≠分支没执行★ ‖ 临时目录已删"
say "  ★B1 判读尺★＝第二圈 rc=6 且分支命中枚数=1 ⇒ ★让路真停第一次有窗内载体★；rc=0 或命中枚数=0 ⇒ 本批仍没喂到那条分支、按在册形只报不认"
say "== 步0 重跑让路★＝本遍＝第 $((RUNS+1)) 遍 ‖〈上一遍遗留〉已改名让路＝${SKIPPED:-（无・本目录首次跑）} ‖〈本遍自建〉＝${BUILT:-（无）}（★甲已修＝建目录在让路之后⇒这一枚不进让路清单★）‖ 遍次来源＝$R/.runs（★预检通过才自增★）★"
# ★S1（r517q 缺席清单 S1）★＝「撞名让路」此前只在★/tmp scratch★上走过、★真 $R 目录★上 0 枚载体 ⇒ 本步在★真目录★上跑两圈
#   ★清单临时只含一枚非承重探针件★＝六枚承重件（含 00_window_log.txt 与 run/）此刻★不在★让路清单里；守卫不过⇒当场 exit 8（★绝不允许把窗志搬走★）
#   ★命令替换里的函数对全局 YIELDED 的赋值不回父壳★⇒ 这一格★只报 rc 与命中、不报「让开枚数」★（追加一百零七⑤）
S1_SAVED_LIST="$YIELD_LIST"
YIELD_LIST="zz_probe.txt"
[ "$(printf "%s\n" $YIELD_LIST | wc -l)" = 1 ] || { echo "★S1 守卫不过＝让路清单不止一枚★" >&2; exit 8; }
case " $YIELD_LIST " in *" run "*|*" 05_cmp.txt "*|*" 00_window_log.txt "*) echo "★S1 守卫不过＝让路清单里还有承重件＝当场中止★" >&2; exit 8;; esac
touch "$R/zz_probe.txt"
S1A_ERR=$(yield_prev "$R" "$STAMP" 2>&1 >/dev/null); S1A=$?
S1A_HITS=$(printf "%s\n" "$S1A_ERR" | grep -c "撞名即停")
touch "$R/zz_probe.txt"
S1B_ERR=$(yield_prev "$R" "$STAMP" 2>&1 >/dev/null); S1B=$?
S1B_HITS=$(printf "%s\n" "$S1B_ERR" | grep -c "撞名即停")
S1B_HITSO=$(printf "%s\n" "$S1B_ERR" | grep -o "撞名即停" | wc -l)
S1PREV=$(ls -a "$R" | grep -c "^\.prev_.*zz_probe")
YIELD_LIST="$S1_SAVED_LIST"
[ -f "$R/00_window_log.txt" ] && [ -d "$R/run" ] || { echo "★S1 后检不过＝承重件被让路函数搬走了★" >&2; exit 8; }
say "  ★S1 真目录两圈（目录＝$R 本身 ‖ 清单临时只含 zz_probe.txt ‖ STAMP 同枚）★＝第一圈 rc=$S1A（命中行=$S1A_HITS・★期望 0＝真让路★） ‖ ★第二圈 rc=$S1B（命中行=$S1B_HITS ‖ 命中次=$S1B_HITSO・期望 6／1＝真停★）"
say "  ★S1 判读尺★＝第二圈 rc=6 ⇒ ★撞名让路在真 $R 目录上第一次有载体★；rc=0 ⇒ 本窗仍未走到那条分支＝★延二那一格继续半填、枚名不摘★"
say "  ★S1 stderr 原文（亲手搬进窗志）＝$S1B_ERR ‖ 盘上留证＝$S1PREV 枚 .prev_*_zz_probe.txt（★留在本目录作载体、不删★） ‖ 清单已还原＝$YIELD_LIST"
say "  ★S1 结构上限现测★＝★命中枚数恒 ≤1★（件内首枚撞名即 return 6 ⇒ 循环不继续）＝这枚是布尔、不是计数；★行数口径与次数口径随句并报、不得互顶★"
say "  ★S1 两道守卫★＝①让路清单临时只含★一枚★探针件（不止一枚⇒exit 8）②★跑完后窗志与 run/ 仍在位★（被动走⇒exit 8）＝走到下面这一行＝★两枚守卫都过★"
say "== 步0 预检变体当刻（在册预检件一字未动）=="
bash /opt/ros1-slam/scripts/verify/v_window_preflight_x86_variant.sh > "$R/01_preflight.txt" 2>&1
PRC=$?
say "  预检 rc=$PRC"
[ "$PRC" != 0 ] && { say "★预检未过 ⇒ 不起窗"; exit 7; }
echo $((RUNS+1)) > "$R/.runs"   # ★B＝自增移到预检之后⇒「第 N 遍」只数真起过的窗（W498 在预检前自增＝含未起窗的尝试）★
SEED=7
TRAJ=$(rospack find lite3_drivers 2>/dev/null)/config/traj_maze.yaml
[ -f "$TRAJ" ] || { say "★traj 不在位 ⇒ 不起窗（STATE 行不可信）★"; exit 9; }
say "  ★名集＝$NSZ 名｜STATE 常量＝seed=$SEED traj=$TRAJ master=$ROS_MASTER_URI★"
say "== 步0a 副本生成＋白名单逐行验＋在册四件跑前自证 =="
python3 "$R/02_make_copies_r521.py" /opt/ros1-slam >> "$R/00_window_log.txt" 2>&1
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
# ★B4（r514q 缺席清单 B4）＝r514 那版「步1a2 开窗前闸式自测」排在 roscore 之后（见本件下方 roscore 那行）⇒「开窗前」相对 master 不成立
#   ⇒ 本批整块★前移到 roscore 之前★、标题改「步0a2 四态首跑之前、且 roscore 尚未起」、★rc≠0 则不起 master★
DT="$R/../../../_prewindow_drytest.sh"
if [ -f "$DT" ]; then
  DTOUT=$(bash "$DT" "$R/_cmp_r521.py" 2>&1); DTRC=$?
  DTSAM=$(printf '%s\n' "$DTOUT" | grep -oE '不合枚数＝[0-9]+／共 [0-9]+ 枚样本' | tail -1)
  say "== 步0a2 ★闸式自测＝四态首跑之前、且 roscore 尚未起★＝本件 rc=$DTRC ‖ $DTSAM（★0／N＝式子能区分真值行与反例行；被试件 sha 由该件自带★）=="
else
  say "== 步0a2 ★闸式自测＝采集件不在位（$DT）⇒ 本枚记★未证★、不假装跑过=="
  DTRC=9
fi
if [ "${DTRC:-9}" != "0" ]; then say "★闸式自测未过（rc=$DTRC）⇒ 不起 master、不起窗★"; exit 7; fi
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

# run_state <态> <yaml 或 REG> <带靶 yes|no>；REF 态＝★在册形原样跑（不设 R521_YAML、用在册冒烟件与在册 yaml）★
run_state(){
  local tag="$1" yml="$2" withpub="$3"
  local out="$RUN/smoke_${tag}.txt" t0 rc sp el f s1=0 s2=0 sm="$SM_CP"
  if [ "$yml" = "REG" ]; then unset R521_YAML || true; sm="$S_REG"; else export R521_YAML="$yml"; fi
  printf 'STATE=%s master=%s seed=%s traj=%s load1=%s precheck=%s wpid=%s uuid=%s\n' "$tag" "$ROS_MASTER_URI" "$SEED" "$TRAJ" "$(cut -d' ' -f1 /proc/loadavg)" "$PRC" "$WPID" "$UUID" | tee -a "$R/00_window_log.txt"
  t0=$(date +%s)
  say "== 跑态 $tag（R521_YAML=${R521_YAML:-（★在册形・未设★）} ‖ 冒烟件＝${sm##*/} ‖ 超时 300s）=="
  timeout 300 bash "$sm" > "$out" 2>&1 &
  sp=$!
  if [ "$withpub" = "yes" ]; then
    python3 "$R/pub_rect_probe_r521.py" 170 > "$RUN/pub_${tag}.txt" 2>&1 &
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
      timeout 90 python3 "$R/sub_costmap_dump_r521.py" "$CSV/cm_${tag}.csv" 80 > "$RUN/sample_${tag}.txt" 2>&1
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
    timeout 45 python3 "$R/03_sub_elev_count.py" > "$RUN/sub_elev_C.txt" 2>&1
    SSC=$?
    say "  ★真第二来源（订阅侧回读 /elev_map）★ rc=$SSC ‖ $(grep -o 'SECSUB.*' "$RUN/sub_elev_C.txt")"
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
say "  ★判定行五数＝基线=$BN｜D=$DN｜K=$KN｜D∖K=$DPK｜端口 TCP=$TCP UDP=$UDP★（现读 $NSZ 名计数=$CN） TIDY5=OK"
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
python3 -u "$R/_cmp_r521.py" "$RUN/smoke_REF.txt" "$RUN/smoke_A.txt" "$RUN/smoke_Ap.txt" "$RUN/smoke_C.txt" "$DUMP" "$CSV" "$RUN" "$R/00_window_log.txt" /opt/ros1-slam > "$R/05_cmp.txt" 2>&1
CRC=$?
say "  对照件 rc=$CRC ‖ 输出行数=$(wc -l < "$R/05_cmp.txt")（★非空自证★）"
sed 's/^/    ｜/' "$R/05_cmp.txt" | tee -a "$R/00_window_log.txt"

say "== 步10a ★R521 改形④＝机检位报 0 时同报离窗复跑值★（同一份冻存件、第二遍独立重跑对照件）=="
python3 -u "$R/_cmp_r521.py" "$RUN/smoke_REF.txt" "$RUN/smoke_A.txt" "$RUN/smoke_Ap.txt" "$RUN/smoke_C.txt" "$DUMP" "$CSV" "$RUN" "$R/00_window_log.txt" /opt/ros1-slam > "$R/05_cmp_复跑.txt" 2>&1
CRC2=$?
DIFFN=$(diff "$R/05_cmp.txt" "$R/05_cmp_复跑.txt" 2>/dev/null | grep -c "^[<>]")
# ★S3（r517q 缺席清单 S3）＝本件把★遍次与时刻★打进自己的输出 ⇒ 两遍 sha★必然不同★、「sha 相同＝真同值」这枚判据★永不为真★
#   ⇒ 判据换成这一枚＝★除时刻行外逐行差★，并把★时刻行的行位★随句报出（差在哪一行不再靠人眼找）
TSROW=$(grep -n '★遍次（件自身带）' "$R/05_cmp.txt" | cut -d: -f1 | paste -sd, -)
TSROW2=$(grep -n '★遍次（件自身带）' "$R/05_cmp_复跑.txt" | cut -d: -f1 | paste -sd, -)
DIFFX=$(diff <(grep -v '★遍次（件自身带）' "$R/05_cmp.txt") <(grep -v '★遍次（件自身带）' "$R/05_cmp_复跑.txt") 2>/dev/null | grep -c "^[<>]")
say "  对照件第一遍 rc=$CRC ‖ ★离窗复跑第二遍 rc=$CRC2★ ‖ 两遍逐行差＝$DIFFN 行（★行位口径＝diff 两侧各计一行★） ‖ ★S3/R521＝除时刻行外逐行差＝$DIFFX 行★（★判据取这一枚；sha 相同这枚永不为真★） ‖ 时刻行行位＝第一遍[$TSROW] ‖ 第二遍[$TSROW2]（★两枚行位相同才算差只长在时刻串上、不长在别处★）"
# ★R521 改形⑧＝离窗复跑两遍必须★同句★并报「件自身 sha」与「输入件 sha」（r508q Q2 缺口那格：两遍同崩时逐行差也是 0）
LOGSHA_BEFORE=$(sha256sum "$R/00_window_log.txt" 2>/dev/null | cut -c1-16)
LOGLN_BEFORE=$(wc -l < "$R/00_window_log.txt")
# ★A4（r514q 缺席清单 A4）＝r514 那版两枚 sha 之间★没有任何写窗志的动作★ ⇒ 同一瞬间对同一份字节的两次读、同值恒真 ⇒ 标签「跑前／跑后」名实不符
#   ⇒ 本批在两读之间★真写一次窗志★，使两枚分处两次写入的两端；并同句带★该件此刻行数★（r511q:86 点名那枚）
say "  ★A4 插桩＝两枚窗志 sha 之间★真写一次窗志★（写入前窗志行数＝$LOGLN_BEFORE）＝★不插这一句则两读落在同一瞬间★"
SELF_SHA1=$(sha256sum "$R/05_cmp.txt" 2>/dev/null | cut -c1-16); SELF_SHA2=$(sha256sum "$R/05_cmp_复跑.txt" 2>/dev/null | cut -c1-16)
INP_SHA=$(sha256sum "$R/00_window_log.txt" 2>/dev/null | cut -c1-16); CMPJ_SHA=$(sha256sum "$R/_cmp_r521.py" 2>/dev/null | cut -c1-16)
# ★R521 改形⑤＝窗志那枚 sha 是★时点读数★（r511q 新三：志内 978725ec… 事后不可复现）⇒ 现在★跑前／跑后各取一枚、并同句写明它只代表那一刻★
LOGLN_AFTER=$(wc -l < "$R/00_window_log.txt")
say "  ★改形⑤＋A4 窗志 sha 两枚＝跑前 sha16=$LOGSHA_BEFORE（该刻行数＝$LOGLN_BEFORE） ‖ 跑后 sha16=$INP_SHA（该刻行数＝$LOGLN_AFTER） ‖ ★两枚不同＝$([ \"$LOGSHA_BEFORE\" != \"$INP_SHA\" ] && echo True || echo False)★ ‖ ★随句带行数＝r511q:86 点名的那枚★（★两枚都只表示各自那一刻、事后不可复现；凡引用它必须带这句★）"

say "  ★改形⑧四枚同句＝第一遍件 sha16=$SELF_SHA1 ‖ 复跑件 sha16=$SELF_SHA2 ‖ 输入（本窗志）sha16=$INP_SHA ‖ 被试件（对照件本体）sha16=$CMPJ_SHA‖两遍 rc＝$CRC/$CRC2‖逐行差＝$DIFFN★（★两遍输出 sha 相同＝真同值；不同＝看差在哪；两遍 rc 相同而 sha 也相同＋无异常才算交叉验证成立★）"
say "== 步10c ★S10/R521＝判读锚由件内现取★（手写锚错过一次：八栏被抄成第 32—35 行）⇒ 下面这些行号由★本窗件★打出，判读面只准抄这里、不准凭记忆写★★"
for pat in "──【P100 行】" "──【P90 行】" "G1 闸" "判词" "注：皆致命" "★遍次（件自身带）"; do
  CROW=$(grep -n -F "$pat" "$R/05_cmp.txt" 2>/dev/null | cut -d: -f1 | paste -sd, -)
  LROW=$(grep -n -F "$pat" "$R/00_window_log.txt" 2>/dev/null | cut -d: -f1 | paste -sd, -)
  say "  锚＝$pat ‖ 05_cmp.txt 行号＝${CROW:-（无）} ‖ 窗志行号＝${LROW:-（无）}（★两枚载体不互顶、各报各的★）"
done
if [ "$DIFFN" != "0" ]; then diff "$R/05_cmp.txt" "$R/05_cmp_复跑.txt" | head -14 | sed 's/^/    差｜/' | tee -a "$R/00_window_log.txt"; fi

say "== 步10a2 ★R521 改形④＝SWAP 负例在★窗内★跑一遍★（r511q 延一：证件有 rc、窗志无那一遍）★=="
python3 -u "$R/01_复读件_r521.py" /opt/ros1-slam "r472_收口窗重跑规范架与出处随行与勾对表" "$(basename "$R")" --swap > "$R/08_复读_swap负例.txt" 2>&1
SRC=$?
say "  ★SWAP 遍 进程 rc 回读＝$SRC（★这枚数是从 $? 读回来的、不是打印串里的自陈★） ‖ 证件内自陈行＝$(grep -c '即将以 rc=5 退出' "$R/08_复读_swap负例.txt") 枚 ‖ 该件不进任何在册件的 rc★"
grep -E '即将以 rc=5 退出|被引件不在位' "$R/08_复读_swap负例.txt" | sed 's/^/    ｜/' | tee -a "$R/00_window_log.txt"
say "== 步10b ★真读复读件（P100 与 P90 两行都复、留底侧真 open 被引件）★=="
python3 -u "$R/01_复读件_r521.py" /opt/ros1-slam "r472_收口窗重跑规范架与出处随行与勾对表" "$(basename "$R")" > "$R/07_复读.txt" 2>&1
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
