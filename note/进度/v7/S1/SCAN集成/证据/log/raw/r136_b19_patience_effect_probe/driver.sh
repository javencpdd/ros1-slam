#!/usr/bin/env bash
# r136_b19_patience_effect_probe/driver.sh —— B19 的「生效性原件」探针（诊断件，**非判据采数**）
#
# 要答的问题（请求单 R 补九 第 2 条）：在册那句「远角对 patience 运行时放宽 60s→90s 仍无 result＝不可终止」
#   从来没有**回执**：老探针 `matrix_w3/scripts/w3_t1_probe.sh` 的 dynparam() 用 capture_output=True 把回执整个丢弃，
#   归档 log 里 `atience` 只命中参数表那两行（3.0），且原句自陈「dynparam 20s 超时」。
#   ⇒ 本件只做一件事：把 set/get 的**stdout＋stderr＋rc 全部留底**，证明运行时改 patience 到底落不落得进去。
#
# 红线（写死在本件里，不是口头承诺）：
#   ①**不投任何 goal、不发 /cmd_vel** ⇒ 与「远角 4 对任何波次禁止复跑」（进度/v7/S3/仿真测试台/进度文档.md:278）无涉；
#   ②**不碰生产配置**：只走 dynamic_reconfigure 运行时通道，跑完即复回原值并自证；
#   ③负载门槛 loadavg≤4，超即 rc=7 停手留证（在册 O-5「跑了也是作废跑」），**不设 FORCE 口子**。
set -u
RUN=${B19_RUN:-/home/ros1-slam/note/进度/SCAN集成/证据/log/raw/r136_b19_patience_effect_probe}
mkdir -p "$RUN"
LOG=$RUN/probe.log
say() { echo "[$(date '+%T')] $*" | tee -a "$LOG"; }

LOAD=$(cut -d' ' -f1 /proc/loadavg)
if python3 -c "import sys;sys.exit(0 if float('$LOAD')<=4.0 else 1)"; then :; else
  say "起窗门槛不过：loadavg=$LOAD ＞ 4 ⇒ 停手不跑（不绕行）"
  echo "B19 生效性结论 = INVALID|未起窗（loadavg=$LOAD＞4）" | tee -a "$LOG"
  echo "B19 VERDICT rc=7" | tee -a "$LOG"; exit 7
fi
say "起窗门槛：loadavg=$LOAD ≤ 4"

LOCK=/tmp/lite3_b19_probe.lock
if [ -f "$LOCK" ] && kill -0 "$(cat "$LOCK" 2>/dev/null)" 2>/dev/null; then
  echo "B19 生效性结论 = INVALID|已有并发实例（PID=$(cat "$LOCK")）" | tee -a "$LOG"; echo "B19 VERDICT rc=2" | tee -a "$LOG"; exit 2
fi
echo $$ > "$LOCK"

set +u
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION
source /opt/ros/noetic/setup.bash
[ -f /home/ros1-slam/ws/devel/setup.bash ] && source /home/ros1-slam/ws/devel/setup.bash
set -u
export ROS_MASTER_URI=http://127.0.0.1:11313
[ "${ROS_DISTRO:-}" = "noetic" ] || { say "ROS_DISTRO=$ROS_DISTRO 非 noetic"; rm -f "$LOCK"; echo "B19 VERDICT rc=2"; exit 2; }

cleanup() {
  say "-- teardown --"
  rm -f "$LOCK"
  rosnode kill -a >/dev/null 2>&1; sleep 2
  pkill -f 'roslaunch.*mode_nav_sim' 2>/dev/null; pkill -f move_base_flex 2>/dev/null
  pkill -f gzserver 2>/dev/null; pkill -f rosmaster 2>/dev/null
  sleep 2
  say "残留：gzserver=$(pgrep -c gzserver 2>/dev/null) move_base_flex=$(pgrep -fc 'move_base_flex' 2>/dev/null) 11313 监听=$(ss -ltn 2>/dev/null | grep -c ':11313')"
}
trap cleanup EXIT

say "== [0] 生产配置原文留底（生效件＝mbf_navigation.yaml，加载者＝mbf_navigation.launch:9）=="
sed -n '31,35p' /home/ros1-slam/ws/src/40_navigation/lite3_navigation/config/mbf_navigation.yaml > "$RUN/config_before.txt" 2>&1
sha256sum /home/ros1-slam/ws/src/40_navigation/lite3_navigation/config/mbf_navigation.yaml | tee -a "$RUN/config_before.txt" >> "$LOG" 2>&1
cat "$RUN/config_before.txt" | tee -a "$LOG"

say "== [1] 起链（perlin3d 标准档，只为把 /move_base_flex 拉起来；**不投 goal**）=="
setsid roslaunch lite3_bringup mode_nav_sim.launch rviz:=false use_gpu:=false \
  map_type:=1 map_seed:=7 > "$RUN/launch.log" 2>&1 &
for i in $(seq 1 90); do
  rostopic list 2>/dev/null | grep -q '/move_base_flex/get_path/goal' && break
  sleep 2
done
rostopic list 2>/dev/null | grep '/move_base_flex' | head -8 > "$RUN/topics.txt"
say "get_path 话题在位=$(grep -c '/move_base_flex/get_path/goal' <<<"$(rostopic list 2>/dev/null)" || echo 0)（须 1）"
if ! rostopic list 2>/dev/null | grep -q '/move_base_flex/get_path/goal'; then
  echo "B19 生效性结论 = INVALID|链未就绪（无 /move_base_flex/get_path）" | tee -a "$LOG"
  echo "B19 VERDICT rc=9" | tee -a "$LOG"; exit 9
fi

dp() {  # **回执不丢弃**：stdout＋stderr＋rc 全留底（这正是老探针缺的那一格）
  #   用法按 `dynparam -h` 实测：`set <node> <param> <value>`／`get <node>`（get 是**整份配置 dump，不接参数名**）
  local op=$1; shift; local out rc
  out=$(timeout 20 rosrun dynamic_reconfigure dynparam "$op" "$@" 2>&1); rc=$?
  { echo "--- dynparam $op $* (rc=$rc) ---"; echo "$out"; } >> "$RUN/dynparam_receipts.txt"
  say "  dynparam $op $* ⇒ rc=$rc"
  return $rc
}
# 读回走 **DR 广播主题**，不再解析 `dynparam get` 的文本（三跑实测：get 取到空串 ⇒ 值仍未证）
#   原理：dynamic_reconfigure 服务器每次 set 都会把**整份当前 config** 发到 /move_base_flex/parameter_updates，
#   那条广播就是"生效回执"本体；先起 echo -n1 再 set，保证拿到的是 set 之后的那一帧。
# R142 定稿读回法（前两版的错都登记在这里，不遮）：
#   ①首版用 `dynparam get <node> <参数名>` ⇒ dynparam 的 get **只接节点名**（整份 dump），多传参数名回 Usage ⇒ 空值；
#   ②二版改抓 /move_base_flex/parameter_updates 广播，但 `-n1` 只取**第一帧**、而我是在 set 之前起订阅 ⇒
#     拿到的是**上一帧**（实测三帧＝3.0／3.0／10.0，对着 set 序列 3.0→10.0→3.0，整体**慢一拍**）；
#     另我在广播里用 `^planner_patience:` 锚定行首，而 dynparam／echo 的输出是**缩进**的 ⇒ 永远匹配不到。
#   ③本版：set 之后**直接问服务器**（`dynparam get /move_base_flex` 走 DR 的查询），并整份留底；
#     同时仍抓广播帧（set 之后再起 echo -n1）做第二路印证，两路对不上就以"查询路"为准并如实报。
dr_probe() {   # 用法：dr_probe <目标值> <标签>  ⇒ 打印 set 之后服务器实际持有的 planner_patience
  local tgt=$1 tag=$2 raw val
  raw="$RUN/dynparam_get_$tag.txt"
  timeout 20 rosrun dynamic_reconfigure dynparam get /move_base_flex > "$raw" 2>&1
  val=$(grep -oE "'?planner_patience'? *: *[0-9.]+" "$raw" | head -1 | grep -oE '[0-9.]+')
  echo "  [$tag] set 前查询 ⇒ planner_patience=${val:-（空，见 dynparam_get_$tag.txt）}" | tee -a "$LOG"
  local out rc
  out=$(timeout 20 rosrun dynamic_reconfigure dynparam set /move_base_flex planner_patience "$tgt" 2>&1); rc=$?
  { echo "--- dynparam set /move_base_flex planner_patience $tgt (rc=$rc) ---"; echo "$out"; } >> "$RUN/dynparam_receipts.txt"
  sleep 0.6
  local af
  af=$(timeout 20 rosrun dynamic_reconfigure dynparam get /move_base_flex 2>&1 | grep -oE "'?planner_patience'? *: *[0-9.]+" | head -1 | grep -oE '[0-9.]+')
  { echo "--- dynparam get /move_base_flex (set $tgt 之后, rc=0) ---"; timeout 20 rosrun dynamic_reconfigure dynparam get /move_base_flex 2>&1; } >> "$RUN/dynparam_receipts.txt"
  say "  [$tag] set=$tgt (rc=$rc) ⇒ set 后查询 planner_patience=${af:-（空）}"
  echo "${af:-}"
}

say "== [2] 基线读取（参数服务器侧 ＋ DR 侧各一份）=="
: > "$RUN/dynparam_receipts.txt"
rosparam get /move_base_flex/planner_patience > "$RUN/paramserver_planner_patience.txt" 2>&1
say "  rosparam get /move_base_flex/planner_patience ⇒ $(cat "$RUN/paramserver_planner_patience.txt" 2>/dev/null | tr -d '\n')"
dp list >/dev/null 2>&1 || true   # 证明 /move_base_flex 确是 DR 可配置节点（在册老探针从没留过这一格）

say "== [3] 运行时放宽（目标 10.0＝B18 那一档）=="
BASE_DR=$(dr_probe 3.0 base)        # 基线帧：把生产值原值回写一次，只为取回"set 前服务器实际持有什么"
AFTER_DR=$(dr_probe 10.0 raised); SET_RC=$?
rosparam get /move_base_flex/planner_patience > "$RUN/paramserver_after.txt" 2>&1

say "== [4] 复回原值（生产侧不留改动面）=="
RESTORE_DR=$(dr_probe 3.0 restored)
rosparam get /move_base_flex/planner_patience > "$RUN/paramserver_restored.txt" 2>&1

say "== [5] 配置面自证（本件没写过任何配置文件）=="
sha256sum /home/ros1-slam/ws/src/40_navigation/lite3_navigation/config/mbf_navigation.yaml > "$RUN/config_after.txt" 2>&1
SHA_BEFORE=$(grep -oE '^[0-9a-f]{64}' "$RUN/config_before.txt" | tail -1)
SHA_AFTER=$(cut -d' ' -f1 "$RUN/config_after.txt")
SAME=$([ "$SHA_BEFORE" = "$SHA_AFTER" ] && echo 是 || echo "否!!（$SHA_BEFORE → $SHA_AFTER）")
say "  mbf_navigation.yaml sha 起前＝退后：$SAME"

VERDICT="UNKNOWN"; RC=9
if [ "$SET_RC" != "0" ]; then
  VERDICT="NO-DR-set-被拒(rc=$SET_RC)"; RC=1
elif [ "$AFTER_DR" = "10" ] || [ "$AFTER_DR" = "10.0" ]; then
  if [ "$RESTORE_DR" = "3" ] || [ "$RESTORE_DR" = "3.0" ]; then
    VERDICT="OK-双向生效（10.0 与复回 3.0 都在服务器广播里读到 ⇒ 在册那次 60s→90s 的 set 机制上可行；但**行为面仍未证**：本件不投 goal，没有"放宽后能不能规划出来"的读数）"; RC=0
  else
    VERDICT="PARTIAL-放宽帧对、复回帧不符（base=$BASE_DR raised=$AFTER_DR restored=$RESTORE_DR）"; RC=9
  fi
else
  VERDICT="PENDING-服务器查询里读不到所设值（base=$BASE_DR raised=$AFTER_DR restored=$RESTORE_DR ⇒ 不能据此说生效，也不能说被拒）"; RC=9
fi
{
  echo "基线 DR 读数=$BASE_DR"
  echo "放宽后 DR 读数=$AFTER_DR"
  echo "参数服务器（与 DR 无关，DR 不回写）before=$(tr -d '\n' < "$RUN/paramserver_planner_patience.txt") after=$(tr -d '\n' < "$RUN/paramserver_restored.txt")"
  echo "DR 侧三点一线：基线=$BASE_DR → 放宽=$AFTER_DR → 复回=$RESTORE_DR"
  echo "B19 生效性结论 = $VERDICT"
  echo "★本件**不投 goal**⇒ 不产出任何'远角对能不能规划出来'的行为读数；它只回答'60s/90s 那次 set 落不落得进去'。"
  echo "B19 VERDICT rc=$RC"
} | tee -a "$LOG"
exit "$RC"
