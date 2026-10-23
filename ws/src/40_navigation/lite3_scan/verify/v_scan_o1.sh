#!/usr/bin/env bash
# v_scan_o1.sh —— O-1「到达信号口径」验收件（小明，2026-10-08）
# 判据表 A/B 出自小张设计件 note/进度/SCAN集成/设计/o1_到达信号与联锁设计.md §5。（★路径已迁★＝真身＝note/进度/v7/S1/SCAN集成/设计/o1_到达信号与联锁设计.md；R606／批258 按 L1 问一丁档处置＝加向前指针、★本行原文不回改★）
# 覆盖：J1 观测件（A-1 只读归属 / A-2 唯一发布者 / A-3 类型与命名 / 注入式功能四用例）
#       J2 参数单源（B-1 全名对账 / B-2 活体取值 / B-4 反双嵌套 / B-5 arg 联动 / B-6 死配置件已删+上游基线全等）
# 不覆盖（明确留 P3，不在本件造绿）：A-4 到达真值、A-6 双 goal 连发的运动学面 —— 需车真动，
#       而 P3 前置（J3 速度指令接 M6、F-D 的 /Odometry 双写者）未闭合。
# 隔离域：master 由 roslaunch 自启在 11313（**不得另起 roscore**：它会绑默认 11311 留孤儿 master，
#   本件首版即踩此坑，见 04 台账）；不启 Gazebo；生产 11312 零接触。
set -u
# ★R606／批258 核出、批259 落裁★＝原 ROOT 写死容器面路径，宿主面恒不可达；按 L1 问三裁甲改成 git 派生＋缺根即退（与 D7 件第 45 行同形）。
ROOT=$(git rev-parse --show-toplevel 2>/dev/null) || ROOT=/home/ros1-slam
PKG=$(rospack find lite3_scan)
BASE_DIR=$ROOT/note/进度/v7/S1/SCAN集成/证据
O=${O1_OUT:-/tmp/o1_out}
mkdir -p "$O"
export ROS_MASTER_URI=http://127.0.0.1:11313
PASS=0; FAIL=0
chk() { if [ "$1" = "0" ]; then echo "[PASS] $2"; PASS=$((PASS+1)); else echo "[FAIL] $2"; FAIL=$((FAIL+1)); fi; }
q() { timeout 15 "$@" 2>&1; }
pubs() { q rostopic info "$1" | awk '/^Publishers:/{f=1;next} /^Subscribers:/{f=0} f&&NF' | tr -d ' '; }
subs() { q rostopic info "$1" | awk '/^Subscribers:/{f=1;next} /^Publishers:/{f=0} f&&NF' | tr -d ' '; }

cleanup() {
  [ -n "${RL:-}" ] && kill $RL >/dev/null 2>&1
  sleep 2; pkill -f "rosmaster --core -p 11313" >/dev/null 2>&1
  pkill -f "lite3_scan/verify/scan_arrival_monitor.py" >/dev/null 2>&1
  pkill -f "topic_tools" >/dev/null 2>&1
  sleep 2
  R=$(pgrep -f 'roslaunch lite3_scan|rosmaster --core -p 1131|scan_planner_node|closed_loop_controller|scan_arrival_monitor|topic_tools' | tr '\n' ' ')
  echo "退净 残留:[$R]"
  echo "退净 端口 11311=$(ss -lnt 2>/dev/null | grep -c ':11311') 11312=$(ss -lnt 2>/dev/null | grep -c ':11312') 11313=$(ss -lnt 2>/dev/null | grep -c ':11313')"
  [ -z "$R" ] || echo "★ 退净不为空，本件判 FAIL 附加项"
}
trap cleanup EXIT

echo "### 一、静态判据（不起节点）"
L=$PKG/launch/70_scan.launch
roslaunch --dump-params "$L" > "$O/dump_default.txt" 2>"$O/dump_err.txt"; ER=$?
chk $ER "B-0 dump-params 可解析（stderr 首行: $(head -1 "$O/dump_err.txt" | cut -c1-60)）"
N=$(grep -c "^/closed_loop_controller/" "$O/dump_default.txt")
[ "$N" = "8" ]; chk $? "B-1 closed_loop_controller 全名键恰 8 条（实测 $N）"
for kv in "finish_dist: 0.15" "max_vy: 0.05" "max_vyaw: 1.0" "kp_pos: 0.8" "kp_yaw: 1.5" "time_forward: 0.8" "heading_error_threshold: 0.8"; do
  grep -qx "/closed_loop_controller/$kv" "$O/dump_default.txt"; chk $? "B-1 值对账 $kv"
done
grep -qx "/closed_loop_controller/max_vx: 0.4" "$O/dump_default.txt"; chk $? "B-1 max_vx == arg max_vel 缺省值"
D=$(grep -c "/closed_loop_controller/closed_loop_controller" "$O/dump_default.txt")
[ "$D" = "0" ]; chk $? "B-4 无双嵌套全名（命中 $D）"
roslaunch --dump-params "$L" max_vel:=0.55 2>/dev/null | grep -qx "/closed_loop_controller/max_vx: 0.55"
chk $? "B-5 max_vel:=0.55 联动到 max_vx（无第二源劫持）"
[ ! -f "$PKG/config/scan_params.yaml" ]; chk $? "B-6a 死配置件 config/scan_params.yaml 已不在位"
( cd "$ROOT/ws/src/30_3rd" && sha256sum -c "$BASE_DIR/scan_src_baseline_49.sha256" >/dev/null 2>&1 )
chk $? "B-6b 30_3rd 基线 49 件内容全等（advanced_param.xml 等上游件未被改动）"
CNT=$(find "$ROOT/ws/src/30_3rd" -type f | wc -l)
[ "$CNT" = "49" ]; chk $? "C-7 30_3rd 文件集合计数 = 49（sha256sum -c 只检清单内内容、对新增文件不敏感，故补此断言）"
echo "### 二、活体判据（SCAN 计算链自启 master 于 11313，不另起 roscore）"
roslaunch lite3_scan 70_scan.launch > "$O/launch.log" 2>&1 & RL=$!
sleep 24
[ "$(ss -lnt 2>/dev/null | grep -c ':11313')" -ge 1 ]; chk $? "I-0 master 自启并绑在 11313（隔离域仲裁位）"
[ "$(ss -lnt 2>/dev/null | grep -c ':11311')" = 0 ]; chk $? "I-1 11311 零监听（首版另起 roscore 留孤儿 master 的回归断言）"
[ "$(ss -lnt 2>/dev/null | grep -c ':11312')" = 0 ]; chk $? "I-2 生产 11312 零监听"
q rosnode list | grep -qx "/scan_arrival_monitor"; chk $? "A-1a 节点 /scan_arrival_monitor 在位"
for T in /lite3/scan/arrived /lite3/scan/arrival_dist; do
  P=$(pubs $T); M=$(echo "$P" | grep -c .)
  [ "$M" = "1" ] && echo "$P" | grep -q "/scan_arrival_monitor"; chk $? "A-2 $T 唯一发布者 = /scan_arrival_monitor（实测 $M 个: ${P:-无}）"
done
q rostopic info /lite3/scan/arrived | grep -qx "Type: std_msgs/Bool"; chk $? "A-3a arrived 类型 std_msgs/Bool"
q rostopic info /lite3/scan/arrival_dist | grep -qx "Type: std_msgs/Float32"; chk $? "A-3b arrival_dist 类型 std_msgs/Float32"
q rostopic list | grep -qE "^/scan_planner/(arrived|status|arrival_dist)$"; [ $? = 1 ]; chk $? "A-3c 我方信号未发在 /scan_planner/ 族下（归属可由话题名判定）"
q rostopic echo -n 1 /lite3/scan/arrived | grep -q "data: False"; chk $? "A-3d latch 初值 False（未拿 goal 前不得报到达）"
GS=$(subs /scan_planner/goal); echo "$GS" | grep -q "/scan_arrival_monitor"; chk $? "A-1b 观测件是 goal 的读者（订阅者: $GS）"
GP=$(pubs /scan_planner/goal | grep -c .)
[ "$GP" = "0" ]; chk $? "A-1c 观测件不写任何上游话题（goal 发布者 $GP 个）"
BS=$(subs /planning/bspline | grep -c "/scan_arrival_monitor")
[ "$BS" = "0" ]; chk $? "A-1d 观测件不订阅 bspline（与设计 §2.2 的偏差已在 04 登记：不消费即不订）"
[ "$(q rosparam get /closed_loop_controller/finish_dist)" = "0.15" ]; chk $? "B-2a 活体 finish_dist = 0.15"
[ "$(q rosparam get /scan_arrival_monitor/tol_arrive)" = "0.2" ]; chk $? "B-2b 活体 tol_arrive = 0.2"
TX=$(grep -c "AttributeError\|Traceback" "$O/launch.log" 2>/dev/null | head -1); TX=${TX:-0}
[ "$TX" = "0" ]; chk $? "A-1e 监测件运行期零异常（launch.log 里 Traceback/AttributeError 命中 $TX）"
echo "### 三、注入式功能用例（无车：假 goal + 假里程计）"
timeout 150 python3 -u "$PKG/verify/o1_arrival_probe.py" > "$O/probe.txt" 2>&1; PR=$?
sed 's/^/    /' "$O/probe.txt"
for k in case1_far case2_near case3_reset case4_break case4_reconverge; do
  grep -q "O1 $k=PASS" "$O/probe.txt"; chk $? "功能 $k"
done
[ "$PR" = "0" ]; chk $? "功能用例脚本 exit=$PR"
echo "### 汇总 PASS=$PASS FAIL=$FAIL"
[ $FAIL = 0 ] && echo "SCAN O1 VERDICT: PASS" || echo "SCAN O1 VERDICT: FAIL"
exit $(( FAIL != 0 ))
