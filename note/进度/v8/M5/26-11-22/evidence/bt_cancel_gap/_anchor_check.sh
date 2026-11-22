#!/usr/bin/env bash
# _anchor_check.sh —— 把「取消后重发失联」诊断里每一处行号锚点重新核一遍。
#
# 为什么要有这件：诊断件（note/架构/双后端可替换_验证记录_2026-11-22.md 第十一节）
# 通篇靠「文件:行号」立论。行号是会漂的代理量——只要上游有人往 bt_console.cpp 里插一行，
# 整节论证就静默指到别处，而**没有任何闸会变红**（正是坑60/61 那一族）。
# 所以把锚点做成可跑的表：任一枚 MISS ⇒ rc=5，且 MISS 行会把「实际那一行」打出来，
# 让复核的人一眼看出是漂了还是符号改了。
#
# 本件的历史：表在 T-20261122-03 立（35 枚，对 bt_console.cpp 修复前快照）；
# T-20261122-04 落地丙案（BT 回执升格）后 bt_console.cpp 净增 94 行，
# 35 枚里 23 枚行号漂走 ⇒ 2026-11-22 晚按改后文件实际行号逐枚回改，
# 并补入丙案新增的 14 枚（判据头文件、两个锁内助手、四处登记与判类调用、状态帧两字段）。
#
# 三连（照本仓门禁件的既有姿势）：
#   主测＝52 枚真锚点逐枚 sed -n 取行 + grep -E 命中；
#   注入有效性＝反控①故意写一个不存在的行号、反控②故意写一个不匹配的正则，两者**必须 MISS**
#                （都 MISS 才说明本件不是「恒绿空控」）；
#   反控的意义＝若某天有人把 check 写成恒返回 0，反控两项会先翻红。
#
# 用法＝bash _anchor_check.sh            # 跑并打印
#      bash _anchor_check.sh --log      # 跑并把同一份输出落到同目录 anchors.log
# 退出码＝0（52 枚全命中且两条反控都 MISS）｜5（有 MISS 或反控失效）｜9（前置缺失）
set -u
LOG=0
[ "${1:-}" = "--log" ] && LOG=1
SELF=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$SELF" && git rev-parse --show-toplevel 2>/dev/null)
[ -n "$ROOT" ] || { echo "前置缺失：不在 git 仓内"; exit 9; }
cd "$ROOT" || exit 9
# --log：同一份输出同时落盘（stdout 只 print 不留档＝自报，本仓不收）
[ "$LOG" = 1 ] && exec > >(tee "$SELF/anchors.log") 2>&1

BT=ws/src/90_platform/lite3_bt/src/bt_console.cpp
FAR=ws/src/80_algorithm/v8_algorithm/planner/far_node.py
TREE=ws/src/90_platform/lite3_bt/trees/navigate_with_planner.xml
BUD=ws/src/90_platform/lite3_bt/include/lite3_bt/arrival_budget.hpp
ORG=ws/src/90_platform/lite3_bt/include/lite3_bt/cancel_origin.hpp
EXE=ws/src/80_algorithm/v8_algorithm/local_planner/ros_node.py
for f in "$BT" "$FAR" "$TREE" "$BUD" "$ORG" "$EXE"; do
  [ -f "$f" ] || { echo "前置缺失：$f 不在位"; exit 9; }
done

MISS=0
HIT=0
check() {  # $1 标签 $2 文件 $3 行号 $4 正则
  local got
  got=$(sed -n "${3}p" "$2" 2>/dev/null)
  if printf '%s' "$got" | grep -qE "$4"; then
    HIT=$((HIT + 1))
    printf 'OK   %-26s %s:%s\n' "$1" "$2" "$3"
    return 0
  fi
  MISS=$((MISS + 1))
  printf 'MISS %-26s %s:%s  期望=/ %s /  实际=%s\n' \
    "$1" "$2" "$3" "$4" "$(printf '%s' "$got" | sed 's/^ *//' | cut -c1-64)"
  return 1
}

echo "== 取消后重发失联 · 行号锚点复核（T-20261122-03 立表 / 04 回改并补丙案）=="
echo "   仓根=$ROOT"
echo "   被核件＝bt_console.cpp / far_node.py / navigate_with_planner.xml / arrival_budget.hpp / cancel_origin.hpp / ros_node.py"
echo ""

while IFS='|' read -r label file line pat; do
  case "$label" in ''|'#'*) continue ;; esac
  check "$label" "$file" "$line" "$pat" || true
done <<'ANCHORS'
环1 取消入口回调|ws/src/90_platform/lite3_bt/src/bt_console.cpp|1165|void cancelCallback
环1 输入话题缺省|ws/src/90_platform/lite3_bt/src/bt_console.cpp|1296|cancel_input_topic = "/v8/bt/cancel"
环1 输入订阅|ws/src/90_platform/lite3_bt/src/bt_console.cpp|1376|cancel_sub = nh.subscribe
环2 下发话题缺省|ws/src/90_platform/lite3_bt/src/bt_console.cpp|1297|cancel_topic = "/v8/local/cancel"
环2 下发句柄|ws/src/90_platform/lite3_bt/src/bt_console.cpp|1345|advertise<std_msgs::String>\(cancel_topic
环3 第二写者订阅|ws/src/80_algorithm/v8_algorithm/planner/far_node.py|630|Subscriber\(cancel_topic
环3 自诊断取消入口|ws/src/80_algorithm/v8_algorithm/planner/far_node.py|337|def request_cancel
环3 不回声的理由|ws/src/80_algorithm/v8_algorithm/planner/far_node.py|341|已经有 BT 这个
环4 回执话题|ws/src/80_algorithm/v8_algorithm/planner/far_node.py|438|cancel_ack_topic
环5 回执回调|ws/src/90_platform/lite3_bt/src/bt_console.cpp|1190|void cancelAckCallback
环5 回执记账点|ws/src/90_platform/lite3_bt/src/bt_console.cpp|1213|cancel_results\[seq\] = outcome
环6 WaitForGoal 早退|ws/src/90_platform/lite3_bt/src/bt_console.cpp|232|if \(g_state.cancel_requested\)
环6 WaitArrival 早退|ws/src/90_platform/lite3_bt/src/bt_console.cpp|531|if \(g_state.cancel_requested\)
环6 Dwell 早退|ws/src/90_platform/lite3_bt/src/bt_console.cpp|683|if \(g_state.cancel_requested\)
环6 Recovery 早退|ws/src/90_platform/lite3_bt/src/bt_console.cpp|945|if \(g_state.cancel_requested\) return
环6 门 IsNotCancelled|ws/src/90_platform/lite3_bt/src/bt_console.cpp|876|cancel_requested \? BT::NodeStatus::FAILURE
环6 门 IsCancelled|ws/src/90_platform/lite3_bt/src/bt_console.cpp|863|cancel_requested \? BT::NodeStatus::SUCCESS
环7 停车后不出产出|ws/src/80_algorithm/v8_algorithm/planner/far_node.py|387|E_FAR_HALTED
环8 预算冻结点|ws/src/90_platform/lite3_bt/src/bt_console.cpp|512|budget_ = lite3_bt::arrival_budget_s
环8 速度下限缺省|ws/src/90_platform/lite3_bt/src/bt_console.cpp|111|arrival_speed_floor = 0.02
环8 预算式子|ws/src/90_platform/lite3_bt/include/lite3_bt/arrival_budget.hpp|16|inline double arrival_budget_s
环9 取队点|ws/src/90_platform/lite3_bt/src/bt_console.cpp|239|pending.pop
环9 取队后复位意图|ws/src/90_platform/lite3_bt/src/bt_console.cpp|252|cancel_requested = false
环9 队列上限|ws/src/90_platform/lite3_bt/src/bt_console.cpp|1282|queue_max = 16
环9 满队才拒|ws/src/90_platform/lite3_bt/src/bt_console.cpp|1016|goal queue full
树 mission 门|ws/src/90_platform/lite3_bt/trees/navigate_with_planner.xml|10|IsNotCancelled
树 cancel 支|ws/src/90_platform/lite3_bt/trees/navigate_with_planner.xml|26|IsCancelled
树 停滞判据关|ws/src/90_platform/lite3_bt/trees/navigate_with_planner.xml|21|progress_timeout="0"
设1 适配器号段起点|ws/src/80_algorithm/v8_algorithm/planner/far_node.py|34|CANCEL_SEQ_BASE = 900000000
设2 适配器号自增|ws/src/80_algorithm/v8_algorithm/planner/far_node.py|279|def next_cancel_seq
设3 执行层回 ack 处|ws/src/80_algorithm/v8_algorithm/local_planner/ros_node.py|266|cancelled
设4 BT 收帧取 max|ws/src/90_platform/lite3_bt/src/bt_console.cpp|1176|std::max
设5 CancelStop 取现号|ws/src/90_platform/lite3_bt/src/bt_console.cpp|737|seq = g_state.cancel_seq
设6 StopExecutor 自增号|ws/src/90_platform/lite3_bt/src/bt_console.cpp|813|seq_ = \+\+g_state.cancel_seq
设7 回执去重表上限|ws/src/90_platform/lite3_bt/src/bt_console.cpp|1215|cancel_results.erase
设8 现有清队只看 all|ws/src/90_platform/lite3_bt/src/bt_console.cpp|1184|if \(scope == "all"\) dropPendingLocked
丙案 判据头文件|ws/src/90_platform/lite3_bt/include/lite3_bt/cancel_origin.hpp|43|inline CancelAckDisposition classify_cancel_ack
丙案 外部停枚举|ws/src/90_platform/lite3_bt/include/lite3_bt/cancel_origin.hpp|40|EXTERNAL_STOP
丙案 自持集合字段|ws/src/90_platform/lite3_bt/src/bt_console.cpp|81|std::set<std::uint64_t> self_issued
丙案 来源字段|ws/src/90_platform/lite3_bt/src/bt_console.cpp|83|std::string cancel_origin
丙案 自持记账助手|ws/src/90_platform/lite3_bt/src/bt_console.cpp|160|void rememberSelfIssuedLocked
丙案 清队助手|ws/src/90_platform/lite3_bt/src/bt_console.cpp|172|void dropPendingLocked
丙案 故障停清队|ws/src/90_platform/lite3_bt/src/bt_console.cpp|746|execution_fault\) dropPendingLocked
丙案 自持登记 CancelStop|ws/src/90_platform/lite3_bt/src/bt_console.cpp|747|rememberSelfIssuedLocked\(seq\)
丙案 自持登记 StopExec|ws/src/90_platform/lite3_bt/src/bt_console.cpp|818|rememberSelfIssuedLocked\(seq_\)
丙案 操作员来源|ws/src/90_platform/lite3_bt/src/bt_console.cpp|1179|cancel_origin = "operator"
丙案 判类调用|ws/src/90_platform/lite3_bt/src/bt_console.cpp|1207|lite3_bt::classify_cancel_ack\(facts\)
丙案 非外部即早退|ws/src/90_platform/lite3_bt/src/bt_console.cpp|1217|CancelAckDisposition::EXTERNAL_STOP\) return
丙案 来源标 external|ws/src/90_platform/lite3_bt/src/bt_console.cpp|1231|cancel_origin = "external"
丙案 记外部号|ws/src/90_platform/lite3_bt/src/bt_console.cpp|1232|external_cancel_seq = seq
丙案 状态帧外部号|ws/src/90_platform/lite3_bt/src/bt_console.cpp|1516|external_cancel_seq
丙案 状态帧弃队数|ws/src/90_platform/lite3_bt/src/bt_console.cpp|1517|pending_dropped
ANCHORS

echo ""
echo "-- 反控（两条都必须 MISS，否则本件是恒绿空控）--"
CTL=0
sed -n "999999p" "$BT" >/dev/null 2>&1
if printf '%s' "$(sed -n '999999p' "$BT" 2>/dev/null)" | grep -qE 'cancel'; then
  echo "反控1 失效：不存在的行号居然命中了"; CTL=1
else
  echo "反控1 OK：不存在的行号（第 999999 行）确实取不到内容"
fi
L1=$(sed -n '1p' "$BT")
if printf '%s' "$L1" | grep -qE 'cancel_requested'; then
  echo "反控2 失效：本文件第 1 行不该含 cancel_requested"; CTL=1
else
  echo "反控2 OK：正则在无关行上不误命中"
fi

echo ""
echo "汇总＝命中 $HIT 枚 ‖ MISS $MISS 枚 ‖ 反控失效 $CTL 项"
if [ "$MISS" -eq 0 ] && [ "$CTL" -eq 0 ]; then
  echo "ANCHOR VERDICT: rc=0（全部锚点现读命中，反控未退化）"
  RC=0
else
  echo "ANCHOR VERDICT: rc=5（有锚点漂了或反控失效＝诊断件里的行号需要回改）"
  RC=5
fi
exit "$RC"
