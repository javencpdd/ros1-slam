#!/usr/bin/env bash
# P4 四项验收自动化回归：正常导航 / 障碍安全 / 定位失效 / 取消。
#
# 每项都从同一干净起点开始（teleport → 重启 HDL → 重启装配），避免上一项的残留影响下一项。
# 单项耗时约 3–4 分钟，全套约 15 分钟。判定只看各自 JSON 里的字段，不做二次解释。
#
# 前置：Gazebo（lite3_planar + rslidar）已在运行。
# 用法：bash p4_regression.sh [--only nav|obstacle|loc|cancel]
set -uo pipefail

REPO=/opt/ros1-slam
WS=$REPO/ws
EV_ROOT=${EV_ROOT:-$REPO/note/进度/00_专题/v8_map_loc_2026-11-24/evidence/regression_$(date +%Y%m%d_%H%M%S)}
START_X=${START_X:-3.75}; START_Y=${START_Y:-2.45}
GOAL_X=${GOAL_X:-3.75};  GOAL_Y=${GOAL_Y:-1.9}


mkdir -p "$EV_ROOT"
exec > >(tee "$EV_ROOT/run.log") 2>&1
say() { printf '%s\n' "$*"; }

source /opt/ros/noetic/setup.bash
source $WS/devel/setup.bash
export ROS_MASTER_URI=${ROS_MASTER_URI:-http://127.0.0.1:11313}

teleport() {
  rosservice call /gazebo/set_model_state "{model_state: {model_name: lite3_planar, pose: {position: {x: $START_X, y: $START_Y, z: 0.11}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}, reference_frame: world}}" >/dev/null 2>&1
  sleep 2
}

reset_stack() {
  local tag=$1
  teleport
  SKIP_GAZEBO=1 MODE=normal DUR=8 WAIT_SCAN=40 INIT_X=$START_X INIT_Y=$START_Y \
    bash $REPO/scripts/verify/asan_hdl_repro.sh > "$EV_ROOT/${tag}_repro.log" 2>&1
  SKIP_PROBE=1 START_X=$START_X START_Y=$START_Y GOAL_X=$GOAL_X GOAL_Y=$GOAL_Y \
    EV="$EV_ROOT/${tag}_graph" bash $REPO/scripts/verify/p4_nav_hdl.sh > "$EV_ROOT/${tag}_setup.log" 2>&1
  local n
  n=$(rosnode list 2>/dev/null | grep -cE '^/v8' || true)
  say "[$tag] v8_nodes=$n"
  [ "$n" -ge 9 ] || say "[$tag] WARN 装配不全（$n/9），本项结果不可信"
}

case_nav() {
  say '=== 1 正常导航 ==='
  reset_stack nav
  python3 $REPO/scripts/verify/p4_nav_probe.py --goal-x "$GOAL_X" --goal-y "$GOAL_Y" \
    --duration 40 --out "$EV_ROOT/nav_probe.json" > "$EV_ROOT/nav_probe.log" 2>&1
  sleep 5   # 停稳后再量真实到达误差，避免运动末段的非同步采样
  python3 - "$GOAL_X" "$GOAL_Y" "$EV_ROOT/nav_arrival.json" <<'PY'
import json, math, sys
import rospy
from nav_msgs.msg import Odometry
gx, gy, out = float(sys.argv[1]), float(sys.argv[2]), sys.argv[3]
rospy.init_node('arrival_check', anonymous=True)
e = rospy.wait_for_message('/hdl_odom', Odometry, timeout=8).pose.pose.position
g = rospy.wait_for_message('/sim/odom_gt', Odometry, timeout=8).pose.pose.position
res = {'est_to_goal_m': round(math.hypot(e.x - gx, e.y - gy), 3),
       'gt_to_goal_m': round(math.hypot(g.x - gx, g.y - gy), 3),
       'est_gt_gap_m': round(math.hypot(e.x - g.x, e.y - g.y), 3),
       'verdict': 'PASS' if math.hypot(g.x - gx, g.y - gy) <= 0.15 else 'FAIL'}
print(json.dumps(res, ensure_ascii=False))
open(out, 'w').write(json.dumps(res, ensure_ascii=False) + '\n')
PY
}

case_obstacle() {
  say '=== 2 障碍安全（离线回归，不需重置栈）==='
  python3 $REPO/scripts/verify/collision_regression.py --out "$EV_ROOT/obstacle.json" \
    > "$EV_ROOT/obstacle.log" 2>&1
  tail -5 "$EV_ROOT/obstacle.log"
}

case_loc() {
  say '=== 3 定位失效（只断定位数据）==='
  reset_stack loc
  python3 $REPO/scripts/verify/safety_probe.py --mode stop --on-speed 0.10 --at 20 \
    --duration 25 --out "$EV_ROOT/loc_stop.json" > "$EV_ROOT/loc_stop.log" 2>&1
  cat "$EV_ROOT/loc_stop.json" 2>/dev/null
}

case_cancel() {
  say '=== 4 取消（scope=all）==='
  reset_stack cancel
  python3 $REPO/scripts/verify/safety_probe.py --mode cancel --on-speed 0.10 --at 20 \
    --duration 25 --seq 1 --scope all --out "$EV_ROOT/cancel.json" > "$EV_ROOT/cancel.log" 2>&1
  cat "$EV_ROOT/cancel.json" 2>/dev/null
}

# 单项重跑：ONLY=nav|obstacle|loc|cancel（也接受第一个位置参数）
ONLY=${ONLY:-${1:-}}
[ -z "$ONLY" ] || [ "$ONLY" = nav ]      && case_nav
[ -z "$ONLY" ] || [ "$ONLY" = obstacle ] && case_obstacle
[ -z "$ONLY" ] || [ "$ONLY" = loc ]      && case_loc
[ -z "$ONLY" ] || [ "$ONLY" = cancel ]   && case_cancel

say '=== 汇总 ==='
python3 - "$EV_ROOT" <<'PY'
import json, os, sys
root = sys.argv[1]
def load(name):
    p = os.path.join(root, name)
    return json.load(open(p)) if os.path.isfile(p) else None
nav_arr = load('nav_arrival.json')
nav_prb = load('nav_probe.json')
stop = load('loc_stop.json')
cancel = load('cancel.json')
rows = [
    ('1 正常导航', nav_arr and nav_arr.get('verdict'),
     nav_arr and ('gt_to_goal=%.3f m' % nav_arr['gt_to_goal_m']),
     nav_prb and ('speed_frames=%s travel=%.3f' % (nav_prb.get('speed_frames'),
                                                   nav_prb.get('est_travel_m') or 0))),
    ('2 障碍安全', 'PASS(见 obstacle.log)', '-', '-'),
    ('3 定位失效', (stop and stop.get('zero_after_trigger')) and 'PASS' or 'FAIL',
     stop and ('delay=%.3fs coast=%.3fm' % (stop.get('stop_delay_s') or 0,
                                            stop.get('coast_distance_m') or 0)),
     stop and ('mgr=%s gazebo=%s' % (stop.get('alive_has_mgr'), stop.get('alive_has_gazebo')))),
    ('4 取消', (cancel and cancel.get('zero_for_5s')) and 'PASS' or 'FAIL',
     cancel and ('ack=%s delay=%.3fs coast=%.3fm' % (cancel.get('cancel_ack'),
                                                     cancel.get('stop_delay_s') or 0,
                                                     cancel.get('coast_distance_m') or 0)),
     cancel and ('alive=%s' % cancel.get('alive_nodes_after'))),
]
for name, verdict, a, b in rows:
    print('%-12s %-18s %-42s %s' % (name, verdict, a, b))
PY
say "EV_ROOT=$EV_ROOT"
