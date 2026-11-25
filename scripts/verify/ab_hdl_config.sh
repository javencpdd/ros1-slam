#!/usr/bin/env bash
# 受控 A/B：只切换底座是否自带 HDL（hdl / no-hdl），其余（路线、目标、图、探针）完全一致。
#
# 为什么需要它：之前出现过一对互相矛盾的事实——
#   base hdl    → /hdl_odom 有 2 个发布者、4 Hz、最大断档 0.64 s，FAR nav **会动**（0.65/1.3 m）
#   base no-hdl → 单发布者 9.1 Hz，FAR nav **一步不动**
# 直觉上"位姿更稳就该更好"，事实相反 ⇒ 说明还有第二个变量没控制住。这个脚本把唯一变量锁成
# "底座是否自带 HDL"，其余全部固定，并同时采集三组读数：FAR 是否出路径、local 停因、位姿断档。
#
# 用法：bash scripts/verify/ab_hdl_config.sh
set -uo pipefail

W=/opt/ros1-slam
GRAPH=$W/ws/src/90_platform/v8_platform/framework/fixtures/graphs/graph_3d_m1_hdl_bt_far.yaml
EV=${EV:-/tmp/ab_hdl_$(date +%H%M%S)}
START_X=${START_X:-5.2}; START_Y=${START_Y:-2.45}
GOAL_X=${GOAL_X:-3.9};   GOAL_Y=${GOAL_Y:-2.45}
mkdir -p "$EV"

source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=${ROS_MASTER_URI:-http://127.0.0.1:11313}
export GAZEBO_MASTER_URI=${GAZEBO_MASTER_URI:-http://127.0.0.1:11345}

say() { printf '[ab] %s\n' "$*"; }

cleanup() {
  pkill -x bt_console 2>/dev/null || true
  pkill -x nodelet 2>/dev/null || true
  pkill -f "noetic/bin/[r]oslaunch" 2>/dev/null || true
  sleep 2
}

run_case() {  # $1=hdl|no-hdl
  local mode="$1"
  local d="$EV/$mode"
  mkdir -p "$d"
  say "===== case=$mode ====="
  cleanup
  bash "$W/docker/sim.sh" base m1 "$mode" > "$d/base.log" 2>&1 || { say "base 失败见 $d/base.log"; return 1; }
  tail -1 "$d/base.log"

  python3 "$W/scripts/verify/pose_gap_probe.py" --duration 34 --out "$d/gaps.json" > "$d/gaps.out" 2>&1 &
  local gap_pid=$!

  ( cd "$W/ws/src/90_platform/v8_platform/framework" && \
    python3 cli.py plan fixtures/manifests "$GRAPH" --package-index fixtures/package_index.json \
      --out-plan "$d/plan.json" --out-launch "$d/graph.launch" ) > "$d/plan.out" 2>&1
  export ROS_PACKAGE_PATH="$W/ws/.profiles/x86_sim/src:$W/tooling/far_overlay/src:/opt/ros/noetic/share"
  export LD_LIBRARY_PATH="$W/tooling/far_overlay/devel/lib:${LD_LIBRARY_PATH:-}"
  roslaunch --screen "$d/graph.launch" > "$d/graph.log" 2>&1 &
  sleep 28

  timeout 3 rostopic pub -r 3 /v8/3d/global/start geometry_msgs/PoseStamped \
    "{header: {frame_id: map}, pose: {position: {x: $START_X, y: $START_Y}, orientation: {w: 1.0}}}" >/dev/null 2>&1
  sleep 1
  timeout 3 rostopic pub -r 3 /v8/bt/goal geometry_msgs/PoseStamped \
    "{header: {frame_id: map}, pose: {position: {x: $GOAL_X, y: $GOAL_Y}, orientation: {w: 1.0}}}" >/dev/null 2>&1
  sleep 22

  {
    echo "hdl_odom_pubs=$(rostopic info /hdl_odom 2>/dev/null | sed -n '/Publishers/,$p' | grep -c '^ \*')"
    echo "hdl_odom_hz=$(timeout 6 rostopic hz /hdl_odom 2>/dev/null | tail -1)"
    echo "path_hz=$(timeout 6 rostopic hz /v8/3d/global/path 2>/dev/null | tail -1)"
    echo "localgoal_hz=$(timeout 6 rostopic hz /v8/local/goal 2>/dev/null | tail -1)"
    echo "cmd_sample=$(timeout 5 rostopic echo -n3 /cmd_vel/linear/x 2>/dev/null | tr -d '\n')"
    echo "local_status=$(timeout 5 rostopic echo -n1 /v8/local/status 2>/dev/null | head -2 | tr -d '\n')"
    echo "path_ready=$(grep -c 'global path ready' "$d/graph.log" 2>/dev/null)"
  } > "$d/readings.txt" 2>&1
  wait $gap_pid 2>/dev/null
  say "reads:"; sed 's/^/    /' "$d/readings.txt"
  say "gaps:"; python3 - "$d/gaps.json" <<'PY'
import json, sys
try:
    d = json.load(open(sys.argv[1]))
    for k in ('pose', 'lio'):
        v = d[k]
        print('    %-5s n=%s rate=%s max_gap=%s gaps>thr=%s'
              % (k, v['n'], v.get('rate_hz'), v['max_gap_s'], len(v['gaps'])))
except Exception as e:
    print('    (gaps 读取失败: %s)' % e)
PY
  pkill -f "noetic/bin/[r]oslaunch" 2>/dev/null || true
  sleep 2
}

run_case hdl
run_case no-hdl
say "EV=$EV"
