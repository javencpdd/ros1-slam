#!/usr/bin/env bash
# BT host tests + XML load (container). Read-only; starts no ROS master.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PKG="$ROOT/ws/src/90_platform/lite3_bt"
CXX="${CXX:-g++}"
TMP="${TMPDIR:-/tmp}/v8_bt_tests"
mkdir -p "$TMP"
for name in test_goal_queue test_arrival_budget test_bt_policy; do
  "$CXX" -std=c++11 -Wall -Wextra -I"$PKG/include" "$PKG/test/$name.cpp" -o "$TMP/$name"
  "$TMP/$name"
done
if docker ps --format '{{.Names}}' | grep -qx ros1-slam-sim-viz; then
  docker exec ros1-slam-sim-viz bash -c 'source /opt/ros/noetic/setup.bash && B=/opt/ros1-slam && g++ -std=c++14 -I$B/opt/btcpp_v3/include $B/ws/src/90_platform/lite3_bt/test/test_tree_load.cpp -L$B/opt/btcpp_v3/lib -lbehaviortree_cpp_v3 -Wl,-rpath,$B/opt/btcpp_v3/lib -pthread -o /tmp/lite3_bt_tree_load && /tmp/lite3_bt_tree_load $B/ws/src/90_platform/lite3_bt/trees/navigate_with_planner.xml && /tmp/lite3_bt_tree_load $B/ws/src/90_platform/lite3_bt/trees/navigate_single_goal.xml'
else
  echo 'SKIP  BT XML tree load (sim-viz not running)'
fi
