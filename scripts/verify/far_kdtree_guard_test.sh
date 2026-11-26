#!/usr/bin/env bash
# 编并跑 FAR 空树防护的 C++ 反控（2026-11-26 裁决①）。产物落在 /tmp，不入库。
set -euo pipefail
REPO=${REPO:-/opt/ros1-slam}
OUT=${OUT:-/tmp/far_kdtree_guard_test}
SRC="$REPO/scripts/verify/cpp/far_kdtree_guard_test.cc"
INC="$REPO/tooling/far_overlay/src/far_planner/include"

/usr/bin/g++ -std=c++14 -O1 -g \
  -I"$INC" -I/usr/include/pcl-1.10 -I/usr/include/eigen3 -I/usr/include/opencv4 -I/opt/ros/noetic/include -I"$REPO/tooling/far_overlay/devel/include" \
  "$SRC" -o "$OUT" \
  -lpcl_kdtree -lpcl_common -lboost_system

"$OUT"
