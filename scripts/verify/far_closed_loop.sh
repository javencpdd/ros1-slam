#!/usr/bin/env bash
# FAR / Support 后端对照的一键入口（2026-11-26 收敛版）。
#
# 为什么要改成薄壳（真人裁决②③）：
#   · 旧版自己 roslaunch M2 底座、自己起图、自己跑 `far_closed_loop_probe.py`。而
#     `p4_nav_hdl.sh`/`p4_regression.sh` 已经有一套完整的底座+HDL+图+探针+judge；
#     两者并存导致"同一批 manager/relay 被准备两次"，也导致 FAR 的验收口径与 Support 不一致
#     （旧探针直接发 localization_ok=false、固定目标、用估计位移、取消 scope 也是旧的，
#      且没有完整 judge）。
#   · 现在**只做后端切换**：把 GRAPH/EXPECT_NODES/POSE_TOPIC 交给 p4_regression，其余完全复用
#     已通过的 Support 验收链（四项 + judge）。
#
# 用法（容器内）：
#   bash scripts/verify/far_closed_loop.sh                 # 默认 M1 + HDL + FAR 图
#   BASE=m2 bash scripts/verify/far_closed_loop.sh          # LIO 组合（M2 时间戳未修好前不作基准）
#   EV_ROOT=/path/to/evidence bash scripts/verify/far_closed_loop.sh
#
# 退出码沿用 p4_regression：0＝四项全 PASS。
set -uo pipefail

REPO="${REPO:-/opt/ros1-slam}"
BASE="${BASE:-m1}"
FW="$REPO/ws/src/90_platform/v8_platform/framework/fixtures/graphs"

if [ "$BASE" = "m1" ]; then
  export GRAPH="${GRAPH:-$FW/graph_3d_m1_hdl_bt_far.yaml}"
  export EXPECT_NODES="${EXPECT_NODES:-8}"     # far_stack 以 include 起 far_planner + v8_global_far
  export POSE_TOPIC="${POSE_TOPIC:-/hdl_odom}"
else
  export M2=1
  export GRAPH="${GRAPH:-$FW/graph_3d_m2_lio_bt_far.yaml}"
  export EXPECT_NODES="${EXPECT_NODES:-10}"
  export POSE_TOPIC="${POSE_TOPIC:-/v8/local/pose}"
fi

echo "[far] BASE=$BASE GRAPH=$(basename "$GRAPH") EXPECT_NODES=$EXPECT_NODES POSE_TOPIC=$POSE_TOPIC"
echo "[far] 旧 far_closed_loop_probe.py 已停用（口径与 Support 不一致），验收统一走 p4_regression"

exec bash "$REPO/scripts/verify/p4_regression.sh"
