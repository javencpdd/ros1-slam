#!/usr/bin/env bash
# m2_negctl_tf_runner.sh —— 阴性对照跑的「被试包装」（R41 之二 的活体现场面；小明 2026-10-08）
# 本件不设任何门值、不改任何判据，只做一件事：按跑序把 TF 源置回 static，
#   走 mode_sim_teb.launch L12–L13 那个 [M2R3] 复验口子（原文注明「tf_source:=static 仅供复验做阴性对照，证明 G5 有分辨力」）。
# 为什么需要它：BR 的 SM_CMD／SM_ARGS 是全批共用的，活体批里没法逐跑换被试；
#   用一个数跑序的包装，就能在同一批里放进「设计外应 FAIL」的那一跑，从而拿到非 0 期望位的现场证据。
# 用法：NEGCTL_CNT=<计数件> NEGCTL_STATIC_RUN=<第几跑走 static> 由外层（BR 的父 shell）导出；本件不自设缺省。
set -u
SM=/home/ros1-slam/ws/src/95_simulation/lite3_gazebo_sim/verify/m2_smoke.sh
n=0
if [ -n "${NEGCTL_CNT:-}" ]; then
  n=$(cat "$NEGCTL_CNT" 2>/dev/null || echo 0); n=$((n+1)); echo "$n" > "$NEGCTL_CNT"
fi
if [ -n "${NEGCTL_STATIC_RUN:-}" ] && [ "$n" = "$NEGCTL_STATIC_RUN" ]; then
  echo "[negctl] 跑$n ⇒ TF_SOURCE=static（阴性对照，预期 harness 判 FAIL）"
  export TF_SOURCE=static
else
  echo "[negctl] 跑$n ⇒ TF_SOURCE 缺省（fresh）"
fi
exec bash "$SM" "$@"
