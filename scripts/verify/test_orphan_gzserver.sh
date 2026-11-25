#!/usr/bin/env bash
# 孤儿 gzserver 判据的三连自测：正常底座不动手 / 孤儿被识别 / 反控（只有 master 没有进程时也不动手）。
#
# 为什么用注入而不是真起 gzserver：这三条判的是"判据逻辑"，真起 Gazebo 既慢又会污染端口；
# 用一个 argv 里带 `worlds/m1_world.world __name:=gazebo` 的替身进程即可精确复现 /proc 侧特征。
# 反控的必要性：若判据恒真（永远报孤儿），测试必须能翻红。
set -uo pipefail

HERE=$(cd "$(dirname "$0")" && pwd)
ORPHAN="${ORPHAN:-$HERE/orphan_gzserver.sh}"
pass=0; fail=0
ok()  { echo "[PASS] $*"; pass=$((pass+1)); }
bad() { echo "[FAIL] $*"; fail=$((fail+1)); }
cleanup() { [ -n "${FAKE_PID:-}" ] && kill "$FAKE_PID" 2>/dev/null; true; }
trap cleanup EXIT

# ---------- 用例 1：正常底座（有进程 + master 上有 /gazebo）⇒ 不动手 ----------
out=$(PGREP_CMD_="echo 1" ROSNODE_CMD_="echo /gazebo" DRY=0 bash "$ORPHAN" 2>&1); rc=$?
if [ "$rc" = "0" ] && printf '%s' "$out" | grep -q "属正常底座"; then
  ok "正常底座：识别为在册，不动手"
else
  bad "正常底座被误判（rc=$rc）: $out"
fi

# ---------- 用例 2：孤儿（进程在、master 无 /gazebo）⇒ DRY 干跑应识别并给出计划 ----------
# 造一个 argv 含 m1_world 与 __name:=gazebo 的替身进程
bash -c 'exec -a "gzserver -e ode /opt/ros1-slam/ws/.profiles/x86_sim/src/lite3_gazebo_sim/worlds/m1_world.world __name:=gazebo" sleep 120' &
FAKE_PID=$!
sleep 1
out=$(PGREP_CMD_="pgrep -f worlds/m1_world.world" ROSNODE_CMD_="true" DRY=1 bash "$ORPHAN" 2>&1); rc=$?
if [ "$rc" = "0" ] && printf '%s' "$out" | grep -q "检出孤儿" && printf '%s' "$out" | grep -q "计划清理"; then
  ok "孤儿：识别并给出清理计划（DRY 不执行）"
else
  bad "孤儿未被识别（rc=$rc）: $out"
fi

# ---------- 用例 3：反控——只有 master 上没有 /gazebo、但没有进程 ⇒ 不得报孤儿 ----------
out=$(PGREP_CMD_="true" ROSNODE_CMD_="true" DRY=0 bash "$ORPHAN" 2>&1); rc=$?
if [ "$rc" = "0" ] && printf '%s' "$out" | grep -q "无需清理"; then
  ok "反控：无进程时判据不误报"
else
  bad "反控失败（rc=$rc）: $out"
fi

# ---------- 用例 4：真清理（非 DRY）后进程应消失 ----------
out=$(PGREP_CMD_="pgrep -f worlds/m1_world.world" ROSNODE_CMD_="true" DRY=0 bash "$ORPHAN" 2>&1); rc=$?
if [ "$rc" = "0" ] && printf '%s' "$out" | grep -q "已清干净"; then
  ok "真清理：替身进程被回收"
else
  bad "真清理失败（rc=$rc）: $out"
fi

echo "---- 结果：$pass PASS / $fail FAIL ----"
[ "$fail" = "0" ] || exit 1
