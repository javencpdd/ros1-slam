#!/usr/bin/env bash
# r85_b1_345_driver.sh —— 批1 复跑清单的第③④⑤条（stageA 有效复取 ＋ SCAN P1 ＋ SCAN P2 含 O-1 同窗取数）
#
# 为什么只做这三条：⑥ FD-0/1/3/4 三取样点、⑦ FD-6 哨兵、⑧ V6 结构等价、⑨ W4 复测**各自的命令与判据我还没在册里
# 核对到可直接执行的粒度**——不猜口径跑出来的绿不算数（02 坑45 同族）。这三条的命令形状照在册
# `证据/log/raw/b1_retake/b1_retake_driver.sh` 抄，不自创。
#
# ③ 的口径：stageA 首跑曾 rc=3 INVALID（A0 运动窗 RTF 0.240 < 0.25 门，而 A①8.3Hz／A②188.8Hz／A④飞点 0 全过）
#   ⇒ 按 M2R2 裁决**作废重跑、最多三次取有效**；三次都不有效就停手上报，不改成"降低门值"。
# 独占面：本脚本占 `m2_smoke` 链窗；不与其他链并发（锁由 m2_smoke 自身管）。
set +u
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION
source /opt/ros/noetic/setup.bash
set -u
[ -f /home/ros1-slam/ws/devel/setup.bash ] && source /home/ros1-slam/ws/devel/setup.bash
export GAZEBO_PLUGIN_PATH=/opt/ros/noetic/lib:/home/ros1-slam/ws/devel/lib
export ROS_MASTER_URI=http://127.0.0.1:11313 GAZEBO_MASTER_URI=http://127.0.0.1:11345

O=${R85_OUT:-/home/ros1-slam/note/进度/SCAN集成/证据/log/raw/r85_b1_345}
mkdir -p "$O" || { echo "建不出证据目录"; exit 2; }
LOG="$O/driver.log"
D=/home/ros1-slam/ws/src/95_simulation/lite3_gazebo_sim
P=/home/ros1-slam/ws/src/40_navigation/lite3_scan/verify

say() { echo "[$(date '+%T')] $*" | tee -a "$LOG"; }

say "===== 0) 被测件与 harness 指纹（跑前）====="
sha256sum $D/urdf/lite3_planar.urdf.xacro $D/verify/m2_smoke.sh $D/verify/m2_exitclean_lib.sh \
  $D/verify/v_pose_source.sh $P/v_scan_p1.sh $P/v_scan_p2.sh $P/p2_goal_plan.py \
  /home/ros1-slam/ws/src/95_simulation/lite3_gazebo_sim/worlds/m1_world.world \
  /home/ros1-slam/data/maps/lite3.pcd > "$O/sha_inputs_before.txt" 2>&1
cat "$O/sha_inputs_before.txt" | sed 's/^/    /' | tee -a "$LOG"
say "跑前净空：在册端口=$(ss -ltnH 2>/dev/null | grep -cE ':(11311|11312|11313|11345)\b') 残留=$(ps -eo comm= | grep -cE '^(gzserver|rosmaster|roscore|rosout|run_mapping_online|scan_planner_node)$')"

say "===== ③ stageA 有效复取（最多三次取有效；rtf_motion 门 0.25）====="
okA=0
for i in 1 2 3; do
  TF_SOURCE="" bash $D/verify/m2_smoke.sh stageA > "$O/stageA_try$i.log" 2>&1; rc=$?
  rtf=$(grep -oE "rtf_motion=[0-9.]+" "$O/stageA_try$i.log" | head -1 | cut -d= -f2)
  say "  try$i rc=$rc rtf_motion=[$rtf] 判定=$(grep -cE '^\[PASS\]' "$O/stageA_try$i.log")P/$(grep -cE '^\[FAIL\]' "$O/stageA_try$i.log")F"
  grep -E "^\[(PASS|FAIL|INVALID)\]|POSE SOURCE|posesrc|drift=" "$O/stageA_try$i.log" | tail -12 | sed 's/^/      /' | tee -a "$LOG"
  if [ "$rc" = "0" ]; then okA=1; say "  ⇒ 第 $i 次取到有效 stageA（rc=0），复取完成"; break; fi
  if [ "$rc" = "3" ] || [ "$rc" = "9" ]; then say "  ⇒ 第 $i 次 INVALID（rc=$rc）＝作废不计 FAIL，续跑下一次"; else
    say "  ⇒ 第 $i 次 rc=$rc 非 0/3/9 ⇒ 这是**跑不过**或异常，**停手不续跑**，留待人工"; break; fi
  sleep 5
done
[ "$okA" = "1" ] || say "  ⇒ 【三次都未取有效】按 M2R2 上限停手上报，不自降门值"

say "===== ④ SCAN P1 ====="
bash $P/v_scan_p1.sh > "$O/p1.log" 2>&1; rc1=$?
say "  p1 rc=$rc1"; grep -E "^\[(PASS|FAIL|INVALID)\]|SCAN P1 VERDICT" "$O/p1.log" | tail -8 | sed 's/^/      /' | tee -a "$LOG"

say "===== ⑤ SCAN P2（同窗取 O-1 endpoint_err）====="
bash $P/v_scan_p2.sh > "$O/p2.log" 2>&1; rc2=$?
say "  p2 rc=$rc2"; grep -E "^\[(PASS|FAIL|INVALID)\]|VERDICT|endpoint_err|react_s" "$O/p2.log" | tail -12 | sed 's/^/      /' | tee -a "$LOG"

say "===== 9) 跑后指纹与净空（与跑前逐行对）====="
sha256sum $D/urdf/lite3_planar.urdf.xacro $D/verify/m2_smoke.sh $D/verify/m2_exitclean_lib.sh \
  $D/verify/v_pose_source.sh $P/v_scan_p1.sh $P/v_scan_p2.sh $P/p2_goal_plan.py \
  /home/ros1-slam/ws/src/95_simulation/lite3_gazebo_sim/worlds/m1_world.world \
  /home/ros1-slam/data/maps/lite3.pcd > "$O/sha_inputs_after.txt" 2>&1
if diff -q "$O/sha_inputs_before.txt" "$O/sha_inputs_after.txt" >/dev/null; then
  say "  被测件／harness／受保护件 sha **跑前＝跑后**（零改动面自证）"
else
  say "  【红】sha 对不上："; diff "$O/sha_inputs_before.txt" "$O/sha_inputs_after.txt" | sed 's/^/      /' | tee -a "$LOG"
fi
say "  跑后净空：在册端口=$(ss -ltnH 2>/dev/null | grep -cE ':(11311|11312|11313|11345)\b') 残留=$(ps -eo comm= | grep -cE '^(gzserver|rosmaster|roscore|rosout|run_mapping_online|scan_planner_node)$')"
say "===== 汇总 ====="
say "  ③stageA=$([ $okA = 1 ] && echo 有效复取成功 || echo 三次未取有效) ④P1 rc=$rc1 ⑤P2 rc=$rc2"
say "  证据目录=$O"
