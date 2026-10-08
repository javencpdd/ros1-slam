#!/usr/bin/env bash
# v_dwa_recheck.sh —— W2 遗留项「DWA 经 mbf nav_core 包装不可用」的重核驱动（小明 2026-10-08）
# 前置已满足的证据链：W2 三败签名 = 位姿获取域（age 5.372s > tolerance 3s），而 M2R5/U1 已根除
#   该家族根因（base_link 双父 + 静态 map->odom 零锚点）⇒ 小美「非算法否定、待后续重核」的条件成立。
# 判据（roadmap C #4 原句）：mbf 试加载插件成功 ＋ ExePath outcome=0 ×3（三连）。
# 入口走 TOP 编排件 lite3_bringup/mode_sim_teb.launch（K3：平级模块禁互 include；与 m2_smoke.sh 同款）。
# 用法：bash v_dwa_recheck.sh [轮数=3]
# 注：catkin 的 profile.d/1.ros_distro.sh 读 ROS_DISTRO 时不防未绑定 ⇒ 必须在 source 段关 -u，
#   source 完再开（02 既有坑，m2_smoke.sh 同款处置）。
set +u
N=${1:-3}
PKG=/home/ros1-slam/ws/src/40_navigation/lite3_navigation
export DWA_OUT=${DWA_OUT:-/tmp/dwa_recheck}
mkdir -p "$DWA_OUT"
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION
source /opt/ros/noetic/setup.bash
source /home/ros1-slam/ws/devel/setup.bash
set -u
export ROS_MASTER_URI=http://127.0.0.1:11313 GAZEBO_MASTER_URI=http://127.0.0.1:11345
export GAZEBO_PLUGIN_PATH=/opt/ros/noetic/lib:/home/ros1-slam/ws/devel/lib

cleanup() { rosnode kill -a >/dev/null 2>&1; sleep 3
  pkill -f gzserver >/dev/null 2>&1; pkill -f gzclient >/dev/null 2>&1
  pkill -f "roslaunch.*mode_sim_" >/dev/null 2>&1; pkill -f move_base_flex >/dev/null 2>&1
  pkill -f sim_adapter >/dev/null 2>&1; pkill -f "rosmaster --core -p 11313" >/dev/null 2>&1
  sleep 4
  { echo "# 退净核验 @$(date '+%F %T')"
    echo "-- 命中进程原样 ps 行 --"
    pgrep -af 'roslaunch|rosmaster|rosout|gzserver|gzclient|move_base_flex|sim_adapter|dwa_recheck' || echo "  （零命中）"
    echo "-- 端口 TCP/UDP 分查 --"
    for p in 11311 11312 11313 11345; do echo "  port $p TCP=$(ss -lnt 2>/dev/null | grep -c ":$p\b") UDP=$(ss -lnu 2>/dev/null | grep -c ":$p\b")"; done
  } > "$DWA_OUT/exitclean.txt" 2>&1
  sed 's/^/[exitclean] /' "$DWA_OUT/exitclean.txt"; }
trap cleanup EXIT

echo "### 一、mbf 是否真把 DWA 装进控制器表（加载判据，零节点侧改动）"
roslaunch lite3_bringup mode_sim_teb.launch > "$DWA_OUT/mbf.log" 2>&1 &
LP=$!
T=0
until [ $T -ge 300 ]; do
  if rostopic list 2>/dev/null | grep -qx /clock && timeout 10 rostopic echo -n 1 /clock >/dev/null 2>&1; then break; fi
  sleep 2; T=$((T+2))
done
[ $T -lt 300 ] || { echo "FAIL: /clock 300s 未出"; exit 1; }
echo "  /clock 就绪 t=${T}s"
# 加载判据必须**有界轮询**：mbf 装插件晚于 /clock 出现数十秒，单次即查会把「还没装」误判成「装不上」，
#   更糟的是随即 exit → trap 关掉整台仿真（首版即踩此坑，log 见 Reason given for shutdown: user request）。
# 就绪信号必须等**目标那一条**：planners（GlobalPlanner/NavfnROS）先于 controllers 装载，
#   拿「loaded successfully 计数 >=2」当就绪会在 DWA 还没轮到时就读到 2 而提前判定（首版即踩）。
DWA_LOADED=0; TEB_LOADED=0; T2=0
while [ $T2 -lt 300 ]; do
  DWA_LOADED=$(grep -ac 'dwa_local_planner/DWAPlannerROS" has been loaded successfully' "$DWA_OUT/mbf.log" 2>/dev/null | head -1); DWA_LOADED=${DWA_LOADED:-0}
  TEB_LOADED=$(grep -ac 'teb_local_planner/TebLocalPlannerROS" has been loaded successfully' "$DWA_OUT/mbf.log" 2>/dev/null | head -1); TEB_LOADED=${TEB_LOADED:-0}
  [ "$DWA_LOADED" -ge 1 ] && [ "$TEB_LOADED" -ge 1 ] && break
  sleep 4; T2=$((T2+4))
done
LOAD=$(grep -ac 'has been loaded successfully' "$DWA_OUT/mbf.log" 2>/dev/null | head -1)
echo "  等待 ${T2}s；DWA 命中=$DWA_LOADED teb 命中=$TEB_LOADED（loaded 总累计=$LOAD，含 planners/recovery）"
[ "${DWA_LOADED:-0}" -ge 1 ] || { echo "  [FAIL] DWA 未加载 ⇒ 复测前置不成立，本批作废（不判 DWA 可用性）"; exit 1; }
LIST=$(timeout 15 rosparam get /move_base_flex/controllers 2>/dev/null | tr -d '\n' | grep -o "name: '[A-Za-z]*'")
echo "  控制器注册表: $LIST"
echo "  （注：root-inertia WARN 现指 base_link，小张 M2R5 风险①预测成立；已 grep 全仓无脚本消费该串）"

PASS=0; FAIL=0
for i in $(seq 1 "$N"); do
  echo "### 二、DWA 第 $i 跑（三连判据）"
  python3 "$PKG/verify/dwa_recheck.py" DWAPlannerROS "dwa$i" > "$DWA_OUT/dwa$i.txt" 2>&1
  RC=$?
  sed 's/^/    /' "$DWA_OUT/dwa$i.txt"
  POSEERR=$(grep -acE "Could not get the current robot pose|robot pose is .* old" "$DWA_OUT/mbf.log" | head -1)
  echo "  rc=$RC ；mbf 日志内 pose 类命中累计=${POSEERR:-0}"
  if [ $RC -eq 0 ]; then PASS=$((PASS+1)); echo "  [PASS] dwa 第 $i 跑"; else FAIL=$((FAIL+1)); echo "  [FAIL] dwa 第 $i 跑 rc=$RC"; fi
  sleep 3
done
echo "### 三、同场 teb 对照（证 harness 本身没坏）"
python3 "$PKG/verify/dwa_recheck.py" TebLocalPlannerROS teb_ctrl > "$DWA_OUT/teb_ctrl.txt" 2>&1
TC=$?; sed 's/^/    /' "$DWA_OUT/teb_ctrl.txt"
[ $TC -eq 0 ] && echo "  [PASS] teb 对照" || echo "  [FAIL] teb 对照 rc=$TC"
echo "### 汇总 DWA PASS=$PASS FAIL=$FAIL /$N；teb 对照 rc=$TC"
kill $LP >/dev/null 2>&1
[ "$PASS" = "$N" ] && [ "$TC" = "0" ] && echo "DWA RECHECK VERDICT: PASS（DWA 已可用）" || echo "DWA RECHECK VERDICT: FAIL（DWA 仍不可用）"
