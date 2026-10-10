#!/usr/bin/env bash
# verify_ledger_citations.sh —— 台账面 VCC 黄/红名单**逐条回原文核实**（R102 的 v 稿取证件；只读，零节点，零写盘）
# 为什么每条都要给"可直接重放的命令"：v 稿的全部价值就在于**不要求读者相信我的结论**（坑58 修法①）。
# 覆盖＝本次 VCC 缺省扫描面的 1 红 + 21 黄（去重后 22 项）；重放本件前先跑
#   VCC_MAX=200 bash scripts/verify/v_citation_check.sh
# 取到同样的名单，否则说明名单已变、本件的结论不该照抄。
set -u
cd /home/ros1-slam || exit 9
INC=/opt/ros/noetic/include
show() { printf '\n### %s\n' "$1"; }
T() { # 行文本截 110 列，避免长中文行刷屏
  printf '    $ sed -n "%sp" %s\n' "$1" "$2"
  sed -n "$1p" "$2" 2>/dev/null | cut -c1-110 | sed 's/^/      ⇒ /'
}

show "0) 名单本身（去重后的红/黄全集，应与本件条目一一对应）"
echo '    $ VCC_MAX=200 bash scripts/verify/v_citation_check.sh 2>&1 | grep -E "^\[(RED|AMBER)\]" | sort -u'

show "1) 红·04:237 引 ws/src/CMakeLists.txt:192（越界，该件仅 71 行）"
echo "    $ wc -l ws/src/CMakeLists.txt"; wc -l ws/src/CMakeLists.txt | sed 's/^/      ⇒ /'
echo '    $ sed -n "192p" opt/upstream/lvr2/CMakeLists.txt   # 真所指（04:750 已在册勘误）'
T 192 opt/upstream/lvr2/CMakeLists.txt

show "2) 04 引 mockamap/src/maps.cpp:599（键名 block）"
T 599 ws/src/95_simulation/mockamap/src/maps.cpp
T 600 ws/src/95_simulation/mockamap/src/maps.cpp

show "3) 04 引 faster_lio_sim.yaml:29（键名 mapping.extrinsic_est_en）"
T 29 ws/src/95_simulation/config/faster_lio_sim.yaml
T 30 ws/src/95_simulation/config/faster_lio_sim.yaml

show "4) 04 引 faster_lio_sim.yaml:37（键名 preprocess.blind）"
T 37 ws/src/95_simulation/config/faster_lio_sim.yaml
T 38 ws/src/95_simulation/config/faster_lio_sim.yaml

show "5) 04 引 global_planner_params.yaml:12（键名 allow_unknown）"
T 12 ws/src/40_navigation/lite3_navigation/config/global_planner_params.yaml
T 8  ws/src/40_navigation/lite3_navigation/config/global_planner_params.yaml

show "6) 04 引 note/02_问题与坑.md:314（键名 坑42）"
T 314 note/02_问题与坑.md
T 452 note/02_问题与坑.md

show "7) 02 引 70_scan.launch:61（键名 remap）"
T 61 ws/src/40_navigation/lite3_scan/launch/70_scan.launch
T 76 ws/src/40_navigation/lite3_scan/launch/70_scan.launch

show "8) 简写名·costmap_navigation_server.h:236/:239（真所在 $INC/mbf_costmap_nav/）"
F=$INC/mbf_costmap_nav/costmap_navigation_server.h
echo "    $ ls $F"; ls "$F" 2>/dev/null | sed 's/^/      ⇒ /'
T 236 "$F"; T 239 "$F"

show "9) 简写名·nodelet.cpp:68（实名 hdl_localization_nodelet.cpp）"
T 68 ws/src/20_localization/hdl_localization/apps/hdl_localization_nodelet.cpp

show "10) 简写名·mbf 三码 GetPathResult.h:111 / ExePathResult.h:126 / RecoveryResult.h:87"
for h in GetPathResult:111 ExePathResult:126 RecoveryResult:87; do
  n=${h%%:*}; l=${h##*:}; T "$l" "$INC/mbf_msgs/$n.h"
done

show "11) 简写名·move_base_action.h:116（总览引；$INC/mbf_abstract_nav/，**非** mbf_msgs/MoveBaseAction.h）"
T 116 $INC/mbf_abstract_nav/move_base_action.h
echo "    配套两码表：$ grep -n OSCILLATION /opt/ros/noetic/share/mbf_msgs/action/{MoveBase,ExePath}.action"
grep -n OSCILLATION /opt/ros/noetic/share/mbf_msgs/action/MoveBase.action /opt/ros/noetic/share/mbf_msgs/action/ExePath.action 2>/dev/null | cut -c1-90 | sed 's/^/      ⇒ /'

show "12) 简写名·fresh{1..5}n.log（真名带 m2r5_ 前缀，同目录 raw/；首条已给全路径，后四条省写）"
D=note/进度/SCAN集成/证据/log/raw
for p in 1:19 2:23 3:27 4:19 5:23; do T "${p##*:}" "$D/m2r5_fresh${p%%:*}n.log"; done

show "13) fd 稿引 hdl_localization.launch:89（键名 enable_robot_odometry_prediction）"
L=ws/src/20_localization/lite3_localization/launch/hdl_localization.launch
T 89 "$L"; echo "    该键实所在："; grep -n "enable_robot_odometry_prediction" "$L" | cut -c1-110 | sed 's/^/      ⇒ /'

show "14) fd 稿引 hdl_localization.launch:38/:80/:89-90（键名 odom_child_frame_id）"
grep -n "odom_child_frame_id" "$L" | cut -c1-110 | sed 's/^/      ⇒ /'

show "15) fd 稿:55 机读点 m2_smoke.sh:193/:379/:380 ⇒ 行号随 SM 生长漂移"
S=ws/src/95_simulation/lite3_gazebo_sim/verify/m2_smoke.sh
T 193 "$S"; T 379 "$S"; T 380 "$S"
echo "    现行真值："; grep -n "sim/odom_gt', Odometry" "$S" | cut -c1-100 | sed 's/^/      ⇒ /'
T 404 "$S"; T 405 "$S"; T 406 "$S"
echo "    当年（M2R6＝3c12fec）确为所指 ⇒ 属漂移非臆造："
git show 3c12fec:$S 2>/dev/null | sed -n '193p;379p;380p' | cut -c1-96 | sed 's/^/      ⇒ /'

echo
echo "核实命令清单即本件；重放：bash note/进度/SCAN集成/证据/log/raw/r102_ledger_citation_table/verify_ledger_citations.sh"
