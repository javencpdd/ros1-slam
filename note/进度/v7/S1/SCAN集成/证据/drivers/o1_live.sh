#!/usr/bin/env bash
set +u
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION
source /opt/ros/noetic/setup.bash; source /home/ros1-slam/ws/devel/setup.bash
set -u
export ROS_MASTER_URI=http://127.0.0.1:11313
O=/tmp/o1_live; mkdir -p $O
roscore > $O/roscore.log 2>&1 &
RC=$!
sleep 4
roslaunch lite3_scan 70_scan.launch > $O/launch.log 2>&1 &
RL=$!
sleep 22
echo "=== rosnode list ===" | tee $O/report.txt
timeout 15 rosnode list 2>&1 | tee -a $O/report.txt
echo "=== A-2/A-3 arrived ===" | tee -a $O/report.txt
timeout 15 rostopic info /lite3/scan/arrived 2>&1 | tee -a $O/report.txt
echo "=== A-2/A-3 arrival_dist ===" | tee -a $O/report.txt
timeout 15 rostopic info /lite3/scan/arrival_dist 2>&1 | tee -a $O/report.txt
echo "=== A-1 bspline 订阅者（只增读者核对）===" | tee -a $O/report.txt
timeout 15 rostopic info /planning/bspline 2>&1 | tee -a $O/report.txt
echo "=== A-1 goal 归属（FSM 侧订阅 /scan_planner/goal）===" | tee -a $O/report.txt
timeout 15 rostopic info /scan_planner/goal 2>&1 | tee -a $O/report.txt
echo "=== B-2 活体取值 ===" | tee -a $O/report.txt
timeout 15 rosparam get /closed_loop_controller/finish_dist 2>&1 | tee -a $O/report.txt
timeout 15 rosparam get /scan_arrival_monitor/tol_arrive 2>&1 | tee -a $O/report.txt
echo "=== B-4 双嵌套全名（应无）===" | tee -a $O/report.txt
timeout 15 rosparam list 2>&1 | grep -c "/closed_loop_controller/closed_loop_controller" | tee -a $O/report.txt
echo "=== 功能用例（无车：注入假 goal + 假 odom）===" | tee -a $O/report.txt
timeout 120 python3 -u /tmp/o1_func.py 2>&1 | tee -a $O/report.txt
echo "=== 话题最终值 ===" | tee -a $O/report.txt
timeout 15 rostopic echo -n 1 /lite3/scan/arrived 2>&1 | tee -a $O/report.txt
echo "=== scan_arrival_monitor 节点日志（stdout 片段）===" | tee -a $O/report.txt
grep -E "goal received|arrived|monitor up" $O/launch.log | head -12 | tee -a $O/report.txt
kill $RL $RC >/dev/null 2>&1; sleep 3
pkill -f "rosmaster --core -p 11313" >/dev/null 2>&1; sleep 2
echo "=== 退净 ===" ; echo "残留:[$(pgrep -f 'roslaunch|rosmaster|roscore|scan_planner_node|closed_loop_controller|scan_arrival_monitor' | tr '\n' ' ')]" | tee -a $O/report.txt
echo "O1_LIVE_DONE $(date '+%T')" | tee -a $O/report.txt
