#!/usr/bin/env bash
# V8-03 第二切片无头烟测：roscore + scan_planner_node(mode3) + 合成输入，退净检查后退出。
# 注意不能 set -u：ROS setup 脚本链对未定义变量敏感（首跑实测 5 秒退 1 即此因）。
set -o pipefail
source /opt/ros/noetic/setup.bash
source /opt/ros1-slam/r633/ws/devel/setup.bash
# network=none 容器无主机名解析，ROS 必须钉死回环（首跑实测 RLException: Unable to contact my own server）
export ROS_HOSTNAME=127.0.0.1
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_LOG_DIR=/opt/ros1-slam/r633/logs
mkdir -p /opt/ros1-slam/r633/logs

roscore -p 11311 > /opt/ros1-slam/r633/logs/roscore.log 2>&1 &
CORE_PID=$!
cleanup() {
  [ -n "${RL_PID:-}" ] && kill -INT "$RL_PID" 2>/dev/null; sleep 3
  [ -n "${RL_PID:-}" ] && kill "$RL_PID" 2>/dev/null
  [ -n "$CORE_PID" ] && kill "$CORE_PID" 2>/dev/null
}
trap cleanup EXIT

for _ in $(seq 1 40); do rosnode list >/dev/null 2>&1 && break; sleep 0.5; done
rosnode list >/dev/null 2>&1 || { echo "roscore not ready"; exit 9; }

roslaunch /opt/ros1-slam/r633/inputs/smoke.launch > /opt/ros1-slam/r633/logs/roslaunch.log 2>&1 &
RL_PID=$!
sleep 4
if ! rosnode list 2>/dev/null | grep -q /scan_planner_node; then
  echo "scan_planner_node not up"; tail -30 /opt/ros1-slam/r633/logs/roslaunch.log; exit 9
fi

python3 -B /opt/ros1-slam/r633/inputs/smoke_driver.py
RC=$?

kill -INT "$RL_PID" 2>/dev/null; sleep 3; kill "$RL_PID" 2>/dev/null
kill "$CORE_PID" 2>/dev/null; sleep 2
trap - EXIT

# 退净：容器内不得残留规划进程与 ROS 进程
LEFT=$(pgrep -af "scan_planner_node|rosmaster|roslaunch|rosout" | grep -v pgrep || true)
if [ -n "$LEFT" ]; then echo "LEFTOVER: $LEFT"; exit 7; fi
echo "CLEAN"
exit $RC
