#!/usr/bin/env bash
# g8sim_run.sh —— G8-sim 返工R1 单轮回归（三连之一）
# 用法: bash g8sim_run.sh <run编号>   （以 ros1-slam 身份、ROS 环境已 source 后执行）
# 起序: roscore -p 11312 → 95_sim.launch(use_gpu:=false) → collector → faster_lio(10_slam_mapping_sim.launch)
# 停序: faster_lio 先停（SIGINT，等 "finish done" 日志打完/pcd 落盘）→ sim launch → collector → roscore
# 注：不设 set -u——ROS setup.bash 的 profile.d 段在 -u 下报 ROS_DISTRO unbound（首跑实证）
RUN="$1"
EV=/home/ros1-slam/note/进度/v7/S4/G8sim/证据/log
LOG="$EV/R1_run${RUN}"
source /opt/ros/noetic/setup.bash
source /home/ros1-slam/ws/devel/setup.bash
export ROS_HOME=/home/ros1-slam/.ros ROS_LOG_DIR=/home/ros1-slam/logs
export ROS_MASTER_URI=http://192.168.1.103:11312 ROS_IP=192.168.1.103

# 0) 现场清洁检查：11312 端口不得已占用
if (exec 3<>/dev/tcp/127.0.0.1/11312) 2>/dev/null; then
  exec 3>&- 3<&-
  echo "[run$RUN] FATAL: 端口 11312 已被占用（roscore 残留），拒绝起跑" >&2
  exit 3
fi
echo "== run$RUN start $(date +%T)" > "$LOG.overall.log"

# 0.1) v1 底图保护备份（跑前离位）
cp -a /home/ros1-slam/data/maps/lite3.pcd "/tmp/lite3.pcd.v1bak_run${RUN}"
sha256sum "/tmp/lite3.pcd.v1bak_run${RUN}" >> "$LOG.overall.log"

# 1) roscore
roscore -p 11312 > "$LOG.roscore.log" 2>&1 &
CORE_PID=$!
sleep 3
if ! kill -0 $CORE_PID 2>/dev/null; then echo "[run$RUN] FATAL: roscore 未存活" >&2; exit 3; fi

# 2) sim launch（CPU 渲染；内含 sim_adapter 轨迹源）
roslaunch lite3_drivers 95_sim.launch rviz:=false use_gpu:=false > "$LOG.sim.log" 2>&1 &
SIM_PID=$!
sleep 2

# 3) 采集器（GT 50Hz + /Odometry 全程）
python3 /tmp/g8sim_collector.py "$LOG" > "$LOG.collector.log" 2>&1 &
COLL_PID=$!
sleep 2

# 4) faster_lio（sim yaml）
T0=$(date +%s)
echo "T0=$T0 $(date +%T)" >> "$LOG.overall.log"
roslaunch lite3_slam 10_slam_mapping_sim.launch > "$LOG.lio.log" 2>&1 &
LIO_PID=$!

# 5) 跑满 149s
sleep 149

# 6) 停序①：faster_lio 先停（SIGINT 优雅落盘），等日志打完
kill -INT $LIO_PID 2>/dev/null
DONE=0
for i in $(seq 90); do
  if ! kill -0 $LIO_PID 2>/dev/null; then DONE=1; break; fi
  grep -q "finish done" "$LOG.lio.log" 2>/dev/null && DONE=1 && break
  sleep 1
done
sleep 5   # 日志/落盘缓冲收尾
echo "lio_stopped=$DONE $(date +%T) pcd=$(stat -c %s /home/ros1-slam/data/maps/lite3.pcd 2>/dev/null)" >> "$LOG.overall.log"
sha256sum /home/ros1-slam/data/maps/lite3.pcd >> "$LOG.overall.log"

# 7) 停序②：sim launch（INT roslaunch → 子节点 adapter/render/mockamap）
kill -INT $SIM_PID 2>/dev/null
for i in $(seq 30); do kill -0 $SIM_PID 2>/dev/null || break; sleep 1; done
kill $COLL_PID 2>/dev/null
sleep 2

# 8) roscore 退
kill -INT $CORE_PID 2>/dev/null
sleep 2
kill -9 $CORE_PID 2>/dev/null
sleep 1
echo "== run$RUN end $(date +%T)" >> "$LOG.overall.log"
echo "== run$RUN 退场进程核检（应无 ROS 残留）" >> "$LOG.overall.log"
pgrep -u ros1-slam -f "roscore|roslaunch|rosmaster|rosout|run_mapping_online|sim_adapter|pcl_render|mockamap" >> "$LOG.overall.log" 2>/dev/null || echo "(none)" >> "$LOG.overall.log"
exit 0
