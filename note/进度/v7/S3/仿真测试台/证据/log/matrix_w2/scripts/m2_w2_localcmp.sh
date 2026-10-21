#!/usr/bin/env bash
# m2_w2_localcmp.sh —— 矩阵W2/A4 局部规划对比 T2 harness（小明 2026-10-07）
# m2_smoke.sh stageB 同款门控（G1–G4）+ mbf 按名点单（GetPath→ExePath controller=<NAME>）：
#   teb ×1（录平滑度/evo 数据的对照跑，基线 dt=8.6s/dev=0.179m 在案）+ DWA ×3（三连判据）。
# 判据（C 文 D 表 T2 行）：state=SUCCEEDED(3)/outcome=0、dt≤120s(sim)、dev_odom≤0.35m、平滑度=cmd_vel 角速度一阶差分 RMS。
# A0 RTF 门：运动窗 IMU stamp 增量/墙钟增量，门值 0.25（M2R2 主理人裁决在案；C 文写 0.3 与平台 0.288–0.29 自相矛盾——
#   实测值逐跑上报，双门槛均可复核，冲突已报告待主理人确认）。
# 存档：gt/odom TUM 序列（evo APE 用，M2 旧档无时序 gt/odom——本次采数窗同步存档，偏差登记）+ cmd_vel CSV。
# 用法: bash m2_w2_localcmp.sh <controller> <runlabel>   # controller=TebLocalPlannerROS|DWAPlannerROS
CTRL=$1; LABEL=$2
LOGDIR=/tmp/matrix_w2/logs; mkdir -p "$LOGDIR"
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION
source /opt/ros/noetic/setup.bash
source /home/ros1-slam/ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11313 GAZEBO_MASTER_URI=http://127.0.0.1:11345
export GAZEBO_PLUGIN_PATH=/opt/ros/noetic/lib:/home/ros1-slam/ws/devel/lib

cleanup() { rosnode kill -a 2>/dev/null; sleep 3
  pkill -f gzserver 2>/dev/null; pkill -f gzclient 2>/dev/null
  pkill -f "roslaunch.*m2_teb" 2>/dev/null; pkill -f move_base_flex 2>/dev/null
  pkill -f sim_adapter 2>/dev/null; sleep 2; }
trap cleanup EXIT

echo "[w2-T2] run=$LABEL controller=$CTRL 起 m2_teb.launch..."
setsid roslaunch lite3_gazebo_sim m2_teb.launch > "$LOGDIR/T2_${LABEL}_launch.log" 2>&1 &
# 等 /clock 就绪后再起执行体（保证 use_sim_time 参数与 /clock 已在位，stageB wait_topic 同款）；
# 短 echo 反复重试（D-M2-P4 修法：单次长 echo 会挂起）
T=0
until [ "$T" -ge 300 ]; do
  if rostopic list 2>/dev/null | grep -qx -- "/clock" && timeout 10 rostopic echo -n 1 /clock >/dev/null 2>&1; then
    echo "[w2-T2] /clock 就绪 t=${T}s"; break
  fi
  sleep 2; T=$((T+2))
done
[ "$T" -lt 300 ] || { echo "[w2-T2] FAIL: /clock 300s 未出"; exit 1; }

python3 /tmp/matrix_w2/w2_t2_exec.py "$CTRL" "$LABEL" 2>&1 | tee "$LOGDIR/T2_${LABEL}.txt"
RC=$?
echo "[w2-T2] rc=$RC，退净..."
exit $RC
