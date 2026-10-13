#!/usr/bin/env bash
export ROS_MASTER_URI=http://192.168.1.103:11312
export ROS_IP=192.168.1.103
# ★ 环境隔离（2026-10-06）：清 foxy 预载，锁 noetic——ysc 终端 bashrc 常预载 foxy 生产栈
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION ROS_ETC_DIR ROS_ROOT 2>/dev/null || true
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v "/opt/ros/foxy" | paste -sd:)
export PYTHONPATH=$(echo "$PYTHONPATH" | tr ':' '\n' | grep -v "/opt/ros/foxy" | paste -sd:)
export LD_LIBRARY_PATH=$(echo "$LD_LIBRARY_PATH" | tr ':' '\n' | grep -v "/opt/ros/foxy" | paste -sd:)
export ROS_DISTRO=noetic
export ROS_VERSION=1
export ROS_PYTHON_VERSION=3
# ★ 库来源钉成唯一态（A35＝丙，主理人-佳文 2026-10-14 裁）：不读运行账户的 user-site，
#   否则同一台机器上 numpy 会随账户在两套之间跳（ISO 第 [10] 项长期 rc=1 那一红即此病）。
export PYTHONNOUSERSITE=1
source /opt/ros/noetic/setup.bash
# ★ v3 守卫：devel 不存在时明确报错而不是喷一屏 source 失败
[ -f /home/ros1-slam/ws/devel/setup.bash ] || { echo "[env.sh] ws/devel 不存在——先完成编译（P5）"; return 1 2>/dev/null || exit 1; }
source /home/ros1-slam/ws/devel/setup.bash
export ROS_HOME=/home/ros1-slam/.ros
export ROS_LOG_DIR=/home/ros1-slam/logs
export ROS_PACKAGE_PATH=$(echo "$ROS_PACKAGE_PATH" | tr ':' '\n' | grep -v "/home/ysc" | paste -sd:)
