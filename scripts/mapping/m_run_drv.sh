#!/usr/bin/env bash
source "$(dirname "$0")/m_env.sh"
roslaunch lite3_drivers 01_sensors.launch
# 进程退出后窗内留 bash 便于看报错（勿 exec：秒死即窗关报错丢失）
