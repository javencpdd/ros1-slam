#!/usr/bin/env bash
source "$(dirname "$0")/m_env.sh"
# ⚠ 停机时 Ctrl-C 后必须等 "current scan saved" 与 "finish done" 两行打完（interval=-1 全量落盘）
roslaunch lite3_slam 10_slam_mapping.launch
# 进程退出后窗内留 bash 便于看报错（勿 exec：秒死即窗关报错丢失）
