#!/usr/bin/env bash
source "$(dirname "$0")/m_env.sh"
# ⚠ 停机时 Ctrl-C 后必须等 "current scan saved" 与 "finish done" 两行打完（interval=-1 全量落盘）
exec roslaunch lite3_slam 10_slam_mapping.launch
