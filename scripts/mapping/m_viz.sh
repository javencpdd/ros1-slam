#!/usr/bin/env bash
# m_viz.sh — 可视化伴侣（/map_snapshot 累积快照 + Lite3 模型），Foxglove 连 11312 后自动出现
# 超参: 无
source "$(dirname "$0")/m_env.sh"
exec roslaunch lite3_bringup mapping_viz.launch
