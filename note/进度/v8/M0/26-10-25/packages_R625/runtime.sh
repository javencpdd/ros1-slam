#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/noetic/setup.bash
source /opt/ros1-slam/r625/ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=127.0.0.1
export GAZEBO_MASTER_URI=http://127.0.0.1:11345
export GAZEBO_MODEL_DATABASE_URI= GAZEBO_MODEL_PATH=/opt/ros1-slam/r625
export GAZEBO_RESOURCE_PATH=/usr/share/gazebo-11
export GAZEBO_PLUGIN_PATH=/opt/ros/noetic/lib:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:$LD_LIBRARY_PATH
export LIBGL_ALWAYS_SOFTWARE=1 XDG_CACHE_HOME=/tmp/ros/cache
exec python3 -B /opt/ros1-slam/r625/inputs/runtime.py
