#!/usr/bin/env python3
"""roslaunch 入口 shim（局部规划器）。"""
import os
import sys

import rospkg

sys.path.insert(0, os.path.join(rospkg.RosPack().get_path('v8_platform'), 'local_planner'))

import ros_node as local_planner_node  # noqa: E402

if __name__ == '__main__':
    local_planner_node.main()
