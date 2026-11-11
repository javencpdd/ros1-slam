#!/usr/bin/env python3
"""roslaunch 入口 shim（全局路径 -> 局部目标）。"""
import os
import sys

import rospkg

sys.path.insert(0, os.path.join(rospkg.RosPack().get_path('v8_platform'), 'local_planner'))

import path_goal_node  # noqa: E402

if __name__ == '__main__':
    path_goal_node.main()
