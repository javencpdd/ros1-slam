#!/usr/bin/env python3
"""roslaunch 入口 shim（支撑图发布）。

与 run_global_planner.py 同一模式：roslaunch 的 <node type="..."> 只接受
纯文件名，而 map_file_node.py 在 framework/ 子目录。framework 无同级相对
依赖，但保持与其它 shim 一致的 rospkg 定位写法，devel space 与真机镜像都成立。
"""
import os
import sys

import rospkg

sys.path.insert(0, os.path.join(rospkg.RosPack().get_path('v8_platform'), 'framework'))

import map_file_node  # noqa: E402

if __name__ == '__main__':
    map_file_node.main()
