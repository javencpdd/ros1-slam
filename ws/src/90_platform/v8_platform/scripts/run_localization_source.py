#!/usr/bin/env python3
"""roslaunch 入口 shim（定位源）。

与 run_map_file.py 同一模式：roslaunch 的 <node type="..."> 只接受纯文件名，
而 localization_node.py 在 framework/nodes/ 子目录。

为什么必须补这个 shim：manifest 里 type 写成 framework/nodes/localization_node.py 时，
roslaunch 找不到该类型，进程根本不启动，且日志目录里连 v8-3d-loc-*.log 都不会建，
表现为「图里 8 个实例只起来 7 个」，很容易被数量判据漏掉。
"""
import os
import sys

import rospkg

sys.path.insert(0, os.path.join(rospkg.RosPack().get_path('v8_platform'), 'framework'))

from nodes import localization_node  # noqa: E402

if __name__ == '__main__':
    localization_node.main()
