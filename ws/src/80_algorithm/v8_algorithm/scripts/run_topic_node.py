#!/usr/bin/env python3
"""roslaunch 入口 shim（全局规划器的 topic 接口（plan 请求/路径发布））。

roslaunch 的 <node type="..."> 只认包内**顶层**可执行名（roslib.find_node 搜
`<pkg>/scripts` 与 devel/lib，不递归子目录），而实现在 `planner/` 下。本 shim 与
`v8_platform/scripts/run_*.py` 同一写法：rospkg 定位包路径后改 sys.path，
devel 与真机镜像都成立，不写死绝对路径，也不复制实现。
"""
import os
import sys

import rospkg

sys.path.insert(0, os.path.join(rospkg.RosPack().get_path('v8_algorithm'), 'planner'))

import topic_node  # noqa: E402

if __name__ == '__main__':
    topic_node.main()
