#!/usr/bin/env python3
"""roslaunch 入口 shim（全局规划器）。

roslaunch 的 <node type="..."> 只接受**纯文件名**（在 <pkg>/scripts/ 或
devel/lib/<pkg>/ 下按名查找），而真正的节点源码在 planner/ 子目录，且依赖
同目录模块与 ../contracts、../representation。直接复制源码到包根会让
`__file__` 变化、相对依赖全部失效（实测 exit code 1）。

这里用 rospkg 定位源码包再改 sys.path：devel space 与真机镜像都成立，
不写死绝对路径，也不把模块复制进安装目录。
"""
import os
import sys

import rospkg

sys.path.insert(0, os.path.join(rospkg.RosPack().get_path('v8_platform'), 'planner'))

import topic_node  # noqa: E402

if __name__ == '__main__':
    topic_node.main()
