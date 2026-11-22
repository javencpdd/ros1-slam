#!/usr/bin/env python3
"""roslaunch 入口 shim（FAR 全局后端适配器）。

与 `run_global_planner.py` 同一套 rospkg 定位法：roslaunch 的 `<node type>` 只接受
**纯文件名**，而源码在 `v8_algorithm/planner/` 子目录，且依赖同目录模块。
复制源码到包根会让 `__file__` 变化、相对依赖失效（support 版实测过 exit 1）。
"""
import os
import sys

import rospkg

sys.path.insert(0, os.path.join(rospkg.RosPack().get_path('v8_algorithm'), 'planner'))

import far_node  # noqa: E402

if __name__ == '__main__':
    raise SystemExit(far_node.main())
