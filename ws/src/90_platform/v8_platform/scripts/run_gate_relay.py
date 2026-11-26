#!/usr/bin/env python3
"""roslaunch 入口 shim（LIO 数据闸门）。与其它 shim 同一模式：type 只接受纯文件名。"""
import os
import sys

import rospkg

sys.path.insert(0, os.path.join(rospkg.RosPack().get_path('v8_platform'), 'framework'))

from nodes import gate_relay_node  # noqa: E402

if __name__ == '__main__':
    gate_relay_node.main()
