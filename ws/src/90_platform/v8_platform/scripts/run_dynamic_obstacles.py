#!/usr/bin/env python3
"""roslaunch 入口 shim（动态障碍：点云 → /v8/local/obstacles）。

与 run_map_file.py 同一写法：roslaunch 的 <node type> 只认包内顶层可执行名，
而实现在 framework/nodes/ 子目录。rospkg 定位后改 sys.path，不复制实现。

注意：该模块没有 main()，入口是 `DynamicObstaclesNode().spin()`；节点名由 roslaunch 的
`__name:=perception` 覆盖，因此节点私有参数（/v8/3d/perception/perception/*）照常解析。
"""
import os
import sys

import rospkg
import rospy

sys.path.insert(0, os.path.join(rospkg.RosPack().get_path('v8_platform'), 'framework'))

import nodes.dynamic_obstacles_node as impl  # noqa: E402

if __name__ == '__main__':
    rospy.init_node('dynamic_obstacles')
    impl.DynamicObstaclesNode().spin()
