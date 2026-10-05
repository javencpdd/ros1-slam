#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# sim_map_bridge.py —— mode_nav_sim 仿真测试台真值图桥（小张 2026-10-05，「仿真可用化」payload A2）
#
# 职责：订阅 mockamap 全局真值点云 /map_generator/global_cloud（world 系，0.1m 分辨率）→
#       z 切片 [~z_min, ~z_max] 投影为 2D OccupancyGrid → latched 发布 /map（frame=map，1Hz 重发保活）。
# 必要性：M4 全局 costmap static_layer 订阅 /map（costmap_common.yaml:21），mode_nav_stub 实证无 /map
#       则 GetPath 拒（当时用探针级 stub /map 注入，B 章 P7.2.3 /tmp 零落盘）。长期测试台用「空白 stub 图」
#       会让 GlobalPlanner 穿墙规划（未知=free），无正确性可言——本桥供迷宫真值图，静态层=真墙体，
#       GetPath 的 outcome=0 才有几何含义。
# 坐标约定：world≡map 恒等（与 sim_pose_tf.py 同约定）；迷宫 12×12m 居中，网格 origin=(-size/2,-size/2)。
# 红线：只读点云、只发 /map，不发布/订阅 TF 与速度类话题；/tmp 零落盘（不做 PGM 物化）。
# 参数：~cloud_topic（默认 /map_generator/global_cloud）、~map_topic（默认 /map）、~frame（默认 map）、
#       ~grid_size（默认 12.0m）、~res（默认 0.1m）、~z_min（默认 0.05m）、~z_max（默认 2.0m）、~republish_hz（默认 1.0）。

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2 as pc2
from nav_msgs.msg import OccupancyGrid, MapMetaData


class SimMapBridge(object):
    def __init__(self):
        self.cloud_topic = rospy.get_param('~cloud_topic', '/map_generator/global_cloud')
        self.map_topic = rospy.get_param('~map_topic', '/map')
        self.frame = rospy.get_param('~frame', 'map')
        self.size = float(rospy.get_param('~grid_size', 12.0))
        self.res = float(rospy.get_param('~res', 0.1))
        self.z_min = float(rospy.get_param('~z_min', 0.05))
        self.z_max = float(rospy.get_param('~z_max', 2.0))
        self.republish_hz = float(rospy.get_param('~republish_hz', 1.0))
        self.n = int(round(self.size / self.res))
        self.origin = -self.size / 2.0
        self.grid = None
        self.pub = rospy.Publisher(self.map_topic, OccupancyGrid, queue_size=1, latch=True)
        rospy.Subscriber(self.cloud_topic, PointCloud2, self.on_cloud, queue_size=1)

    def on_cloud(self, msg):
        try:
            pts = pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
            arr = np.array(list(pts), dtype=np.float64)   # 全图一次性物化（≤~50 万点，numpy 秒级）
        except Exception as e:
            rospy.logwarn_throttle(5.0, '[sim_map_bridge] 点云解析失败: %s' % e)
            return
        if arr.size == 0:
            rospy.logwarn_throttle(5.0, '[sim_map_bridge] 空点云，跳过')
            return
        occ = np.zeros((self.n, self.n), dtype=np.int8)   # occ[iy, ix]，row-major 即 OccupancyGrid 布局
        zok = (arr[:, 2] >= self.z_min) & (arr[:, 2] <= self.z_max)
        a = arr[zok]
        if a.size:
            ix = np.floor((a[:, 0] - self.origin) / self.res).astype(np.int64)
            iy = np.floor((a[:, 1] - self.origin) / self.res).astype(np.int64)
            inb = (ix >= 0) & (ix < self.n) & (iy >= 0) & (iy < self.n)
            occ[iy[inb], ix[inb]] = 100                     # 真值墙体占据；其余=free（迷宫外开阔区）
        self.grid = occ
        rospy.loginfo('[sim_map_bridge] /map 已物化：%dx%d @%.2fm，占据格=%d（z∈[%.2f,%.2f]）',
                      self.n, self.n, self.res, int((occ == 100).sum()), self.z_min, self.z_max)

    def spin(self):
        rate = rospy.Rate(self.republish_hz)
        while not rospy.is_shutdown():
            if self.grid is not None:            # latched + 1Hz 重发（晚起节点连接即取，重发保活便于诊断）
                g = OccupancyGrid()
                g.header.stamp, g.header.frame_id = rospy.Time.now(), self.frame
                g.info = MapMetaData()
                g.info.resolution = self.res
                g.info.width = g.info.height = self.n
                g.info.origin.position.x = g.info.origin.position.y = self.origin
                g.info.origin.orientation.w = 1.0
                g.data = self.grid.flatten().tolist()
                self.pub.publish(g)
            rate.sleep()


def main():
    rospy.init_node('sim_map_bridge')
    SimMapBridge().spin()


if __name__ == '__main__':
    main()
