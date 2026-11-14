#!/usr/bin/env python3
# m2_free_map.py —— M2-2 免费格底图发布器（小张，2026-10-06）
# 物理层 teb 闭环无静态 2D 图（Gazebo 建图为 3D pcd，M2-3 之前不进 hdl）：给 costmap static_layer
# 一张全 FREE 的 latched OccupancyGrid（20×20m @0.1），真实障碍由 stvl 障碍层吃 /rslidar_points 实时注入
# （costmap_common.yaml C3 终值配置，rolling_window=true 亦在位）。1Hz 重发 fresh stamp。
# 红线：仅 sim 域；/map 写者唯一=sim_map_bridge（规划层）或本节点（物理层），同 master 不共存。
import rospy
from nav_msgs.msg import OccupancyGrid, MapMetaData

W, H, RES = 200, 200, 0.1  # 20m × 20m，全 FREE（0）
if __name__ == '__main__':
    rospy.init_node('m2_free_map')
    pub = rospy.Publisher('/map', OccupancyGrid, queue_size=1, latch=True)
    rate = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        m = OccupancyGrid()
        m.header.stamp = rospy.Time.now()
        m.header.frame_id = 'map'
        m.info = MapMetaData()
        m.info.resolution = RES
        m.info.width, m.info.height = W, H
        m.info.origin.position.x, m.info.origin.position.y = -W*RES/2.0, -H*RES/2.0
        m.info.origin.orientation.w = 1.0
        m.data = [0] * (W * H)
        pub.publish(m)
        rate.sleep()
