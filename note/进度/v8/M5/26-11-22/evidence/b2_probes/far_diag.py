#!/usr/bin/env python3
"""B2 诊断：世界稳定后取一帧语义地形打分布，并直注 /goal_point 看 FAR 是否出航点。"""
import time

import rospy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

CNT = {'terrain': 0, 'odom': 0, 'waypoint': 0}
SAMPLE = {}


def buckets(values):
    edges = [(0.0, 0.1), (0.1, 0.2), (0.2, 0.5), (0.5, 1.0), (1.0, 1e9)]
    out = []
    for lo, hi in edges:
        out.append(sum(1 for v in values if lo <= v < hi))
    return out


def on_terrain(m):
    CNT['terrain'] += 1
    if CNT['terrain'] != 8:            # 第 8 帧（约 3 s 后）再采样
        return
    pts = list(pc2.read_points(m, field_names=('x', 'y', 'z', 'intensity'), skip_nans=True))
    if not pts:
        SAMPLE['terrain'] = 'EMPTY'
        return
    zs = [p[2] for p in pts]
    hs = [p[3] for p in pts]
    SAMPLE['terrain'] = dict(n=len(pts), z=(round(min(zs), 3), round(max(zs), 3)),
                             h=(round(min(hs), 3), round(max(hs), 3)),
                             h_hist=buckets(hs), frame=m.header.frame_id)


def on_odom(m):
    CNT['odom'] += 1
    if 'odom' not in SAMPLE:
        p = m.pose.pose.position
        SAMPLE['odom'] = (round(p.x, 3), round(p.y, 3), round(p.z, 3))


def on_wp(m):
    CNT['waypoint'] += 1
    SAMPLE['waypoint'] = (round(m.point.x, 3), round(m.point.y, 3), round(m.point.z, 3))


rospy.init_node('far_diag', anonymous=True)
rospy.Subscriber('/terrain_cloud', PointCloud2, on_terrain, queue_size=1)
rospy.Subscriber('/odom_world', Odometry, on_odom, queue_size=5)
rospy.Subscriber('/way_point', PointStamped, on_wp, queue_size=5)
time.sleep(10.0)
print('DIAG_TERRAIN_MSGS=%d' % CNT['terrain'])
print('DIAG_TERRAIN_SAMPLE=%s' % (SAMPLE.get('terrain'),))
print('DIAG_ODOM=%s' % (SAMPLE.get('odom'),))

pub = rospy.Publisher('/goal_point', PointStamped, queue_size=1)
time.sleep(1.0)
g = PointStamped()
g.header.frame_id = 'map'
g.header.stamp = rospy.Time.now()
g.point.x, g.point.y, g.point.z = 2.0, 0.0, 0.11
for _ in range(3):
    pub.publish(g)
    time.sleep(0.5)
time.sleep(25.0)
print('DIAG_WAYPOINTS=%d' % CNT['waypoint'])
print('DIAG_LAST_WAYPOINT=%s' % (SAMPLE.get('waypoint'),))
print('DIAG_TERRAIN_AFTER=%d' % CNT['terrain'])
