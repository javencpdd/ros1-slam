#!/usr/bin/env python3
"""B2 诊断三：取稳定后「最新一帧」语义地形，打 h 分布，判定障碍点到底有没有。"""
import time

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

LAST = {'msg': None, 'n': 0}


def on_terrain(m):
    LAST['msg'] = m
    LAST['n'] += 1


rospy.init_node('far_diag3', anonymous=True)
rospy.Subscriber('/terrain_cloud', PointCloud2, on_terrain, queue_size=1)
time.sleep(15.0)
m = LAST['msg']
print('DIAG3_MSGS=%d' % LAST['n'])
if m is None:
    print('DIAG3_NO_TERRAIN')
else:
    pts = list(pc2.read_points(m, field_names=('x', 'y', 'z', 'intensity'), skip_nans=True))
    hs = [p[3] for p in pts]
    zs = [p[2] for p in pts]
    edges = [(0.0, 0.1), (0.1, 0.2), (0.2, 0.5), (0.5, 1.0), (1.0, 1e9)]
    hist = [sum(1 for v in hs if lo <= v < hi) for lo, hi in edges]
    print('DIAG3_N=%d' % len(pts))
    print('DIAG3_Z=(%.3f, %.3f)' % (min(zs), max(zs)))
    print('DIAG3_H=(%.3f, %.3f)' % (min(hs), max(hs)))
    print('DIAG3_HIST=%s' % hist)
    print('DIAG3_OBS_FRAC=%.4f' % ((sum(1 for v in hs if v >= 0.2) / float(len(pts))) if pts else -1))
