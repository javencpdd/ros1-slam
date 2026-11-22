#!/usr/bin/env python3
"""B2 诊断二（修正版）：每帧都记，报「最大点数」而非首帧——首帧常是空的，只看首帧会把非空误判成空。"""
import time

import rospy
from sensor_msgs.msg import PointCloud2

TOPICS = ['/terrain_cloud', '/FAR_obs_debug', '/FAR_free_debug', '/FAR_new_debug',
          '/FAR_scanGrid_debug', '/FAR_terrain_height_debug']
ST = {t: {'n': 0, 'max': 0, 'last': 0} for t in TOPICS}


def counter(topic):
    def cb(m):
        size = m.width * m.height
        ST[topic]['n'] += 1
        ST[topic]['last'] = size
        if size > ST[topic]['max']:
            ST[topic]['max'] = size
    return cb


rospy.init_node('far_diag2', anonymous=True)
for t in TOPICS:
    rospy.Subscriber(t, PointCloud2, counter(t), queue_size=1)
time.sleep(25.0)
for t in TOPICS:
    s = ST[t]
    print('DIAG2 %-28s msgs=%-4d max_pts=%-6d last_pts=%d' % (t, s['n'], s['max'], s['last']))
