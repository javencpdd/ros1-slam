#!/usr/bin/env python3
"""B2 台架：用合成点云单独驱动 FAR，隔离「我的数据」与「FAR 内部」。

用法：先起 far_stack.launch，再 rosnode kill /v8_global_far（停掉适配器，
免得它继续往 /terrain_cloud 与 /odom_world 上写），然后跑本件。

合成输入刻意做到无争议：
  /odom_world   静态位姿，z=0.436（Lite3 站立雷达高），frame=map，20 Hz
  /terrain_cloud  ±10 m 地面 0.2 m 网格（intensity=0）+ x=3 m 处一面 2 m 宽、
                 1.2 m 高的墙（intensity=1.0），2.5 Hz，frame=map
  /goal_point  5 s 后发一次 x=8,y=0,z=0.436

判据：
  HARNESS_WAYPOINTS > 0        ⇒ FAR 图初始化成功，问题在真实数据的几何/量纲
  HARNESS_WAYPOINTS = 0 且
  HARNESS_OBS = 0              ⇒ FAR 连合成墙都看不见，问题在 FAR 侧配置/参数
"""
import math
import time

import rospy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2

FRAME = 'map'
ROBOT_Z = 0.436
FIELDS = [PointField(name=n, offset=i * 4, datatype=PointField.FLOAT32, count=1)
          for i, n in enumerate(('x', 'y', 'z', 'intensity'))]

COUNT = {'way': 0, 'obs': 0, 'free': 0, 'new': 0, 'th': 0}
LAST = {'way': None}


def on_way(m):
    COUNT['way'] += 1
    LAST['way'] = (m.point.x, m.point.y, m.point.z)


def counter(key):
    def fn(_m):
        COUNT[key] += 1
    return fn


def ground_and_wall():
    pts = []
    step = 0.2
    n = int(10.0 / step)
    for i in range(-n, n + 1):
        for j in range(-n, n + 1):
            pts.append((i * step, j * step, 0.0, 0.0))
    # 墙：x=3，y∈[-1,1]，z∈[0,1.2]
    for iy in range(-5, 6):
        for iz in range(0, 7):
            pts.append((3.0, iy * 0.2, iz * 0.2, 1.0))
    return pts


rospy.init_node('far_harness', anonymous=True)
odom_pub = rospy.Publisher('/odom_world', Odometry, queue_size=1)
cloud_pub = rospy.Publisher('/terrain_cloud', PointCloud2, queue_size=1)
goal_pub = rospy.Publisher('/goal_point', PointStamped, queue_size=1)
rospy.Subscriber('/way_point', PointStamped, on_way, queue_size=10)
for name, key in (('/FAR_obs_debug', 'obs'), ('/FAR_free_debug', 'free'),
                  ('/FAR_new_debug', 'new'), ('/FAR_terrain_height_debug', 'th')):
    rospy.Subscriber(name, PointCloud2, counter(key), queue_size=1)

CLOUD = ground_and_wall()
print('HARNESS_CLOUD_PTS=%d' % len(CLOUD))
start = time.time()
sent_goal = False
while not rospy.is_shutdown() and time.time() - start < 20.0:
    now = rospy.Time.now()
    odom = Odometry()
    odom.header.stamp = now
    odom.header.frame_id = FRAME
    odom.pose.pose.position.z = ROBOT_Z
    odom.pose.pose.orientation.w = 1.0
    odom_pub.publish(odom)
    cloud_pub.publish(pc2.create_cloud(rospy.Header(stamp=now, frame_id=FRAME), FIELDS, CLOUD))
    if not sent_goal and time.time() - start > 5.0:
        g = PointStamped()
        g.header.stamp = now
        g.header.frame_id = FRAME
        g.point.x, g.point.y, g.point.z = 8.0, 0.0, ROBOT_Z
        goal_pub.publish(g)
        sent_goal = True
        print('HARNESS_GOAL_SENT')
    time.sleep(0.05)

print('HARNESS_MSGS way=%d obs=%d free=%d new=%d th=%d'
      % (COUNT['way'], COUNT['obs'], COUNT['free'], COUNT['new'], COUNT['th']))
print('HARNESS_WAYPOINTS=%d' % COUNT['way'])
print('HARNESS_FIRST_WAY=%s' % (LAST['way'],))
