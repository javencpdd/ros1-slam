"""Gazebo m1 目标传感器/真值桥：/rslidar_points -> base_link 点云；/sim/odom_gt -> /LIO/*。"""
import math
import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import Odometry

PITCH = 0.2617993877991494  # base_link -> rslidar: z=0.34, pitch +15°（TF 实测）
C, S = math.cos(PITCH), math.sin(PITCH)
R = np.array([[C, 0.0, S], [0.0, 1.0, 0.0], [-S, 0.0, C]])
T = np.array([0.0, 0.0, 0.34])

rospy.init_node('scan_live_relay', disable_signals=True)
pub_cloud = rospy.Publisher('/LIO/clouds_lidar', PointCloud2, queue_size=5)
pub_veh = rospy.Publisher('/LIO/odom_vehicle', Odometry, queue_size=50)
pub_imu = rospy.Publisher('/LIO/odom_imu', Odometry, queue_size=50)
stat = {'in': 0, 'out': 0, 'drop': 0}

def cb_cloud(m):
    stat['in'] += 1
    try:
        f = {fd.name: fd.offset for fd in m.fields}
        if not all(k in f for k in 'xyz'):
            stat['drop'] += 1
            return
        n = m.width * m.height
        buf = np.frombuffer(m.data, dtype=np.uint8, count=n * m.point_step).reshape(n, m.point_step)
        pts = np.empty((n, 3), dtype=np.float64)
        for i, k in enumerate('xyz'):
            pts[:, i] = buf[:, f[k]:f[k] + 4].copy().view(np.float32).reshape(-1)
        good = np.isfinite(pts).all(axis=1)
        pts = pts[good] @ R.T + T
        out = PointCloud2()
        out.header = m.header
        out.header.frame_id = 'base_link'
        out.height = 1
        out.width = pts.shape[0]
        out.fields = [PointField('x', 0, PointField.FLOAT32, 1),
                      PointField('y', 4, PointField.FLOAT32, 1),
                      PointField('z', 8, PointField.FLOAT32, 1)]
        out.is_bigendian = False
        out.point_step = 12
        out.row_step = 12 * pts.shape[0]
        out.data = pts.astype(np.float32).tobytes()
        out.is_dense = True
        pub_cloud.publish(out)
        stat['out'] += 1
    except Exception as e:
        stat['drop'] += 1
        rospy.logwarn_throttle(5, 'cloud transform fail: %s' % e)

def cb_odom(m):
    pub_veh.publish(m)
    pub_imu.publish(m)

rospy.Subscriber('/rslidar_points', PointCloud2, cb_cloud, queue_size=3)
rospy.Subscriber('/sim/odom_gt', Odometry, cb_odom, queue_size=50)
rospy.Timer(rospy.Duration(10), lambda _: rospy.loginfo('relay %s' % stat))
rospy.spin()
