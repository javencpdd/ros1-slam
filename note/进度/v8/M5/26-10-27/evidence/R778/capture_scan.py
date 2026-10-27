"""抓一帧 /rslidar_points 存 PCD（binary, x y z intensity）。用法：capture_scan.py <out.pcd> [timeout_s]"""
import struct
import sys

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

out = sys.argv[1]
timeout = float(sys.argv[2]) if len(sys.argv) > 2 else 20.0
rospy.init_node('capture_scan', anonymous=True)
got = []


def cb(msg):
    if got:
        return
    pts = [(p[0], p[1], p[2], p[3] if len(p) > 3 else 0.0)
           for p in pc2.read_points(msg, field_names=('x', 'y', 'z', 'intensity'), skip_nans=True)]
    got.append((msg.header.stamp.to_sec(), msg.header.frame_id, pts))


rospy.Subscriber('/rslidar_points', PointCloud2, cb, queue_size=2)
t0 = rospy.Time.now().to_sec()
while not got and rospy.Time.now().to_sec() - t0 < timeout and not rospy.is_shutdown():
    rospy.sleep(0.05)
if not got:
    print('NO_DATA')
    sys.exit(3)
stamp, frame, pts = got[0]
with open(out, 'wb') as f:
    f.write(b'# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z intensity\n'
            b'SIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n')
    f.write(('WIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA binary\n'
             % (len(pts), len(pts))).encode())
    f.write(struct.pack('<%df' % (4 * len(pts)), *[v for p in pts for v in p]))
print('SAVED %s n=%d frame=%s stamp=%.3f' % (out, len(pts), frame, stamp))
