import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
rospy.init_node('probe_scan', anonymous=True)
got = []
rospy.Subscriber('/scan_points', PointCloud2, lambda m: got.append(m) if not got else None, queue_size=2)
t0 = rospy.Time.now().to_sec()
while not got and rospy.Time.now().to_sec() - t0 < 20 and not rospy.is_shutdown():
    rospy.sleep(0.05)
if not got:
    print('NO_DATA'); raise SystemExit(3)
m = got[0]
pts = [p for p in pc2.read_points(m, field_names=('x', 'y', 'z'), skip_nans=True) if abs(p[0]) + abs(p[1]) > 1e-6]
print('n', len(pts))
bins = {}
for p in pts:
    if 0.10 < p[2] < 1.20:
        b = round(p[0] * 2) / 2
        bins[b] = bins.get(b, 0) + 1
print('z-in-band x-hist:', sorted(bins.items())[:20])
sel = [p for p in pts if 1.0 < p[0] < 3.0]
print('1<x<3 n', len(sel))
if sel:
    print('y', round(min(p[1] for p in sel), 2), round(max(p[1] for p in sel), 2),
          'z', round(min(p[2] for p in sel), 2), round(max(p[2] for p in sel), 2))
    print(sel[:5])
