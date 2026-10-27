import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
rospy.init_node('inspect_cloud', anonymous=True)
got = []
rospy.Subscriber('/rslidar_points', PointCloud2, lambda m: got.append(m) if not got else None, queue_size=2)
t0 = rospy.Time.now().to_sec()
while not got and rospy.Time.now().to_sec() - t0 < 15:
    rospy.sleep(0.05)
m = got[0]
names = [f.name for f in m.fields]
print('fields', names, 'width', m.width, 'stamp', m.header.stamp.to_sec())
rows = list(pc2.read_points(m, field_names=('x', 'y', 'z', 'time', 'ring'), skip_nans=True))
print('n', len(rows))
times = sorted(r[3] for r in rows)
print('time min %.6f p50 %.6f max %.6f' % (times[0], times[len(times)//2], times[-1]))
print('sample rows', rows[:3])
