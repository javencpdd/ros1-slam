import collections
import rospy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid
import sensor_msgs.point_cloud2 as pc2

rospy.init_node('diag_z', anonymous=True)
clouds, grids = [], []
rospy.Subscriber('/scan_points', PointCloud2, lambda m: clouds.append(m), queue_size=1)
rospy.Subscriber('/move_base/local_costmap/costmap', OccupancyGrid, lambda m: grids.append(m), queue_size=1)
t0 = rospy.Time.now().to_sec()
while (not clouds or not grids) and rospy.Time.now().to_sec() - t0 < 15:
    rospy.sleep(0.1)
c, g = clouds[-1], grids[-1]
info = g.info
ox, oy, res = info.origin.position.x, info.origin.position.y, info.resolution
pts = [p for p in pc2.read_points(c, field_names=('x', 'y', 'z'), skip_nans=True)
       if abs(p[0]) + abs(p[1]) > 1e-6]


def cell(v, o, n):
    i = int((v - o) / res)
    return i if 0 <= i < n else None


zh = collections.Counter()
samples = []
in_marked = 0
for p in pts:
    cx, cy = cell(p[0], ox, info.width), cell(p[1], oy, info.height)
    if cx is None or cy is None:
        continue
    if g.data[cy * info.width + cx] >= 99:
        in_marked += 1
        zh[round(p[2] * 10) / 10] += 1
        if len(samples) < 8:
            samples.append((round(p[0], 2), round(p[1], 2), round(p[2], 2)))
print('points in marked cells:', in_marked, 'of', len(pts))
print('z hist (marked-cell points):', sorted(zh.items())[:16])
print('samples:', samples)
# 也统计自由格里的点
zh2 = collections.Counter()
for p in pts:
    cx, cy = cell(p[0], ox, info.width), cell(p[1], oy, info.height)
    if cx is None or cy is None:
        continue
    if g.data[cy * info.width + cx] < 99:
        zh2[round(p[2] * 10) / 10] += 1
print('z hist (free cells):', sorted(zh2.items())[:10])
