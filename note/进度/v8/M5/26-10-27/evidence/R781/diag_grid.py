import rospy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid
import sensor_msgs.point_cloud2 as pc2

rospy.init_node('diag_grid', anonymous=True)
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


def dump(name, pick):
    lines = []
    for cy in range(info.height - 1, -1, -1):
        y = oy + (cy + 0.5) * res
        if abs(y) > 1.6:
            continue
        row = ''
        for cx in range(0, info.width, 2):
            row += pick(cx, cy)
        lines.append('%+06.2f %s' % (y, row))
    print('== ' + name)
    print('\n'.join(lines))


def marked(cx, cy):
    v = g.data[cy * info.width + cx]
    return '#' if v >= 99 else '.'


count = {}
for p in pts:
    cx, cy = cell(p[0], ox, info.width), cell(p[1], oy, info.height)
    if cx is None or cy is None:
        continue
    count[(cx, cy)] = count.get((cx, cy), 0) + 1


def pts_grid(cx, cy):
    return '#' if count.get((cx, cy), 0) else '.'


dump('costmap marked (>=99)', marked)
dump('cloud points present (all z)', pts_grid)
