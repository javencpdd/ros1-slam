#!/usr/bin/env python3
"""Drop near-range returns before an external elevation producer.

The ANYbotics ``perfect`` sensor processor has no range cutoff; the sim lidar
sees the carrier's own body at 0.2--0.5 m.  This is wiring (a range gate), not
an elevation algorithm: the open-source producer remains unchanged.
"""
import math
import sys

try:
    import rospy
    from sensor_msgs import point_cloud2
    from sensor_msgs.msg import PointCloud2
except ImportError:  # host self-test without ROS
    rospy = None
    point_cloud2 = None
    PointCloud2 = None


def in_range(x, y, z, min_range, max_range):
    if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
        return False
    r2 = x * x + y * y + z * z
    if r2 < min_range * min_range:
        return False
    if max_range > 0.0 and r2 > max_range * max_range:
        return False
    return True


class RangeFilter(object):
    def __init__(self):
        self.input_topic = rospy.get_param('~input_topic', '/rslidar_points')
        self.output_topic = rospy.get_param('~output_topic', '/rslidar_points_filtered')
        self.min_range = float(rospy.get_param('~min_range', 0.5))
        self.max_range = float(rospy.get_param('~max_range', 0.0))
        self.pub = rospy.Publisher(self.output_topic, PointCloud2, queue_size=2)
        self.sub = rospy.Subscriber(self.input_topic, PointCloud2, self.on_cloud, queue_size=2)
        rospy.loginfo('[range_filter] %s -> %s min=%.2f max=%.2f', self.input_topic,
                      self.output_topic, self.min_range, self.max_range)

    def on_cloud(self, msg):
        names = [field.name for field in msg.fields]
        try:
            ix, iy, iz = names.index('x'), names.index('y'), names.index('z')
        except ValueError:
            rospy.logerr('[range_filter] cloud lacks x/y/z fields')
            return
        kept = [p for p in point_cloud2.read_points(msg, field_names=names, skip_nans=False)
                if in_range(p[ix], p[iy], p[iz], self.min_range, self.max_range)]
        out = point_cloud2.create_cloud(msg.header, msg.fields, kept)
        out.is_dense = False
        self.pub.publish(out)


def main():
    rospy.init_node('pointcloud_range_filter')
    RangeFilter()
    rospy.spin()


def selftest():
    assert in_range(0.5, 0.0, 0.0, 0.5, 0.0)
    assert not in_range(0.49, 0.0, 0.0, 0.5, 0.0)
    assert in_range(0.0, 0.0, 0.6, 0.5, 0.0)
    assert not in_range(0.0, 0.0, 0.4, 0.5, 0.0)
    assert not in_range(float('nan'), 0.0, 0.0, 0.5, 0.0)
    assert not in_range(2.0, 0.0, 0.0, 0.5, 1.0)
    print('RANGE_FILTER_SELFTEST_OK')


if __name__ == '__main__':
    if '--selftest' in sys.argv:
        selftest()
    else:
        main()
