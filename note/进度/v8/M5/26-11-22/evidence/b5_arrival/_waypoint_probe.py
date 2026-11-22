import time, rospy
from geometry_msgs.msg import PointStamped
seen = []
def cb(msg):
    seen.append(msg)
rospy.init_node('b5_wp_probe', anonymous=True, disable_signals=True)
rospy.Subscriber('/way_point', PointStamped, cb, queue_size=5)
end = time.monotonic() + 120.0
while not rospy.is_shutdown() and time.monotonic() < end and not seen:
    time.sleep(0.1)
print('WAYPOINT_SEEN=%d' % len(seen))
