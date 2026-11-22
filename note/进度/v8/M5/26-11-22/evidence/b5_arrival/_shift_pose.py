import sys, time, rospy
from geometry_msgs.msg import PoseStamped
x, y, z, dur = (float(v) for v in sys.argv[1:5])
rospy.init_node('b5_shift_pose', anonymous=True, disable_signals=True)
pub = rospy.Publisher('/v8/local/pose', PoseStamped, queue_size=2)
msg = PoseStamped()
end = time.monotonic() + dur
while not rospy.is_shutdown() and time.monotonic() < end:
    msg.header.frame_id = 'map'
    msg.header.stamp = rospy.Time.now()
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = x, y, z
    msg.pose.orientation.w = 1.0
    pub.publish(msg)
    time.sleep(0.05)
