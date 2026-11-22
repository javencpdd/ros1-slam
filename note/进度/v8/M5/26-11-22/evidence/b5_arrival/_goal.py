import sys, time, rospy
from geometry_msgs.msg import PoseStamped
x, y, z = (float(v) for v in sys.argv[1:4])
rospy.init_node('b5_goal', anonymous=True, disable_signals=True)
pub = rospy.Publisher('/v8/global/goal', PoseStamped, queue_size=1, latch=True)
msg = PoseStamped()
msg.header.frame_id = 'map'
msg.header.stamp = rospy.Time.now()
msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = x, y, z
msg.pose.orientation.w = 1.0
for _ in range(5):
    pub.publish(msg)
    time.sleep(0.1)
print('GOAL_SENT %.3f %.3f %.3f' % (x, y, z))
