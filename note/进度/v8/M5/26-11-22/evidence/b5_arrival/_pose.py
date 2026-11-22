import sys, time, rospy
from geometry_msgs.msg import PoseStamped
got = {}
def cb(msg):
    got['p'] = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
rospy.init_node('b5_pose', anonymous=True, disable_signals=True)
rospy.Subscriber('/v8/local/pose', PoseStamped, cb, queue_size=1)
end = time.monotonic() + 25.0
while not rospy.is_shutdown() and time.monotonic() < end and 'p' not in got:
    time.sleep(0.05)
p = got.get('p', (float('nan'),) * 3)
print('POSE %.3f %.3f %.3f' % p)
