import rospy
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
rospy.init_node('r92_fake_tf', anonymous=True, disable_signals=True)
pub = rospy.Publisher('/tf', TFMessage, queue_size=10)
spur = rospy.Publisher('/tf_static', TFMessage, queue_size=10, latch=True)


def mk(p, c, st):
    t = TransformStamped()
    t.header.frame_id = p
    t.child_frame_id = c
    t.header.stamp = st
    t.transform.translation.x = 0.0
    return t


r = rospy.Rate(20)
n = 0
while n < 200 and not rospy.is_shutdown():
    m = TFMessage()
    m.transforms.append(mk('odom', 'base_link', rospy.get_rostime()))
    m.transforms.append(mk('base_link', 'rslidar', rospy.get_rostime()))
    pub.publish(m)
    if n == 0:
        s = TFMessage()
        s.transforms.append(mk('body', 'base_link', rospy.Time(0)))
        spur.publish(s)
    r.sleep()
    n += 1
print('FAKE_TF done rows=%d' % n)
