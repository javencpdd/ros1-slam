"""全链驱动源：/src/high 恒 0.3 m/s（不直接 hold）。"""
import sys, time
import rospy
from geometry_msgs.msg import Twist

rospy.init_node('haz_chain_driver', disable_signals=True)
pub = rospy.Publisher('/src/high', Twist, queue_size=1)
dur = float(sys.argv[1]) if len(sys.argv) > 1 else 16.0
cmd = Twist(); cmd.linear.x = 0.3
t0 = time.time(); r = rospy.Rate(20)
while time.time() - t0 < dur:
    pub.publish(cmd); r.sleep()
