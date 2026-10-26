"""全链停车驱动：/src/high 恒 0.3 m/s 6 s，然后 hold=true，记录触发时刻。"""
import json, sys, time
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

rospy.init_node('stop_driver', disable_signals=True)
src = rospy.Publisher('/src/high', Twist, queue_size=1)
hold = rospy.Publisher('/lite3/supervisor/hold', Bool, queue_size=1)
extra = float(sys.argv[1]) if len(sys.argv) > 1 else 0.0
V_CMD = float(sys.argv[2]) if len(sys.argv) > 2 else 0.3
time.sleep(3 + extra)  # 等 mux 与 MuJoCo 桥就绪、机器人进入稳态
cmd = Twist(); cmd.linear.x = V_CMD
t0 = time.time()
r = rospy.Rate(20)
while time.time() - t0 < 6.0:
    src.publish(cmd)
    r.sleep()
t_hold = time.time()
while time.time() - t_hold < 4.0:
    hold.publish(Bool(True))
    src.publish(Twist())
    r.sleep()
print(json.dumps({'t_hold': t_hold, 't_zero_src': t_hold}))
