# 在 N 秒墙钟窗口里数 /v8/bt/arrived 的 true 条数与 /global_path 的唯一时间戳数
import sys, time, rospy
from std_msgs.msg import Bool
from nav_msgs.msg import Path
window = float(sys.argv[1])
state = {'true': 0, 'false': 0, 'stamps': set()}
def on_arrived(msg):
    state['true' if msg.data else 'false'] += 1
def on_path(msg):
    state['stamps'].add(msg.header.stamp.to_nsec())
rospy.init_node('b5_watch', anonymous=True, disable_signals=True)
rospy.Subscriber('/v8/bt/arrived', Bool, on_arrived, queue_size=10)
rospy.Subscriber('/global_path', Path, on_path, queue_size=10)
end = time.monotonic() + window
while not rospy.is_shutdown() and time.monotonic() < end:
    time.sleep(0.05)
print('ARRIVED_TRUE=%d ARRIVED_FALSE=%d PATH_UNIQ_STAMPS=%d'
      % (state['true'], state['false'], len(state['stamps'])))
