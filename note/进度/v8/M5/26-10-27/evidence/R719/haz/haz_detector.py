"""障碍探测器：深度图中心区最小距离 < 1.0 m -> 发布 hold=true（一次）。"""
import json, sys, time
import numpy as np, rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

rospy.init_node('haz_detector', disable_signals=True)
out = sys.argv[1] if len(sys.argv) > 1 else '/work/haz_detector.json'
THR = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0
hold = rospy.Publisher('/lite3/supervisor/hold', Bool, queue_size=1, latch=True)
events = []
fired = False


def cb(msg):
    global fired
    t_recv = time.time()
    dep = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
    lo, hi = int(msg.width * 0.3), int(msg.width * 0.7)
    roi = dep[int(msg.height * 0.15):int(msg.height * 0.50), lo:hi]
    ok = np.isfinite(roi) & (roi > 0.05) & (roi < 5.0)
    if not ok.any():
        return
    dmin = float(roi[ok].min())
    events.append({'t_recv': t_recv, 'stamp': msg.header.stamp.to_sec(), 'dmin': dmin})
    if not fired and dmin < THR:
        fired = True
        t_hold = time.time()
        hold.publish(Bool(data=True))
        events.append({'t_hold': t_hold, 'dmin': dmin, 'fired': True})
        with open(out, 'w') as f:
            json.dump({'thr': THR, 'events': events}, f)
        print('FIRED dmin=%.3f t=%.6f' % (dmin, t_hold))


rospy.Subscriber('/haz/depth', Image, cb, queue_size=1)
rospy.spin()
