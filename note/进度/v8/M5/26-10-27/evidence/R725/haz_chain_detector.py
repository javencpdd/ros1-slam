"""全链探测器：深度中心区 dmin<1.0 -> hold=true（仿真时钟）。"""
import json, sys
import numpy as np, rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

rospy.init_node('haz_chain_detector', disable_signals=True)
out = sys.argv[1] if len(sys.argv) > 1 else '/work/haz_chain_det.json'
THR = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0
hold_pub = rospy.Publisher('/lite3/supervisor/hold', Bool, queue_size=1, latch=True)
fired = False
rec = {'thr': THR, 'frames': []}


def cb(msg):
    global fired
    dep = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
    roi = dep[int(msg.height * 0.15):int(msg.height * 0.50), int(msg.width * 0.3):int(msg.width * 0.7)]
    ok = np.isfinite(roi) & (roi > 0.05) & (roi < 5.0)
    if not ok.any() or fired:
        return
    dmin = float(roi[ok].min())
    rec['frames'].append({'stamp': msg.header.stamp.to_sec(), 'dmin': dmin, 'now': rospy.Time.now().to_sec()})
    if dmin < THR:
        fired = True
        rec['t_hold'] = rospy.Time.now().to_sec()
        rec['hold_dmin'] = dmin
        rec['hold_stamp'] = msg.header.stamp.to_sec()
        hold_pub.publish(Bool(data=True))
        json.dump(rec, open(out, 'w'))
        rospy.logwarn('[haz] fired dmin=%.3f t=%.3f' % (dmin, rec['t_hold']))


rospy.Subscriber('/haz/depth', Image, cb, queue_size=1)
rospy.spin()
