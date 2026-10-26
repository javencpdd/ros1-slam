"""SCAN 私有探针：消费点云 stamp -> 规划开始/发布 -> cmd_vel。"""
import json, sys, time, bisect
import rospy
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

rospy.init_node('scan_timing_probe', disable_signals=True)
plans, cmds = [], []
rospy.Subscriber('/debug/plan_timing', Float64MultiArray,
                 lambda m: plans.append([float(x) for x in m.data]), queue_size=200)
rospy.Subscriber('/cmd_vel', Twist, lambda m: cmds.append(rospy.Time.now().to_sec()), queue_size=100)
path_pub = rospy.Publisher('/initial_path', Path, queue_size=1)
WPS = [(-5.5, 5.5), (-2.0, 5.5), (1.5, 5.5), (5.0, 5.5), (8.5, 5.5), (12.0, 5.5)]

def send_path(yoff=0.0):
    pa = Path(); pa.header.frame_id = 'world'; pa.header.stamp = rospy.Time.now()
    for (x, y) in WPS:
        g = PoseStamped(); g.header = pa.header
        g.pose.position.x, g.pose.position.y, g.pose.position.z = x, y + yoff, 0.25
        g.pose.orientation.w = 1.0
        pa.poses.append(g)
    path_pub.publish(pa)

dur = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
out = sys.argv[2] if len(sys.argv) > 2 else '/work/scan_timing.json'
yoff = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
if len(sys.argv) > 4 and sys.argv[4] != '-':
    import json as _json
    WPS = [tuple(p) for p in _json.load(open(sys.argv[4]))]
    def send_path(_yoff=0.0):
        pa = Path(); pa.header.frame_id = 'world'; pa.header.stamp = rospy.Time.now()
        for (x, y) in WPS:
            g = PoseStamped(); g.header = pa.header
            g.pose.position.x, g.pose.position.y, g.pose.position.z = x, y, 0.25
            g.pose.orientation.w = 1.0
            pa.poses.append(g)
        path_pub.publish(pa)
time.sleep(2)
send_path(yoff)
t0 = time.time()
next_send = t0 + 20
while time.time() - t0 < dur:
    if time.time() > next_send:
        send_path(yoff); next_send = time.time() + 20
    time.sleep(0.05)

rows = []
for p in plans:
    if len(p) < 4:
        continue
    cloud, pstart, pend, tid = p[0], p[1], p[2], p[3]
    j = bisect.bisect_left(cmds, pend)
    cmd = cmds[j] if j < len(cmds) else None
    rows.append({'cloud_to_bspline_ms': (pend - cloud) * 1000,
                 'plan_compute_ms': (pend - pstart) * 1000,
                 'bspline_to_cmd_ms': (cmd - pend) * 1000 if cmd else None,
                 'traj_id': tid})
def st(key):
    v = sorted(r[key] for r in rows if r[key] is not None)
    if not v:
        return None
    q = lambda p: v[min(len(v) - 1, int(round(p * (len(v) - 1))))]
    return {'n': len(v), 'p50_ms': round(q(0.5), 2), 'p95_ms': round(q(0.95), 2), 'max_ms': round(v[-1], 2)}
res = {'duration_s': dur, 'n_plans': len(rows), 'plan_rate_hz': round(len(rows) / dur, 3),
       'cloud_to_bspline': st('cloud_to_bspline_ms'),
       'plan_compute': st('plan_compute_ms'),
       'bspline_to_cmd': st('bspline_to_cmd_ms'), 'rows': rows}
open(out, 'w').write(json.dumps(res))
print(json.dumps({k: v for k, v in res.items() if k != 'rows'}, ensure_ascii=False))
