"""FAST-LIO 双轨：驱动平面底盘走一段，同时记录 /Odometry 与 /sim/odom_gt。"""
import json, math, sys, time
import numpy as np, rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

rospy.init_node('lio_dual', disable_signals=True)
out = sys.argv[1] if len(sys.argv) > 1 else '/work/lio_dual.json'
est, gt = {}, {}
est_list, gt_list = [], []

def yaw(q):
    w, x, y, z = q.w, q.x, q.y, q.z
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

def cb_est(m):
    est_list.append((m.header.stamp.to_sec(), m.pose.pose.position.x, m.pose.pose.position.y,
                     m.pose.pose.position.z, yaw(m.pose.pose.orientation)))

def cb_gt(m):
    gt_list.append((m.header.stamp.to_sec(), m.pose.pose.position.x, m.pose.pose.position.y,
                    m.pose.pose.position.z, yaw(m.pose.pose.orientation)))

rospy.Subscriber('/Odometry', Odometry, cb_est, queue_size=50)
rospy.Subscriber('/sim/odom_gt', Odometry, cb_gt, queue_size=50)
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
time.sleep(4)   # 等 LIO 初始化
t0 = time.time()
r = rospy.Rate(20)
segments = [(0.3, 0.0, 6.0), (0.0, 0.3, 3.0), (0.3, 0.0, 6.0), (0.0, 0.0, 2.0)]
for v, w, dur in segments:
    end = time.time() + dur
    cmd = Twist(); cmd.linear.x = v; cmd.angular.z = w
    while time.time() < end:
        pub.publish(cmd); r.sleep()

def interp(series, t):
    if not series:
        return None
    ts = [s[0] for s in series]
    i = int(np.searchsorted(ts, t))
    if i <= 0:
        return series[0]
    if i >= len(series):
        return series[-1]
    a, b = series[i - 1], series[i]
    if b[0] - a[0] == 0:
        return a
    f = (t - a[0]) / (b[0] - a[0])
    return tuple(a[k] + f * (b[k] - a[k]) for k in range(5))

pairs = []
for e in est_list:
    g = interp(gt_list, e[0])
    if g is None:
        continue
    gt_t = min(gt_list, key=lambda s: abs(s[0] - e[0]))[0]
    if abs(gt_t - e[0]) > 0.05:          # 半周期拒绝
        continue
    pairs.append((e[0], e[1] - g[1], e[2] - g[2], e[3] - g[3], e[4] - g[4]))
t_start = est_list[0][0] + 2.0 if est_list else 0
pairs = [p for p in pairs if p[0] >= t_start]
if not pairs:
    print(json.dumps({'ok': False, 'reason': 'no_pairs', 'n_est': len(est_list), 'n_gt': len(gt_list)}))
    sys.exit(0)
# 固定初值偏移（取首个有效配对，非逐段拟合）
dx0, dy0, dz0, dyaw0 = pairs[0][1], pairs[0][2], pairs[0][3], pairs[0][4]
raw_xy = [math.hypot(p[1], p[2]) for p in pairs]
corr_xy = [math.hypot(p[1] - dx0, p[2] - dy0) for p in pairs]
raw_z = [abs(p[3]) for p in pairs]
corr_z = [abs(p[3] - dz0) for p in pairs]
yawerr = [abs(math.atan2(math.sin(p[4] - dyaw0), math.cos(p[4] - dyaw0))) for p in pairs]
def st(v):
    v = sorted(v)
    return {'n': len(v), 'max': round(v[-1], 4), 'p95': round(v[int(round(0.95 * (len(v) - 1)))], 4),
            'p50': round(v[len(v) // 2], 4)}
res = {'ok': True, 'clock': 'sim (use_sim_time)', 'n_est': len(est_list), 'n_gt': len(gt_list),
       'n_pairs': len(pairs), 't_offset0': [round(dx0, 4), round(dy0, 4), round(dz0, 4), round(dyaw0, 4)],
       'raw_xy': st(raw_xy), 'corr_xy': st(corr_xy), 'raw_z': st(raw_z), 'corr_z': st(corr_z),
       'yaw_deg': st([math.degrees(x) for x in yawerr]),
       'gt_end': gt_list[-1] if gt_list else None}
with open(out, 'w') as f:
    json.dump(res, f)
print(json.dumps(res))
