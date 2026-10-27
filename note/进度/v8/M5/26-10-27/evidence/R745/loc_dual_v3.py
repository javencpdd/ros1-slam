"""定位双轨 v3：同一 run 内同时评 LIO（/Odometry）与 NDT（/ndt_odom）。

- LIO 输出 body＝雷达系：按 TF 外参 (0,0,0.34, pitch15°) 转到 base_link 再比。
- SE(2) 初始化窗对齐（每后端各自），raw/aligned 双列、逐样本 CSV、score/accept 合并。
- 路径由 GT 偏航驱动掉头（声明：GT 驱动路径、只评 EST），与 R733/R741 同口径。
"""
import argparse
import csv
import json
import math
import time

import numpy as np
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Float32MultiArray

ap = argparse.ArgumentParser()
ap.add_argument('--out', default='/work/loc_v3.json')
ap.add_argument('--csv', default='/work/loc_v3')
ap.add_argument('--init-sec', type=float, default=2.0)
ap.add_argument('--reject', type=float, default=0.05)
ap.add_argument('--no-drive', action='store_true')
args = ap.parse_args()

rospy.init_node('loc_dual_v3', disable_signals=True)
streams = {'lio': [], 'ndt': []}
gt, scores, dbgs = [], [], []


def yaw_of(q):
    w, x, y, z = q.w, q.x, q.y, q.z
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def rot_of(q):
    w, x, y, z = q.w, q.x, q.y, q.z
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def cb_lio(m):
    p = m.pose.pose
    # T_body_base 平移＝-R_y(15°)^T·(0,0,0.34)＝(0.08797, 0, -0.32840)（base→rslidar: z 0.34, pitch +15°）
    off = rot_of(p.orientation) @ np.array([0.08797, 0.0, -0.32840])
    streams['lio'].append((m.header.stamp.to_sec(), p.position.x + off[0], p.position.y + off[1],
                           p.position.z + off[2], yaw_of(p.orientation)))


def cb_ndt(m):
    p = m.pose.pose
    streams['ndt'].append((m.header.stamp.to_sec(), p.position.x, p.position.y, p.position.z,
                           yaw_of(p.orientation)))


def cb_gt(m):
    p = m.pose.pose
    gt.append((m.header.stamp.to_sec(), p.position.x, p.position.y, p.position.z, yaw_of(p.orientation)))


rospy.Subscriber('/Odometry', Odometry, cb_lio, queue_size=100)
rospy.Subscriber('/ndt_odom', Odometry, cb_ndt, queue_size=100)
rospy.Subscriber('/sim/odom_gt', Odometry, cb_gt, queue_size=300)
rospy.Subscriber('/ndt_score', Float32, lambda m: scores.append((rospy.Time.now().to_sec(), float(m.data))), queue_size=200)
rospy.Subscriber('/ndt_dbg', Float32MultiArray, lambda m: dbgs.append((rospy.Time.now().to_sec(), list(m.data))), queue_size=200)
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)


def norm(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def drive():
    r = rospy.Rate(20)

    def simnow():
        return rospy.Time.now().to_sec()

    def cur_yaw():
        return gt[-1][4] if gt else 0.0

    def forward(dur, v=0.3):
        end = simnow() + dur
        cmd = Twist()
        cmd.linear.x = v
        while simnow() < end:
            pub.publish(cmd)
            r.sleep()

    def turn_to(target):
        end = simnow() + 12.0
        while simnow() < end:
            d = norm(target - cur_yaw())
            if abs(d) < 0.04:
                break
            cmd = Twist()
            cmd.angular.z = max(-0.7, min(0.7, 1.5 * d))
            pub.publish(cmd)
            r.sleep()
        pub.publish(Twist())
        time.sleep(0.3)

    time.sleep(4)
    forward(14.0)
    turn_to(math.pi)
    forward(14.0)
    turn_to(0.0)
    forward(14.0)
    pub.publish(Twist())
    time.sleep(1.0)


if not args.no_drive:
    drive()


def interp(series, t):
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


def stats(v):
    v = sorted(v)
    n = len(v)
    return {'n': n, 'max': round(v[-1], 4), 'p95': round(v[int(round(0.95 * (n - 1)))], 4),
            'p50': round(v[n // 2], 4)}


report = {'ok': True, 'clock': 'sim (use_sim_time)', 'n_gt': len(gt),
          'gt_path_len_m': None, 'backends': {}}
if gt:
    report['gt_path_len_m'] = round(sum(math.hypot(gt[i][1] - gt[i - 1][1], gt[i][2] - gt[i - 1][2])
                                        for i in range(1, len(gt))), 3)

for name, series in streams.items():
    if not series or len(gt) < 10:
        report['backends'][name] = {'ok': False, 'reason': 'no_data', 'n_est': len(series)}
        continue
    pairs, rejected = [], []
    for e in series:
        nearest = min(gt, key=lambda s: abs(s[0] - e[0]))
        if abs(nearest[0] - e[0]) > args.reject:
            rejected.append(e[0])
            continue
        pairs.append((e, nearest))
    t_first = pairs[0][0][0] if pairs else 0.0
    pairs = [p for p in pairs if p[0][0] >= t_first + 2.0]
    if len(pairs) < 20:
        report['backends'][name] = {'ok': False, 'reason': 'too_few_pairs', 'n_pairs': len(pairs)}
        continue
    init = [p for p in pairs if p[0][0] <= t_first + 2.0 + args.init_sec] or pairs[:20]
    E = np.array([[p[0][1], p[0][2]] for p in init])
    G = np.array([[p[1][1], p[1][2]] for p in init])
    Ec, Gc = E.mean(axis=0), G.mean(axis=0)
    A, B = E - Ec, G - Gc
    psi = math.atan2(float((A[:, 0] * B[:, 1] - A[:, 1] * B[:, 0]).sum()),
                     float((A[:, 0] * B[:, 0] + A[:, 1] * B[:, 1]).sum()))
    R = np.array([[math.cos(psi), -math.sin(psi)], [math.sin(psi), math.cos(psi)]])
    tvec = Gc - R @ Ec

    rows = []
    for e, g in pairs:
        ax, ay = R @ np.array([e[1], e[2]]) + tvec
        rows.append({'t': round(e[0], 4), 'est_x': round(e[1], 4), 'est_y': round(e[2], 4),
                     'est_z': round(e[3], 4), 'est_yaw': round(e[4], 5),
                     'gt_x': round(g[1], 4), 'gt_y': round(g[2], 4), 'gt_z': round(g[3], 4),
                     'gt_yaw': round(g[4], 5),
                     'raw_xy': round(math.hypot(e[1] - g[1], e[2] - g[2]), 5),
                     'ali_xy': round(math.hypot(ax - g[1], ay - g[2]), 5),
                     'raw_yaw_deg': round(math.degrees(abs(norm(e[4] - g[4]))), 3),
                     'ali_yaw_deg': round(math.degrees(abs(norm(e[4] + psi - g[4]))), 3),
                     'raw_z': round(abs(e[3] - g[3]), 4)})
    worst = max(rows, key=lambda r: r['ali_xy'])
    score_at_worst = None
    accept_at_worst = None
    if name == 'ndt' and scores:
        nearest = min(scores, key=lambda s: abs(s[0] - worst['t']))
        if abs(nearest[0] - worst['t']) <= 0.2:
            score_at_worst = nearest[1]
    if name == 'ndt' and dbgs:
        nearest = min(dbgs, key=lambda s: abs(s[0] - worst['t']))
        if abs(nearest[0] - worst['t']) <= 0.2:
            accept_at_worst = nearest[1]
    csv_path = '%s_%s.csv' % (args.csv, name)
    with open(csv_path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    report['backends'][name] = {
        'ok': True, 'n_est': len(series), 'n_pairs': len(rows), 'n_rejected': len(rejected),
        'reject_rate': round(len(rejected) / max(1, len(series)), 4),
        'init_pairs': len(init), 'init_sec': args.init_sec,
        'se2_psi_rad': round(psi, 6), 'se2_t': [round(float(tvec[0]), 4), round(float(tvec[1]), 4)],
        'raw_xy': stats([r['raw_xy'] for r in rows]),
        'ali_xy': stats([r['ali_xy'] for r in rows]),
        'raw_yaw_deg': stats([r['raw_yaw_deg'] for r in rows]),
        'ali_yaw_deg': stats([r['ali_yaw_deg'] for r in rows]),
        'raw_z': stats([r['raw_z'] for r in rows]),
        'worst_ali': {k: worst[k] for k in ('t', 'ali_xy', 'raw_xy', 'ali_yaw_deg')},
        'score_at_worst': score_at_worst, 'accept_at_worst': accept_at_worst,
        'n_scores': len(scores) if name == 'ndt' else None,
        'n_dbg': len(dbgs) if name == 'ndt' else None,
        'score_stats': (stats([s[1] for s in scores]) if (scores and name == 'ndt') else None),
        'accept_rate': (round(sum(1 for d in dbgs if d[1] and d[1][0] > 0.5) / len(dbgs), 4)
                        if (dbgs and name == 'ndt') else None),
        'csv': csv_path,
    }

with open(args.out, 'w') as f:
    json.dump(report, f)
print(json.dumps(report))
