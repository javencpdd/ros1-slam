#!/usr/bin/env python3
"""z 契约取证：同源戳配对原始 LIO / 转换后位姿 / TF / GT / 支撑面。

为什么必须同戳：z 的差异有三种可能来源（LIO 世界基准、机体→规划参考点、地图支撑面），
不同时刻各取一个数做减法，分不清是哪一种。这里按源戳最近邻配对，只统计**配对成功**的样本。

用法：z_contract_probe.py [--seconds 30] [--out out.json]
只读：不发目标、不改参数。
"""
import argparse
import json
import statistics as st
import time

import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


def _pct(v, q):
    s = sorted(v)
    return s[min(len(s) - 1, int(q * (len(s) - 1)))]


def _summary(v):
    if not v:
        return None
    return {'n': len(v), 'min': round(min(v), 4), 'max': round(max(v), 4),
            'median': round(st.median(v), 4), 'std': round(st.pstdev(v), 5),
            'p95': round(_pct(v, .95), 4)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=30.0)
    ap.add_argument('--out', default='')
    a = ap.parse_args()

    rospy.init_node('z_contract_probe', anonymous=True, disable_signals=True)
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf)

    lio, conv, gt, tfz = [], [], [], []

    def on_lio(m):
        p = m.pose.pose.position
        lio.append((m.header.stamp.to_sec(), p.x, p.y, p.z))

    def on_conv(m):
        p = m.pose.position
        conv.append((m.header.stamp.to_sec(), p.x, p.y, p.z))

    def on_gt(m):
        p = m.pose.pose.position
        gt.append((m.header.stamp.to_sec(), p.x, p.y, p.z))

    rospy.Subscriber('/Odometry', Odometry, on_lio, queue_size=100)
    rospy.Subscriber('/v8/local/pose', PoseStamped, on_conv, queue_size=100)
    rospy.Subscriber('/sim/odom_gt', Odometry, on_gt, queue_size=100)

    t0 = time.monotonic()
    while time.monotonic() - t0 < a.seconds and not rospy.is_shutdown():
        try:
            tr = buf.lookup_transform('map', 'base_link', rospy.Time(0))
            tfz.append((rospy.get_time(), tr.transform.translation.x,
                        tr.transform.translation.y, tr.transform.translation.z))
        except Exception:
            pass
        rospy.sleep(0.1)

    def nearest(series, stamp, tol=0.15):
        best, dist = None, 1e9
        for row in series:
            d = abs(row[0] - stamp)
            if d < dist:
                best, dist = row, d
        return best if best is not None and dist <= tol else None

    pairs_conv_lio, pairs_conv_gt, pairs_conv_tf = [], [], []
    for row in conv:
        l = nearest(lio, row[0])
        g = nearest(gt, row[0])
        t = nearest(tfz, row[0])
        if l:
            pairs_conv_lio.append(row[3] - l[3])
        if g:
            pairs_conv_gt.append(row[3] - g[3])
        if t:
            pairs_conv_tf.append(row[3] - t[3])

    result = {
        'seconds': a.seconds,
        'channels': {'lio_z': _summary([r[3] for r in lio]),
                     'converted_z': _summary([r[3] for r in conv]),
                     'gt_z': _summary([r[3] for r in gt]),
                     'tf_z': _summary([r[3] for r in tfz])},
        'paired_delta_z': {'converted_minus_lio': _summary(pairs_conv_lio),
                           'converted_minus_gt': _summary(pairs_conv_gt),
                           'converted_minus_tf': _summary(pairs_conv_tf)},
        'pairing': {'n_conv': len(conv), 'paired_with_lio': len(pairs_conv_lio),
                    'paired_with_gt': len(pairs_conv_gt), 'paired_with_tf': len(pairs_conv_tf)},
    }
    print(json.dumps(result, ensure_ascii=False, indent=1))
    if a.out:
        with open(a.out, 'w', encoding='utf-8') as fh:
            json.dump(result, fh, ensure_ascii=False, indent=1)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
