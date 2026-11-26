#!/usr/bin/env python3
"""est/gt 受控 A/B 探针（2026-11-26 裁决③）：同一地图/HDL/初始位姿/任务下比较 Support 与 FAR。

要回答的只有一个问题：19 cm 的"到了却没到"到底是**定位偏差**、**执行差异**还是**到达语义**。
所以本探针只做观测，不改任何阈值、不给任何补偿：

  · `init_err`            起始时刻的 est↔gt 差（同一时刻、按消息戳配对）
  · `err_timeline`        全程逐秒的 est↔gt 水平/竖向误差（中位、最大）
  · `hdl_hz`              `/hdl_odom` 实测频率（配准健康度的代理量）
  · `first_reached_wall`  BT 第一次报到达（`outcome` 离开 RUNNING 或 path_state 变化）
  · `stop_wall`           速度归零的时刻
  · `final_pose`          结束时 est 与 gt 各自位置、二者差
  · `cpu`                 采样窗内的 loadavg 与关键进程 CPU

配对口径：est 与 gt 都带 header.stamp，按戳匹配（≤`--pair-tol` 秒取最近），
**不用"各自最新"**——那会把两帧不同时刻的位姿相减，误差里混进速度×时差。

用法：
  est_gt_ab_probe.py --seconds 150 --out /tmp/ab.json
"""
import argparse
import json
import math
import os
import statistics
import time

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import String


def _pos(msg):
    return (float(msg.pose.pose.position.x), float(msg.pose.pose.position.y),
            float(msg.pose.pose.position.z))


def _cpu_sample():
    out = {}
    try:
        out['loadavg'] = [round(v, 2) for v in os.getloadavg()]
    except Exception:
        out['loadavg'] = None
    try:
        res = os.popen("ps -eo pcpu,comm --sort=-pcpu | head -4").read().strip().splitlines()
        out['top'] = [line.split(None, 1) for line in res[1:4]]
    except Exception:
        out['top'] = None
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=150.0)
    ap.add_argument('--pair-tol', type=float, default=0.02)
    ap.add_argument('--est-topic', default='/hdl_odom')
    ap.add_argument('--gt-topic', default='/sim/odom_gt')
    ap.add_argument('--bt-status-topic', default='/v8/bt/status')
    ap.add_argument('--out', default='')
    args = ap.parse_args()

    rospy.init_node('est_gt_ab_probe', anonymous=True, disable_signals=True)
    st = {'est': {}, 'gt': {}, 'bt': [], 'pairs': []}

    def on_est(msg):
        st['est'][round(msg.header.stamp.to_sec(), 3)] = _pos(msg)

    def on_gt(msg):
        st['gt'][round(msg.header.stamp.to_sec(), 3)] = _pos(msg)

    def on_bt(msg):
        try:
            value = json.loads(msg.data)
        except Exception:
            return
        st['bt'].append({'t': round(time.monotonic(), 2),
                         'outcome': value.get('outcome'),
                         'path_state': value.get('path_state'),
                         'path_points': value.get('path_points'),
                         'goal_seq': value.get('goal_seq')})

    rospy.Subscriber(args.est_topic, Odometry, on_est, queue_size=50)
    rospy.Subscriber(args.gt_topic, Odometry, on_gt, queue_size=50)
    rospy.Subscriber(args.bt_status_topic, String, on_bt, queue_size=50)

    t0 = time.monotonic()
    cpu_before = _cpu_sample()
    rospy.sleep(max(1.0, args.seconds))
    wall = max(1e-6, time.monotonic() - t0)

    # ---- 按戳配对 ----
    est_keys = sorted(st['est'])
    gt_keys = sorted(st['gt'])
    pairs = []
    if est_keys and gt_keys:
        j = 0
        for k in est_keys:
            while j + 1 < len(gt_keys) and abs(gt_keys[j + 1] - k) <= abs(gt_keys[j] - k):
                j += 1
            if abs(gt_keys[j] - k) <= args.pair_tol:
                e, g = st['est'][k], st['gt'][gt_keys[j]]
                pairs.append({'t': k,
                              'dxy': math.hypot(e[0] - g[0], e[1] - g[1]),
                              'dz': abs(e[2] - g[2]),
                              'est': e, 'gt': g})
    dxy = [p['dxy'] for p in pairs]
    dz = [p['dz'] for p in pairs]

    # ---- 每秒聚合 ----
    per_sec = []
    if pairs:
        bucket = {}
        for p in pairs:
            bucket.setdefault(int(p['t']), []).append(p['dxy'])
        for sec in sorted(bucket):
            per_sec.append({'sim_s': sec, 'n': len(bucket[sec]),
                            'median_dxy': round(statistics.median(bucket[sec]), 4),
                            'max_dxy': round(max(bucket[sec]), 4)})

    first_reached = None
    for row in st['bt']:
        if row['outcome'] in ('success', 'succeeded', 'reached'):
            first_reached = row['t']
            break

    result = {
        'window_s': round(wall, 2),
        'est_frames': len(est_keys), 'gt_frames': len(gt_keys), 'pairs': len(pairs),
        # ★频率必须用**消息戳**算，不能用墙钟★：仿真时钟远快于墙钟，
        # 拿 len/墙钟 会得出 4.4 Hz 这种假读数（实测踩到）。
        'hdl_hz': (round((len(est_keys) - 1) / (est_keys[-1] - est_keys[0]), 2)
                   if len(est_keys) > 1 and est_keys[-1] > est_keys[0] else None),
        'init_err': ({'dxy': round(dxy[0], 4), 'dz': round(dz[0], 4)}
                     if dxy else None),
        'err_median_dxy': round(statistics.median(dxy), 4) if dxy else None,
        'err_p95_dxy': round(sorted(dxy)[int(0.95 * (len(dxy) - 1))], 4) if dxy else None,
        'err_max_dxy': round(max(dxy), 4) if dxy else None,
        'err_max_dz': round(max(dz), 4) if dz else None,
        'final': ({'est': pairs[-1]['est'], 'gt': pairs[-1]['gt'],
                   'dxy': round(pairs[-1]['dxy'], 4)} if pairs else None),
        'first_reached_wall_s': None if first_reached is None else round(first_reached - t0, 2),
        'bt_outcomes': sorted({r['outcome'] for r in st['bt'] if r['outcome']}),
        'cpu_before': cpu_before, 'cpu_after': _cpu_sample(),
        'err_timeline': per_sec[::max(1, len(per_sec) // 30)] if per_sec else [],
    }
    if args.out:
        with open(args.out, 'w') as fh:
            fh.write(json.dumps(result, ensure_ascii=False) + '\n')
    print(json.dumps({k: result[k] for k in
                      ('hdl_hz', 'pairs', 'init_err', 'err_median_dxy', 'err_max_dxy',
                       'err_max_dz', 'first_reached_wall_s', 'bt_outcomes')},
                     ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
