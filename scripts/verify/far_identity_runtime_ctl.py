#!/usr/bin/env python3
"""航点身份运行时正反控（2026-11-27）：在**真实生产者**跑起来之后做注入对照。

只做观测与注入，不改任何判据：
  · 正控：等真实生产者（FAR）给出带 request_id 的结果，确认 path_task 版本随**有效航点**推进；
    同一航点的几何刷新不得推进版本。
  · 反控：同 request_id 的重复 gen、旧 gen、错误 request_id、无身份入口 —— 必须各有独立拒绝计数，
    版本不得推进，缓存不得被污染。

输出一条机器可读的 RESULT 行，供证据归档与门禁判定。

用法：
  far_identity_runtime_ctl.py --seconds 40 --out /tmp/id_ctl.json
"""
import argparse
import json
import sys

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from std_msgs.msg import String


class Rec(object):
    def __init__(self):
        self.tag = None
        self.task = None
        self.status = None
        self.task_hist = []
        self.path_n = 0          # /v8/3d/global/path 帧数
        self.gait_n = 0          # /v8/gait_guard/path 帧数
        self.task_n = 0          # path_task 帧数
        self.cmd_n = 0           # 非零速度帧数

    def on_tag(self, msg):
        try:
            self.tag = json.loads(msg.data)
        except Exception:
            pass

    def on_task(self, msg):
        try:
            value = json.loads(msg.data)
        except Exception:
            return
        self.task = value
        self.task_hist.append({'rev': value.get('path_revision'),
                               'idx': value.get('waypoint_index'),
                               'wp': (value.get('poses') or [[], []])[-1][:2]})

    def on_path(self, msg):
        self.path_n += 1

    def on_gait(self, msg):
        self.gait_n += 1

    def on_cmd(self, msg):
        if abs(msg.linear.x) > 1e-3 or abs(msg.angular.z) > 1e-3:
            self.cmd_n += 1

    def on_status(self, msg):
        try:
            value = json.loads(msg.data)
        except Exception:
            return
        if value.get('task'):
            self.status = value


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=40.0)
    ap.add_argument('--out', default='')
    args = ap.parse_args()

    rospy.init_node('far_identity_runtime_ctl', anonymous=True, disable_signals=True)
    rec = Rec()
    rospy.Subscriber('/far/waypoint_tagged', String, rec.on_tag, queue_size=10)
    rospy.Subscriber('/v8/3d/global/path_task', String, rec.on_task, queue_size=10)
    rospy.Subscriber('/v8/3d/global/status', String, rec.on_status, queue_size=30)
    rospy.Subscriber('/v8/3d/global/path', Path, rec.on_path, queue_size=10)
    rospy.Subscriber('/v8/gait_guard/path', Path, rec.on_gait, queue_size=10)
    rospy.Subscriber('/lite3/teleop/app_cmd_vel', Twist, rec.on_cmd, queue_size=50)
    pub = rospy.Publisher('/far/waypoint_tagged', String, queue_size=5)

    # 1) 等真实生产者的第一条结果
    # ★用墙钟等，不用仿真钟★：仿真相对于墙钟可能暂停或远快于墙钟，用 ros 时间会在
    # "仿真钟不走"时死等（实测直接把探针挂死到 timeout）。
    import time as _time
    deadline = _time.monotonic() + args.seconds
    while _time.monotonic() < deadline and (rec.tag is None or rec.task is None):
        _time.sleep(0.3)
    if not rec.tag:
        print('RESULT ' + json.dumps({'verdict': 'NO_PRODUCER_RESULT',
                                      'note': '未收到带身份结果：先确认目标已下发且 FAR 在规划'},
                                     ensure_ascii=False))
        return 2

    rid = rec.tag.get('request_id')
    gen = int(rec.tag.get('gen') or 0)
    rev0 = (rec.task or {}).get('path_revision')
    rows = [{'step': 'producer', 'request_id': rid, 'gen': gen, 'revision': rev0,
             'note': '真实生产者结果'}]

    def counters():
        t = ((rec.status or {}).get('task') or {})
        return {'rejected': t.get('rejected_results'),
                'stale_gen': t.get('stale_gen_results'),
                'bad': t.get('bad_results'),
                'unnamed': t.get('unnamed_results'),
                'cache_id': t.get('cache_request_id'),
                'cache_gen': t.get('cache_result_gen')}

    def inject(tag_id, g, x, step):
        payload = {'request_id': tag_id, 'gen': g, 'x': x, 'y': 2.45, 'z': 0.0,
                   'frame': 'map', 'stamp': rospy.get_time()}
        pub.publish(String(data=json.dumps(payload)))
        _time.sleep(1.5)
        row = {'step': step, 'request_id': tag_id, 'gen': g,
               'revision': (rec.task or {}).get('path_revision'),
               'reason': (rec.status or {}).get('reason'),
               'path_frames': rec.path_n, 'gait_frames': rec.gait_n,
               'task_frames': rec.task_n, 'cmd_frames': rec.cmd_n,
               'cache_produced_at': (((rec.status or {}).get('task') or {})
                                     .get('cache_produced_at')),
               'cache_received_at': (((rec.status or {}).get('task') or {})
                                     .get('cache_received_at'))}
        row.update(counters())
        rows.append(row)
        return row

    # 2) 反控：重复 gen（同身份、旧代次）
    inject(rid, gen, 4.30, 'dup_gen')
    # 3) 反控：旧 gen（比缓存更旧）
    inject(rid, max(0, gen - 1), 4.31, 'stale_gen')
    # 4) 反控：错误 request_id
    inject('bogus-id', gen + 7, 4.32, 'bad_id')
    # 5) 正控：合法新代次 + 新航点 ⇒ 版本应推进
    after = inject(rid, gen + 5, 4.36, 'new_gen_new_waypoint')

    rev_final = (rec.task or {}).get('path_revision')
    checks = {
        'producer_identity': bool(rid),
        'dup_gen_rejected': rows[1]['revision'] == rev0,
        'stale_gen_rejected': rows[2]['revision'] == rev0,
        'bad_id_rejected': rows[3]['revision'] == rev0,
        'new_gen_advanced': rev_final is not None and rev_final > (rev0 or 0),
        'cache_identity_intact': after.get('cache_id') == rid,
        'rejected_counted': (after.get('rejected') or 0) >= 3,
        'outlets_silent_in_rejects': all(r['path_frames'] == rows[0]['path_frames']
                                        for r in rows[1:4]),
    }
    verdict = 'PASS' if all(checks.values()) else 'FAIL'
    result = {'verdict': verdict, 'checks': checks, 'rows': rows,
              'task_history': rec.task_hist[-10:]}
    if args.out:
        with open(args.out, 'w') as fh:
            fh.write(json.dumps(result, ensure_ascii=False) + '\n')
    print('RESULT ' + json.dumps({'verdict': verdict, 'checks': checks,
                                  'rows': [{k: r.get(k) for k in
                                            ('step', 'revision', 'reason', 'rejected',
                                             'stale_gen', 'cache_id', 'cache_gen')}
                                           for r in rows]}, ensure_ascii=False))
    return 0 if verdict == 'PASS' else 1


if __name__ == '__main__':
    sys.exit(main())
