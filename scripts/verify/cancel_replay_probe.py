#!/usr/bin/env python3
"""取消反控探针：证明「取消后拒旧路径」不是靠运气，而是可区分的新旧判据。

做法是**成对**注入，缺一不可：
  1) 旧 stamp 的路径（stamp < cancelled_at）⇒ 必须被拒，reason=stale_after_cancel；
  2) 新 stamp 的路径（stamp > cancelled_at）⇒ 必须被收，generation +1。
只做 (1) 会得到"拒了"的假绿 —— 万一节点是整体挂了、或者路径格式不对，也一样拒。
只做 (2) 会得到"收了"，但证明不了取消态参与判定。

判据里 `generation` 是**任务身份**（写进 goal.header.seq），不是本地计数器：
拒掉的路径不该让它 +1。

★取消必须与重放同一个进程、紧挨着做★：cancelled_at 是"最近一次取消的时刻"，
若隔几分钟再重放，`now - 30 s` 反而比那次取消更新，会被正常收下（实测踩过）。
所以本探针默认自己先发一条取消，再紧接着喂旧路径。

用法：
  cancel_replay_probe.py --x 3.9 --y 2.45 --out replay.json
"""
import argparse
import json
import sys
import time

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import String


def make_path(x, y, stamp, n=4, frame='map'):
    path = Path()
    path.header.stamp = stamp
    path.header.frame_id = frame
    for i in range(n):
        ps = PoseStamped()
        ps.header.stamp = stamp
        ps.header.frame_id = frame
        # 沿 -x 排布 0.15 m 间隔，终点落在 (x, y)：走廊内，不与障碍相交
        ps.pose.position.x = x + 0.15 * (n - 1 - i)
        ps.pose.position.y = y
        ps.pose.position.z = 0.0
        ps.pose.orientation.w = 1.0
        path.poses.append(ps)
    return path


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--x', type=float, default=3.9)
    ap.add_argument('--y', type=float, default=2.45)
    ap.add_argument('--status-topic', default='/v8/3d/path_goal/status')
    ap.add_argument('--path-topic', default='/v8/3d/global/path')
    ap.add_argument('--stale-back-s', type=float, default=5.0)
    ap.add_argument('--settle-s', type=float, default=1.5)
    ap.add_argument('--cancel-topic', default='/v8/bt/cancel')
    ap.add_argument('--cancel-seq', type=int, default=1)
    ap.add_argument('--no-cancel', action='store_true',
                    help='跳过自发取消（复用上一次取消态，需自行保证时间口径）')
    ap.add_argument('--out', default='')
    args = ap.parse_args()

    rospy.init_node('cancel_replay_probe', anonymous=True)
    seen = []

    def on_status(msg):
        try:
            seen.append(json.loads(msg.data))
        except Exception:
            pass

    rospy.Subscriber(args.status_topic, String, on_status, queue_size=50)
    pub = rospy.Publisher(args.path_topic, Path, queue_size=5)
    cancel_pub = rospy.Publisher(args.cancel_topic, String, queue_size=5)
    t_conn = time.monotonic()
    while (pub.get_num_connections() == 0
           or cancel_pub.get_num_connections() == 0) and time.monotonic() - t_conn < 20.0:
        time.sleep(0.1)

    result = {'conns': pub.get_num_connections(),
              'cancel_conns': cancel_pub.get_num_connections()}

    if not args.no_cancel:
        # 只有自己刚刚取消过，cancelled_at 才确定是"现在"，后续新旧比较才有意义
        cancel_pub.publish(String(data='%d:all:probe' % args.cancel_seq))
        time.sleep(1.0)

    # --- (1) 旧 stamp：应被拒 ---
    now = rospy.Time.now()
    old = make_path(args.x, args.y, now - rospy.Duration(args.stale_back_s))
    mark = len(seen)
    pub.publish(old)
    time.sleep(args.settle_s)
    stale = [s for s in seen[mark:]]
    result['stale_replies'] = stale[:3]
    result['stale_rejected'] = any(
        s.get('reason') == 'stale_after_cancel' for s in stale)
    gen_after_stale = [s.get('generation') for s in stale if s.get('ok')]
    result['stale_generation_bump'] = bool(gen_after_stale)

    # --- (2) 新 stamp：应被收 ---
    now2 = rospy.Time.now()
    fresh = make_path(args.x, args.y, now2 + rospy.Duration(0.5))
    mark = len(seen)
    pub.publish(fresh)
    time.sleep(args.settle_s)
    new = [s for s in seen[mark:]]
    result['fresh_replies'] = new[:3]
    ok = [s for s in new if s.get('ok')]
    result['fresh_accepted'] = bool(ok)
    result['fresh_generation'] = ok[0].get('generation') if ok else None

    result['pass'] = bool(result['stale_rejected'] and result['fresh_accepted']
                          and not result['stale_generation_bump'])
    if args.out:
        open(args.out, 'w').write(json.dumps(result, ensure_ascii=False) + '\n')
    print(json.dumps(result, ensure_ascii=False), flush=True)
    return 0 if result['pass'] else 1


if __name__ == '__main__':
    sys.exit(main())
