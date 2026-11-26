#!/usr/bin/env python3
"""取消反控（原子通道版）：证明"取消＝任务撤销"是可区分的判据，不是靠运气。

三相位，缺一不可：
  S 播种：用 mission=M 走**原子通道** `/v8/3d/global/path_task` 下发一次 ⇒ 必须被收；
  R 重放：在 `/v8/local/cancel` 发取消后，把**同一条 mission M 原样**再发一次
          （内容不变、位姿消息自带的时间戳是全新的）⇒ 必须以 `task_revoked` 拒绝；
  N 新任务：mission=M+1 ⇒ 必须被收（反控：撤销只封旧 mission，不能把系统锁死）。

为什么不再用 stamp 当判据（本探针的旧实现就是那样，已作废）：
  跨 topic 没有保序，原子契约下任务身份是 `mission_id`；旧任务重放时 timestamp 一定更新，
  只比 stamp 区分不开"取消后的合法新任务"与"旧任务复活"。
  旧实现在原子模式下每次只拿到 `{ok:true, legacy_ignored:true}` —— 它测的是 legacy 通道，
  已经不在控制路径上了（false-green 的来源）。

用法：
  cancel_replay_probe.py --x 3.9 --y 2.45 --mission 1 --out replay.json
"""
import argparse
import json
import sys
import time

import rospy
from std_msgs.msg import String


def payload(mission, x, y, n=6, spacing=0.15, revision=0):
    # 沿 -x 排布，终点落在 (x, y)：走廊内，不与障碍相交
    poses = [[round(x + spacing * (n - 1 - i), 3), y, 0.0] for i in range(n)]
    return json.dumps({'mission_id': mission, 'path_revision': revision,
                       'waypoint_index': 0, 'execution_generation': 0,
                       'poses': poses}, ensure_ascii=False)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--x', type=float, default=3.9)
    ap.add_argument('--y', type=float, default=2.45)
    ap.add_argument('--mission', type=int, default=1, help='播种用的 mission_id')
    ap.add_argument('--cancel-seq', type=int, default=5)
    ap.add_argument('--path-task-topic', default='/v8/3d/global/path_task')
    ap.add_argument('--status-topic', default='/v8/3d/path_goal/status')
    ap.add_argument('--cancel-topic', default='/v8/local/cancel')
    ap.add_argument('--settle-s', type=float, default=1.5)
    ap.add_argument('--out', default='')
    args = ap.parse_args()

    rospy.init_node('cancel_replay_probe', anonymous=True, disable_signals=True)
    seen = []

    def on_status(msg):
        try:
            seen.append(json.loads(msg.data))
        except Exception:
            pass

    rospy.Subscriber(args.status_topic, String, on_status, queue_size=100)
    task_pub = rospy.Publisher(args.path_task_topic, String, queue_size=5)
    cancel_pub = rospy.Publisher(args.cancel_topic, String, queue_size=5)

    t_conn = time.monotonic()
    while (task_pub.get_num_connections() == 0
           or cancel_pub.get_num_connections() == 0) and time.monotonic() - t_conn < 20.0:
        time.sleep(0.1)
    result = {'path_task_conns': task_pub.get_num_connections(),
              'cancel_conns': cancel_pub.get_num_connections()}

    def publish_and_collect(text, settle):
        mark = len(seen)
        task_pub.publish(String(data=text))
        time.sleep(settle)
        return list(seen[mark:])

    # ★播种编号不能靠猜★：取消刚发生过时 `revoked_mission` 已等于当前任务号，
    # 用同号播种会被直接拒（那会把"探针自己没播上"错判成"系统锁死"）。
    # path_goal 的 status 里现在会带 `mission` 与 `revoked_mission`，据此取 max+1。
    base = None
    t_wait = time.monotonic()
    while time.monotonic() - t_wait < 5.0:
        for st in reversed(seen):
            # 回执里可能只带 revoked_mission（例如刚刚那条 task_revoked），那也够用
            if ('mission' in st or 'revoked_mission' in st
                    or 'mission_id' in st or 'current' in st):
                cand = [st.get('mission'), st.get('revoked_mission'),
                        st.get('mission_id'), st.get('current')]
                vals = [int(v) for v in cand if isinstance(v, (int, float))]
                if vals:
                    base = max(vals)
                    break
        if base is not None:
            break
        time.sleep(0.1)
    seed = (base + 1) if base is not None else args.mission
    result['seed_mission'] = seed
    result['mission_base'] = base

    # --- S 播种 ---
    # 若连一份带编号的回执都没拿到（栈刚起来、还没人发过任务），先探一次低编号：
    # 它要么被拒（回执里带撤销线），要么被收（那就已经拿到 mission 号）——两种情况都能定出 base。
    if base is None:
        probe = publish_and_collect(payload(0, args.x, args.y), 0.8)
        cand = []
        for st in probe:
            for k in ('mission', 'revoked_mission', 'mission_id', 'current'):
                v = st.get(k)
                if isinstance(v, (int, float)):
                    cand.append(int(v))
        if cand:
            base = max(cand)
            seed = base + 1
            result['seed_mission'] = seed
            result['mission_base'] = base
    replies = publish_and_collect(payload(seed, args.x, args.y), args.settle_s)
    result['seed_replies'] = replies[:3]
    result['seeded'] = any(r.get('ok') and not r.get('legacy_ignored') for r in replies)

    # --- 取消（与重放紧挨着，保证"撤销线"就是刚刚那次取消）---
    cancel_pub.publish(String(data='%d:all:probe' % args.cancel_seq))
    time.sleep(1.0)

    # --- R 重放同一条 mission（内容完全相同，任何时间戳都是新的）---
    replies = publish_and_collect(payload(seed, args.x, args.y), args.settle_s)
    result['replay_replies'] = replies[:3]
    result['replay_rejected'] = any(r.get('reason') == 'task_revoked' for r in replies)
    result['replay_accepted_badly'] = any(r.get('ok') for r in replies)

    # --- N 新 mission（反控）---
    replies = publish_and_collect(payload(seed + 1, args.x, args.y), args.settle_s)
    result['fresh_replies'] = replies[:3]
    result['fresh_accepted'] = any(r.get('ok') and not r.get('legacy_ignored') for r in replies)

    result['pass'] = bool(result['seeded'] and result['replay_rejected']
                          and not result['replay_accepted_badly'] and result['fresh_accepted'])
    if args.out:
        with open(args.out, 'w') as fh:
            fh.write(json.dumps(result, ensure_ascii=False) + '\n')
    print(json.dumps(result, ensure_ascii=False), flush=True)
    return 0 if result['pass'] else 1


if __name__ == '__main__':
    sys.exit(main())
