#!/usr/bin/env python3
"""给 BT 下**一条**目标，并确认它真的被接下了（不是把目标刷成多个任务）。

为什么必须只发一条（2026-11-26 实测事故）：
  `bt_console` 的 `/v8/bt/goal` 是 **FIFO 多任务入口**——每收到一条消息就
  `current_seq = ++goal_seq` 并把目标入队（`bt_console.cpp` "goal queued (depth=N)"），
  这是为真实多目标任务设计的。而验收脚本里惯用的
  `timeout 3 rostopic pub -r 5 …` 会在 3 秒内发 15 条 ⇒ **同一件事被拆成 15 个任务**：
  每出队一个就重发一次全局路径、重发一次 `follow path`，`path_goal` 把 local 反复重置；
  更要紧的是"出队下一个目标"会把上一个目标记成 `goal_outcome=reached`
  ——于是 BT 报 arrived，而车其实还差 0.6 m（实测 est_to_goal=0.595、gt=0.677）。
  验收判据于是得到「bt_reached=true 但没到达」这种自相矛盾的读数。

做法：发一条、读 `/v8/bt/status` 的 `current_seq` 确认受理；没受理才补发（最多 N 次）。
`/v8/bt/status` 是 std_msgs/String + JSON，字段见 bt_console.cpp 的 status builder。
"""
import argparse
import json
import sys
import time

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--x', type=float, required=True)
    ap.add_argument('--y', type=float, required=True)
    ap.add_argument('--z', type=float, default=0.0)
    ap.add_argument('--frame', default='map')
    ap.add_argument('--goal-topic', default='/v8/bt/goal')
    ap.add_argument('--status-topic', default='/v8/bt/status')
    ap.add_argument('--retries', type=int, default=3)
    ap.add_argument('--wait-s', type=float, default=8.0,
                    help='单次等待"受理凭据"的墙钟秒数；BT 在恢复/重规划时会晚一点出队')
    ap.add_argument('--out', default='')
    args = ap.parse_args()

    rospy.init_node('bt_goal_once', anonymous=True, disable_signals=True)
    state = {'status': {}, 'seen': 0}

    def on_status(msg):
        try:
            state['status'] = json.loads(msg.data)
            state['seen'] += 1
        except Exception:
            pass

    rospy.Subscriber(args.status_topic, String, on_status, queue_size=20)
    goal_pub = rospy.Publisher(args.goal_topic, PoseStamped, queue_size=1)

    # 等发布端握手（这条必须有）；状态只是"确认受理"的凭据，可以短等。
    # ★不要等到 20 s★：验收探针的窗口是墙钟且通常 45 s，等太久会把目标挤到窗口之后
    # （实测：探针已结束、目标才发出，读数成了"车一步没走"）。
    t_conn = time.monotonic()
    while goal_pub.get_num_connections() == 0 and time.monotonic() - t_conn < 10.0:
        rospy.sleep(0.1)
    t_st = time.monotonic()
    while state['seen'] == 0 and time.monotonic() - t_st < 6.0:
        rospy.sleep(0.1)

    def make():
        m = PoseStamped()
        m.header.stamp = rospy.Time.now()
        m.header.frame_id = args.frame
        m.pose.position.x, m.pose.position.y, m.pose.position.z = args.x, args.y, args.z
        m.pose.orientation.w = 1.0
        return m

    before = int(state['status'].get('current_seq', 0) or 0)
    depth_before = state['status'].get('queue_depth')
    have_status = state['seen'] > 0
    attempts = 0
    accepted = False
    queued_only = False
    for i in range(max(1, args.retries)):
        attempts += 1
        goal_pub.publish(make())          # ★单发★：绝不 -r 循环刷
        end = time.monotonic() + args.wait_s
        while time.monotonic() < end:
            now = int(state['status'].get('current_seq', 0) or 0)
            depth = state['status'].get('queue_depth')
            if now != before:
                accepted = True
                break
            # ★入队也算受理★：BT 在"定位恢复中/重规划中"会先入队、稍后才出队（实测 current_seq
            # 停住 3 s 以上，只等 current_seq 会把"已受理"误判成"被拒"；随后它照样跑完了）。
            if isinstance(depth, int) and isinstance(depth_before, int) and depth > depth_before:
                accepted = True
                queued_only = (now == before)
                break
            rospy.sleep(0.05)
        if accepted:
            break
        if not have_status:
            # 拿不到状态就没有"受理凭据"：这时只按**连接数**判"已发出"，并在结果里如实标注，
            # 不谎称已受理。
            break

    if not have_status and not accepted:
        result_verdict = 'unconfirmed'   # 状态话题没数据，只能证明"发出去了"
    else:
        result_verdict = 'accepted' if accepted else 'rejected'
    result = {'accepted': accepted, 'verdict': result_verdict, 'attempts': attempts,
              'queued_only': queued_only, 'depth_before': depth_before,
              'seq_before': before,
              'seq_after': int(state['status'].get('current_seq', 0) or 0),
              'queue_depth': state['status'].get('queue_depth'),
              'goal': [args.x, args.y, args.z]}
    if args.out:
        with open(args.out, 'w') as fh:
            fh.write(json.dumps(result, ensure_ascii=False) + '\n')
    print(json.dumps(result, ensure_ascii=False), flush=True)
    return 0 if accepted else 2


if __name__ == '__main__':
    sys.exit(main())
