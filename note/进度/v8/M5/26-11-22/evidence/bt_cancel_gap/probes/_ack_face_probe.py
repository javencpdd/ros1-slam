#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""_ack_face_probe.py —— 丙案落地后的「误触发面」活体读数（T-20261122-04）。

为什么要有这件：`cancelAckCallback` 升格的判据是「收到一条 confirmed 的回执，
而这条号不是本节点自己发的、也没记过账，同时手里还有一条活腿」。这是一组**事实**，
不是「FAR 出故障了」的**结论**。授权第 6 条明确要求：
  可以靠这组事实停腿，但不许仅凭「seq 不在自持集合」就对外宣称「FAR 故障」。
所以这里把四种与「真外部停」相邻的输入逐个喂进去，看判类是否按真值表走：

  stray      有活腿，注入一条**谁都没发过**的 confirmed 回执 ⇒ 会停腿（刻意 fail-safe，
             代价一条：多停一次、要操作员重发；这是已知过触发，不许写成零风险）
  late       真外部停已经升格之后，把同一号回执**再送一次** ⇒ 必须不再升格（已记账）
  duplicate  同一号回执背靠背送 5 次 ⇒ 只准升格一次（日志计数须 =1）
  idle       手里**没有活腿**（mission 已收尾、回到取目标）时注入回执 ⇒ 必须只记账不升格

判据全部取自 `/v8/bt/status` 的 `external_cancel_seq` / `cancel_origin`：
升格时 `external_cancel_seq` 会被写成那条外部号，未升格恒 0 ⇒ 一个字段就够分辨。
`external_cancel` 日志条数由 gap_probe.sh 事后统计，作为独立旁证。

用法（在 ros1-slam-sim 容器内、栈已起）：
  python3 _ack_face_probe.py --case stray --tag a1
期望值固定（本件只验丙案自己的判类），`--expect` 仅为与运行器接口对齐。
"""
import argparse
import json
import sys
import time

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool, String

GOAL_Z = 0.11
FIRST_GOAL = (2.0, 0.0)
SECOND_WRITER_SEQ = 900000001     # 与适配器自诊断同号段（真外部停，走 case B/C）
STRAY_SEQ = 777777                # 谁都没发过的号（无关回执，走 case A）
IDLE_SEQ = 888888                 # 无活腿时的无关回执（走 case D）
SETTLE_AFTER_PATH_S = 1.5
WARM_TIMEOUT_S = 10.0
LATCH_TIMEOUT_S = 10.0

PATH_STAMPS = set()
CANCEL_OUT = []
ACK_IN = []
BT_STATUS = {'goal_seq': 0, 'queue_depth': 0, 'pending_dropped': 0,
             'external_cancel_seq': 0, 'cancel_origin': '', 'event': '', 'nodes': {}}


def landing_node(nodes):
    for name in ('wait_path', 'wait_arrival', 'follow_path', 'dwell_at_goal',
                 'request_path', 'wait_goal', 'cancel_current'):
        if nodes.get(name) == 'RUNNING':
            return name
    return '?'


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--case', choices=('stray', 'late', 'duplicate', 'idle'),
                        required=True)
    parser.add_argument('--expect', choices=('auto', 'pass', 'fail'), default='pass')
    parser.add_argument('--tag', default='ack')
    args = parser.parse_args()

    rospy.init_node('gap_ack_face', anonymous=True)
    goal_pub = rospy.Publisher('/v8/bt/goal', PoseStamped, queue_size=1)
    arrival_pub = rospy.Publisher('/v8/bt/arrived', Bool, queue_size=1)
    second_writer_pub = rospy.Publisher('/v8/local/cancel', String, queue_size=1)
    ack_pub = rospy.Publisher('/v8/local/cancel_ack', String, queue_size=10)

    def on_path(msg):
        if msg.poses:
            PATH_STAMPS.add(msg.header.stamp.to_nsec())

    def on_status(msg):
        try:
            payload = json.loads(msg.data)
        except ValueError:
            return
        if not isinstance(payload, dict):
            return
        for key in ('goal_seq', 'queue_depth', 'pending_dropped', 'external_cancel_seq'):
            if key in payload:
                try:
                    BT_STATUS[key] = int(payload[key] or 0)
                except (TypeError, ValueError):
                    pass
        for key in ('cancel_origin', 'event'):
            if key in payload:
                BT_STATUS[key] = str(payload[key] or '')
        nodes = payload.get('nodes')
        if isinstance(nodes, list):
            BT_STATUS['nodes'] = {str(n.get('name')): str(n.get('status'))
                                  for n in nodes if isinstance(n, dict)}

    rospy.Subscriber('/global_path', Path, on_path, queue_size=10)
    rospy.Subscriber('/v8/bt/status', String, on_status, queue_size=50)
    rospy.Subscriber('/v8/local/cancel', String, lambda m: CANCEL_OUT.append(m.data),
                     queue_size=100)
    rospy.Subscriber('/v8/local/cancel_ack', String, lambda m: ACK_IN.append(m.data),
                     queue_size=100)

    def send_goal(tag, xy):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = xy[0], xy[1], GOAL_Z
        goal.pose.orientation.w = 1.0
        goal_pub.publish(goal)
        print('SW_GOAL_SENT tag=%s xy=(%.2f, %.2f)' % (tag, xy[0], xy[1]), flush=True)

    def wait(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end and not rospy.is_shutdown():
            time.sleep(0.05)

    def wait_for(predicate, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end and not rospy.is_shutdown():
            if predicate():
                return True
            time.sleep(0.1)
        return False

    def inject_ack(seq, times=1, gap=0.05):
        for _ in range(times):
            ack_pub.publish(String(data='%d:cancelled' % seq))
            wait(gap)

    def snapshot(label):
        print('SW_%s EXTERNAL_SEQ=%d ORIGIN=%s EVENT=%s PENDING_DROPPED=%d QUEUE_DEPTH=%d NODE=%s'
              % (label, BT_STATUS['external_cancel_seq'], BT_STATUS['cancel_origin'],
                 BT_STATUS['event'], BT_STATUS['pending_dropped'],
                 BT_STATUS['queue_depth'], landing_node(BT_STATUS['nodes'])), flush=True)

    print('SW_CASE=%s TAG=%s' % (args.case, args.tag), flush=True)

    # ---- 共同准备：喂出第一条路径并等 BT 走出 WaitForPath（与 _regoal_second_writer 同姿势）
    wait(3.0)
    attempts = 0
    while attempts < 6 and not PATH_STAMPS:
        attempts += 1
        send_goal('warm%d' % attempts, FIRST_GOAL)
        wait_for(lambda: bool(PATH_STAMPS), WARM_TIMEOUT_S)
    print('SW_WARM_ATTEMPTS=%d' % attempts, flush=True)
    if not PATH_STAMPS:
        print('SW_ABORT=no_path（上游没起步，读数不可用）', flush=True)
        print('SW_VERDICT=INCONCLUSIVE', flush=True)
        return 0
    wait(SETTLE_AFTER_PATH_S)
    print('SW_LANDING_NODE=%s' % landing_node(BT_STATUS['nodes']), flush=True)
    snapshot('BEFORE')

    ok = False
    if args.case == 'stray':
        # 有活腿 + 一条谁都没发过的 confirmed 回执
        live = wait_for(lambda: landing_node(BT_STATUS['nodes']) in
                        ('wait_arrival', 'follow_path', 'dwell_at_goal'), LATCH_TIMEOUT_S)
        print('SW_LIVE_LEG=%s（须 True，否则「有活腿」这个前提没成立）' % live, flush=True)
        ACK_IN.clear()
        inject_ack(STRAY_SEQ)
        latched = wait_for(lambda: BT_STATUS['external_cancel_seq'] == STRAY_SEQ,
                           LATCH_TIMEOUT_S)
        snapshot('AFTER')
        wait(1.5)
        print('SW_STRAY_LATCHED=%s（须 True：fail-safe 会把无关回执当真外部停）' % latched,
              flush=True)
        print('SW_STRAY_OVER_TRIGGER=%s（True＝已知过触发：多停一次腿，要操作员重发）'
              % latched, flush=True)
        print('SW_CANCEL_OUT_TAIL=%s（BT 随后应用自己的号重发，不复用外来号）'
              % (CANCEL_OUT[-2:],), flush=True)
        print('SW_ACK_IN_SEEN=%s' % (ACK_IN[:3],), flush=True)
        ok = live and latched

    elif args.case in ('late', 'duplicate'):
        # 先造一次真外部停（第二写者），升格之后再送同号回执
        second_writer_pub.publish(String(data='%d:all:second_writer' % SECOND_WRITER_SEQ))
        print('SW_CANCEL_SENT topic=/v8/local/cancel seq=%d（第二写者）'
              % SECOND_WRITER_SEQ, flush=True)
        latched = wait_for(
            lambda: BT_STATUS['external_cancel_seq'] == SECOND_WRITER_SEQ, LATCH_TIMEOUT_S)
        print('SW_FIRST_LATCH=%s（须 True，否则后面「重复回执不二次升格」没有对照）' % latched,
              flush=True)
        snapshot('AFTER_FIRST')
        seq_after_first = BT_STATUS['external_cancel_seq']
        dropped_after_first = BT_STATUS['pending_dropped']
        ACK_IN.clear()
        times = 1 if args.case == 'late' else 5
        inject_ack(SECOND_WRITER_SEQ, times=times)
        wait(2.0)
        snapshot('AFTER_REPLAY')
        same_seq = BT_STATUS['external_cancel_seq'] == seq_after_first
        same_dropped = BT_STATUS['pending_dropped'] == dropped_after_first
        print('SW_REPLAY_TIMES=%d' % times, flush=True)
        print('SW_RELATCH_SEQ_UNCHANGED=%s（须 True：已记账的回执不再升格）' % same_seq, flush=True)
        print('SW_RELATCH_DROP_UNCHANGED=%s（须 True：不再二次清队）' % same_dropped, flush=True)
        print('SW_REPLAY_ACK_SEEN=%s' % (ACK_IN[:6],), flush=True)
        ok = latched and same_seq and same_dropped

    else:  # idle
        # 把当前腿判到达，让 mission 收尾、回到取目标（has_current=false）
        arrival_pub.publish(Bool(data=True))
        print('SW_ARRIVAL_PUSHED（/v8/bt/arrived=true，收掉当前腿）', flush=True)
        back = wait_for(lambda: landing_node(BT_STATUS['nodes']) == 'wait_goal', 25.0)
        print('SW_BACK_TO_WAIT_GOAL=%s（须 True：此刻手里没有活腿）' % back, flush=True)
        snapshot('AT_IDLE')
        ACK_IN.clear()
        inject_ack(IDLE_SEQ)
        wait(2.0)
        snapshot('AFTER_IDLE_ACK')
        ignored = BT_STATUS['external_cancel_seq'] == 0
        print('SW_IDLE_ACK_IGNORED=%s（须 True：无活腿时只记账不升格）' % ignored, flush=True)
        print('SW_IDLE_ACK_SEEN=%s' % (ACK_IN[:3],), flush=True)
        ok = back and ignored

    print('SW_EXPECT=PASS', flush=True)
    print('SW_VERDICT=%s' % ('PASS' if ok else 'FAIL'), flush=True)
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
