#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""_ack_face_probe.py —— 取消来源判据的误触发面活体读数（T-20261122-04 第二施工段）。

背景：`/v8/local/cancel` 不止一个写者。执行层收到谁写的都照办，回一条 `seq:outcome`
到 `/v8/local/cancel_ack`，回执里没有来源字段。第一施工段只靠回执判来源，活体实测
暴露两个洞：腿在 mission 成功收尾时不清零（空闲期也「有活腿」）、孤证回执与真取消
在回执面上不可分（无关回执会误停）。

第二施工段改成**同号三通道互证**（判据在 `cancel_origin.hpp`，纯函数、有真值表）：
甲＝线上帧、乙＝后端回显 `E_FAR_CANCEL_RECEIVED` 带 seq、丙＝执行层回执。
甲或乙任一成立即可停腿；丙必须被甲或乙作证才算数。

本件逐格喂进去，看判据是不是按真值表走：

  stray      有活腿，只注入一条陌生 confirmed 回执（无线上证据） ⇒ **必须不停腿**
  late       真取消升格之后，同号回执再送一次                ⇒ 不再升格
  duplicate  同号回执背靠背送 5 次                            ⇒ 只升格一次
  idle       mission 已收尾（leg_active 假）后注入回执         ⇒ 必须不停腿
  wire       有活腿，直接发一条外来取消帧（模拟第二写者）      ⇒ 必须停腿（来源 external_wire）
  noack      ★无 ACK 场景★ 只发后端回显、不发帧不收回执        ⇒ 必须停腿（来源 external_backend）
  idle_wire  空闲期发外来取消帧                                ⇒ 不停腿，只记账
  race       ★取队竞态★ 队列里有旧目标、腿刚被到达信号收掉，同拍发外来帧
             ⇒ 不论外来帧抢在取队之前还是之后，旧目标都不得被派发

判据取 `/v8/bt/status`：`cancel_origin`／`external_cancel_seq`／`pending_dropped`／
`leg_active`／`wire_foreign_seen`／`backend_cancel_seen`／`stray_ack_seen`。
日志条数由运行器事后统计，作独立旁证。

用法（在 ros1-slam-sim 容器内、栈已起）：
  python3 _ack_face_probe.py --case stray --tag a1
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
SECOND_WRITER_SEQ = 900000001     # 真外来停：帧 + 回执都在（case late/duplicate）
WIRE_SEQ = 900000005              # 只发帧（case wire）
MARKER_SEQ = 900000006            # 只发后端回显（case noack）
STRAY_SEQ = 777777                # 谁都没发过（case stray）
IDLE_SEQ = 888888                 # 空闲期的陌生回执（case idle）
IDLE_WIRE_SEQ = 900000007         # 空闲期的外来帧（case idle_wire）
RACE_SEQ = 900000008              # 取队竞态那一拍的外来帧（case race）
OLD_A = (3.0, 0.0)                # race 里排在队列里的两条旧目标
OLD_B = (4.0, 0.0)
SETTLE_AFTER_PATH_S = 1.5
WARM_TIMEOUT_S = 10.0
LATCH_TIMEOUT_S = 10.0

PATH_STAMPS = set()
CANCEL_OUT = []
ACK_IN = []
GLOBAL_GOALS = []   # (x, y, t_monotonic)：/v8/global/goal 上真被派发出去的坐标
BT_STATUS = {'goal_seq': 0, 'queue_depth': 0, 'pending_dropped': 0,
             'external_cancel_seq': 0, 'cancel_origin': '', 'event': '', 'nodes': {},
             'leg_active': False, 'wire_foreign_seen': 0, 'backend_cancel_seen': 0,
             'stray_ack_seen': 0, 'stale_queue_dropped': 0}


def landing_node(nodes):
    for name in ('wait_path', 'wait_arrival', 'follow_path', 'dwell_at_goal',
                 'request_path', 'wait_goal', 'cancel_current'):
        if nodes.get(name) == 'RUNNING':
            return name
    return '?'


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--case', required=True,
                        choices=('stray', 'late', 'duplicate', 'idle', 'wire',
                                 'noack', 'idle_wire', 'race'))
    parser.add_argument('--expect', choices=('auto', 'pass', 'fail'), default='pass')
    parser.add_argument('--tag', default='ack')
    # race 专用：到达信号与外来帧之间的间隔。两个分支只差这一拍——
    #   0 秒  帧抢在取队之前/腿还没收掉 ⇒ 有活腿，走升格并连带清队
    #   >0 秒 腿已收掉再发          ⇒ 无活腿，走「只清队不升格」
    parser.add_argument('--race-delay', type=float, default=0.0)
    args = parser.parse_args()

    rospy.init_node('gap_ack_face', anonymous=True)
    goal_pub = rospy.Publisher('/v8/bt/goal', PoseStamped, queue_size=1)
    arrival_pub = rospy.Publisher('/v8/bt/arrived', Bool, queue_size=1)
    wire_pub = rospy.Publisher('/v8/local/cancel', String, queue_size=1)
    ack_pub = rospy.Publisher('/v8/local/cancel_ack', String, queue_size=10)
    status_pub = rospy.Publisher('/global_path/status', String, queue_size=1)

    def on_path(msg):
        if msg.poses:
            PATH_STAMPS.add(msg.header.stamp.to_nsec())

    def on_global_goal(msg):
        p = msg.pose.position
        GLOBAL_GOALS.append((round(p.x, 2), round(p.y, 2), time.monotonic()))

    def on_status(msg):
        try:
            payload = json.loads(msg.data)
        except ValueError:
            return
        if not isinstance(payload, dict):
            return
        for key in ('goal_seq', 'queue_depth', 'pending_dropped', 'external_cancel_seq',
                    'wire_foreign_seen', 'backend_cancel_seen', 'stray_ack_seen',
                    'stale_queue_dropped'):
            if key in payload:
                try:
                    BT_STATUS[key] = int(payload[key] or 0)
                except (TypeError, ValueError):
                    pass
        for key in ('cancel_origin', 'event'):
            if key in payload:
                BT_STATUS[key] = str(payload[key] or '')
        if 'leg_active' in payload:
            BT_STATUS['leg_active'] = bool(payload['leg_active'])
        nodes = payload.get('nodes')
        if isinstance(nodes, list):
            BT_STATUS['nodes'] = {str(n.get('name')): str(n.get('status'))
                                  for n in nodes if isinstance(n, dict)}

    rospy.Subscriber('/global_path', Path, on_path, queue_size=10)
    rospy.Subscriber('/v8/global/goal', PoseStamped, on_global_goal, queue_size=20)
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

    def inject_wire(seq):
        wire_pub.publish(String(data='%d:all:second_writer' % seq))
        print('SW_WIRE_INJECTED seq=%d（/v8/local/cancel，外来帧）' % seq, flush=True)

    def inject_marker(seq):
        status_pub.publish(String(data=json.dumps(
            {'ok': False, 'points': 0, 'reason': 'E_FAR_CANCEL_RECEIVED', 'seq': seq})))
        print('SW_MARKER_INJECTED seq=%d（/global_path/status 后端回显，无 ACK）' % seq,
              flush=True)

    def snapshot(label):
        print('SW_%s EXTERNAL_SEQ=%d ORIGIN=%s EVENT=%s LEG_ACTIVE=%s PENDING_DROPPED=%d '
              'QUEUE_DEPTH=%d WIRE=%d BACKEND=%d STRAY=%d STALEQ=%d NODE=%s'
              % (label, BT_STATUS['external_cancel_seq'], BT_STATUS['cancel_origin'],
                 BT_STATUS['event'], BT_STATUS['leg_active'],
                 BT_STATUS['pending_dropped'], BT_STATUS['queue_depth'],
                 BT_STATUS['wire_foreign_seen'], BT_STATUS['backend_cancel_seen'],
                 BT_STATUS['stray_ack_seen'], BT_STATUS['stale_queue_dropped'],
                 landing_node(BT_STATUS['nodes'])), flush=True)

    print('SW_CASE=%s TAG=%s' % (args.case, args.tag), flush=True)

    # ---- 共同准备：喂出第一条路径并等 BT 走出 WaitForPath
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
        # 有活腿 + 一条谁都没发过的 confirmed 回执，且线上没有任何同号帧
        live = wait_for(lambda: BT_STATUS['leg_active'], LATCH_TIMEOUT_S)
        print('SW_LIVE_LEG=%s（须 True，否则「有活腿」这个前提没成立）' % live, flush=True)
        ACK_IN.clear()
        inject_ack(STRAY_SEQ)
        wait(2.0)
        snapshot('AFTER')
        latched = BT_STATUS['external_cancel_seq'] == STRAY_SEQ
        counted = BT_STATUS['stray_ack_seen'] >= 1
        print('SW_STRAY_LATCHED=%s（须 False：孤证回执不得停腿）' % latched, flush=True)
        print('SW_STRAY_COUNTED=%s（须 True：回执被记账，不是被丢掉）' % counted, flush=True)
        print('SW_LEG_STILL_ACTIVE=%s（须 True：腿没被误停）'
              % BT_STATUS['leg_active'], flush=True)
        ok = live and (not latched) and counted and BT_STATUS['leg_active']

    elif args.case in ('late', 'duplicate'):
        # 先造一次真外部停（帧 + 回执都在），升格之后再送同号回执
        inject_wire(SECOND_WRITER_SEQ)
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

    elif args.case == 'idle':
        # 把当前腿判到达，让 mission 收尾 → Stop 节点必须把 leg_active 清掉
        arrival_pub.publish(Bool(data=True))
        print('SW_ARRIVAL_PUSHED（/v8/bt/arrived=true，收掉当前腿）', flush=True)
        back = wait_for(lambda: landing_node(BT_STATUS['nodes']) == 'wait_goal'
                        and not BT_STATUS['leg_active'], 25.0)
        print('SW_BACK_TO_WAIT_GOAL=%s（须 True：此刻没有活腿）' % back, flush=True)
        snapshot('AT_IDLE')
        ACK_IN.clear()
        inject_ack(IDLE_SEQ)
        wait(2.0)
        snapshot('AFTER_IDLE_ACK')
        latched = BT_STATUS['external_cancel_seq'] == IDLE_SEQ
        print('SW_IDLE_ACK_LATCHED=%s（须 False：无活腿时只记账不升格）' % latched, flush=True)
        ok = back and (not latched)

    elif args.case == 'wire':
        # 有活腿 + 外来取消帧（帧本身就是证据，回执可有可无）
        live = wait_for(lambda: BT_STATUS['leg_active'], LATCH_TIMEOUT_S)
        print('SW_LIVE_LEG=%s' % live, flush=True)
        inject_wire(WIRE_SEQ)
        latched = wait_for(lambda: BT_STATUS['external_cancel_seq'] == WIRE_SEQ,
                           LATCH_TIMEOUT_S)
        snapshot('AFTER_WIRE')
        origin_ok = BT_STATUS['cancel_origin'].startswith('external_wire')
        print('SW_WIRE_LATCHED=%s（须 True）' % latched, flush=True)
        print('SW_WIRE_ORIGIN=%s（须 external_wire 开头）' % BT_STATUS['cancel_origin'],
              flush=True)
        ok = live and latched and origin_ok

    elif args.case == 'noack':
        # ★无 ACK 场景★：只有后端回显，没有取消帧也没有回执
        live = wait_for(lambda: BT_STATUS['leg_active'], LATCH_TIMEOUT_S)
        print('SW_LIVE_LEG=%s' % live, flush=True)
        ACK_IN.clear()
        inject_marker(MARKER_SEQ)
        latched = wait_for(lambda: BT_STATUS['external_cancel_seq'] == MARKER_SEQ,
                           LATCH_TIMEOUT_S)
        wait(1.0)
        snapshot('AFTER_MARKER')
        origin_ok = BT_STATUS['cancel_origin'].startswith('external_backend')
        # 「无回执」指的是**注入的那个号**没有回执。升格之后 BT 会用自己的号重发一条取消，
        # 执行层对它回执本来就是正常链路（实测到这里会看到 `[local] cancel accepted seq=1`
        # 与一条 `1:cancelled`）；拿「收件箱为空」当判据等于把恢复动作判成缺陷（首轮实测
        # 就是这样红的：MARKER_LATCHED=True 而 NO_ACK=False）。
        marker_acks = [d for d in ACK_IN if d.split(':', 1)[0] == str(MARKER_SEQ)]
        no_ack = not marker_acks
        print('SW_MARKER_ACKS=%s' % (ACK_IN[:6],), flush=True)
        print('SW_MARKER_LATCHED=%s（须 True：没有回执也要停腿）' % latched, flush=True)
        print('SW_MARKER_ORIGIN=%s（须 external_backend 开头）' % BT_STATUS['cancel_origin'],
              flush=True)
        print('SW_MARKER_NO_ACK=%s（须 True：注入的这个号没有任何回执）' % no_ack, flush=True)
        ok = live and latched and origin_ok and no_ack

    elif args.case == 'idle_wire':
        arrival_pub.publish(Bool(data=True))
        back = wait_for(lambda: landing_node(BT_STATUS['nodes']) == 'wait_goal'
                        and not BT_STATUS['leg_active'], 25.0)
        print('SW_BACK_TO_WAIT_GOAL=%s（须 True）' % back, flush=True)
        snapshot('AT_IDLE')
        inject_wire(IDLE_WIRE_SEQ)
        wait(2.0)
        snapshot('AFTER_IDLE_WIRE')
        latched = BT_STATUS['external_cancel_seq'] == IDLE_WIRE_SEQ
        remembered = BT_STATUS['wire_foreign_seen'] >= 1
        print('SW_IDLE_WIRE_LATCHED=%s（须 False：无活腿时外来帧只记账）' % latched, flush=True)
        print('SW_IDLE_WIRE_REMEMBERED=%s（须 True）' % remembered, flush=True)
        ok = back and (not latched) and remembered

    else:  # race —— 取队与外来帧撞在同一拍
        # 队列里先塞两条旧目标，再用到达信号把当前腿收掉；收掉的那一拍外来帧同时到。
        # 两种先后都必须收敛到同一个结果：旧目标不派发。
        #   · 外来帧先到 ⇒ 没有活腿，走「只清队不升格」（dropStaleQueueLocked）
        #   · 取队先到   ⇒ 新腿已经起来，走升格（latchExternalStopLocked）并连带清队
        leg = wait_for(lambda: BT_STATUS['leg_active'], LATCH_TIMEOUT_S)
        print('SW_LIVE_LEG=%s（须 True）' % leg, flush=True)
        send_goal('oldA', OLD_A)
        wait(0.4)
        send_goal('oldB', OLD_B)
        wait(0.4)
        seq_before = BT_STATUS['goal_seq']
        depth_before = BT_STATUS['queue_depth']
        # 采样深度只作上下文：/v8/bt/status 是 2 Hz，两次入队相隔 0.4 s 时它可能只读到 1
        # （首轮 t1.06 就是这样把 FAIL 判在了前提上）。真正的前提用「清掉了几个」来证明。
        print('SW_RACE_SETUP GOAL_SEQ=%d QUEUE_DEPTH=%d（深度是 2 Hz 采样，仅作上下文）'
              % (seq_before, depth_before), flush=True)
        t_race = time.monotonic()
        arrival_pub.publish(Bool(data=True))
        if args.race_delay > 0.0:
            wait(args.race_delay)
        inject_wire(RACE_SEQ)
        wait(6.0)
        snapshot('AFTER_RACE')
        old_xy = {OLD_A, OLD_B}
        dispatched = [g for g in GLOBAL_GOALS if g[2] > t_race and (g[0], g[1]) in old_xy]
        seq_after = BT_STATUS['goal_seq']
        staleq = BT_STATUS['stale_queue_dropped']
        branch = ('stale_queue(无活腿)' if staleq > 0
                  else 'latch(有活腿)' if BT_STATUS['cancel_origin'].startswith('external_')
                  else 'none')
        print('SW_RACE_DELAY=%.3f BRANCH=%s' % (args.race_delay, branch), flush=True)
        print('SW_RACE_GOAL_SEQ=%d->%d（须不变：不得取走旧目标）' % (seq_before, seq_after),
              flush=True)
        print('SW_RACE_OLD_DISPATCHED=%s（须 False：/v8/global/goal 上不得出现旧目标坐标）'
              % (bool(dispatched),), flush=True)
        print('SW_RACE_QUEUE_DEPTH=%d（须 0：旧目标被清掉，不是留着等下一拍）'
              % BT_STATUS['queue_depth'], flush=True)
        print('SW_RACE_BRANCH ORIGIN=%s PENDING_DROPPED=%d STALEQ=%d WIRE=%d'
              '（两分支都合法，只要能清干净）'
              % (BT_STATUS['cancel_origin'], BT_STATUS['pending_dropped'], staleq,
                 BT_STATUS['wire_foreign_seen']), flush=True)
        ok = (leg and BT_STATUS['pending_dropped'] >= 2 and seq_after == seq_before
              and not dispatched and BT_STATUS['queue_depth'] == 0
              and branch != 'none')

    print('SW_EXPECT=PASS', flush=True)
    print('SW_VERDICT=%s' % ('PASS' if ok else 'FAIL'), flush=True)
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
