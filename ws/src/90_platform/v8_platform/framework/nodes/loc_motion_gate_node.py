#!/usr/bin/env python3
"""定位运动门控：收敛／质量／新鲜度／跳变任一异常 ⇒ 发**取消**中止任务，并锁存。

为什么不用 BT 自带的 `CheckLocalization`：它只返回 FAILURE，而 `recovery_mode=skip` 与
`retry_max>=1` 会把失败目标重新排队 —— 定位一恢复就自动接着跑旧任务。本节点在异常发生
的那一刻往 BT 的取消入口发一条 `seq:scope:reason`（文本契约，JSON 会被判 malformed 丢弃），
BT 收到即清队；恢复后本节点**不发任何目标**，旧任务只能由操作者用新目标重新下达。

输入：`localization_ok`（carrier 已把新鲜度/收敛/质量/跳变四条折进去）+ `carrier/status`
（带 health.reasons，仅用于取消理由与日志）。
输出：`cancel`（std_msgs/String）与 `gate_status`（std_msgs/String，只读观测）。
"""
import json
import os
import sys
import time

import rospy
from std_msgs.msg import Bool, String

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))))   # .../v8_platform，common 在其下
from common.real_gate import MotionGate  # noqa: E402


def main():
    rospy.init_node('v8_loc_motion_gate')
    localization_topic = rospy.get_param('~localization_topic', '/v8/local/localization_ok')
    status_topic = rospy.get_param('~carrier_status_topic', '/v8/3d/carrier/status')
    cancel_topic = rospy.get_param('~cancel_topic', '/v8/bt/cancel')
    gate_status_topic = rospy.get_param('~gate_status_topic', '/v8/safety/loc_gate/status')
    republish_s = float(rospy.get_param('~republish_s', 2.0))
    rate_hz = float(rospy.get_param('~rate_hz', 20.0))

    gate = MotionGate(republish_s=republish_s)
    state = {'healthy': None, 'detail': None}

    cancel_pub = rospy.Publisher(cancel_topic, String, queue_size=5)
    status_pub = rospy.Publisher(gate_status_topic, String, queue_size=5)

    def on_loc(message):
        state['healthy'] = bool(message.data)

    def on_status(message):
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError):
            return
        health = payload.get('health') or {}
        reasons = health.get('reasons') or []
        state['detail'] = reasons[0] if reasons else None

    rospy.Subscriber(localization_topic, Bool, on_loc, queue_size=10)
    rospy.Subscriber(status_topic, String, on_status, queue_size=10)
    rospy.loginfo('[loc_gate] %s → 取消入口 %s（异常即中止，恢复不自动重启）',
                  localization_topic, cancel_topic)

    rate = rospy.Rate(rate_hz)
    while not rospy.is_shutdown():
        if state['healthy'] is not None:
            actions = gate.update(state['healthy'], time.monotonic(), state['detail'])
            for action in actions:
                if action['type'] == 'cancel':
                    cancel_pub.publish(String(data=action['payload']))
                    rospy.logwarn('[loc_gate] 定位异常，已发取消：%s', action['payload'])
                else:
                    rospy.logwarn('[loc_gate] %s', action['text'])
            status_pub.publish(String(data=json.dumps({
                'latched': gate.latched, 'seq': gate.seq,
                'healthy': state['healthy'], 'detail': state['detail'],
                'republish_s': gate.republish_s})))
        rate.sleep()


if __name__ == '__main__':
    main()
