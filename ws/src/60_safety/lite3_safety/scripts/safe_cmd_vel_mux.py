#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# safe_cmd_vel_mux.py —— M6 安全 mux（v4 C8 终稿；v3 基线四设计 + v4 N5/N6 增补）
#
# 管道规约（v4 N6）：controller(M4) → /cmd_vel_raw → safe_cmd_vel_mux(M6) → /cmd_vel → ros2qnx（仅真机 mode_navigation）
# 四设计（v3 C8）：源优先级表（批1/J3 起七源）/ 超时淘汰 / 无源发零速 / 恒 20 Hz（Lite3 底盘 250 ms 超时不能断）
# v4 增补（C8①②）：
#   ① 监督保持输入：订阅 /lite3/supervisor/hold（std_msgs/Bool，唯一发布者 = SUP，C13），
#      true 时无视各源直接输出零速并保持直到 false（SUP 绝不直写 /cmd_vel，§2.6 硬规则 a）；
#   ② 全系统 /cmd_vel 唯一写者 = 本节点（自检 [6] 承接）；hold 是控制输入，登记口径：批1/J3 起七源。
# 源表外化：config/cmd_vel_mux.yaml（经 ~mux_config 注入；源名/话题为架构师默认，见 yaml 头登记注）。

import threading

import rospy
import yaml
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


class SafeCmdVelMux(object):
    def __init__(self):
        with open(rospy.get_param('~mux_config')) as f:
            cfg = yaml.safe_load(f)
        out_topic = cfg['output_topic']              # /cmd_vel（唯一写者 = 本节点）
        self.rate_hz = float(cfg['output_rate_hz'])  # 恒 20 Hz
        hold_topic = cfg['hold_topic']               # /lite3/supervisor/hold（C8①）
        self.lock = threading.Lock()
        self.hold = False
        self.current = 'INIT'
        # 源表按优先级排序（1 = 最高）；订阅全部源话题（批1/J3 起七源），超时淘汰在 _tick 内判定
        self.sources = sorted(cfg['sources'], key=lambda s: s['priority'])
        for s in self.sources:
            s['msg'] = None
            s['stamp'] = None
            s['active'] = False
            rospy.Subscriber(s['topic'], Twist, self._make_cb(s['name']), queue_size=1)
        rospy.Subscriber(hold_topic, Bool, self._hold_cb, queue_size=1)
        self.pub = rospy.Publisher(out_topic, Twist, queue_size=1)
        rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self._tick)   # 恒 20 Hz（底盘 250 ms 超时不能断）
        rospy.loginfo('[M6] safe_cmd_vel_mux up: %d sources, out=%s @%.0f Hz, hold_in=%s',
                      len(self.sources), out_topic, self.rate_hz, hold_topic)

    def _make_cb(self, name):
        def _cb(msg):
            with self.lock:
                for s in self.sources:
                    if s['name'] == name:
                        s['msg'], s['stamp'] = msg, rospy.Time.now()
                        break
        return _cb

    def _hold_cb(self, msg):
        with self.lock:
            if self.hold != bool(msg.data):
                rospy.logwarn('[M6] 监督保持输入 hold=%s（/lite3/supervisor/hold，唯一发布者=SUP）', bool(msg.data))
            self.hold = bool(msg.data)

    @staticmethod
    def _zero_twist():
        z = Twist()
        z.linear.x = z.linear.y = z.linear.z = 0.0
        z.angular.x = z.angular.y = z.angular.z = 0.0
        return z

    def _tick(self, _event):
        with self.lock:
            now = rospy.Time.now()
            chosen = None
            for s in self.sources:
                fresh = s['stamp'] is not None and (now - s['stamp']) <= rospy.Duration(s['timeout_s'])
                if s['active'] and not fresh:            # 超时淘汰
                    rospy.logwarn('[M6] 源 %s 超时（%.2fs 无数据），淘汰', s['name'], s['timeout_s'])
                    s['active'] = False
                if fresh:
                    s['active'] = True
                    if chosen is None:                   # 优先级最高且未超时的源
                        chosen = s
            if self.hold:                                # C8①：hold 期无视各源直接零速
                out, cur = self._zero_twist(), 'HOLD'
            elif chosen is not None:
                out, cur = chosen['msg'], chosen['name']
            else:                                        # C8：无源发零速（/cmd_vel 恒有输出，底盘超时不断）
                out, cur = self._zero_twist(), 'NONE'
            if cur != self.current:
                rospy.logwarn('[M6] 活跃源切换: %s -> %s', self.current, cur)
                self.current = cur
        self.pub.publish(out)                            # 恒 20 Hz 输出


if __name__ == '__main__':
    rospy.init_node('safe_cmd_vel_mux')
    SafeCmdVelMux()
    rospy.spin()
