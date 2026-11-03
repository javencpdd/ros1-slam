#!/usr/bin/env python3
import os
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
from std_msgs.msg import Float32
from velocity_limit import clamp
from std_msgs.msg import Bool


class SafeCmdVelMux(object):
    def __init__(self):
        config_path = rospy.get_param('~mux_config')
        if not os.path.isabs(config_path):
            # Relative paths resolve against this package, so the same launch works
            # in the x86 container (/opt/ros1-slam) and on the Jetson (/home/ros1-slam).
            import rospkg
            config_path = os.path.join(rospkg.RosPack().get_path('lite3_safety'), config_path)
        with open(config_path) as f:
            cfg = yaml.safe_load(f)
        out_topic = cfg['output_topic']              # /cmd_vel（唯一写者 = 本节点）
        self.rate_hz = float(cfg['output_rate_hz'])  # 恒 20 Hz
        hold_topic = cfg['hold_topic']               # /lite3/supervisor/hold（C8①）
        self.lock = threading.Lock()
        self.hold = False
        self.current = 'INIT'
        # 源表按优先级排序（1 = 最高）；订阅全部源话题（批1/J3 起七源），超时淘汰在 _tick 内判定
        # Outer speed bound. The chassis monitor also limits velocity; this is the
        # last software stage before /cmd_vel, so it enforces the same cap.
        self.max_linear = cfg.get('max_linear_mps')
        self.max_angular = cfg.get('max_angular_rps')
        # 动态限速（前方超声保底）：有话题时取 min(静态上限, 动态 cap)；
        # cap 过期且 speed_cap_required=true 时退到 blind_cap。
        self.cap_topic = cfg.get('speed_cap_topic')
        self.cap_timeout = float(cfg.get('speed_cap_timeout_s', 0.5))
        self.cap_required = bool(cfg.get('speed_cap_required', False))
        self.blind_cap = cfg.get('blind_cap_mps', 0.1)
        self.cap_value = None
        self.cap_stamp = None
        if self.cap_topic:
            rospy.Subscriber(self.cap_topic, Float32, self._cap_cb, queue_size=1)
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

    def _cap_cb(self, msg):
        self.cap_value = float(msg.data)
        self.cap_stamp = rospy.Time.now()

    def _effective_linear(self):
        limit = self.max_linear
        if self.cap_topic:
            now = rospy.Time.now()
            fresh = (self.cap_stamp is not None
                     and (now - self.cap_stamp) <= rospy.Duration(self.cap_timeout))
            if fresh:
                cap = self.cap_value
            elif self.cap_required:
                cap = self.blind_cap
            else:
                cap = None
            if cap is not None:
                limit = cap if limit is None else min(limit, cap)
        return limit

    def _limited(self, msg):
        limit = self._effective_linear()
        if limit is None and self.max_angular is None:
            return msg
        values = clamp(msg.linear.x, msg.linear.y, msg.linear.z,
                       msg.angular.x, msg.angular.y, msg.angular.z,
                       limit, self.max_angular)
        out = Twist()
        out.linear.x, out.linear.y, out.linear.z = values[0], values[1], values[2]
        out.angular.x, out.angular.y, out.angular.z = values[3], values[4], values[5]
        return out

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
                out, cur = self._limited(chosen['msg']), chosen['name']
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
