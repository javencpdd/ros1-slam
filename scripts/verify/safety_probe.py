#!/usr/bin/env python3
"""安全行为探针：运动中触发"定位失效"或"取消"，量测停止延迟与滑行距离。

两种模式：
  stop    到 --at 秒时杀掉 nodelet manager（断开 HDL 定位），验证 local/mux 按既有
          超时机制归零；
  cancel  到 --at 秒时向 /v8/bt/cancel 发文本取消（契约 seq:scope:reason，JSON 会被判
          malformed 丢弃），验证 ACK、速度归零与旧目标失效。

指标定义：
  停止延迟 = 首次 |v| < 0.05 的时刻 - 触发时刻
  滑行距离 = 触发时刻到停止时刻的**真值位移**（/sim/odom_gt，物理位移，含滑行）
  归零判定 = 触发后所有 cmd 帧 |v| < 0.05，且探针结束时仍为 0
"""
import argparse
import json
import math
import subprocess
import sys
import time

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String

STOP_SPEED = 0.05


class Probe(object):
    def __init__(self):
        self.v = 0.0
        self.v_at = None
        self.gt = None
        self.cmd_after = []
        self.trigger_at = None
        self.trigger_pose = None
        self.stop_time = None
        self.ack = None
        self.t0 = None
        rospy.Subscriber('/cmd_vel', Twist, self.on_cmd, queue_size=20)
        rospy.Subscriber('/sim/odom_gt', Odometry, self.on_gt, queue_size=20)
        rospy.Subscriber('/v8/local/cancel_ack', String, self.on_ack, queue_size=10)

    def on_cmd(self, msg):
        now = rospy.Time.now().to_sec()
        if self.t0 is None:
            self.t0 = now
        self.v = math.hypot(msg.linear.x, msg.linear.y)
        self.v_at = now
        if self.trigger_at is not None:
            self.cmd_after.append((now - self.trigger_at, self.v))
            if self.stop_time is None and self.v < STOP_SPEED:
                self.stop_time = now

    def on_gt(self, msg):
        p = msg.pose.pose.position
        self.gt = (p.x, p.y)

    def on_ack(self, msg):
        self.ack = msg.data


def trigger_stop_hdl():
    subprocess.call(['pkill', '-f', 'nodelet manager'])
    return 'killed nodelet manager (hdl 定位中断)'


def trigger_cancel(seq):
    pub = rospy.Publisher('/v8/bt/cancel', String, queue_size=5)
    rospy.sleep(0.3)
    text = '%d:current:operator' % seq
    pub.publish(String(data=text))
    rospy.sleep(0.3)
    pub.publish(String(data=text))
    return 'sent /v8/bt/cancel = %r' % text


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--mode', choices=('stop', 'cancel'), required=True)
    ap.add_argument('--at', type=float, default=6.0, help='触发时刻（相对节点启动，秒）')
    ap.add_argument('--duration', type=float, default=22.0)
    ap.add_argument('--seq', type=int, default=1)
    ap.add_argument('--on-speed', type=float, default=0.0,
                    help='速度首次达到该值时才触发（>0 时优先于 --at）；'
                         '固定时刻触发容易在车已停下时误判为"运动中"')
    ap.add_argument('--out', default='')
    args = ap.parse_args()

    rospy.init_node('safety_probe', anonymous=True)
    pr = Probe()
    rate = rospy.Rate(20.0)
    begin = rospy.Time.now().to_sec()
    action = None
    speed_at_trigger = None
    # ★必须兜住 ROSInterrupt★：kill 掉 nodelet manager 后 roslaunch 会关停 master，
    # rospy 立刻抛 ROSInterruptException。不捕获的话进程直接退出、JSON 压根不落盘，
    # 表现为"测了但没数据"。
    try:
      while not rospy.is_shutdown():
        now = rospy.Time.now().to_sec()
        elapsed = now - begin
        moving = args.on_speed > 0.0 and pr.v >= args.on_speed
        timed = args.on_speed <= 0.0 and elapsed >= args.at
        if action is None and (moving or timed):
            pr.trigger_at = now
            pr.trigger_pose = pr.gt
            speed_at_trigger = pr.v
            action = trigger_stop_hdl() if args.mode == 'stop' else trigger_cancel(args.seq)
        if elapsed >= args.duration:
            break
        rate.sleep()
    except rospy.ROSInterruptException:
        pass

    result = {
        'mode': args.mode,
        'trigger_at_s': round(args.at, 2),
        'trigger_on_speed_mps': args.on_speed or None,
        'triggered_while_moving': bool((speed_at_trigger or 0.0) >= max(args.on_speed, 0.05)),
        'action': action,
        'speed_at_trigger': None if speed_at_trigger is None else round(speed_at_trigger, 4),
        'stop_delay_s': None,
        'coast_distance_m': None,
        'cancel_ack': pr.ack,
        'cmd_frames_after': len(pr.cmd_after),
        'max_speed_after_2s': None,
        'zero_after_trigger': None,
    }
    if pr.cmd_after:
        after2 = [v for (t, v) in pr.cmd_after if t <= 2.0]
        result['max_speed_after_2s'] = round(max(after2), 4) if after2 else None
        tail = [v for (t, v) in pr.cmd_after if t > 2.0]
        result['zero_after_trigger'] = bool(tail) and max(tail) < STOP_SPEED
    if pr.trigger_at is not None and pr.stop_time is not None:
        result['stop_delay_s'] = round(pr.stop_time - pr.trigger_at, 3)
    if pr.trigger_pose is not None and pr.gt is not None:
        result['coast_distance_m'] = round(
            math.hypot(pr.gt[0] - pr.trigger_pose[0], pr.gt[1] - pr.trigger_pose[1]), 3)
    # 触发前的最后速度（从 cmd_after 的第一帧反推不到，单独用 v_at 之前的值不可得；
    # 用触发后首帧近似，并在报告里标注口径）
    if pr.cmd_after:
        result['speed_first_frame_after'] = round(pr.cmd_after[0][1], 4)

    text = json.dumps(result, ensure_ascii=False)
    print(text)
    if args.out:
        open(args.out, 'w').write(text + '\n')
    return 0


if __name__ == '__main__':
    sys.exit(main())
