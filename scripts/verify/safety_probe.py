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
        self.first_zero_at = None
        self.last_nonzero_at = None
        self.ack = None
        self.t0 = None
        rospy.Subscriber('/cmd_vel', Twist, self.on_cmd, queue_size=20)
        rospy.Subscriber('/sim/odom_gt', Odometry, self.on_gt, queue_size=20)
        rospy.Subscriber('/v8/local/cancel_ack', String, self.on_ack, queue_size=10)

    def on_cmd(self, msg):
        # ★用 wall clock 而不是 rospy.Time★：后者是仿真时间，Gazebo 一暂停 /clock 不走，
        # elapsed 永远为 0，探针在 timeout 边界上死等（实测卡住 160 s 以上不返回）。
        now = time.monotonic()
        if self.t0 is None:
            self.t0 = now
        self.v = math.hypot(msg.linear.x, msg.linear.y)
        self.v_at = now
        if self.trigger_at is not None:
            self.cmd_after.append((now - self.trigger_at, self.v))
            if self.v >= STOP_SPEED:
                # ★真正"停下"要等最后一个非零帧★：局部规划器是步进式的，tick 之间会出现
                # 短暂零命令，用"首次 <阈值"当停止时刻会把一次 tick 间隙误判成停止，
                # 从而把后续正常推进屈解为"复动"。
                self.last_nonzero_at = now
            if self.stop_time is None and self.v < STOP_SPEED:
                self.first_zero_at = now

    def on_gt(self, msg):
        p = msg.pose.pose.position
        self.gt = (p.x, p.y)

    def on_ack(self, msg):
        self.ack = msg.data


def trigger_stop_hdl():
    """只中断 HDL 定位数据，不碰 master 与导航进程。

    先前用 pkill nodelet manager：那会连带让 roslaunch 关停整个启动文件、master 随之消失，
    于是"速度归零"其实是所有节点一起死掉的结果，测不出门控本身。改成 rosnode kill 只卸载
    hdl 这个 nodelet 实例，/mgr 与 9 个导航节点都应继续存活。
    """
    subprocess.call(['rosnode', 'kill', '/hdl'])
    return 'rosnode kill /hdl (only localization data interrupted)'


def trigger_cancel(seq, scope='all'):
    pub = rospy.Publisher('/v8/bt/cancel', String, queue_size=5)
    rospy.sleep(0.3)
    text = '%d:%s:operator' % (seq, scope)
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
    ap.add_argument('--scope', default='all',
                    help='current 只取消当前腿；all 还会 dropPendingLocked() 清空目标队列。'
                         '取消语义要求「只有明确的新任务才能重启」，故默认 all')
    ap.add_argument('--on-speed', type=float, default=0.0,
                    help='速度首次达到该值时才触发（>0 时优先于 --at）；'
                         '固定时刻触发容易在车已停下时误判为"运动中"')
    ap.add_argument('--out', default='')
    args = ap.parse_args()

    rospy.init_node('safety_probe', anonymous=True)
    pr = Probe()
    begin = time.monotonic()
    action = None
    speed_at_trigger = None
    # ★必须兜住 ROSInterrupt★：kill 掉 nodelet manager 后 roslaunch 会关停 master，
    # rospy 立刻抛 ROSInterruptException。不捕获的话进程直接退出、JSON 压根不落盘，
    # 表现为"测了但没数据"。
    try:
      while not rospy.is_shutdown():
        now = time.monotonic()
        elapsed = now - begin
        moving = args.on_speed > 0.0 and pr.v >= args.on_speed
        timed = args.on_speed <= 0.0 and elapsed >= args.at
        if action is None and (moving or timed):
            pr.trigger_at = now
            pr.trigger_pose = pr.gt
            speed_at_trigger = pr.v
            action = (trigger_stop_hdl() if args.mode == 'stop'
                      else trigger_cancel(args.seq, args.scope))
        if elapsed >= args.duration:
            break
        time.sleep(0.05)
    except rospy.ROSInterruptException:
        pass

    # 健康检查：取消/断定位之后，框架本身必须还在
    alive = []
    try:
        out = subprocess.check_output(['rosnode', 'list'], stderr=subprocess.DEVNULL, timeout=10)
        alive = [n for n in out.decode().split() if n.startswith('/v8') or n in ('/mgr', '/gazebo', '/hdl')]
    except Exception:
        alive = []
    result = {
        'mode': args.mode,
        'alive_nodes_after': len(alive),
        'alive_has_mgr': '/mgr' in alive,
        'alive_has_gazebo': '/gazebo' in alive,
        'alive_has_hdl': '/hdl' in alive,
        'alive_v8_nodes': len([n for n in alive if n.startswith('/v8')]),
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
    # 定位失效后可能残留短暂非零命令：把每一条的时间与幅度都留下来，
    # 不能只报一句 zero_after_trigger（那会掩盖"抖了多久、抖多大"）。
    nonzero = [(round(t, 3), round(v, 3)) for (t, v) in pr.cmd_after if v >= STOP_SPEED]
    result['nonzero_after_count'] = len(nonzero)
    result['nonzero_after_max'] = max([v for (_, v) in nonzero], default=0.0)
    if nonzero:
        result['nonzero_after_t_range'] = [nonzero[0][0], nonzero[-1][0]]
        result['nonzero_after_head'] = nonzero[:20]
        # 首条非零命令相对触发时刻的延迟（这才是"门控反应"的真实时间）
        result['first_nonzero_after_s'] = nonzero[0][0]
    else:
        result['nonzero_after_t_range'] = None

    # 口径按用户要求拆成四项：首次归零、最后非零帧、持续零速窗口、真实滑行
    if pr.trigger_at is not None and pr.first_zero_at is not None:
        result['first_zero_at_s'] = round(pr.first_zero_at - pr.trigger_at, 3)
    if pr.trigger_at is not None and pr.last_nonzero_at is not None:
        result['last_nonzero_at_s'] = round(pr.last_nonzero_at - pr.trigger_at, 3)
        result['stop_delay_s'] = result['last_nonzero_at_s']
        # 持续零速窗口：从最后非零帧到探针结束
        tail = [v for (t, v) in pr.cmd_after if t > result['last_nonzero_at_s']]
        result['quiet_window_frames'] = len(tail)
        result['quiet_window_all_zero'] = bool(tail) and max(tail) < STOP_SPEED
        result['quiet_window_s'] = (round(pr.cmd_after[-1][0] - result['last_nonzero_at_s'], 3)
                                    if pr.cmd_after else None)
    if pr.trigger_at is not None and pr.stop_time is not None:
        # 保留旧口径供对照，但不再覆盖 stop_delay_s（后者已按"最后非零帧"定义）
        result['first_zero_delay_s'] = round(pr.stop_time - pr.trigger_at, 3)
        # 用户判据：停止后必须**持续 5 s** 保持零速，而不只是某一瞬为零
        stop_rel = pr.stop_time - pr.trigger_at
        window = [v for (t, v) in pr.cmd_after if stop_rel <= t <= stop_rel + 5.0]
        result['zero_for_5s_window_frames'] = len(window)
        result['zero_for_5s'] = bool(window) and max(window) < STOP_SPEED
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
