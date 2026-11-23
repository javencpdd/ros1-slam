#!/usr/bin/env python3
"""P2 运行期探针：在一个**真实运行的隔离 ROS 图**上验证「目标→规划→路径→局部目标→取消→停止」。

不做算法判断，只回答"消息有没有按声明流动"，并把每步的**时间戳与消息内容**落盘。
所有字段都来自实际订阅，缺失即判 FAIL —— 这条是防假绿的核心。

用法（容器内，隔离 master）：
    python3 p2_runtime_probe.py --combo trad_support --evidence-dir /tmp/p2/trad_support \
        --goal 1.0 0.0 --pose 0 0 --duration 40
"""
import argparse
import json
import os
import sys
import time

import rospy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from std_msgs.msg import String


class Probe:
    def __init__(self, args):
        self.args = args
        self.t0 = time.time()
        self.ev = {
            'combo': args.combo, 'started_at': time.strftime('%Y-%m-%dT%H:%M:%S'),
            'master': os.environ.get('ROS_MASTER_URI'), 'host': os.uname().nodename,
            'goal': {'x': args.goal[0], 'y': args.goal[1]},
            'pose_origin': {'x': args.pose[0], 'y': args.pose[1]},
            'events': [], 'counts': {}, 'errors': [],
        }
        self.seen = {'path': False, 'status': False, 'local_goal': False,
                     'cancel_ack': False, 'planner_goal': False}
        self.last = {}
        self.cmd = []

        self.pose_pub = rospy.Publisher('/v8/local/pose', PoseStamped, queue_size=1)
        self.goal_pub = rospy.Publisher(args.goal_topic, PoseStamped, queue_size=1)
        self.cancel_pub = rospy.Publisher(args.cancel_topic, String, queue_size=1)

        rospy.Subscriber(args.planner_goal_topic, PoseStamped, self.on_planner_goal, queue_size=1)
        rospy.Subscriber('/global_path', Path, self.on_path, queue_size=1)
        rospy.Subscriber('/global_path/status', String, self.on_status, queue_size=1)
        rospy.Subscriber('/v8/local/goal', PoseStamped, self.on_local_goal, queue_size=1)
        rospy.Subscriber('/v8/local/cancel_ack', String, self.on_cancel_ack, queue_size=1)
        rospy.Subscriber('/cmd_vel', Twist, self.on_cmd, queue_size=50)
        rospy.Subscriber('/cmd_vel_raw', Twist, self.on_cmd_raw, queue_size=50)

    # ---------------- 回调：只记录事实
    def mark(self, key, **kw):
        self.seen[key] = True
        rec = {'t': round(time.time() - self.t0, 3), 'event': key}
        rec.update(kw)
        self.ev['events'].append(rec)
        self.last[key] = rec
        self.ev['counts'][key] = self.ev['counts'].get(key, 0) + 1

    def on_planner_goal(self, msg):
        p = msg.pose.position
        self.mark('planner_goal', x=round(p.x, 3), y=round(p.y, 3), frame=msg.header.frame_id)

    def on_path(self, msg):
        if not msg.poses:
            self.mark('path_empty')
            return
        p = msg.poses[-1].pose.position
        self.mark('path', n=len(msg.poses), end_x=round(p.x, 3), end_y=round(p.y, 3))

    def on_status(self, msg):
        self.mark('status', data=msg.data[:80])

    def on_local_goal(self, msg):
        p = msg.pose.position
        self.mark('local_goal', x=round(p.x, 3), y=round(p.y, 3))

    def on_cancel_ack(self, msg):
        self.mark('cancel_ack', data=msg.data[:80])

    def on_cmd(self, msg):
        self.cmd.append(('cmd_vel', round(msg.linear.x, 3), round(msg.angular.z, 3),
                         round(time.time() - self.t0, 2)))

    def on_cmd_raw(self, msg):
        self.cmd.append(('cmd_vel_raw', round(msg.linear.x, 3), round(msg.angular.z, 3),
                         round(time.time() - self.t0, 2)))

    # ---------------- 流程
    def publish_pose(self, n=40, hz=20.0):
        r = rospy.Rate(hz)
        for _ in range(n):
            m = PoseStamped()
            m.header.stamp = rospy.Time.now()
            m.header.frame_id = 'map'
            m.pose.position.x = self.args.pose[0]
            m.pose.position.y = self.args.pose[1]
            m.pose.orientation.w = 1.0
            self.pose_pub.publish(m)
            r.sleep()

    def wait(self, key, timeout):
        end = time.time() + timeout
        while time.time() < end and not rospy.is_shutdown():
            if self.seen.get(key):
                return True
            time.sleep(0.05)
        return bool(self.seen.get(key))

    def run(self):
        rospy.init_node('p2_runtime_probe', anonymous=True, disable_signals=True)
        time.sleep(1.0)
        n_pub = self.goal_pub.get_num_connections()
        self.ev['goal_subscribers'] = n_pub
        if n_pub == 0 and not self.args.allow_no_goal_sub:
            self.ev['errors'].append("目标话题无人订阅：%s" % self.args.goal_topic)
            return self.finish()

        # 1) 喂位姿（传统链无载体时用），再发目标；graph 有 carrier 时加 --no-pose
        if not self.args.no_pose:
            self.publish_pose()
        g = PoseStamped()
        g.header.stamp = rospy.Time.now()
        g.header.frame_id = 'map'
        g.pose.position.x = self.args.goal[0]
        g.pose.position.y = self.args.goal[1]
        g.pose.orientation.w = 1.0
        self.mark('goal_sent', x=self.args.goal[0], y=self.args.goal[1])
        self.goal_pub.publish(g)

        # 2) 等四个环节
        self.wait('planner_goal', self.args.duration)
        self.wait('path', self.args.duration)
        self.wait('status', 5)
        self.wait('local_goal', self.args.duration)

        # 3) 取消
        # 取消契约是文本 "<seq>:<scope>:<reason>"（bt_policy.hpp:72 parse_cancel），不是 JSON
        c = String()
        c.data = '%d:all:p2-probe' % self.args.cancel_seq
        self.mark('cancel_sent')
        self.cancel_pub.publish(c)
        self.wait('cancel_ack', 5)

        # 4) 收尾：观察是否还有非零速度
        time.sleep(1.0)
        return self.finish()

    def finish(self):
        self.ev['counts'] = dict(self.ev['counts'])
        self.ev['cmd_samples'] = self.cmd[-20:]
        nonzero = [c for c in self.cmd if abs(c[1]) > 1e-6 or abs(c[2]) > 1e-6]
        self.ev['nonzero_cmd_samples'] = len(nonzero)
        self.ev['verdict'] = self.verdict()
        out = os.path.join(self.args.evidence_dir, 'probe_%s.json' % self.args.combo)
        os.makedirs(self.args.evidence_dir, exist_ok=True)
        with open(out, 'w') as f:
            json.dump(self.ev, f, ensure_ascii=False, indent=1)
        print(json.dumps({'combo': self.args.combo, 'verdict': self.ev['verdict'],
                          'counts': self.ev['counts'], 'evidence': out}, ensure_ascii=False))
        return 0 if self.ev['verdict'] == 'PASS' else 1

    def verdict(self):
        need = ['planner_goal', 'path', 'local_goal']
        missing = [k for k in need if not self.seen.get(k)]
        if missing:
            return 'FAIL:缺 %s' % ','.join(missing)
        if not self.seen.get('cancel_ack') and self.args.expect_cancel_ack:
            return 'FAIL:取消无 ACK'
        return 'PASS'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--combo', required=True)
    ap.add_argument('--evidence-dir', required=True)
    ap.add_argument('--goal', nargs=2, type=float, default=(1.0, 0.0))
    ap.add_argument('--pose', nargs=2, type=float, default=(0.0, 0.0))
    ap.add_argument('--goal-topic', default='/v8/bt/goal')
    ap.add_argument('--cancel-topic', default='/v8/bt/cancel')
    ap.add_argument('--planner-goal-topic', default='/v8/global/goal')
    ap.add_argument('--duration', type=float, default=25.0)
    ap.add_argument('--cancel-seq', type=int, default=1)
    ap.add_argument('--expect-cancel-ack', action='store_true', default=True)
    ap.add_argument('--allow-no-goal-sub', action='store_true')
    ap.add_argument('--no-pose', action='store_true', help='图内已有载体发布位姿时不要注入')
    a = ap.parse_args()
    rc = Probe(a).run()
    sys.exit(rc)


if __name__ == '__main__':
    main()
