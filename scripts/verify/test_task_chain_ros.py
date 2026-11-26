#!/usr/bin/env python3
"""真实 ROS 消息链测试：跨话题乱序 / 丢包 / 重复 / 晚到 / 节点重启（真人 2026-11-26 第 5 条）。

为什么必须是"真消息链"：`test_path_goal_atomic.py` 是零 ROS 依赖的离线单测，它证明的是
**函数语义**；但这次的结构问题是"Path / path_meta / path_task 三路写同一状态"，
只有把真正的 `path_goal_node.py` 起起来、真的往 topic 上发消息，才能证明
"legacy 真的让位、原子通道真的独占、撤销线真的跨消息生效"。

做法：起一个**私有话题域**的 path_goal 实例（真实节点代码，`__name:=pgtest` + 私有话题重映射），
对它发真实消息，读它真实发布的 goal / status / goal_task。不动验收栈里的实例。

用例：
  A 乱序：同一 mission 先 rev2 后 rev1 ⇒ 拒（stale_revision），目标不重发；
  B 丢包：rev 从 2 直接跳到 5（中间消息丢了）⇒ 必须接受（丢包不能把任务卡死）；
  C 重复：把 rev5 原样再发一次 ⇒ 必须 no-op（目标不再发、进度不重置）；
  D 晚到：新 mission 生效后，旧 mission 才到 ⇒ 拒（stale_mission）；
  E 取消重放：cancel 后旧 mission 带"新时间戳"重放 ⇒ 拒（task_revoked）；
  F 反控：取消后编号更大的新 mission ⇒ 必须接受；
  G 节点重启：杀掉实例再起，2.5 s 内不许自发重发旧目标；随后新 mission 正常工作。
"""
import argparse
import json
import os
import signal
import subprocess
import sys
import threading
import time

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
NODE = os.path.join(REPO, 'ws/src/80_algorithm/v8_algorithm/local_planner/path_goal_node.py')


def task(mission, revision, poses=((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (2.0, 0.0, 0.0))):
    return {'mission_id': mission, 'path_revision': revision, 'waypoint_index': 0,
            'execution_generation': 0, 'poses': [list(p) for p in poses]}


class Chain(object):
    def __init__(self):
        self.lock = threading.Lock()
        self.goals = []          # (wall, x, y)
        self.status = []         # (wall, dict)
        self.tasks = []          # (wall, dict)
        self.path_task = rospy.Publisher('/t/path_task', String, queue_size=10)
        self.cancel = rospy.Publisher('/t/cancel', String, queue_size=10)
        self.pose = rospy.Publisher('/t/pose', PoseStamped, queue_size=10)
        rospy.Subscriber('/t/goal', PoseStamped, self.on_goal, queue_size=20)
        rospy.Subscriber('/t/status', String, self.on_status, queue_size=50)
        rospy.Subscriber('/t/goal_task', String, self.on_task, queue_size=20)
        self.proc = None

    # ---------- 观测 ----------
    def on_goal(self, msg):
        with self.lock:
            self.goals.append((time.monotonic(), msg.pose.position.x, msg.pose.position.y))

    def on_status(self, msg):
        try:
            value = json.loads(msg.data)
        except Exception:
            return
        with self.lock:
            self.status.append((time.monotonic(), value))

    def on_task(self, msg):
        try:
            value = json.loads(msg.data)
        except Exception:
            return
        with self.lock:
            self.tasks.append((time.monotonic(), value))

    def snap(self):
        with self.lock:
            return len(self.goals), len(self.status), len(self.tasks)

    def last_status(self):
        with self.lock:
            return dict(self.status[-1][1]) if self.status else {}

    def reason_after(self, at, want, timeout=4.0):
        """在 at 之后是否出现过指定 reason（用真实回传的 status 判，不看本地推断）。"""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            with self.lock:
                for wall, value in self.status:
                    if wall >= at and value.get('reason') == want:
                        return True
            rospy.sleep(0.05)
        return False

    # ---------- 节点生命周期 ----------
    def start(self):
        argv = [sys.executable, NODE, '__name:=pgtest',
                '_path_task_topic:=/t/path_task', '_path_topic:=/t/path',
                '_path_meta_topic:=/t/path_meta', '_goal_topic:=/t/goal',
                '_goal_task_topic:=/t/goal_task', '_goal_meta_topic:=/t/goal_meta',
                '_status_topic:=/t/status', '_pose_topic:=/t/pose',
                '_cancel_topic:=/t/cancel', '_arrive_m:=0.30']
        self.proc = subprocess.Popen(argv, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                                     preexec_fn=os.setsid)
        # 就绪判据：本实例自己的 status 里出现任何一条（tick 不会说话，靠首帧 path_task 触发），
        # 所以这里只等"订阅建立"——真实 proof 是后面第一条 path_task 的回声。
        end = time.monotonic() + 15
        while time.monotonic() < end:
            if self.path_task.get_num_connections() > 0:
                return True
            rospy.sleep(0.2)
        return False

    def stop(self):
        if self.proc and self.proc.poll() is None:
            try:
                os.killpg(os.getpgid(self.proc.pid), signal.SIGINT)
            except Exception:
                self.proc.terminate()
            try:
                self.proc.wait(timeout=8)
            except Exception:
                os.killpg(os.getpgid(self.proc.pid), signal.SIGKILL)
        self.proc = None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default='/tmp/task_chain_ros.json')
    args = ap.parse_args()
    rospy.init_node('task_chain_ros_test', anonymous=True, disable_signals=True)

    chain = Chain()
    cases = []

    def record(name, ok, detail):
        cases.append({'name': name, 'verdict': 'PASS' if ok else 'FAIL', 'detail': detail})
        print('[%s] %s %s' % ('PASS' if ok else 'FAIL', name, detail))

    ok = chain.start()
    record('node_start', ok, '订阅建立=%s' % ok)
    if not ok:
        return finish(cases, args.out)

    # 基线位姿：让 _first_unreached 有参照（原点）
    for _ in range(3):
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.pose.orientation.w = 1.0
        chain.pose.publish(p)
        rospy.sleep(0.1)
    rospy.sleep(0.5)

    # A 乱序：rev2 先到，rev1 后到 ⇒ 拒
    g0, _, _ = chain.snap()
    chain.path_task.publish(String(data=json.dumps(task(1, 2))))
    rospy.sleep(1.0)
    g1, _, _ = chain.snap()
    at = time.monotonic()
    chain.path_task.publish(String(data=json.dumps(task(1, 1, poses=((0, 0, 0), (9, 0, 0))))))
    rejected = chain.reason_after(at, 'stale_revision')
    rospy.sleep(0.6)
    g2, _, _ = chain.snap()
    record('A_order_revision', rejected and g2 == g1,
           'rev2→rev1 被拒=%s 目标数 %d→%d(%d)' % (rejected, g1, g2, g0))

    # B 丢包：rev2 → rev5（3、4 丢）。点列换成**不同坐标**：publish_current 的键含 xyz，
    # 坐标与前一条相同时会被去重吃掉，那会把"接受但没发"误读成"拒绝"。
    P5 = ((0.0, 0.0, 0.0), (1.5, 0.0, 0.0), (2.5, 0.0, 0.0))
    _, s0, _ = chain.snap()
    chain.path_task.publish(String(data=json.dumps(task(1, 5, poses=P5))))
    rospy.sleep(1.0)
    g3, s1, _ = chain.snap()
    record('B_lossy_revision', s1 > s0 and g3 > g2, '丢包后仍接受 rev5：status %d→%d 目标 %d→%d'
           % (s0, s1, g2, g3))

    # C 重复：一模一样再发一次 ⇒ no-op
    g4a, s4a, t4a = chain.snap()
    chain.path_task.publish(String(data=json.dumps(task(1, 5, poses=P5))))
    rospy.sleep(1.0)
    g4b, s4b, t4b = chain.snap()
    record('C_duplicate_noop', g4b == g4a and t4b == t4a,
           '重复消息未产生新目标/新 goal_task：目标 %d→%d task %d→%d' % (g4a, g4b, t4a, t4b))

    # D 晚到：mission 2 生效后，mission 1 才到 ⇒ 拒
    chain.path_task.publish(String(data=json.dumps(task(2, 0, poses=((0.0, 0.0, 0.0), (0.0, 1.5, 0.0))))))
    rospy.sleep(1.0)
    g5, _, _ = chain.snap()
    at = time.monotonic()
    chain.path_task.publish(String(data=json.dumps(task(1, 6))))
    stale = chain.reason_after(at, 'stale_mission')
    rospy.sleep(0.6)
    g6, _, _ = chain.snap()
    record('D_late_mission', stale and g6 == g5, '旧 mission 晚到被拒=%s 目标 %d→%d' % (stale, g5, g6))

    # E 取消重放：cancel 后旧 mission 带新内容重放 ⇒ 拒（判据是 mission，不是时间戳）
    chain.cancel.publish(String(data='99:current:test'))
    rospy.sleep(0.8)
    at = time.monotonic()
    chain.path_task.publish(String(data=json.dumps(task(2, 9, poses=((0, 0, 0), (5.0, 5.0, 0))))))
    revoked = chain.reason_after(at, 'task_revoked')
    rospy.sleep(0.6)
    g7, _, _ = chain.snap()
    record('E_cancel_replay', revoked and g7 == g6, '取消后旧 mission 重放被拒=%s 目标 %d→%d'
           % (revoked, g6, g7))

    # F 反控：编号更大的新 mission 必须放行
    chain.path_task.publish(String(data=json.dumps(task(3, 0, poses=((0.0, 0.0, 0.0), (-1.5, 0.0, 0.0))))))
    rospy.sleep(1.2)
    g8, _, _ = chain.snap()
    record('F_new_mission_after_cancel', g8 > g7, '取消后新 mission 放行：目标 %d→%d' % (g7, g8))

    # G 节点重启：不许自发重发旧目标；随后新 mission 可用
    chain.stop()
    rospy.sleep(1.0)
    g_before_restart, _, _ = chain.snap()
    ok2 = chain.start()
    rospy.sleep(2.5)
    g_after_restart, _, _ = chain.snap()
    no_spontaneous = ok2 and g_after_restart == g_before_restart
    record('G_restart_no_reissue', no_spontaneous,
           '重启后 2.5 s 内无自发目标：%d→%d（重启成功=%s）'
           % (g_before_restart, g_after_restart, ok2))
    chain.path_task.publish(String(data=json.dumps(task(4, 0, poses=((0.0, 0.0, 0.0), (0.0, -1.5, 0.0))))))
    rospy.sleep(1.2)
    g9, _, _ = chain.snap()
    record('G_restart_accepts_new', g9 > g_after_restart, '重启后新 mission 生效：目标 %d→%d'
           % (g_after_restart, g9))

    chain.stop()
    return finish(cases, args.out)


def finish(cases, out):
    passed = all(c['verdict'] == 'PASS' for c in cases) and cases
    payload = {'verdict': 'PASS' if passed else 'FAIL', 'cases': cases,
               'ts': time.strftime('%Y-%m-%dT%H:%M:%S')}
    with open(out, 'w') as fh:
        json.dump(payload, fh, ensure_ascii=False, indent=1)
    print('WROTE %s verdict=%s' % (out, payload['verdict']))
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
