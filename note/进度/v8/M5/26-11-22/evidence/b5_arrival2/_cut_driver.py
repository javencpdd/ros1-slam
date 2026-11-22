"""§4.4 验收 2：**只断云**，其余全活。

为什么这条不能与「杀 FAR 进程」合并：验收 1 断的是「航点＋状态」两个通道，
本条只断「喂给 FAR 的地形」。如果判据其实盯着「点云有没有来」，两条会得出不同结果。
本测试要证明的是：判据落在航点/状态这一层（`waypoint_timeout_s`），不是云没了才反应。

注入点：仿真里 `/rslidar_points` 的发布者是 `/sim_adapter`，`rosnode info` 现读它只发
`/rosout` 与 `/rslidar_points` 两条话题 ⇒ 杀它等于精确断云，不连带动到时钟与位姿
（这正是不能直接 kill `gzserver` 的原因：那会同时断掉仿真时钟与 odom，观测面被污染）。

判据（与验收 1 同口径）：
  * `status` 出现 `E_FAR_WAYPOINT_STALE`（`since_s` 应≈`waypoint_timeout_s`，仿真时钟 0.44 倍）
  * 适配器自报取消（`/v8/local/cancel`，seq 从 900000000 起）且收到 ack
  * **自停之后**`/global_path` 不再出现新时间戳（停产出）、`/v8/local/goal` 也不再新增。
    断云到自停之间 FAR 仍会用最后一片云继续出航点，那一段不算「没停」——第一轮就是
    把断云瞬间当停产起点，把 20 条正常收敛的路径读成了失败。
"""
import subprocess
import sys
import time

import rospy
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

OBSTACLE_INTENSITY = 0.2      # 与其他探针同判据：intensity ≥ 0.2 才算障碍格


class CutTest(object):
    def __init__(self):
        rospy.init_node('b5_cut', disable_signals=True)
        self.goal_pub = rospy.Publisher('/v8/global/goal', PoseStamped, queue_size=1, latch=True)
        self.pose = None
        self.waypoints = 0
        self.path_stamps = set()
        self.local_goal_stamps = set()
        self.status = None
        self.status_seen = []
        self.cancels = []
        self.acks = []
        self.terrain = {'frames': 0, 'obs': 0, 'settled': False}
        rospy.Subscriber('/v8/local/pose', PoseStamped, self._on_pose, queue_size=2)
        rospy.Subscriber('/way_point', PointStamped, self._on_waypoint, queue_size=5)
        rospy.Subscriber('/global_path', Path, self._on_path, queue_size=5)
        rospy.Subscriber('/v8/local/goal', PoseStamped, self._on_local_goal, queue_size=5)
        rospy.Subscriber('/global_path/status', String, self._on_status, queue_size=10)
        rospy.Subscriber('/v8/local/cancel', String, self._on_cancel, queue_size=5)
        rospy.Subscriber('/v8/local/cancel_ack', String, self._on_ack, queue_size=5)
        rospy.Subscriber('/terrain_cloud', PointCloud2, self._on_terrain, queue_size=2)

    def _on_terrain(self, msg):
        self.terrain['frames'] += 1
        if self.terrain['settled']:
            return
        try:
            values = [p[3] for p in pc2.read_points(msg, field_names=('x', 'y', 'z', 'intensity'),
                                                   skip_nans=True)]
        except Exception:
            return
        if sum(1 for v in values if v >= OBSTACLE_INTENSITY) >= 1:
            self.terrain['obs'] = 1
            self.terrain['settled'] = True

    def _on_pose(self, msg):
        p = msg.pose.position
        self.pose = (p.x, p.y, p.z)

    def _on_waypoint(self, _msg):
        self.waypoints += 1

    def _on_path(self, msg):
        self.path_stamps.add(msg.header.stamp.to_nsec())

    def _on_local_goal(self, msg):
        self.local_goal_stamps.add(msg.header.stamp.to_nsec())

    def _on_status(self, msg):
        self.status = msg.data
        # ★ 只留「最新一帧」会漏事件 ★ stale 帧与它的 ack 帧只隔约 2 ms
        #   （实测 23.400 取消、23.402 ack），10 Hz 轮询几乎必然读不到中间那一帧。
        #   所以按到达顺序留一份去重历史，判据在历史里找，而不是在最新值里找。
        if msg.data not in self.status_seen and len(self.status_seen) < 500:
            self.status_seen.append(msg.data)

    def _on_cancel(self, msg):
        self.cancels.append((round(rospy.get_time(), 3), msg.data))

    def _on_ack(self, msg):
        self.acks.append((round(rospy.get_time(), 3), msg.data))

    def wait_for(self, predicate, timeout, note):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return True
            time.sleep(0.1)
        say('TIMEOUT %s after %.0f s' % (note, timeout))
        return False

    def run(self):
        # 就绪门多两条：地形云里真有障碍格（FAR 主循环的前置，`far_planner.cpp:669`）
        # 与位姿/状态到齐。第一轮之所以 INCONCLUSIVE，就是目标在 FAR 冷启动窗口里发出、
        # 被 `!is_graph_init_` 守卫丢掉，之后 150 s 一直等不到航点。
        ready = self.wait_for(lambda: (self.goal_pub.get_num_connections() >= 1
                                       and self.pose is not None and self.status is not None
                                       and self.terrain['obs'] >= 1),
                              180, 'connections')
        if not ready:
            return 5
        gx, gy, gz = self.pose
        say('READY pose=%.3f %.3f %.3f terrain_frames=%d obs=%d status=%s'
            % (gx, gy, gz, self.terrain['frames'], self.terrain['obs'], self.status))

        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.pose.orientation.w = 1.0
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = gx + 3.0, gy, gz

        # 目标要**重发到有航点为止**：上游在冷启动窗口里静默丢弃目标（同 §4.5 的 C0），
        # 只发一次就可能整场没有航点，那时「断云后静默」分不清是注入还是本来就没起来。
        before = self.waypoints
        got = False
        sent = 0
        deadline = time.monotonic() + 180.0
        while time.monotonic() < deadline:
            msg.header.stamp = rospy.Time.now()
            self.goal_pub.publish(msg)
            sent += 1
            say('GOAL %.3f %.3f %.3f attempt=%d' % (gx + 3.0, gy, gz, sent))
            if self.wait_for(lambda: self.waypoints > before, 10, 'baseline waypoint'):
                got = True
                break
        say('BASELINE_WAYPOINT_GOT=%s attempts=%d waypoints=%d path_stamps=%d'
            % (got, sent, self.waypoints, len(self.path_stamps)))
        if not got:
            say('CUT_RESULT=INCONCLUSIVE 没有基线航点，断云后的静默分不清是注入造成的')
            return 6

        say('CUT_BEGIN kill /sim_adapter')
        subprocess.run(['rosnode', 'kill', '/sim_adapter'], check=False)
        cut_at = rospy.get_time()
        say('CUT_AT=%.3f' % cut_at)

        # 观测窗分两段——「停产」只能在**自停之后**才开始算：
        #   A 段：等适配器自己判 stale 并发取消。stale 状态帧与取消帧走不同话题，
        #        投递顺序不保证，所以两者并列记，不能看到取消就退出去等状态。
        #   B 段：自停之后再观察 12 s，期间不许再有新路径、新局部目标。
        # 第一轮把「断云那一刻」当停产起点，于是断云后仍在生成的 20 条路径被算成失败——
        # 那是 FAR 还在用最后一片云继续出航点（正常收敛），不是「没停」。
        halt_at = None
        halt_path = halt_goal = None
        stale_at = None
        settle_until = None
        deadline = time.monotonic() + 120.0
        while time.monotonic() < deadline:
            if stale_at is None:
                hit = [t for t in self.status_seen if 'E_FAR_WAYPOINT_STALE' in t]
                if hit:
                    stale_at = rospy.get_time()
                    say('CUT_STALE_STATUS=%s' % hit[0])
                    say('CUT_STALE_AFTER_ROS_S=%.3f' % (stale_at - cut_at))
            if halt_at is None and self.cancels and self.acks:
                halt_at = rospy.get_time()
                halt_path = len(self.path_stamps)
                halt_goal = len(self.local_goal_stamps)
                settle_until = time.monotonic() + 12.0
                say('CUT_HALT_AT=%.3f CUT_HALT_AFTER_ROS_S=%.3f' % (halt_at, halt_at - cut_at))
            if settle_until is not None and time.monotonic() >= settle_until:
                break
            time.sleep(0.1)

        base_paths = halt_path if halt_path is not None else len(self.path_stamps)
        base_goals = halt_goal if halt_goal is not None else len(self.local_goal_stamps)
        new_paths = len(self.path_stamps) - base_paths
        new_goals = len(self.local_goal_stamps) - base_goals
        say('CUT_SELF_CANCEL=%s' % (self.cancels or 'none'))
        say('CUT_ACK=%s' % (self.acks or 'none'))
        say('CUT_STATUS_SEEN=%s' % self.status_seen)
        say('CUT_POST_HALT_PATH_STAMPS=%d CUT_POST_HALT_LOCAL_GOAL_STAMPS=%d'
            % (new_paths, new_goals))
        ok = (stale_at is not None and bool(self.cancels) and bool(self.acks)
              and new_paths == 0 and new_goals == 0)
        say('CUT_RESULT=%s' % ('PASS' if ok else 'FAIL'))
        return 0 if ok else 7


def say(text):
    print(text, flush=True)


if __name__ == '__main__':
    raise SystemExit(CutTest().run())
