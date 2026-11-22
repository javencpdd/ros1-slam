"""场景驱动器（v3）：一个节点顺序跑完四个场景，标记与目标由同一个发布者发出。

v2 的两次失败各留一条教训，这里都改了：

* **C0 的目标发得太早，被上游静默丢弃。** 上游 `WaypointCallBack`
  （far_planner.cpp:699-703）在 `!is_graph_init_` 时直接 return，而 v2 在 ROS 时间
  0.85 s 就发了目标（V-Graph 初始化的打印出现在 1.0 s 前后）。所以 C0 不再靠「等
  固定秒数」，而是**重发直到被接受**：接受的可观测证据是 `/far_reach_goal_status`
  开始出帧（上游 `PlanningCallBack` 只有在 `is_graph_init_` 且**有目标**时才走到
  发布那一行，far_planner.cpp:237-305）。
* **C4 在抬位姿之前，上一场景的目标还挂着。** v2 里 C2 的目标是 `(0,0,z+3)`，
  位姿一抬到 3.0 m，判据立刻判到达——那一帧 arrived=true 是**对的**，但它落在
  新目标之前，被算进了 C4 的窗口（见 `../b5_arrival2/analyze_attemptB.log`）。
  这里先发一个 5 m 外的中间目标把判据压成 false，再抬位姿，最后才发地面目标；
  这样整个 C4 窗口里任何 arrived=true 都是真失败。

判据面：`/v8/bt/arrived`（适配器按 `|pose−goal|_3d ≤ hypot(0.25,0.30)` 自判，latch）。
* C0_liveness    远目标 3.0 m：证明「目标被接受」且远目标不判到达
* C1_arrival     目标 0.2 m（< 0.3905）：**正向对照**，必须出现 arrived=true
* C2_cross_floor 同 xy、z 抬 3.0 m：arrived 必须一直是 false
* C3_offset      平地静止读 pose.z − goal.z，作为 z_tol 的标定依据
* C4_pose_shift  位姿抬到 z+3.0、目标留在地面：不得出现 arrived=true
"""
import subprocess
import threading
import time

import rospy
from geometry_msgs.msg import PointStamped, PoseStamped
from std_msgs.msg import Bool, String


class Driver(object):
    def __init__(self):
        rospy.init_node('b5_driver', disable_signals=True)
        self.goal_pub = rospy.Publisher('/v8/global/goal', PoseStamped, queue_size=1, latch=True)
        self.mark_pub = rospy.Publisher('/_b5_mark', String, queue_size=10)
        self.pose_pub = rospy.Publisher('/v8/local/pose', PoseStamped, queue_size=2)
        self.pose = None
        self.waypoints = 0
        self.reach_false = 0
        self.reach_true = 0
        self.goal_point_seen = 0
        self.goal_point_last = None
        self.ground = None
        rospy.Subscriber('/v8/local/pose', PoseStamped, self._on_pose, queue_size=2)
        rospy.Subscriber('/way_point', PointStamped, self._on_waypoint, queue_size=5)
        rospy.Subscriber('/far_reach_goal_status', Bool, self._on_reach, queue_size=5)
        rospy.Subscriber('/goal_point', PointStamped, self._on_goal_point, queue_size=5)

    def _on_goal_point(self, msg):
        self.goal_point_seen += 1
        point = msg.point
        self.goal_point_last = (round(point.x, 3), round(point.y, 3), round(point.z, 3))

    def _on_pose(self, msg):
        position = msg.pose.position
        self.pose = (position.x, position.y, position.z)

    def _on_waypoint(self, _msg):
        self.waypoints += 1

    def _on_reach(self, msg):
        if msg.data:
            self.reach_true += 1
        else:
            self.reach_false += 1

    # ---- 基础动作 ----
    def wait_for(self, predicate, timeout, note, tick=0.1):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return True
            time.sleep(tick)
        say('TIMEOUT %s after %.0f s' % (note, timeout))
        return False

    def mark(self, tag):
        for _ in range(3):
            self.mark_pub.publish(String(data=tag))
            time.sleep(0.05)

    def goal(self, xyz, quiet=False, stamp='now'):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = rospy.Time.now() if stamp == 'now' else rospy.Time(0)
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = xyz
        msg.pose.orientation.w = 1.0
        self.goal_pub.publish(msg)
        if not quiet:
            say('GOAL %.3f %.3f %.3f stamp=%s' % (xyz[0], xyz[1], xyz[2], stamp))

    def accepted_since(self, before):
        return (self.reach_false > before[0] or self.waypoints > before[1])

    def ab_goal(self, xyz):
        """同一点、三种发法的 A/B/C：定位「目标在哪一步消失」。

        A 本节点 Publisher，`stamp=now`；B `rostopic pub -1`（独立进程、`stamp=0`）；
        C 本节点 Publisher，`stamp=0`。
        `far_node` 收到 goal 就会发 `/goal_point`，所以三条都同时看 `goal_point_seen`：
        它涨了说明适配器那一跳通了，没涨说明目标根本没到适配器。
        """
        for tag, kwargs, via in (('A_driver_stamp_now', {'stamp': 'now'}, 'driver'),
                                 ('C_driver_stamp_zero', {'stamp': 'zero'}, 'driver'),
                                 ('B_rostopic_pub', {'stamp': 'zero'}, 'rostopic')):
            before = (self.reach_false, self.waypoints, self.goal_point_seen)
            if via == 'driver':
                self.goal(xyz, stamp=kwargs['stamp'])
            else:
                subprocess.run(['rostopic', 'pub', '-1', '/v8/global/goal',
                                'geometry_msgs/PoseStamped',
                                '{header: {frame_id: "map"}, pose: {position: {x: %.3f, y: %.3f, z: %.3f}, orientation: {w: 1.0}}}'
                                % xyz], check=False,
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            ok = self.wait_for(lambda: self.accepted_since(before[:2]), 20, 'AB ' + tag)
            say('AB_%s ok=%s reach_delta=%d wp_delta=%d goal_point_delta=%d last=%s'
                % (tag, ok, self.reach_false - before[0], self.waypoints - before[1],
                   self.goal_point_seen - before[2], self.goal_point_last))
            if ok:
                self.snapshot('AB_END')
                return tag
        return None

    def begin(self, tag):
        self.mark(tag)
        # 标记先到、目标后到：跨话题投递顺序不能保证，留 0.5 s 让标记先落盘
        time.sleep(0.5)

    def snapshot(self, tag):
        say('%s reach_false=%d reach_true=%d waypoints=%d'
            % (tag, self.reach_false, self.reach_true, self.waypoints))

    # ---- 场景 ----
    def run(self):
        ready = self.wait_for(
            lambda: (self.goal_pub.get_num_connections() >= 1
                     and self.mark_pub.get_num_connections() >= 1
                     and self.pose is not None),
            60, 'driver connections')
        if not ready:
            return 5
        say('DRIVER_READY goal_conn=%d mark_conn=%d pose=%.3f %.3f %.3f'
            % (self.goal_pub.get_num_connections(), self.mark_pub.get_num_connections(),
               self.pose[0], self.pose[1], self.pose[2]))
        self.ground = tuple(round(v, 3) for v in self.pose)
        gx, gy, gz = self.ground

        # 先做 A/B/C：同一个 1.0 m 目标，三种发法，看「目标在哪一步消失」
        self.begin('C0_ab')
        winner = self.ab_goal((gx + 1.0, gy, gz))
        say('AB_WINNER=%s' % winner)

        # C0：远目标，重发直到上游接受（接受＝`/far_reach_goal_status` 开始出帧）
        self.begin('C0_liveness')
        attempts = 0
        started = time.monotonic()
        accepted = False
        while attempts < 40 and time.monotonic() - started < 150.0 and not accepted:
            attempts += 1
            self.goal((gx + 3.0, gy, gz), quiet=(attempts > 1))
            accepted = self.wait_for(lambda: self.reach_false > 0 or self.waypoints > 0,
                                     4.0, 'C0 accept', tick=0.05) and True
        say('C0_ATTEMPTS=%d C0_ACCEPTED=%s C0_ELAPSED_S=%.1f'
            % (attempts, accepted, time.monotonic() - started))
        self.wait_for(lambda: self.reach_false >= 5, 15, 'C0 reach ticks')
        self.snapshot('C0_END')

        # C1：正向对照，0.2 m < 0.3905 ⇒ 必须出现 arrived=true
        self.begin('C1_arrival')
        self.goal((gx + 0.2, gy, gz))
        time.sleep(12.0)
        self.snapshot('C1_END')

        # C2：跨层同 xy，z 差 3.0 m ⇒ arrived 必须一直是 false
        self.begin('C2_cross_floor')
        self.goal((gx, gy, gz + 3.0))
        time.sleep(25.0)
        self.snapshot('C2_END')

        # C3：偏置标定（平地静止，goal 取地面标高）
        self.begin('C3_offset')
        say('OFFSET pose_z=%.3f goal_z=%.3f diff=%.3f' % (self.pose[2], gz, self.pose[2] - gz))
        time.sleep(3.0)

        # C4：位姿挪层。先把判据压成 false，再抬位姿，最后发地面目标
        self.begin('C4_pose_shift')
        self.goal((gx + 5.0, gy, gz))
        time.sleep(2.0)
        subprocess.run(['rosnode', 'kill', '/sim_pose_source'], check=False)
        time.sleep(2.0)
        stop = threading.Event()
        thread = threading.Thread(target=self._publish_shift,
                                  args=((gx, gy, gz + 3.0), stop), daemon=True)
        thread.start()
        shifted = self.wait_for(lambda: abs(self.pose[2] - (gz + 3.0)) < 0.05,
                                15, 'C4 pose shift readback')
        say('C4_POSE_SHIFTED=%s pose_z=%.3f' % (shifted, self.pose[2]))
        self.goal((gx, gy, gz))
        time.sleep(25.0)
        self.snapshot('C4_END')
        stop.set()
        thread.join(timeout=2.0)
        say('DRIVER_DONE')
        return 0 if accepted else 6

    def _publish_shift(self, xyz, stop_event):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.pose.orientation.w = 1.0
        while not stop_event.is_set() and not rospy.is_shutdown():
            msg.header.stamp = rospy.Time.now()
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = xyz
            self.pose_pub.publish(msg)
            time.sleep(0.05)


def say(text):
    print(text, flush=True)


if __name__ == '__main__':
    raise SystemExit(Driver().run())
