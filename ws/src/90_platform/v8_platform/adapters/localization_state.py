"""定位时效/一致性租约与 ROS1 下游速度门控。

健康只表示消息及时、有限且时间/位姿连续，不证明定位精度。
失效或重定位会撤销当前租约；健康恢复后必须提交新的租约号，旧轨迹不自动复活。
ROS 适配仅在 main 导入，状态机可在宿主独立验证。
"""
import argparse
import json
import math
import threading
import time


class LocalizationState:
    def __init__(self, timeout_s=0.25, max_jump_m=0.5, stable_samples=3):
        if timeout_s <= 0 or max_jump_m <= 0 or stable_samples < 1:
            raise ValueError('positive timeout/jump and stable_samples >= 1 required')
        self.timeout_s = timeout_s
        self.max_jump_m = max_jump_m
        self.stable_samples = stable_samples
        self.state = 'INITIALIZING'
        self.reason = 'E_LOC_NOT_READY'
        self.last_received = None
        self.last_stamp = None
        self.last_position = None
        self.samples = 0
        self.active_lease = None
        self.last_lease = 0
        self.generation = 0
        self.revocations = 0
        self.recoveries = 0
        self.faults = 0
        self.backend_valid = True

    def _invalidate(self, reason, state='LOST'):
        # One transition invalidates once; repeated timer ticks must not grow epochs.
        if self.state != state or self.reason != reason:
            self.generation += 1
            self.faults += 1
        if self.active_lease is not None:
            self.revocations += 1
        self.active_lease = None
        self.samples = 0
        self.state, self.reason = state, reason

    def relocalize(self):
        self._invalidate('E_LOC_RELOCALIZING', 'RELOCALIZING')
        # A new localization frame may legitimately have a new pose/stamp origin.
        self.last_position = self.last_stamp = self.last_received = None

    def backend_health(self, valid):
        self.backend_valid = bool(valid)
        if not self.backend_valid:
            self._invalidate('E_LOC_BACKEND_REJECTED')

    def observe(self, stamp, position, received, valid=True):
        if not self.backend_valid:
            self._invalidate('E_LOC_BACKEND_REJECTED')
            return False
        finite = (len(position) == 3 and all(isinstance(v, (int, float))
                  and not isinstance(v, bool) and math.isfinite(v) for v in position)
                  and isinstance(stamp, (int, float)) and math.isfinite(stamp)
                  and stamp > 0 and math.isfinite(received))
        if not valid or not finite:
            self._invalidate('E_LOC_INVALID')
            return False
        if self.last_stamp is not None and stamp <= self.last_stamp:
            self._invalidate('E_LOC_STAMP_NONMONOTONIC')
            return False
        if self.last_received is not None and received - self.last_received > self.timeout_s:
            self._invalidate('E_LOC_STALE')
        old_position = self.last_position
        self.last_stamp, self.last_position = stamp, tuple(position)
        self.last_received = received
        if old_position is not None and math.dist(old_position, position) > self.max_jump_m:
            self._invalidate('E_LOC_JUMP')
            return False
        self.samples += 1
        if self.samples >= self.stable_samples:
            if self.state != 'HEALTHY' and self.faults:
                self.recoveries += 1
            self.state, self.reason = 'HEALTHY', 'ok'
        return self.state == 'HEALTHY'

    def tick(self, now):
        if self.last_received is not None and now - self.last_received > self.timeout_s:
            self._invalidate('E_LOC_STALE')
        return self.state == 'HEALTHY' and self.active_lease is not None

    def activate(self, lease, now):
        self.tick(now)
        if (self.state != 'HEALTHY' or isinstance(lease, bool)
                or not isinstance(lease, int) or lease <= self.last_lease):
            return False
        self.active_lease = self.last_lease = lease
        return True

    def report(self):
        return {'state': self.state, 'reason': self.reason, 'generation': self.generation,
                'active_lease': self.active_lease, 'last_lease': self.last_lease,
                'revocations': self.revocations, 'recoveries': self.recoveries,
                'faults': self.faults, 'stable_samples': self.samples}


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--namespace', default='/a6')
    parser.add_argument('--odom-topic', default='')
    parser.add_argument('--health-topic', default='')
    parser.add_argument('--timeout-s', type=float, default=0.25)
    parser.add_argument('--command-timeout-s', type=float, default=0.20)
    parser.add_argument('--keep-alive-s', type=float, default=60)
    args = parser.parse_args(argv)
    import rospy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Bool, String, UInt64

    rospy.init_node('localization_gate', anonymous=True)
    prefix = args.namespace.rstrip('/')
    state = LocalizationState(timeout_s=args.timeout_s)
    lock = threading.RLock()
    candidate = {'twist': Twist(), 'received': None}
    output = rospy.Publisher(prefix + '/cmd_vel', Twist, queue_size=20)
    status = rospy.Publisher(prefix + '/status', String, queue_size=20)

    def odometry(message):
        point = message.pose.pose.position
        covariance = message.pose.covariance
        valid = all(math.isfinite(covariance[i]) and covariance[i] >= 0 for i in (0, 7, 14))
        with lock:
            state.observe(message.header.stamp.to_sec(), (point.x, point.y, point.z),
                          time.monotonic(), valid=valid)

    def command(message):
        numbers = (message.linear.x, message.linear.y, message.linear.z,
                   message.angular.x, message.angular.y, message.angular.z)
        with lock:
            candidate['twist'] = message if all(math.isfinite(v) for v in numbers) else Twist()
            candidate['received'] = time.monotonic()

    def activate(message):
        with lock:
            if state.activate(message.data, time.monotonic()):
                # A candidate from the old trajectory may never survive a new lease.
                candidate['received'] = None

    def relocated(message):
        if message.data:
            with lock:
                state.relocalize()
                candidate['received'] = None

    def backend_health(message):
        with lock:
            state.backend_health(message.data)
            if not message.data:
                candidate['received'] = None

    subscriptions = [
        rospy.Subscriber(args.odom_topic or prefix + '/odom', Odometry, odometry, queue_size=20),
        rospy.Subscriber(prefix + '/candidate', Twist, command, queue_size=20),
        rospy.Subscriber(prefix + '/lease', UInt64, activate, queue_size=20),
        rospy.Subscriber(prefix + '/relocalized', Bool, relocated, queue_size=20),
    ]
    if args.health_topic:
        state.backend_health(False)  # A configured external health channel must report first.
        subscriptions.append(rospy.Subscriber(args.health_topic, Bool, backend_health, queue_size=20))
    deadline = time.monotonic() + args.keep_alive_s
    try:
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            with lock:
                now = time.monotonic()
                allow = state.tick(now) and candidate['received'] is not None \
                    and now - candidate['received'] <= args.command_timeout_s
                output.publish(candidate['twist'] if allow else Twist())
                status.publish(String(data=json.dumps(state.report())))
            time.sleep(0.02)  # Wall time: a stopped simulation clock must still revoke commands.
    finally:
        output.publish(Twist())
        for subscription in subscriptions:
            subscription.unregister()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
