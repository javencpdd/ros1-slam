"""A6 定位消息失效/重定位的 ROS1 下游撤销与恢复证据。

默认以带实际 seed 扰动的 nav_msgs/Odometry 夹具输入；--source-odom 可从已有话题
复制消息到独立故障注入命名空间，不停止也不修改上游定位节点。此驱动不证明
LIO/NDT 精度或可互换性，也不把 /cmd_vel 零命令当作物理制动。
"""
import argparse
import copy
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import time

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(HERE))
import runtime_evaluator as evaluator


class RuntimeProbe:
    def __init__(self, profile, repeat, source_topic=''):
        import rospy
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from std_msgs.msg import Bool, String, UInt64

        self.rospy, self.Twist, self.Odometry = rospy, Twist, Odometry
        self.Bool, self.UInt64 = Bool, UInt64
        self.prefix = '/a6_s%s_r%s' % (profile['seed'], repeat)
        self.t0 = time.monotonic()
        self.profile = profile
        self.record = {'profile': profile, 'profile_hash': evaluator.digest(profile),
                       'repeat': repeat, 'source_topic': source_topic,
                       'input_mode': 'relayed_existing_odometry' if source_topic else 'declared_fixture',
                       'inputs': [], 'outputs': [], 'statuses': [], 'cases': []}
        self.latest = None
        self.last_source_stamp = None
        self.latest_status = {}
        self.offset, self.stamp, self.lease = 0.0, 1.0, 0
        self.odom_pub = rospy.Publisher(self.prefix + '/odom', Odometry, queue_size=20)
        self.cmd_pub = rospy.Publisher(self.prefix + '/candidate', Twist, queue_size=20)
        self.lease_pub = rospy.Publisher(self.prefix + '/lease', UInt64, queue_size=20)
        self.relocal_pub = rospy.Publisher(self.prefix + '/relocalized', Bool, queue_size=20)

        def output(message):
            self.record['outputs'].append(dict(t=self.now(), vx=message.linear.x,
                vy=message.linear.y, vz=message.linear.z, wx=message.angular.x,
                wy=message.angular.y, wz=message.angular.z))

        def status(message):
            self.latest_status = json.loads(message.data)
            self.record['statuses'].append(dict(t=self.now(), **self.latest_status))

        self.subscriptions = [rospy.Subscriber(self.prefix + '/cmd_vel', Twist, output, queue_size=200),
                              rospy.Subscriber(self.prefix + '/status', String, status, queue_size=200)]
        if source_topic:
            def source(message):
                self.latest = copy.deepcopy(message)
            self.subscriptions.append(rospy.Subscriber(source_topic, Odometry, source, queue_size=20))

    def now(self):
        return time.monotonic() - self.t0

    def odometry(self, regressed=False):
        if self.record['source_topic']:
            if self.latest is None:
                return
            message = copy.deepcopy(self.latest)
            if not regressed and message.header.stamp.to_sec() == self.last_source_stamp:
                return
            self.last_source_stamp = message.header.stamp.to_sec()
            if regressed:
                message.header.stamp = self.rospy.Time.from_sec(max(0.001, self.last_source_stamp - 1))
            for axis, bias in zip(('x', 'y', 'z'), self.profile['bias_xyz']):
                setattr(message.pose.pose.position, axis,
                        getattr(message.pose.pose.position, axis) + bias)
            message.pose.pose.position.x += self.offset
        else:
            self.stamp += 0.04
            message = self.Odometry()
            message.header.frame_id = 'map'
            message.child_frame_id = 'base_link'
            message.header.stamp = self.rospy.Time.from_sec(self.stamp - (1 if regressed else 0))
            xyz = list(self.profile['bias_xyz'])
            xyz[0] += self.offset
            message.pose.pose.position.x, message.pose.pose.position.y, message.pose.pose.position.z = xyz
            message.pose.pose.orientation.w = 1.0
        p = message.pose.pose.position
        self.record['inputs'].append({'t': self.now(), 'kind': 'odom',
                                      'stamp': message.header.stamp.to_sec(), 'xyz': [p.x, p.y, p.z]})
        self.odom_pub.publish(message)

    def candidate(self):
        message = self.Twist()
        message.linear.x = self.profile['candidate_speed']
        self.record['inputs'].append({'t': self.now(), 'kind': 'candidate', 'vx': message.linear.x})
        self.cmd_pub.publish(message)

    def pump(self, duration, odom=True):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline and not self.rospy.is_shutdown():
            if odom:
                self.odometry()
            self.candidate()
            time.sleep(0.03)

    def wait(self, predicate, timeout=3):
        deadline = time.monotonic() + timeout
        while not predicate():
            if time.monotonic() >= deadline:
                raise RuntimeError('A6 condition timed out: %r' % self.latest_status)
            self.pump(0.04)

    def grant(self, lease=None):
        if lease is None:
            self.lease += 1
        else:
            self.lease = lease
        when = self.now()
        self.record['inputs'].append({'t': when, 'kind': 'lease', 'lease': self.lease})
        self.lease_pub.publish(self.UInt64(data=self.lease))
        return when

    def exercise(self):
        self.wait(lambda: self.odom_pub.get_num_connections() > 0
                  and self.cmd_pub.get_num_connections() > 0
                  and self.lease_pub.get_num_connections() > 0
                  and self.relocal_pub.get_num_connections() > 0
                  and bool(self.record['outputs']))
        self.wait(lambda: self.latest_status.get('state') == 'HEALTHY')
        self.grant()
        for name in ('stale', 'jump', 'relocalization', 'stamp_regression'):
            armed_start = self.now()
            self.pump(0.18)
            armed_end = self.now()
            if name == 'stale':
                fault = next(row['t'] for row in reversed(self.record['inputs']) if row['kind'] == 'odom')
                self.pump(self.profile['stale_pause_s'], odom=False)
                earliest_stop = fault + 0.25
            elif name == 'jump':
                self.offset += self.profile['jump_m']
                fault = self.now()
                self.odometry()
                earliest_stop = fault
                self.pump(0.10)
            elif name == 'relocalization':
                fault = self.now()
                self.record['inputs'].append({'t': fault, 'kind': 'relocalized', 'value': True})
                self.relocal_pub.publish(self.Bool(data=True))
                earliest_stop = fault
                self.pump(0.10)
            else:
                fault = self.now()
                self.odometry(regressed=True)
                earliest_stop = fault
                self.pump(0.10)
            self.wait(lambda: self.latest_status.get('state') == 'HEALTHY')
            # Resume the input stream before testing; stale's wait() pumps fresh odometry.
            hold_start = self.now()
            self.pump(0.12)
            self.grant(self.lease)  # The revoked old lease must remain rejected.
            self.pump(0.09)
            hold_end = self.now()
            resume = self.grant(self.lease + 1)
            self.pump(0.18)
            self.record['cases'].append({'case': name, 'fault_at': fault,
                'armed_window': [armed_start, armed_end],
                'stop_window': [earliest_stop, hold_start + 0.10],
                'hold_window': [hold_start, hold_end], 'resume_at': resume,
                'resume_window': [resume, self.now()]})
        self.record['final_status'] = dict(self.latest_status)
        self.record['evaluation'] = evaluator.evaluate(self.record)
        return self.record

    def close(self):
        self.cmd_pub.publish(self.Twist())
        for subscription in self.subscriptions:
            subscription.unregister()


def run(profile, repeat=1, source_topic=''):
    probe = RuntimeProbe(profile, repeat, source_topic)
    with tempfile.TemporaryFile() as log:
        command = [sys.executable, str(ROOT / 'adapters' / 'localization_state.py'),
                   '--namespace', probe.prefix, '--keep-alive-s', '60']
        proc = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
        try:
            return probe.exercise()
        except Exception as error:
            log.seek(0)
            raise RuntimeError('%s; node log: %s' % (error, log.read()[-5000:].decode(errors='replace'))) from error
        finally:
            probe.close()
            if proc.poll() is None:
                proc.terminate()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait(timeout=5)


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--seeds', default='0,1,2')
    parser.add_argument('--repetitions', type=int, default=1)
    parser.add_argument('--source-odom', default='')
    parser.add_argument('--out', default='')
    args = parser.parse_args(argv)
    import rospy
    rospy.init_node('a6_runtime', anonymous=True)
    records = [run(evaluator.input_profile(seed), repeat, args.source_odom)
               for seed in map(int, args.seeds.split(',')) for repeat in range(1, args.repetitions + 1)]
    result = {'records': records, 'matrix': evaluator.matrix(records)}
    if args.out:
        Path(args.out).write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result['matrix']))
    return 0 if result['matrix']['ok'] else 6


if __name__ == '__main__':
    raise SystemExit(main())
