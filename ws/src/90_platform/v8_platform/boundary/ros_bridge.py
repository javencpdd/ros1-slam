#!/usr/bin/env python3
"""S2 ROS wrapper: mux-rate intent -> fixed-rate UDP frames; telemetry -> ROS.

Pure core is importable without ROS. The node keeps the boundary as the single
UDP command-socket owner; it never publishes /cmd_vel and never talks to the
vendor QNX stack (real-robot S5 stays L2).
"""
import json
import math
import time

try:
    from .protocol import (FAULT_NONFINITE, FAULT_POLICY, MODE_ACTIVE, MODE_HOLD,
                           TELEM_FAULT, FLAG_PLANNER_OK)
    from .rate_bridge import CommandBridge, Limits
except ImportError:  # direct script execution from the boundary directory
    from protocol import (FAULT_NONFINITE, FAULT_POLICY, MODE_ACTIVE, MODE_HOLD,
                          TELEM_FAULT, FLAG_PLANNER_OK)
    from rate_bridge import CommandBridge, Limits


class NodeCore:
    """Fixed-rate command decision + link-health state, independent of ROS."""

    def __init__(self, limits=None, link_timeout_s=.5, clock=time.monotonic):
        self.bridge = CommandBridge(limits)
        self.link_timeout_s = link_timeout_s
        self.clock = clock
        self.telemetry = None
        self.telemetry_at = None
        self.telemetry_frames = 0
        self.fault = None
        self.sent = 0

    def on_command(self, vx, vy, wz, now=None):
        self.bridge.update(vx, vy, wz, self.clock() if now is None else now)

    def on_telemetry(self, frame, now=None):
        self.telemetry = frame
        self.telemetry_at = self.clock() if now is None else now
        self.telemetry_frames += 1

    def telemetry_age(self, now=None):
        if self.telemetry_at is None:
            return None
        return (self.clock() if now is None else now) - self.telemetry_at

    def tick(self, now=None):
        now = self.clock() if now is None else now
        vx, vy, wz = self.bridge.command(now)
        age = self.telemetry_age(now)
        if self.telemetry is None:
            self.fault = 'telemetry_missing'
            mode, out = MODE_HOLD, (0., 0., 0.)
        elif age > self.link_timeout_s:
            self.fault = 'telemetry_stale'
            mode, out = MODE_HOLD, (0., 0., 0.)
        elif self.telemetry.state == TELEM_FAULT:
            self.fault = 'robot_fault'
            mode, out = MODE_HOLD, (0., 0., 0.)
        elif self.bridge.mode == 'active':
            self.fault = None
            mode, out = MODE_ACTIVE, (vx, vy, wz)
        else:
            self.fault = None
            mode, out = MODE_HOLD, (0., 0., 0.)
        flags = FLAG_PLANNER_OK if mode == MODE_ACTIVE else 0
        self.sent += 1
        return out, mode, flags

    def status(self, now=None):
        now = self.clock() if now is None else now
        telemetry_age = self.telemetry_age(now)
        planner_age = self.bridge.age(now)
        return {'state': self.fault or self.bridge.mode,
                'fault': self.fault,
                'planner_age_s': None if planner_age is None else round(planner_age, 4),
                'telemetry_age_s': None if telemetry_age is None else round(telemetry_age, 4),
                'telemetry_frames': self.telemetry_frames,
                'frames_sent': self.sent,
                'bridge': dict(self.bridge.stats),
                'robot_state': None if self.telemetry is None else self.telemetry.state}


def policy_command(vx, vy, wz):
    """Map body-frame velocity intent to the policy's forward+yaw interface."""
    values = (float(vx), float(vy), float(wz))
    if not all(math.isfinite(v) for v in values):
        raise ValueError('non-finite command')
    speed = math.hypot(values[0], values[1])
    heading = 0. if speed <= 1e-9 else math.atan2(values[1], values[0])
    return speed, float(max(-.5, min(.5, values[2] + 3.0 * heading)))


def telemetry_status(frame):
    return {'seq': frame.seq, 'state': frame.state, 'fault': frame.fault,
            'nonfinite': bool(frame.fault & FAULT_NONFINITE),
            'policy': bool(frame.fault & FAULT_POLICY)}


def main():
    import rospy
    from geometry_msgs.msg import Twist, TwistStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu, JointState
    from std_msgs.msg import String

    try:
        from .udp_link import CommandSender, TelemetryReceiver
    except ImportError:
        from udp_link import CommandSender, TelemetryReceiver

    rospy.init_node('v8_boundary_adapter')
    target = rospy.get_param('~target')
    host, port = target.rsplit(':', 1)
    bind = rospy.get_param('~bind', '127.0.0.1:0')
    bind_host, bind_port = bind.rsplit(':', 1)
    rate_hz = float(rospy.get_param('~rate_hz', 500.))
    if not (50. <= rate_hz <= 500.):
        raise ValueError('rate_hz must be within 50..500')
    limits = Limits(
        max_speed=float(rospy.get_param('~max_speed', 0.8)),
        max_wz=float(rospy.get_param('~max_wz', 1.0)),
        max_accel=float(rospy.get_param('~max_accel', 1.5)),
        max_yaw_accel=float(rospy.get_param('~max_yaw_accel', 3.0)),
        command_timeout_s=float(rospy.get_param('~command_timeout_s', 0.25)))
    core = NodeCore(limits=limits,
                    link_timeout_s=float(rospy.get_param('~link_timeout_s', 0.5)))
    sender = CommandSender((host, int(port)))
    receiver = TelemetryReceiver(bind=(bind_host, int(bind_port)))
    rospy.loginfo('boundary udp -> %s, telemetry <- %s', target, receiver.address)

    odom_pub = rospy.Publisher(rospy.get_param('~odom_topic', '/boundary/odom'),
                               Odometry, queue_size=10)
    imu_pub = rospy.Publisher(rospy.get_param('~imu_topic', '/boundary/imu'),
                              Imu, queue_size=10)
    joint_pub = rospy.Publisher(rospy.get_param('~joint_topic', '/boundary/joint_states'),
                                JointState, queue_size=10)
    status_pub = rospy.Publisher(rospy.get_param('~status_topic', '/boundary/status'),
                                 String, queue_size=10)
    command_topic = rospy.get_param('~command_topic', '/cmd_vel')
    command_type = rospy.get_param('~command_type', 'twist_stamped')

    def on_twist_stamped(message):
        core.on_command(message.twist.linear.x, message.twist.linear.y,
                        message.twist.linear.z)

    def on_twist(message):
        core.on_command(message.linear.x, message.linear.y, message.linear.z)

    if command_type == 'twist_stamped':
        rospy.Subscriber(command_topic, TwistStamped, on_twist_stamped, queue_size=10)
    elif command_type == 'twist':
        rospy.Subscriber(command_topic, Twist, on_twist, queue_size=10)
    else:
        raise ValueError('command_type must be twist_stamped or twist')

    def stamp():
        now = rospy.Time.now()
        return now if now.to_sec() > 0 else rospy.Time.from_sec(time.time())

    def publish_telemetry(frame):
        stamp_now = stamp()
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id = stamp_now, 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = (
            frame.x, frame.y, frame.z)
        odom.pose.pose.orientation.w, odom.pose.pose.orientation.x = frame.qw, frame.qx
        odom.pose.pose.orientation.y, odom.pose.pose.orientation.z = frame.qy, frame.qz
        odom.twist.twist.linear.x, odom.twist.twist.linear.y = frame.vx, frame.vy
        odom.twist.twist.linear.z = frame.vz
        odom.twist.twist.angular.x, odom.twist.twist.angular.y = frame.gx, frame.gy
        odom.twist.twist.angular.z = frame.gz
        odom_pub.publish(odom)
        imu = Imu()
        imu.header.stamp, imu.header.frame_id = stamp_now, 'base_link'
        imu.orientation.w, imu.orientation.x = frame.qw, frame.qx
        imu.orientation.y, imu.orientation.z = frame.qy, frame.qz
        imu.angular_velocity.x, imu.angular_velocity.y = frame.gx, frame.gy
        imu.angular_velocity.z = frame.gz
        imu.linear_acceleration.x, imu.linear_acceleration.y = frame.ax, frame.ay
        imu.linear_acceleration.z = frame.az
        imu_pub.publish(imu)
        joints = JointState()
        joints.header.stamp = stamp_now
        joints.name = ['joint_%d' % index for index in range(len(frame.joint_pos))]
        joints.position, joints.velocity = list(frame.joint_pos), list(frame.joint_vel)
        joint_pub.publish(joints)
        return stamp_now

    period = 1. / rate_hz
    next_tick = time.monotonic()
    last_status = 0.
    ticks = 0
    try:
        while not rospy.is_shutdown():
            frame = receiver.poll()
            if frame is not None:
                core.on_telemetry(frame)
                publish_telemetry(frame)
            now = time.monotonic()
            (vx, vy, wz), mode, flags = core.tick(now)
            sender.send(vx, vy, wz, mode, flags)
            ticks += 1
            if now - last_status >= .2:
                payload = core.status(now)
                payload.update(receiver=dict(receiver.stats),
                               telemetry_jitter=receiver.jitter(),
                               rate_hz=rate_hz, ticks=ticks)
                status_pub.publish(String(data=json.dumps(payload, sort_keys=True)))
                last_status = now
            next_tick += period
            delay = next_tick - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_tick = time.monotonic()
    finally:
        sender.close()
        receiver.close()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
