#!/usr/bin/env python3
"""S2 ROS smoke: fake robot endpoint + /cmd_vel publisher + telemetry subscribers.

Run after starting roscore and boundary.ros_bridge with matching ports.
"""
import argparse
import json
import threading
import time

from .protocol import TelemetryFrame, TELEM_OK  # noqa: E402
from .udp_link import CommandReceiver, TelemetrySender  # noqa: E402


def main():
    import rospy
    from geometry_msgs.msg import TwistStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu, JointState
    from std_msgs.msg import String

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=3.)
    parser.add_argument('--bind', default='127.0.0.1:5016')
    parser.add_argument('--target', default='127.0.0.1:5015')
    parser.add_argument('--out')
    args = parser.parse_args()
    rospy.init_node('v8_boundary_s2_check')
    bind_host, bind_port = args.bind.rsplit(':', 1)
    target_host, target_port = args.target.rsplit(':', 1)
    receiver = CommandReceiver((bind_host, int(bind_port)))
    receiver.sock.setblocking(False)
    sender = TelemetrySender((target_host, int(target_port)))
    command_pub = rospy.Publisher('/cmd_vel', TwistStamped, queue_size=10)
    received = {'odom': 0, 'imu': 0, 'joint': 0, 'status': 0}
    seen = {'odom': None, 'status': None, 'joint_names': None}

    def on_odom(message):
        received['odom'] += 1
        seen['odom'] = {'position': [message.pose.pose.position.x,
                                     message.pose.pose.position.y,
                                     message.pose.pose.position.z],
                        'linear': [message.twist.twist.linear.x, message.twist.twist.linear.y]}

    def on_imu(_message):
        received['imu'] += 1

    def on_joint(message):
        received['joint'] += 1
        seen['joint_names'] = list(message.name)

    def on_status(message):
        received['status'] += 1
        try:
            seen['status'] = json.loads(message.data)
        except ValueError:
            pass

    rospy.Subscriber('/boundary/odom', Odometry, on_odom, queue_size=10)
    rospy.Subscriber('/boundary/imu', Imu, on_imu, queue_size=10)
    rospy.Subscriber('/boundary/joint_states', JointState, on_joint, queue_size=10)
    rospy.Subscriber('/boundary/status', String, on_status, queue_size=10)
    stop = threading.Event()
    sent = [0]

    def telemetry_loop():
        seq = 0
        next_tick = time.monotonic()
        while not stop.is_set() and not rospy.is_shutdown():
            frame = TelemetryFrame(seq, time.monotonic_ns(), 1., 2., .3, .1, 0., 0.,
                                   1., 0., 0., 0., 0., 0., 0., 0., 0., 9.8,
                                   TELEM_OK, 0, tuple([0.] * 12), tuple([0.] * 12))
            try:
                sender.send(frame)
                sent[0] += 1
            except Exception:
                pass
            seq = (seq + 1) & 0xFFFFFFFF
            next_tick += .02
            delay = next_tick - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_tick = time.monotonic()

    thread = threading.Thread(target=telemetry_loop, daemon=True)
    thread.start()
    commands = []
    started = time.monotonic()
    next_cmd = started
    try:
        while time.monotonic() - started < args.seconds and not rospy.is_shutdown():
            message = TwistStamped()
            message.header.frame_id, message.header.stamp = 'map', rospy.Time.now()
            message.twist.linear.x = .2
            command_pub.publish(message)
            frame = receiver.poll()
            while frame is not None:
                commands.append([frame.seq, frame.mode, frame.vx])
                frame = receiver.poll()
            next_cmd += .05
            delay = next_cmd - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_cmd = time.monotonic()
    finally:
        stop.set()
        sender.close()
        receiver.close()
    summary = {'seconds': round(time.monotonic() - started, 3),
               'telemetry_sent': sent[0],
               'commands_received': len(commands),
               'command_modes': sorted(set(c[1] for c in commands)),
               'ros': received, 'odom': seen['odom'], 'joint_names': seen['joint_names'],
               'status': seen['status'],
               'command_loss': receiver.stats['loss'], 'command_bad': receiver.stats['bad']}
    print(json.dumps(summary, indent=1))
    if args.out:
        with open(args.out, 'w', encoding='utf-8') as stream:
            json.dump(summary, stream, indent=1)
            stream.write('\n')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
