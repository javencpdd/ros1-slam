#!/usr/bin/env python3
"""S3 Gazebo smoke: send UDP commands, receive telemetry, check model motion."""
import argparse
import json
import socket
import time

from .protocol import MODE_ACTIVE, CommandFrame
from .udp_link import TelemetryReceiver


def main():
    import rospy
    from gazebo_msgs.srv import GetModelState

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', default='s3_box')
    parser.add_argument('--seconds', type=float, default=3.)
    parser.add_argument('--command', default='0.2,0,0')
    parser.add_argument('--target', default='127.0.0.1:5026')
    parser.add_argument('--bind', default='127.0.0.1:5025')
    parser.add_argument('--out')
    args = parser.parse_args()
    rospy.init_node('v8_boundary_s3_check')
    command = tuple(float(v) for v in args.command.split(','))
    target_host, target_port = args.target.rsplit(':', 1)
    bind_host, bind_port = args.bind.rsplit(':', 1)
    telemetry = TelemetryReceiver((bind_host, int(bind_port)))
    telemetry.sock.setblocking(False)
    sock_commands = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    target = (target_host, int(target_port))
    rospy.wait_for_service('/gazebo/get_model_state', timeout=20.)
    get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    seq = 0
    frames = 0
    first = last = None
    started = time.monotonic()
    next_tick = started
    try:
        while time.monotonic() - started < args.seconds:
            frame = CommandFrame(seq, time.monotonic_ns(), command[0], command[1],
                                 command[2], MODE_ACTIVE)
            sock_commands.sendto(frame.encode(), target)
            seq = (seq + 1) & 0xFFFFFFFF
            telemetry_frame = telemetry.poll()
            while telemetry_frame is not None:
                first = first or telemetry_frame
                last = telemetry_frame
                frames += 1
                telemetry_frame = telemetry.poll()
            next_tick += .002
            delay = next_tick - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_tick = time.monotonic()
    finally:
        sock_commands.close()
        telemetry.close()
    state = get_state(args.model, 'world')
    pose = state.pose.position
    summary = {'seconds': round(time.monotonic() - started, 3),
               'commands_sent': seq, 'telemetry_frames': frames,
               'telemetry_loss': telemetry.stats['loss'], 'telemetry_bad': telemetry.stats['bad'],
               'first_telemetry': None if first is None else [first.x, first.y, first.z],
               'last_telemetry': None if last is None else [last.x, last.y, last.z],
               'gazebo_position': [round(pose.x, 4), round(pose.y, 4), round(pose.z, 4)],
               'expected': [round(command[0] * args.seconds, 3), 0., 0.]}
    summary['moved'] = pose.x > .05
    print(json.dumps(summary, indent=1))
    if args.out:
        with open(args.out, 'w', encoding='utf-8') as stream:
            json.dump(summary, stream, indent=1)
            stream.write('\n')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
