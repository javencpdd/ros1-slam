#!/usr/bin/env python3
"""S3 Gazebo kinematic carrier endpoint for the boundary UDP contract.

Applies body-frame CommandFrames to a static Gazebo model with SetModelState
and returns TelemetryFrames. Kinematic pose sink only: no Gazebo contact
dynamics, no legged climb, no real robot.
"""
import argparse
import json
import math
import signal
import socket
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))

from boundary.protocol import (FAULT_POLICY, MODE_ACTIVE, ProtocolError, TELEM_FAULT,
                               TELEM_OK, TelemetryFrame, decode)


def integrate(position, yaw, vx, vy, wz, dt):
    """Body-frame velocity -> world position/yaw (planar kinematic carrier)."""
    yaw = yaw + wz * dt
    cosine, sine = math.cos(yaw), math.sin(yaw)
    world_vx = cosine * vx - sine * vy
    world_vy = sine * vx + cosine * vy
    return (position[0] + world_vx * dt, position[1] + world_vy * dt, position[2]), yaw


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', required=True)
    parser.add_argument('--bind', default='127.0.0.1:5026')
    parser.add_argument('--telemetry-target', required=True)
    parser.add_argument('--initial', default='0,0,0')
    parser.add_argument('--control-hz', type=float, default=50.)
    parser.add_argument('--telemetry-hz', type=float, default=50.)
    parser.add_argument('--command-timeout-s', type=float, default=.25)
    parser.add_argument('--center-z', type=float, default=.1)
    parser.add_argument('--seconds', type=float, default=0.)
    parser.add_argument('--summary-out')
    args = parser.parse_args()

    import rospy
    from gazebo_msgs.msg import ModelState
    from gazebo_msgs.srv import SetModelState

    from boundary.udp_link import TelemetrySender

    position = tuple(float(v) for v in args.initial.split(','))
    yaw = 0.
    bind_host, bind_port = args.bind.rsplit(':', 1)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((bind_host, int(bind_port)))
    sock.setblocking(False)
    target_host, target_port = args.telemetry_target.rsplit(':', 1)
    sender = TelemetrySender((target_host, int(target_port)))
    rospy.init_node('v8_gazebo_boundary_endpoint')
    rospy.wait_for_service('/gazebo/set_model_state', timeout=20.)
    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    velocity = (0., 0., 0.)
    command_at = -1e9
    fault = 0
    seq = ticks = received = sent = 0
    stop = {'flag': False}

    def on_signal(_signum, _frame):
        stop['flag'] = True

    signal.signal(signal.SIGTERM, on_signal)
    signal.signal(signal.SIGINT, on_signal)
    period = 1. / args.control_hz
    telemetry_hz = min(args.telemetry_hz, args.control_hz)
    telemetry_every = max(1, int(round(args.control_hz / telemetry_hz)))
    started = time.monotonic()
    next_tick = started
    last = started
    try:
        while not stop['flag'] and not rospy.is_shutdown():
            if args.seconds > 0 and time.monotonic() - started >= args.seconds:
                break
            while True:
                try:
                    payload, _ = sock.recvfrom(2048)
                except BlockingIOError:
                    break
                try:
                    frame = decode(payload)
                except ProtocolError:
                    continue
                if frame.mode == MODE_ACTIVE:
                    velocity = (frame.vx, frame.vy, frame.wz)
                    command_at = time.monotonic()
                else:
                    velocity = (0., 0., 0.)
                    command_at = -1e9
                received += 1
            now = time.monotonic()
            dt = max(0., now - last)
            last = now
            if now - command_at > args.command_timeout_s:
                velocity = (0., 0., 0.)
            position, yaw = integrate(position, yaw, velocity[0], velocity[1],
                                      velocity[2], min(dt, period))
            state = ModelState()
            state.model_name, state.reference_frame = args.model, 'world'
            state.pose.position.x, state.pose.position.y = position[0], position[1]
            state.pose.position.z = position[2] + args.center_z
            state.pose.orientation.w = math.cos(yaw / 2.)
            state.pose.orientation.z = math.sin(yaw / 2.)
            try:
                set_state(state)
            except rospy.ServiceException:
                fault |= FAULT_POLICY
            ticks += 1
            if ticks % telemetry_every == 0:
                frame = TelemetryFrame(
                    seq, time.monotonic_ns(), position[0], position[1], position[2],
                    velocity[0], velocity[1], velocity[2],
                    math.cos(yaw / 2.), 0., 0., math.sin(yaw / 2.),
                    0., 0., velocity[2], 0., 0., 0.,
                    TELEM_FAULT if fault else TELEM_OK, fault,
                    tuple([0.] * 12), tuple([0.] * 12))
                try:
                    sender.send(frame)
                    sent += 1
                except ProtocolError:
                    fault |= FAULT_POLICY
                seq = (seq + 1) & 0xFFFFFFFF
            next_tick += period
            delay = next_tick - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_tick = time.monotonic()
    finally:
        sender.close()
        sock.close()
        summary = {'ticks': ticks, 'commands_received': received, 'telemetry_sent': sent,
                   'fault': fault, 'wall_s': round(time.monotonic() - started, 3),
                   'control_hz': args.control_hz, 'telemetry_hz': telemetry_hz,
                   'final_position': [round(v, 4) for v in position],
                   'final_yaw': round(yaw, 4)}
        if args.summary_out:
            Path(args.summary_out).write_text(json.dumps(summary, indent=1) + '\n',
                                              encoding='utf-8')
        print(json.dumps(summary))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
