#!/usr/bin/env python3
"""MuJoCo robot endpoint for the v8 boundary UDP contract (S2).

Receives 33-byte CommandFrames at up to 500 Hz, holds the latest intent, runs
the 270-state policy at the control rate, and returns TelemetryFrames. Pure
simulation: no ROS, no vendor stack, no real robot.
"""
import argparse
import json
import math
import signal
import socket
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
BOUNDARY = HERE.parents[2] / '90_platform' / 'v8_platform'
sys.path.insert(0, str(BOUNDARY))

import mj_dreamwaq_walk as walk  # noqa: E402
from boundary.protocol import (FAULT_NONFINITE, FAULT_POLICY,  # noqa: E402
                               ProtocolError, TELEM_FAULT, TELEM_OK, TelemetryFrame,
                               decode)


def policy_command(vx, vy, wz, max_speed=.25, max_wz=.5):
    """Body-frame velocity -> forward speed + yaw rate for this policy.

    The 270 policy's demonstrated envelope is slower than the adapter protocol
    limit; the endpoint clamps to the carrier envelope instead of pretending
    the policy can track the full command range.
    """
    values = (float(vx), float(vy), float(wz))
    if not all(math.isfinite(v) for v in values):
        raise ValueError('non-finite command')
    speed = math.hypot(values[0], values[1])
    heading = 0. if speed <= 1e-9 else math.atan2(values[1], values[0])
    return (min(speed, max_speed),
            float(max(-max_wz, min(max_wz, values[2] + 3.0 * heading))))


def body_imu(qpos, qvel, qacc):
    rotation = walk.rot(qpos[3:7])
    velocity = rotation.T @ np.asarray(qvel[:3])
    gyro = rotation.T @ np.asarray(qvel[3:6])
    specific = rotation.T @ (np.asarray(qacc[:3]) + np.array([0., 0., 9.81]))
    return [float(v) for v in velocity], [float(v) for v in gyro], [float(v) for v in specific]


def telemetry_from_state(seq, t_ns, qpos, qvel, qacc, fault=0):
    velocity, gyro, accel = body_imu(qpos, qvel, qacc)
    joints = [float(v) for v in qpos[7:19]]
    joint_vel = [float(v) for v in qvel[6:18]]
    return TelemetryFrame(
        seq, t_ns, float(qpos[0]), float(qpos[1]), float(qpos[2]),
        velocity[0], velocity[1], velocity[2],
        float(qpos[3]), float(qpos[4]), float(qpos[5]), float(qpos[6]),
        gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2],
        TELEM_FAULT if fault else TELEM_OK, fault,
        tuple(joints), tuple(joint_vel))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--xml', required=True)
    parser.add_argument('--policy', required=True)
    parser.add_argument('--bind', default='127.0.0.1:5006')
    parser.add_argument('--target', required=True)
    parser.add_argument('--control-hz', type=float, default=50.)
    parser.add_argument('--telemetry-hz', type=float, default=50.)
    parser.add_argument('--max-speed', type=float, default=.25)
    parser.add_argument('--max-wz', type=float, default=.5)
    parser.add_argument('--command-timeout-s', type=float, default=.25)
    parser.add_argument('--initial-x', type=float, default=0.)
    parser.add_argument('--initial-y', type=float, default=0.)
    parser.add_argument('--initial-z', type=float, default=.40)
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--perturb', action='store_true')
    parser.add_argument('--viewer', action='store_true')
    parser.add_argument('--seconds', type=float, default=0.)
    parser.add_argument('--summary-out')
    args = parser.parse_args()

    import mujoco
    import onnxruntime as ort

    from boundary.udp_link import TelemetrySender

    bind_host, bind_port = args.bind.rsplit(':', 1)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((bind_host, int(bind_port)))
    sock.setblocking(False)
    target_host, target_port = args.target.rsplit(':', 1)
    sender = TelemetrySender((target_host, int(target_port)))
    model = mujoco.MjModel.from_xml_path(args.xml)
    data = mujoco.MjData(model)
    perturb = walk.sample_initial(args.seed, args.perturb)
    data.qpos[:3] = [args.initial_x + perturb['dx'], args.initial_y + perturb['dy'],
                     args.initial_z + perturb['dz']]
    data.qpos[3:7] = [math.cos(perturb['dyaw'] / 2.), 0., 0., math.sin(perturb['dyaw'] / 2.)]
    data.qpos[7:19] = walk.DEFAULT_POSE
    mujoco.mj_forward(model, data)
    viewer = None
    if args.viewer:
        import mujoco.viewer
        viewer = mujoco.viewer.launch_passive(model, data)
        viewer.cam.lookat[:] = [1.5, 0., .5]
        viewer.cam.distance = 5.0
        viewer.cam.azimuth = 90.
        viewer.cam.elevation = -16.
    session = ort.InferenceSession(args.policy, providers=['CPUExecutionProvider'])
    inp = session.get_inputs()[0].name
    history = [np.zeros(walk.OBS_DIM, dtype=np.float32) for _ in range(walk.HISTORY)]
    last_action = np.zeros(12, dtype=np.float32)
    joint_target = walk.DEFAULT_POSE.copy()
    dt = model.opt.timestep
    stride = max(1, int(round(1. / args.control_hz / dt)))
    telemetry_hz = min(args.telemetry_hz, args.control_hz)
    telemetry_every = max(1, int(round(args.control_hz / telemetry_hz)))
    command = (0., 0., 0.)
    command_at = -1e9
    fault = 0
    fault_tick = fault_sim = None
    nonfinite_actions = 0
    seq = 0
    ticks = received = sent = 0
    stop = {'flag': False}

    def on_signal(_signum, _frame):
        stop['flag'] = True

    signal.signal(signal.SIGTERM, on_signal)
    signal.signal(signal.SIGINT, on_signal)
    started = time.monotonic()
    next_tick = started
    try:
        while not stop['flag']:
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
                if frame.mode == 1:
                    command = (frame.vx, frame.vy, frame.wz)
                    command_at = time.monotonic()
                else:
                    command = (0., 0., 0.)
                    command_at = -1e9
                received += 1
            speed, yaw_rate = policy_command(*command, max_speed=args.max_speed,
                                              max_wz=args.max_wz)
            if time.monotonic() - command_at > args.command_timeout_s:
                speed, yaw_rate = 0., 0.
            obs = walk.observation(data.qpos, data.qvel,
                                   np.array([speed, 0., yaw_rate], dtype=np.float32),
                                   last_action, walk.DEFAULT_POSE)
            state = walk.state_vector(obs, history, newest_first=True)
            action = session.run(None, {inp: state.reshape(1, -1)})[0].reshape(-1)
            if not np.all(np.isfinite(action)):
                fault |= FAULT_POLICY
                nonfinite_actions += 1
                if fault_tick is None:
                    fault_tick, fault_sim = ticks, float(data.time)
                last_action = np.zeros(12, dtype=np.float32)
                joint_target = walk.DEFAULT_POSE.copy()
            else:
                joint_target = walk.joint_target(action, walk.DEFAULT_POSE)
                last_action = action.astype(np.float32)
                history = history[1:] + [obs]
            for _ in range(stride):
                tau = np.clip(25. * (joint_target - data.qpos[7:19])
                              - .5 * data.qvel[6:18], -30., 30.)
                data.qfrc_applied[6:18] = tau
                mujoco.mj_step(model, data)
            ticks += 1
            if ticks % telemetry_every == 0:
                frame = telemetry_from_state(seq, time.monotonic_ns(), data.qpos,
                                             data.qvel, data.qacc, fault)
                try:
                    sender.send(frame)
                    sent += 1
                except ProtocolError:
                    fault |= FAULT_NONFINITE
                seq = (seq + 1) & 0xFFFFFFFF
            if viewer is not None and not viewer.is_running():
                break
            if viewer is not None and ticks % 5 == 0:
                viewer.sync()
            next_tick += 1. / args.control_hz
            delay = next_tick - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_tick = time.monotonic()
    finally:
        if viewer is not None:
            viewer.close()
        sender.close()
        sock.close()
        summary = {'ticks': ticks, 'commands_received': received, 'telemetry_sent': sent,
                   'fault': fault, 'wall_s': round(time.monotonic() - started, 3),
                   'control_hz': args.control_hz, 'telemetry_hz': telemetry_hz,
                   'max_speed': args.max_speed, 'max_wz': args.max_wz,
                   'fault_tick': fault_tick, 'fault_sim_time': fault_sim,
                   'nonfinite_actions': nonfinite_actions}
        if args.summary_out:
            Path(args.summary_out).write_text(json.dumps(summary, indent=1) + '\n',
                                              encoding='utf-8')
        print(json.dumps(summary))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
