#!/usr/bin/env python3
"""S2 adapter-side loopback: send CommandFrames, receive TelemetryFrames.

Run the MuJoCo endpoint separately, then:
  python3 -m boundary.s2_loopback --target 127.0.0.1:5006 \
    --bind 127.0.0.1:5005 --seconds 5 --out /tmp/s2.json
"""
import argparse
import json
import math
import threading
import time

from .protocol import MODE_HOLD
from .rate_bridge import CommandBridge, Limits
from .recording import Recorder
from .udp_link import CommandSender, TelemetryReceiver


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--target', required=True)
    parser.add_argument('--bind', default='127.0.0.1:0')
    parser.add_argument('--seconds', type=float, default=5.)
    parser.add_argument('--rate-hz', type=float, default=500.)
    parser.add_argument('--planner-hz', type=float, default=20.)
    parser.add_argument('--command', default='0.3,0,0')
    parser.add_argument('--out')
    parser.add_argument('--record')
    args = parser.parse_args()
    command = tuple(float(v) for v in args.command.split(','))
    target_host, target_port = args.target.rsplit(':', 1)
    bind_host, bind_port = args.bind.rsplit(':', 1)
    sender = CommandSender((target_host, int(target_port)))
    receiver = TelemetryReceiver((bind_host, int(bind_port)))
    receiver.sock.setblocking(False)
    bridge = CommandBridge(Limits())
    recorder = Recorder(args.record) if args.record else None
    record_lock = threading.Lock()
    period = 1. / args.rate_hz
    planner_every = max(1, int(round(args.rate_hz / args.planner_hz)))
    started = time.monotonic()
    next_tick = started
    sent = planner_updates = 0
    first_telemetry = last_telemetry = None
    active_velocity = hold_velocity = None
    hold_started = [None]
    stop_rx = threading.Event()

    def receive_loop():
        nonlocal first_telemetry, last_telemetry, active_velocity, hold_velocity
        while not stop_rx.is_set():
            frame = receiver.poll()
            if frame is None:
                time.sleep(.0005)
                continue
            if recorder is not None:
                with record_lock:
                    recorder.write('telemetry', frame.encode())
            first_telemetry = first_telemetry or frame
            last_telemetry = frame
            if hold_started[0] is not None and time.monotonic() - hold_started[0] > .4:
                hold_velocity = [frame.vx, frame.vy, frame.vz]
            elif hold_started[0] is None and active_velocity is None:
                active_velocity = [frame.vx, frame.vy, frame.vz]

    rx_thread = threading.Thread(target=receive_loop, daemon=True)
    rx_thread.start()
    try:
        while time.monotonic() - started < args.seconds:
            now = time.monotonic()
            elapsed = now - started
            if elapsed < args.seconds * .6:
                if sent % planner_every == 0:
                    bridge.update(command[0], command[1], command[2], now)
                    planner_updates += 1
            elif hold_started[0] is None:
                hold_started[0] = now
            (vx, vy, wz) = bridge.command(now)
            mode = 1 if bridge.mode == 'active' else MODE_HOLD
            sent_frame = sender.send(vx, vy, wz, mode)
            if recorder is not None:
                with record_lock:
                    recorder.write('command', sent_frame.encode(), mode=mode,
                                   planner_age_s=bridge.age(now))
            sent += 1
            next_tick += period
            delay = next_tick - time.monotonic()
            if delay > 0:
                if delay > .0008:
                    time.sleep(delay - .0008)
                while time.monotonic() < next_tick:
                    pass
            else:
                next_tick = time.monotonic()
    finally:
        stop_rx.set()
        rx_thread.join(timeout=1.)
        if recorder is not None:
            recorder.close()
        sender.close()
        receiver.close()
    stats = {'seconds': round(time.monotonic() - started, 3), 'rate_hz': args.rate_hz,
             'commands_sent': sent, 'planner_updates': planner_updates,
             'telemetry_frames': receiver.stats['frames'],
             'telemetry_loss': receiver.stats['loss'], 'telemetry_bad': receiver.stats['bad'],
             'receiver_jitter': receiver.jitter(),
             'telemetry_age_s': None if receiver.age_s() is None else round(receiver.age_s(), 4),
             'first_telemetry': None if first_telemetry is None else {
                 'seq': first_telemetry.seq, 'pos': [first_telemetry.x, first_telemetry.y,
                                                     first_telemetry.z]},
             'last_telemetry': None if last_telemetry is None else {
                 'seq': last_telemetry.seq, 'pos': [last_telemetry.x, last_telemetry.y,
                                                    last_telemetry.z]},
             'active_velocity': active_velocity, 'hold_velocity': hold_velocity}
    print(json.dumps(stats, indent=1))
    if args.out:
        with open(args.out, 'w', encoding='utf-8') as stream:
            json.dump(stats, stream, indent=1)
            stream.write('\n')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
