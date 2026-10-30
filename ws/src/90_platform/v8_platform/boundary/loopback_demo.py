#!/usr/bin/env python3
"""Loopback demo: 20 Hz planner -> 500 Hz boundary stream -> receiver health.

Runs only on 127.0.0.1; no ROS, no robot address. Prints one JSON line with
rate accuracy, loopback loss, inter-arrival p50/p99, transport latency and the
measured deadman stop delay.
"""
import argparse
import json
import threading
import time

from .protocol import MODE_ACTIVE, MODE_HOLD
from .rate_bridge import CommandBridge, Limits
from .udp_link import CommandReceiver, CommandSender


def run(send_hz=500., planner_hz=20., duration_s=3.0, stop_at=1.5, stop_for=0.6,
        timeout_s=0.25):
    receiver = CommandReceiver(('127.0.0.1', 0), period_s=1. / send_hz)
    bridge = CommandBridge(Limits(command_timeout_s=timeout_s))
    sender = CommandSender(receiver.address)
    period, planner_period = 1. / send_hz, 1. / planner_hz
    stop = threading.Event()
    marks = {'stop': None, 'resume': None}

    def pump():
        next_planner = time.monotonic()
        next_tick = time.monotonic()
        while not stop.is_set():
            now = time.monotonic()
            elapsed = now - start
            inactive = stop_at <= elapsed < stop_at + stop_for
            if now >= next_planner:
                if inactive:
                    if marks['stop'] is None:
                        marks['stop'] = now
                else:
                    bridge.update(.5, 0., .2, now)
                    if elapsed >= stop_at + stop_for and marks['resume'] is None:
                        marks['resume'] = now
                next_planner = max(next_planner + planner_period, now + planner_period * .5)
            vx, vy, wz = bridge.command(now)
            sender.send(vx, vy, wz, MODE_ACTIVE if bridge.mode == 'active' else MODE_HOLD)
            next_tick += period
            delay = next_tick - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_tick = time.monotonic()

    worker = threading.Thread(target=pump, daemon=True)
    start = time.monotonic()
    worker.start()
    latencies, deadman = [], None
    try:
        while time.monotonic() - start < duration_s:
            frame = receiver.poll()
            if frame is None:
                continue
            arrival = time.monotonic()
            latencies.append(max(0., arrival - frame.t_ns / 1e9))
            if deadman is None and marks['stop'] is not None and frame.mode == MODE_HOLD:
                deadman = arrival - marks['stop']
    finally:
        stop.set()
        worker.join(timeout=1.)
        elapsed = time.monotonic() - start
        ordered = sorted(latencies)
        result = {
            'send_hz': send_hz, 'planner_hz': planner_hz, 'duration_s': round(elapsed, 3),
            'sent': sender.sent, 'received': receiver.stats['frames'],
            'loss': receiver.stats['loss'], 'dup': receiver.stats['dup'],
            'bad': receiver.stats['bad'],
            'tx_rate_hz': round(sender.sent / elapsed, 1),
            'rx_rate_hz': round(receiver.stats['frames'] / elapsed, 1),
            'interarrival_p50_ms': round(receiver.jitter()['p50'] * 1e3, 3),
            'interarrival_p99_ms': round(receiver.jitter()['p99'] * 1e3, 3),
            'period_ms': round(period * 1e3, 3),
            'transport_p50_ms': round((ordered[len(ordered) // 2] if ordered else 0.) * 1e3, 3),
            'transport_p99_ms': round((ordered[min(len(ordered) - 1, int(len(ordered) * .99))]
                                       if ordered else 0.) * 1e3, 3),
            'deadman_stop_ms': None if deadman is None else round(deadman * 1e3, 1),
            'timeout_ms': round(timeout_s * 1e3, 1),
            'bridge': bridge.stats, 'bridge_mode': bridge.mode,
        }
        sender.close()
        receiver.close()
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--send-hz', type=float, default=500.)
    parser.add_argument('--planner-hz', type=float, default=20.)
    parser.add_argument('--duration-s', type=float, default=3.0)
    parser.add_argument('--stop-at', type=float, default=1.5)
    parser.add_argument('--stop-for', type=float, default=.6)
    parser.add_argument('--timeout-s', type=float, default=.25)
    args = parser.parse_args()
    print(json.dumps(run(args.send_hz, args.planner_hz, args.duration_s, args.stop_at,
                         args.stop_for, args.timeout_s), ensure_ascii=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
