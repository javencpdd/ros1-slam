#!/usr/bin/env python3
"""Replay a recorded V8-X1 bridge command trace at fixed ticks and compare two runs.

This removes ROS command timing from the determinism question: both runs start
from the same recorded qpos/qvel and receive the same recorded per-tick command
and stepping flag.  Any remaining difference is inside the executor (observation
history, ONNX policy, MuJoCo stepping).
"""
import argparse
import gzip
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

FIELDS = ('sim_time', 'qpos', 'qvel', 'command', 'joint_target', 'action')


def same_special(a, b):
    a, b = float(a), float(b)
    if math.isnan(a) and math.isnan(b):
        return True
    return math.isinf(a) and math.isinf(b) and (a > 0) == (b > 0)


def load_rows(path):
    opener = gzip.open if path.endswith('.gz') else open
    with opener(path, 'rt', encoding='utf-8') as stream:
        return [json.loads(line) for line in stream if line.strip()]


def compare_rows(first, second):
    ticks = min(len(first), len(second))
    first_diff = None
    max_abs_diff = 0.0
    for index in range(ticks):
        for field in FIELDS:
            a, b = first[index].get(field), second[index].get(field)
            if a is None or b is None:
                if a != b and first_diff is None:
                    first_diff = {'tick': index, 'field': field, 'a': a, 'b': b}
                continue
            if isinstance(a, list) and isinstance(b, list):
                if len(a) != len(b):
                    if first_diff is None:
                        first_diff = {'tick': index, 'field': field, 'a': a, 'b': b}
                    continue
                for x, y in zip(a, b):
                    if same_special(x, y):
                        continue
                    diff = abs(float(x) - float(y))
                    max_abs_diff = max(max_abs_diff, diff)
                    if diff != 0.0 and first_diff is None:
                        first_diff = {'tick': index, 'field': field, 'a': a, 'b': b,
                                      'max_abs_diff': diff}
            else:
                if same_special(a, b):
                    continue
                diff = abs(float(a) - float(b))
                max_abs_diff = max(max_abs_diff, diff)
                if diff != 0.0 and first_diff is None:
                    first_diff = {'tick': index, 'field': field, 'a': a, 'b': b}
        if first_diff is not None:
            break
    return {'ticks': ticks, 'identical': first_diff is None and len(first) == len(second),
            'first_diff': first_diff, 'max_abs_diff': max_abs_diff}


def stop(process):
    if process is not None and process.poll() is None:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5)


def run_bridge(bridge, args, prefix, trace_out, replay_trace, port):
    env = dict(os.environ, ROS_MASTER_URI='http://127.0.0.1:%d' % port,
               ROS_IP='127.0.0.1', ROS_HOSTNAME='127.0.0.1',
               ROS_HOME='/tmp/a5_replay_ros', ROS_LOG_DIR='/tmp/a5_replay_logs',
               PYTHONPATH='/tmp/r854-policy-runtime:/opt/ros/noetic/lib/python3/dist-packages')
    command = [sys.executable, '-B', bridge, '--map', args.map, '--xml', args.xml,
               '--policy', args.policy, '--prefix', prefix,
               '--replay-trace', replay_trace, '--trace', trace_out,
               '--seed', '0', '--perturb']
    with open(trace_out + '.log', 'wb') as log:
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True, env=env)
        try:
            return process.wait(timeout=args.timeout_s)
        except subprocess.TimeoutExpired:
            stop(process)
            return 'timeout'


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--bridge', default='/repo/ws/src/95_simulation/lite3_gazebo_sim/dynamics/a5_v8x1_bridge.py')
    parser.add_argument('--map', default='/repo/data/maps/nist_stairs_120_s020x170_support.json')
    parser.add_argument('--xml', default='/tmp/mjcf/Lite3.xml')
    parser.add_argument('--policy', default='/tmp/new_policy.onnx')
    parser.add_argument('--source-trace', required=True)
    parser.add_argument('--out-dir', required=True)
    parser.add_argument('--port', type=int, default=11690)
    parser.add_argument('--timeout-s', type=float, default=120.0)
    args = parser.parse_args(argv)
    out = Path(args.out_dir)
    out.mkdir(parents=True, exist_ok=True)
    Path('/tmp/a5_replay_ros').mkdir(exist_ok=True)
    Path('/tmp/a5_replay_logs').mkdir(exist_ok=True)
    source = load_rows(args.source_trace)
    master = subprocess.Popen(['roscore', '-p', str(args.port)], start_new_session=True,
                              stdout=open('/tmp/a5_replay_master.log', 'ab'),
                              stderr=subprocess.STDOUT,
                              env=dict(os.environ, ROS_IP='127.0.0.1',
                                       ROS_HOSTNAME='127.0.0.1',
                                       ROS_MASTER_URI='http://127.0.0.1:%d' % args.port))
    try:
        time.sleep(5)
        runs = []
        for index in (1, 2):
            trace_out = str(out / ('replay_%d.jsonl' % index))
            returncode = run_bridge(args.bridge, args, '/a5replay%d' % index,
                                    trace_out, args.source_trace, args.port)
            runs.append({'replay': index, 'returncode': returncode,
                         'trace': trace_out, 'rows': len(load_rows(trace_out))
                         if Path(trace_out).exists() else 0})
        first = load_rows(runs[0]['trace'])
        second = load_rows(runs[1]['trace'])
        comparison = compare_rows(first, second)
        result = {'ok': all(run['returncode'] == 0 for run in runs)
                  and comparison['identical'],
                  'source_trace': args.source_trace, 'source_rows': len(source),
                  'runs': runs, 'comparison': comparison,
                  'final_qpos': [first[-1]['qpos'][:3] if first else None,
                                 second[-1]['qpos'][:3] if second else None],
                  'scope': 'executor replay only; ROS command generation bypassed'}
        print(json.dumps(result, ensure_ascii=False, indent=1))
        (out / 'summary.json').write_text(
            json.dumps(result, ensure_ascii=False, indent=1) + '\n')
        return 0 if result['ok'] else 6
    finally:
        stop(master)


if __name__ == '__main__':
    raise SystemExit(main())
