#!/usr/bin/env python3
"""Extend a recorded V8-X1 climb trace with a hold and measure it.

The bridge restores the source end state, MuJoCo warmstart, last action and
five-frame observation history, then runs only the appended rows.  The source
run's world XML must be supplied; using the flat model instead makes the body
leave the stairs and invalidates the hold measurement.  No new climb or
learning is claimed.
"""
import argparse
import gzip
import json
import math
from pathlib import Path
import subprocess
import sys


def load_rows(path):
    opener = gzip.open if path.endswith('.gz') else open
    with opener(path, 'rt', encoding='utf-8') as stream:
        return [json.loads(line) for line in stream if line.strip()]


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--source-trace', required=True)
    parser.add_argument('--xml', default='/tmp/mjcf/Lite3.xml',
                        help='same world XML used for the source run (stairs, not flat)')
    parser.add_argument('--hold-ticks', type=int, default=500)
    parser.add_argument('--hold-vx', type=float, default=0.)
    parser.add_argument('--hold-wz', type=float, default=0.)
    parser.add_argument('--ramp-ticks', type=int, default=0)
    parser.add_argument('--hold-z-drop-m', type=float, default=.25)
    parser.add_argument('--out-dir', required=True)
    parser.add_argument('--replay', default=str(Path(__file__).resolve().parent / 'a5_v8x1_replay.py'))
    args = parser.parse_args(argv)
    out = Path(args.out_dir)
    out.mkdir(parents=True, exist_ok=True)
    rows = load_rows(args.source_trace)
    last = rows[-1]
    extended = list(rows)
    command0 = [float(v) for v in last.get('command', [0., 0., 0.])]
    velocity0 = [float(v) for v in last.get('map_velocity', [0., 0., 0.])]
    for index in range(args.ramp_ticks):
        scale = 1. - (index + 1) / float(max(1, args.ramp_ticks))
        row = dict(last)
        row['tick'] = int(last['tick']) + index + 1
        row['sim_time'] = float(last['sim_time']) + .02 * (index + 1)
        row['wall'] = float(last.get('wall', 0.)) + .02 * (index + 1)
        row['command'] = [scale * value for value in command0]
        row['map_velocity'] = [scale * value for value in velocity0]
        row['target'] = None
        row['finish'] = False
        row['fault'] = None
        row['stepping'] = True
        row['active'] = True
        extended.append(row)
    base_tick = int(last['tick']) + args.ramp_ticks
    base_sim = float(last['sim_time']) + .02 * args.ramp_ticks
    for index in range(args.hold_ticks):
        row = dict(last)
        row['tick'] = base_tick + index + 1
        row['sim_time'] = base_sim + .02 * (index + 1)
        row['wall'] = float(last.get('wall', 0.)) + .02 * (args.ramp_ticks + index + 1)
        row['command'] = [args.hold_vx, 0., args.hold_wz]
        row['map_velocity'] = [args.hold_vx, 0., args.hold_wz]
        row['target'] = None
        row['finish'] = False
        row['fault'] = None
        row['stepping'] = True
        row['active'] = True
        extended.append(row)
    extended_path = out / 'top_hold.trace.jsonl'
    with extended_path.open('w', encoding='utf-8') as stream:
        for row in extended:
            stream.write(json.dumps(row) + '\n')
    replay_dir = out / 'replay'
    command = [sys.executable, '-B', args.replay, '--source-trace', str(extended_path),
               '--xml', args.xml,
               '--start-index', str(len(rows)), '--skip-source-compare',
               '--out-dir', str(replay_dir), '--port', '11698']
    replay = subprocess.run(command, capture_output=True, text=True, timeout=300)
    comparison = None
    summary_path = replay_dir / 'summary.json'
    if summary_path.exists():
        comparison = json.loads(summary_path.read_text())
    replay_rows = load_rows(str(replay_dir / 'replay_1.jsonl')) if (replay_dir / 'replay_1.jsonl').exists() else []
    hold = replay_rows
    support = float(last.get('support') or 0.)
    z0 = float(rows[-1]['qpos'][2])
    xy0 = rows[-1]['qpos'][:2]
    zs = [float(row['qpos'][2]) for row in hold]
    drift = [math.hypot(row['qpos'][0] - xy0[0], row['qpos'][1] - xy0[1]) for row in hold]
    result = {
        'ok': bool(comparison and comparison.get('ok') and hold
                   and min(zs) >= z0 - args.hold_z_drop_m
                   and float(hold[-1]['qpos'][2]) >= support + .10),
        'source_rows': len(rows), 'hold_rows': len(hold),
        'resume_start_index': len(rows),
        'source_end': {'qpos': rows[-1]['qpos'], 'support': support, 'sim_time': rows[-1]['sim_time']},
        'hold_end_qpos': hold[-1]['qpos'] if hold else None,
        'hold_min_z': min(zs) if zs else None,
        'hold_final_z': float(hold[-1]['qpos'][2]) if hold else None,
        'hold_max_xy_drift_m': max(drift) if drift else None,
        'hold_z_drop_m': z0 - min(zs) if zs else None,
        'z_drop_limit_m': args.hold_z_drop_m,
        'support_z': support,
        'replay_ok': bool(comparison and comparison.get('ok')),
        'replay': comparison,
        'scope': 'open-loop recorded climb + appended zero command; no ROS re-planning, no real robot',
        'replay_rc': replay.returncode,
        'replay_stdout': replay.stdout[-2000:], 'replay_stderr': replay.stderr[-2000:],
    }
    print(json.dumps(result, ensure_ascii=False, indent=1))
    (out / 'summary.json').write_text(json.dumps(result, ensure_ascii=False, indent=1) + '\n')
    return 0 if result['ok'] else 6


if __name__ == '__main__':
    raise SystemExit(main())
