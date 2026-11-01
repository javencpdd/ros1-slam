#!/usr/bin/env python3
"""3D Assembly scenario matrix: flat fixture + NIST stairs, repeated runs.

This is a 3D global/local/command/carrier matrix over declared support maps.
It is not the formal A1 2D planner matrix, not Gazebo physics, and not a robot.
"""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
SMOKE = HERE.parent / 'framework' / 'graph_3d_assembly_smoke.py'
FLAT = str(ROOT / 'contracts' / 'fixtures' / 'map_ml_two_surfaces.json')
NIST = '/opt/ros1-slam/data/maps/nist_stairs_120_s020x170_support.json'

SCENARIOS = {
    'flat': {'graph': 'graph_3d_real.yaml', 'map': FLAT,
             'start': '3.0,2.0,0.6', 'goal': '3.5,2.0,0.6'},
    'flat_alt': {'graph': 'graph_3d_real.yaml', 'map': FLAT,
                 'start': '3.0,2.0,0.6', 'goal': '2.2,2.0,0.6'},
    'stairs_top': {'graph': 'graph_3d_real_stairs.yaml', 'map': NIST,
                   'start': '0,-2.175,0', 'goal': '0,1.34,0.890358'},
    'stairs_mid': {'graph': 'graph_3d_real_stairs.yaml', 'map': NIST,
                   'start': '0,-2.175,0', 'goal': '0,-0.1,0.544578'},
}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', required=True)
    parser.add_argument('--port-base', type=int, default=11600)
    parser.add_argument('--repeats', type=int, default=2)
    parser.add_argument('--repeat-indices', default='',
                        help='comma-separated repeat indices; default all 0..repeats-1')
    parser.add_argument('--scenarios', default='flat,flat_alt,stairs_top,stairs_mid')
    parser.add_argument('--timeout-s', type=float, default=60.)
    args = parser.parse_args()
    output = Path(args.out).resolve()
    output.mkdir(parents=True, exist_ok=True)
    names = [name.strip() for name in args.scenarios.split(',') if name.strip()]
    unknown = [name for name in names if name not in SCENARIOS]
    if unknown:
        parser.error('unknown scenarios: %s' % ','.join(unknown))
    repeat_indices = ([int(value) for value in args.repeat_indices.split(',') if value.strip()]
                      if args.repeat_indices else list(range(args.repeats)))
    if any(index < 0 or index >= args.repeats for index in repeat_indices):
        parser.error('--repeat-indices must be within 0..%d' % (args.repeats - 1))
    jobs = []
    index = 0
    for repeat in repeat_indices:
        for name in names:
            index += 1
            scenario = SCENARIOS[name]
            jobs.append({'cell': '%s_r%d' % (name, repeat), 'scenario': name,
                         'repeat': repeat, 'port': args.port_base + index,
                         'state': str(output / ('%s_state.json' % name)),
                         'map': scenario['map'],
                         'graph': str(HERE.parent / 'framework' / 'fixtures'
                                      / 'graphs' / scenario['graph']),
                         'start': scenario['start'], 'goal': scenario['goal'],
                         'out': str(output / ('%s_r%d.json' % (name, repeat)))})
    (output / 'preregistered.json').write_text(
        json.dumps({'jobs': jobs, 'repeats': args.repeats,
                    'repeat_indices': repeat_indices, 'scenarios': names},
                   ensure_ascii=False, indent=1) + '\n')
    rows = []
    started = time.monotonic()
    for job in jobs:
        command = [sys.executable, '-B', str(SMOKE),
                   '--out', job['out'], '--port', str(job['port']),
                   '--state', job['state'], '--map', job['map'],
                   '--graph', job['graph'], '--start', job['start'],
                   '--goal', job['goal'], '--timeout-s', str(args.timeout_s)]
        env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1')
        try:
            completed = subprocess.run(command, capture_output=True, text=True,
                                       timeout=args.timeout_s + 30, env=env)
            returncode, stdout, stderr = completed.returncode, completed.stdout, completed.stderr
        except subprocess.TimeoutExpired as error:
            returncode, stdout, stderr = 'timeout', '', repr(error)
        payload = {}
        if Path(job['out']).exists():
            try:
                payload = json.loads(Path(job['out']).read_text())
            except ValueError:
                payload = {}
        rows.append({'cell': job['cell'], 'scenario': job['scenario'],
                     'repeat': job['repeat'], 'port': job['port'],
                     'returncode': returncode,
                     'ok': bool(payload.get('ok')),
                     'provider_status': payload.get('provider_status'),
                     'path_poses': (payload.get('path') or {}).get('poses'),
                     'local_reason': (payload.get('local_trajectory') or {}).get('reason'),
                     'carrier_distance_m': payload.get('carrier_distance_to_goal_m'),
                     'mux_track_samples': (payload.get('mux') or {}).get('track_samples'),
                     'mux_stale_samples': (payload.get('mux') or {}).get('stale_samples'),
                     'error': payload.get('error') or stderr[-800:]})
    results = {'matrix': '3d_assembly_scenarios', 'jobs': rows,
               'wall_s': time.monotonic() - started,
               'note': 'kinematic 3D carrier + declared support maps; not Gazebo/formal A1'}
    (output / 'results.json').write_text(json.dumps(results, ensure_ascii=False, indent=1) + '\n')
    passed = sum(1 for row in rows if row['ok'])
    worst = max([row['carrier_distance_m'] for row in rows
                 if row['carrier_distance_m'] is not None] or [None]) \
        if any(row['carrier_distance_m'] is not None for row in rows) else None
    summary = {'n': len(rows), 'pass': passed,
               'worst_carrier_distance_m': worst,
               'failures': [row['cell'] for row in rows if not row['ok']],
               'wall_s': results['wall_s']}
    (output / 'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=1) + '\n')
    print(json.dumps(summary, ensure_ascii=False))
    return 0 if passed == len(rows) else 6


if __name__ == '__main__':
    raise SystemExit(main())
