#!/usr/bin/env python3
"""Pre-registered A1 full-combination matrix runner.

Cells: 4 global/local planner graphs x SC01/SC02/SC09 x seed 0/1/2 x truth/noisy
= 72.  Each cell gets its own ROS master and Gazebo master port; jobs are run by
a small worker pool and every result is written immediately.  The registration
file is written before any cell starts.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

HERE = Path(__file__).resolve().parent
GRAPH_DIR = HERE.parent / 'framework/fixtures/graphs'
COMBOS = ('gp_dwa', 'gp_teb', 'navfn_dwa', 'navfn_teb')


def stop(process):
    if process is not None and process.poll() is None:
        try:
            os.killpg(process.pid, signal.SIGTERM)
        except ProcessLookupError:
            return
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            process.wait(timeout=5)


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--out', required=True)
    parser.add_argument('--workers', type=int, default=6)
    parser.add_argument('--ros-port-base', type=int, default=12000)
    parser.add_argument('--gazebo-port-base', type=int, default=12100)
    parser.add_argument('--timeout-s', type=float, default=180.)
    parser.add_argument('--retries', type=int, default=1)
    parser.add_argument('--resume', action='store_true')
    parser.add_argument('--scenarios', default='SC01,SC02,SC09')
    parser.add_argument('--seeds', default='0,1,2')
    parser.add_argument('--tracks', default='truth,noisy')
    args = parser.parse_args(argv)
    out = Path(args.out).resolve()
    out.mkdir(parents=True, exist_ok=True)
    scenarios = args.scenarios.split(',')
    seeds = [int(v) for v in args.seeds.split(',')]
    tracks = args.tracks.split(',')
    jobs = []
    index = 0
    for combo in COMBOS:
        graph = GRAPH_DIR / ('graph_2d_stack_%s.yaml' % combo)
        if not graph.exists():
            raise SystemExit('missing graph ' + str(graph))
        for scenario in scenarios:
            for seed in seeds:
                for track in tracks:
                    jobs.append({'index': index, 'combo': combo, 'graph': str(graph),
                                 'scenario': scenario, 'seed': seed, 'track': track,
                                 'ros_port': args.ros_port_base + index,
                                 'gazebo_port': args.gazebo_port_base + index,
                                 'cell': '%s_%s_s%d_%s' % (combo, scenario, seed, track)})
                    index += 1
    registration = {
        'run_class': 'runtime_experimental',
        'experiment': 'A1 full-combination matrix',
        'jobs': jobs, 'planned': len(jobs),
        'criteria': {'goal_position_m': .30, 'min_clearance_m': .20,
                     'wall_deadline_s': args.timeout_s},
        'graphs': {str(GRAPH_DIR / ('graph_2d_stack_%s.yaml' % combo)):
                   hashlib.sha256((GRAPH_DIR / ('graph_2d_stack_%s.yaml' % combo)).read_bytes()).hexdigest()
                   for combo in COMBOS},
        'git_head': os.environ.get('A1_GIT_HEAD', ''),
        'workers': args.workers, 'retries': args.retries,
    }
    (out / 'preregistered.json').write_text(json.dumps(registration, ensure_ascii=False, indent=1) + '\n')
    lock = threading.RLock()
    completed = {}

    def persist():
        with lock:
            rows = [completed[key] for key in sorted(completed)]
        summary = {
            'planned': len(jobs), 'completed': len(rows),
            'passed': sum(row['verdict'] == 'PASS' for row in rows),
            'failed': sum(row['verdict'] != 'PASS' for row in rows),
            'cells': rows,
        }
        (out / 'results.json.tmp').write_text(json.dumps(summary, ensure_ascii=False, indent=1) + '\n')
        (out / 'results.json.tmp').replace(out / 'results.json')

    def run_cell(job):
        existing = out / job['cell'] / 'driver.json'
        if args.resume and existing.exists():
            driver = json.loads(existing.read_text())
            goal = driver.get('dist_to_goal_m')
            clearance = driver.get('obstacle_min_clearance_m')
            verdict = ('PASS' if (goal is not None and clearance is not None
                                  and goal <= registration['criteria']['goal_position_m']
                                  and clearance >= registration['criteria']['min_clearance_m'])
                       else 'FAIL')
            row = dict(job, verdict=verdict,
                       attempts=[{'resumed': True,
                                  'driver': {key: driver.get(key) for key in
                                             ('dist_to_goal_m', 'path_length_m',
                                              'obstacle_min_clearance_m',
                                              'injected_box_clearance_m', 'track',
                                              'seed', 'scenario_id')}}],
                       elapsed_s=0.0)
            with lock:
                completed[job['index']] = row
                persist()
            return row
        attempts = []
        for attempt in range(args.retries + 1):
            directory = out / (job['cell'] + ('' if attempt == 0 else '_retry%d' % attempt))
            env = dict(os.environ,
                       ROS_MASTER_URI='http://127.0.0.1:%d' % job['ros_port'],
                       GAZEBO_MASTER_URI='http://127.0.0.1:%d' % job['gazebo_port'],
                       ROS_IP='127.0.0.1', ROS_HOSTNAME='127.0.0.1',
                       PYTHONDONTWRITEBYTECODE='1')
            command = [sys.executable, '-B', str(HERE / 'runtime_lab.py'),
                       '--out', str(directory), '--port', str(job['ros_port']),
                       '--gazebo', '--scenario', job['scenario'], '--seed', str(job['seed']),
                       '--localization', job['track'], '--graph', job['graph']]
            started = time.monotonic()
            process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                       start_new_session=True, text=True, env=env)
            timed_out = False
            try:
                output = process.communicate(timeout=args.timeout_s)[0]
            except subprocess.TimeoutExpired:
                timed_out = True
                stop(process)
                output = ''
            elapsed = time.monotonic() - started
            result_path = directory / 'driver.json'
            driver = json.loads(result_path.read_text()) if result_path.exists() else {}
            goal = driver.get('dist_to_goal_m')
            clearance = driver.get('obstacle_min_clearance_m')
            verdict = ('PASS' if (not timed_out and process.returncode == 0
                                  and goal is not None and clearance is not None
                                  and goal <= registration['criteria']['goal_position_m']
                                  and clearance >= registration['criteria']['min_clearance_m'])
                       else 'FAIL')
            attempts.append({'attempt': attempt, 'returncode': process.returncode,
                             'timed_out': timed_out, 'elapsed_s': elapsed,
                             'driver': {key: driver.get(key) for key in
                                        ('dist_to_goal_m', 'path_length_m',
                                         'obstacle_min_clearance_m', 'injected_box_clearance_m',
                                         'track', 'seed', 'scenario_id')},
                             'tail': (output or '')[-2000:]})
            if verdict == 'PASS':
                break
        row = dict(job, verdict=verdict, attempts=attempts,
                   elapsed_s=attempts[-1]['elapsed_s'])
        with lock:
            completed[job['index']] = row
            persist()
        return row

    started = time.monotonic()
    with ThreadPoolExecutor(max_workers=args.workers) as pool:
        results = list(pool.map(run_cell, jobs))
    summary = json.loads((out / 'results.json').read_text())
    summary['elapsed_wall_s'] = time.monotonic() - started
    summary['all_passed'] = summary['completed'] == summary['planned'] and summary['failed'] == 0
    (out / 'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=1) + '\n')
    print(json.dumps({key: summary[key] for key in ('planned', 'completed', 'passed', 'failed',
                                                   'all_passed', 'elapsed_wall_s')},
                     ensure_ascii=False))
    return 0 if summary['all_passed'] else 6


if __name__ == '__main__':
    raise SystemExit(main())
