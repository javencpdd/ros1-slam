"""Preregister a backend experiment, then execute it without dropping failures.

These ground-truth Gazebo trials are separate from the frozen v12 denominator
and from a four-combination localization/backend acceptance matrix.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time

HERE = Path(__file__).resolve().parent


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', required=True)
    parser.add_argument('--backend', default='teb', choices=['teb', 'dwa'])
    parser.add_argument('--seeds', default='0,1,2')
    parser.add_argument('--scenarios', default='SC01,SC02,SC09')
    args = parser.parse_args()
    output = Path(args.out).resolve()
    output.mkdir(parents=True, exist_ok=True)
    registration = output / 'preregistered.json'
    if registration.exists():
        raise ValueError('output already registered; use a new directory')
    jobs = [{'backend': args.backend, 'scenario': scenario, 'seed': seed,
             'repetition': 1, 'localization': 'gazebo_ground_truth'}
            for scenario in args.scenarios.split(',') for seed in map(int, args.seeds.split(','))]
    registration.write_text(json.dumps({'run_class': 'runtime_experimental', 'jobs': jobs,
        'criteria': {'goal_position_m': .30, 'min_clearance_m': .20, 'wall_deadline_s': 120},
        'parameters_sha256': hashlib.sha256((HERE.parent / 'calibration/flat2d/teb_runtime.json')
                                           .read_bytes()).hexdigest()}, indent=2))
    cells = []
    for job in jobs:
        name = '%s_%s_s%d' % (job['backend'], job['scenario'], job['seed'])
        directory = output / name
        started = time.monotonic()
        code = subprocess.call([sys.executable, '-B', str(HERE / 'runtime_lab.py'),
            '--out', str(directory), '--gazebo', '--backend', job['backend'],
            '--scenario', job['scenario'], '--seed', str(job['seed'])])
        result_path = directory / 'driver.json'
        result = json.loads(result_path.read_text()) if result_path.exists() else {}
        cells.append(dict(job, returncode=code, result=result,
                          elapsed_wall_s=time.monotonic() - started))
        (output / 'results.json').write_text(json.dumps({'cells': cells,
            'planned': len(jobs), 'completed': len(cells),
            'passed': sum(c['returncode'] == 0 for c in cells),
            'all_passed': len(cells) == len(jobs) and all(c['returncode'] == 0 for c in cells)}, indent=2))
    return 0 if all(c['returncode'] == 0 for c in cells) else 6


if __name__ == '__main__':
    raise SystemExit(main())
