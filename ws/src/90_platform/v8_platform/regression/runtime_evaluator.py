"""独立运行证据评价器；只读观测与注入记录，不导入被测门控器或规划器。

此评价器证明下游命令撤销/恢复，不把消息健康扩大为 LIO/NDT 精度或四组合验收。
seed 必须同时改变预登记扰动和实际输入；时间戳不同不算输入独立性。
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import random


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                     allow_nan=False).encode()).hexdigest()


def input_profile(seed):
    rng = random.Random(int(seed))
    return {'seed': int(seed), 'bias_xyz': [round(rng.uniform(-0.01, 0.01), 6) for _ in range(3)],
            'candidate_speed': round(rng.uniform(0.15, 0.22), 6),
            'stale_pause_s': round(rng.uniform(0.34, 0.40), 6),
            'jump_m': round(rng.uniform(1.0, 1.5), 6)}


def _moving(row):
    return any(abs(row.get(key, 0)) > 1e-9 for key in ('vx', 'vy', 'vz', 'wx', 'wy', 'wz'))


def evaluate(record):
    errors = []
    outputs = record.get('outputs', [])
    profile = record.get('profile', {})
    if any(not all(isinstance(row.get(key), (int, float))
                   and math.isfinite(row[key]) for key in ('t', 'vx', 'vy', 'vz', 'wx', 'wy', 'wz'))
           for row in outputs):
        errors.append('nonfinite_or_incomplete_output')
    if record.get('profile_hash') != digest(profile):
        errors.append('profile_hash_mismatch')
    odometry = [row for row in record.get('inputs', []) if row.get('kind') == 'odom']
    candidates = [row for row in record.get('inputs', []) if row.get('kind') == 'candidate']
    if not odometry or not candidates:
        errors.append('missing_input_observations')
    elif record.get('input_mode') == 'declared_fixture':
        if odometry[0]['xyz'] != profile.get('bias_xyz'):
            errors.append('seed_not_applied_to_pose')
        if candidates[0]['vx'] != profile.get('candidate_speed'):
            errors.append('seed_not_applied_to_command')
    results = []
    for case in record.get('cases', []):
        failures = []
        before = [row for row in odometry if row['t'] < case['fault_at']]
        after = [row for row in odometry if row['t'] >= case['fault_at']]
        if case['case'] == 'stale':
            resumed_inputs = [row for row in odometry if row['t'] > case['fault_at'] + 1e-9]
            injected = bool(resumed_inputs and resumed_inputs[0]['t'] - case['fault_at'] > 0.25)
        elif case['case'] == 'jump':
            injected = bool(before and after and math.dist(before[-1]['xyz'], after[0]['xyz']) > 0.5)
        elif case['case'] == 'stamp_regression':
            injected = bool(before and after and after[0]['stamp'] <= before[-1]['stamp'])
        else:
            injected = any(row.get('kind') == 'relocalized' and row.get('value') is True
                           and abs(row['t'] - case['fault_at']) < 0.05 for row in record.get('inputs', []))
        if not injected:
            failures.append('fault_injection_unobserved')
        windows = {}
        for name in ('armed', 'stop', 'hold', 'resume'):
            start, end = case[name + '_window']
            windows[name] = [row for row in outputs if start <= row['t'] <= end]
            if not windows[name]:
                failures.append(name + '_unobserved')
        if windows['armed'] and not any(_moving(row) for row in windows['armed']):
            failures.append('never_armed')
        stopped = [row['t'] for row in windows['stop'] if not _moving(row)]
        if not stopped:
            failures.append('no_zero_after_fault')
        if windows['hold'] and any(_moving(row) for row in windows['hold']):
            failures.append('old_lease_revived')
        resumed = [row['t'] for row in windows['resume'] if _moving(row)]
        if not resumed:
            failures.append('new_lease_not_resumed')
        results.append({'case': case['case'], 'ok': not failures, 'failures': failures,
                        'stop_latency_s': None if not stopped else min(stopped) - case['fault_at'],
                        'resume_latency_s': None if not resumed else min(resumed) - case['resume_at'],
                        'hold_zero_samples': sum(not _moving(row) for row in windows['hold'])})
    names = [result['case'] for result in results]
    if len(names) != 4 or set(names) != {'jump', 'relocalization', 'stamp_regression', 'stale'}:
        errors.append('fault_coverage_incomplete')
    return {'ok': not errors and all(result['ok'] for result in results),
            'errors': errors, 'cases': results,
            'scope': 'localization message lease and downstream ROS command revocation',
            'accuracy_backend_switch_tested': False}


def matrix(records):
    seen = set()
    rows, errors = [], []
    by_seed = {}
    actual_inputs = {}
    if not records:
        errors.append('empty_matrix')
    for record in records:
        seed, repeat = record['profile']['seed'], record['repeat']
        identity = (record.get('input_mode'), record.get('source_topic'), seed, repeat)
        if identity in seen:
            errors.append('duplicate_cell:%s' % (identity,))
        seen.add(identity)
        result = evaluate(record)
        rows.append({'seed': seed, 'repeat': repeat, 'profile_hash': record['profile_hash'],
                     'ok': result['ok'], 'evaluation': result})
        by_seed.setdefault(seed, set()).add(record['profile_hash'])
        first_odom = next((row['xyz'] for row in record.get('inputs', [])
                           if row.get('kind') == 'odom'), None)
        first_command = next((row['vx'] for row in record.get('inputs', [])
                              if row.get('kind') == 'candidate'), None)
        actual_inputs.setdefault(seed, set()).add(digest([first_odom, first_command]))
    if any(len(hashes) != 1 for hashes in by_seed.values()):
        errors.append('same_seed_profile_changed')
    if len(by_seed) > 1:
        if len({next(iter(hashes)) for hashes in by_seed.values()}) != len(by_seed):
            errors.append('nominal_seed_profiles_identical')
        if len({item for hashes in actual_inputs.values() for item in hashes}) < len(by_seed):
            errors.append('nominal_seed_inputs_identical')
    return {'ok': not errors and all(row['ok'] for row in rows), 'errors': errors,
            'cells': rows, 'seeds': sorted(by_seed),
            'claim': 'distinct injected input profiles; no statistical independence claimed',
            'uncovered': ['LIO/NDT accuracy switch', 'four-combination navigation matrix',
                          'Gazebo physical braking']}


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('records', nargs='+')
    parser.add_argument('--out', default='')
    args = parser.parse_args(argv)
    records = []
    for path in args.records:
        value = json.loads(Path(path).read_text())
        records.extend(value['records'] if 'records' in value else [value])
    result = matrix(records)
    if args.out:
        Path(args.out).write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result))
    return 0 if result['ok'] else 6


if __name__ == '__main__':
    raise SystemExit(main())
