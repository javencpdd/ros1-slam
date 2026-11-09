#!/usr/bin/env python3
"""Collect the Gazebo 3D test evidence into one run ledger for the web dashboard.

The ledger is read-only: it never runs Gazebo and never rewrites evidence. Every
number comes from an on-disk summary/driver JSON; a missing metric stays None
(the page shows "—"). Sources:

  * R1127 two_floor_nav summaries (3D support planner + kinematic carrier)
  * EVT-PREVIEW summaries (3D app preview)
  * R1122 / R1125 / R1126 SLAM demo summaries (SLAM only, no navigation)
  * R980 / R981 / R866 runtime_lab driver.json cells (2D MBF baselines)

Usage:
  python3 scripts/ops/collect_3d_runs.py
  python3 scripts/ops/collect_3d_runs.py --out web/assets/data/3d_runs.json
"""
from __future__ import annotations

import argparse
import datetime as dt
import json
import os
from pathlib import Path
import re
import subprocess

REPO = Path(__file__).resolve().parents[2]
NOTE = REPO / 'note/进度/v8/M5'
CURRENT_3D = {
    'R1127-upstairs-r17',
    'R1127-downstairs-r14',
    'R1127-upper_detour-r14',
    'R1127-blocked-r14',
}
TYPE_LABELS = {
    '3d_chain': '3D 全链',
    '3d_evt': '3D EVT',
    'slam_demo': 'SLAM 演示线',
    '2d_mbf': '2D MBF 基线',
}


def load(path: Path):
    try:
        return json.loads(path.read_text(encoding='utf-8'))
    except (OSError, ValueError):
        return None


def num(value, digits=3):
    if value is None or isinstance(value, bool):
        return None
    try:
        return round(float(value), digits)
    except (TypeError, ValueError):
        return None


def max_gap(gaps):
    if isinstance(gaps, dict):
        vals = [v for v in gaps.values() if isinstance(v, (int, float))]
        return round(max(vals), 3) if vals else None
    return None


def evidence(items, base=REPO, web=REPO / 'web'):
    out = []
    for label, path in items:
        path = Path(path)
        if not path.is_absolute():
            path = base / path
        if not path.exists():
            continue
        rel = os.path.relpath(path, base)
        link = os.path.relpath(path, web)
        out.append({'label': label, 'path': rel, 'link': link})
    return out


def parse_evt_goals(report: Path):
    """Read the execution table in the EVT report: case -> goal distance."""
    if not report.exists():
        return {}
    goals = {}
    for line in report.read_text(encoding='utf-8').splitlines():
        if not line.startswith('|'):
            continue
        cells = [c.strip() for c in line.strip().strip('|').split('|')]
        case = next((name for name in ('upstairs', 'downstairs', 'blocked')
                     if cells and cells[0].startswith(name)), None)
        if not case or len(cells) < 5:
            continue
        for cell in cells[1:]:
            match = re.fullmatch(r'([0-9]+(?:\.[0-9]+)?)\s*m', cell)
            if match:
                goals[case] = round(float(match.group(1)), 4)
                break
    return goals


def r1127_runs():
    runs = []
    for path in sorted((NOTE / '26-11-07/evidence/R1127').glob('*/summary.json')):
        data = load(path)
        if not data:
            continue
        case = data.get('case')
        verdict = data.get('motion_verdict')
        app_rc = data.get('app_rc')
        if not case or verdict is None or app_rc is None:
            continue
        validation = data.get('validation') or {}
        if case == 'blocked':
            expected = (app_rc == 6 and verdict == 'PASS'
                        and (validation.get('reason') == 'E_PLAN_NO_ROUTE'
                             or not validation.get('global_paths')))
            result = 'PASS（预期拒路）' if expected else 'FAIL'
        else:
            expected = (app_rc == 0 and verdict == 'PASS')
            result = 'PASS' if expected else 'FAIL'
        date = (data.get('updated_at') or '')[:10] or None
        run_id = data.get('window_id') or path.parent.name
        runs.append({
            'id': run_id,
            'type': '3d_chain',
            'type_label': TYPE_LABELS['3d_chain'],
            'batch': 'R1127',
            'date': date,
            'world': 'two_floor_nav（Gazebo kinematic carrier）',
            'scene': case + ' / two_floor_nav',
            'map': 'two_floor_nav（声明几何图）',
            'result': result,
            'success': bool(expected),
            'current': run_id in CURRENT_3D,
            'metrics': {
                'goal_distance_m': num(data.get('goal_distance_m')),
                'path_length_m': None,
                'duration_s': None,
                'command_xy_error_m': num(validation.get('command_integral_max_xy_error_m')),
                'command_z_error_m': num(validation.get('command_integral_max_z_error_m')),
                'max_gap_s': max_gap(data.get('max_sample_gap_s')),
                'global_path_points': data.get('global_path_points'),
                'pose_samples': validation.get('pose_samples'),
                'local_plans': validation.get('local_plans'),
                'clearance_m': None,
                'visited_levels': None,
                'ate_xy_mean_m': None,
                'ate_xy_max_m': None,
                'yaw_mean_rad': None,
            },
            'checks': {
                'preflight_rc': data.get('preflight_rc'),
                'app_rc': app_rc,
                'driver_rc': data.get('driver_rc'),
                'scenario_ok': data.get('scenario_ok'),
                'source_binding': data.get('source_binding_verdict'),
                'capture': data.get('capture_verdict'),
                'cleanup': data.get('cleanup_verdict'),
                'validation_ok': validation.get('ok'),
            },
            'notes': (validation.get('scope') or '') + (
                '；预期负例（E_PLAN_NO_ROUTE）' if case == 'blocked' else ''),
            'evidence': evidence([
                ('summary.json', path),
                ('registration.json', path.parent / 'registration.json'),
            ]),
        })
    return runs


def evt_runs():
    report = NOTE / '26-11-09/evidence/EVT-PREVIEW/预演报告.md'
    goals = parse_evt_goals(report)
    runs = []
    for path in sorted((NOTE / '26-11-09/evidence/EVT-PREVIEW').glob('*/summary.json')):
        data = load(path)
        if not data:
            continue
        case = data.get('case') or path.parent.name
        app_rc = data.get('app_rc')
        validation = data.get('validation') or {}
        if app_rc == 0:
            result, ok = 'PASS', True
        elif case == 'blocked' and app_rc == 6:
            result, ok = 'PASS（预期拒路）', True
        else:
            result, ok = 'FAIL', False
        window = data.get('window') or {}
        levels = validation.get('visited_height_levels') or []
        runs.append({
            'id': window.get('id') or ('EVT-PREVIEW-' + case),
            'type': '3d_evt',
            'type_label': TYPE_LABELS['3d_evt'],
            'batch': 'EVT-PREVIEW',
            'date': '2026-11-09',
            'world': 'two_floor_nav（Gazebo kinematic carrier）',
            'scene': case + ' / EVT 预演',
            'map': 'two_floor_nav（声明几何图）',
            'result': result,
            'success': bool(ok),
            'current': False,
            'metrics': {
                'goal_distance_m': goals.get(case),
                'path_length_m': None,
                'duration_s': None,
                'command_xy_error_m': num(validation.get('command_integral_max_xy_error_m')),
                'command_z_error_m': num(validation.get('command_integral_max_z_error_m')),
                'max_gap_s': max_gap({
                    'pose': validation.get('max_pose_gap_s'),
                    'command': validation.get('max_command_gap_s'),
                    'model': validation.get('max_model_gap_s'),
                }),
                'global_path_points': validation.get('global_paths'),
                'pose_samples': validation.get('pose_samples'),
                'local_plans': validation.get('local_plans'),
                'clearance_m': None,
                'visited_levels': len(levels) or None,
                'ate_xy_mean_m': None,
                'ate_xy_max_m': None,
                'yaw_mean_rad': None,
            },
            'checks': {
                'preflight_rc': data.get('preflight_rc'),
                'app_rc': app_rc,
                'validation_ok': validation.get('ok'),
                'four_foot_physics': validation.get('four_foot_physics'),
                'global_corridor_following': validation.get('global_corridor_following'),
            },
            'notes': (validation.get('scope') or '') + (
                '；终点距离取自预演报告执行表' if case in goals else ''),
            'evidence': evidence([
                ('summary.json', path),
                ('summary.app.json', path.parent / 'summary.app.json'),
                ('预演报告.md', report),
            ]),
        })
    return runs


def slam_runs():
    runs = []
    path = NOTE / '26-11-03/evidence/R1122/summary.json'
    data = load(path)
    if data and (data.get('run') or {}).get('ok'):
        run = data['run']
        trans = run.get('translated') or {}
        rigid = run.get('rigid') or {}
        runs.append({
            'id': 'R1122', 'type': 'slam_demo', 'type_label': TYPE_LABELS['slam_demo'],
            'batch': 'R1122', 'date': '2026-11-03',
            'world': 'm1_world_v2（Gazebo，纯 SLAM）',
            'scene': '纯 SLAM v0：平地直行 20 s',
            'map': '—',
            'result': 'PASS', 'success': True, 'current': False,
            'metrics': {
                'goal_distance_m': None, 'path_length_m': None,
                'duration_s': None, 'command_xy_error_m': None,
                'command_z_error_m': None, 'max_gap_s': None,
                'global_path_points': None, 'pose_samples': run.get('lio_samples'),
                'local_plans': None, 'clearance_m': None,
                'visited_levels': None,
                'ate_xy_mean_m': num(trans.get('xy_m_mean'), 6),
                'ate_xy_max_m': num(trans.get('xy_m_max'), 6),
                'yaw_mean_rad': num(rigid.get('yaw_rad_mean'), 6),
            },
            'checks': {'rigid_xy_mean_m': num(rigid.get('xy_m_mean'), 6),
                       'pairs': run.get('pairs')},
            'notes': '只测 SLAM，不跑导航/运动学；非 formal',
            'evidence': evidence([('summary.json', path),
                                   ('demo_slam_compare.json', path.parent / 'demo_slam_compare.json')]),
        })
    path = NOTE / '26-11-03/evidence/R1125/summary.json'
    data = load(path)
    if data:
        for item in data.get('runs') or []:
            if not item.get('ok'):
                continue
            name = str(item.get('file', '')).replace('demo_hospital_lobby_small_', '').replace('.json', '')
            runs.append({
                'id': 'R1125-' + (name or 'run'),
                'type': 'slam_demo', 'type_label': TYPE_LABELS['slam_demo'],
                'batch': 'R1125', 'date': '2026-11-03',
                'world': data.get('scene', 'hospital.world') + '（Gazebo，纯 SLAM）',
                'scene': 'AWS Hospital 大厅 8 m 小环',
                'map': '—',
                'result': 'PASS', 'success': True, 'current': False,
                'metrics': {
                    'goal_distance_m': None, 'path_length_m': None,
                    'duration_s': num(item.get('duration_s'), 1),
                    'command_xy_error_m': None, 'command_z_error_m': None,
                    'max_gap_s': None, 'global_path_points': None,
                    'pose_samples': item.get('pairs'), 'local_plans': None,
                    'clearance_m': None, 'visited_levels': None,
                    'ate_xy_mean_m': num(item.get('translated_xy_mean_m')),
                    'ate_xy_max_m': num(item.get('translated_xy_max_m')),
                    'yaw_mean_rad': num(item.get('rigid_yaw_mean_rad'), 6),
                },
                'checks': {'pairs': item.get('pairs')},
                'notes': '只测 SLAM，不跑导航/运动学；非 formal',
                'evidence': evidence([
                    ('summary.json', path),
                    (str(item.get('file')), path.parent / str(item.get('file', ''))),
                ]),
            })
    for path in sorted((NOTE / '26-11-03/evidence/R1126').glob('demo_hospital_safe_*.json')):
        data = load(path)
        if not data or not data.get('ok'):
            continue
        compare = data.get('compare') or {}
        trans = compare.get('translated') or {}
        rigid = compare.get('rigid') or {}
        name = path.stem.replace('demo_hospital_safe_', '')
        runs.append({
            'id': 'R1126-safe-' + name,
            'type': 'slam_demo', 'type_label': TYPE_LABELS['slam_demo'],
            'batch': 'R1126', 'date': '2026-11-03',
            'world': 'AWS Hospital 安全走廊（Gazebo，纯 SLAM）',
            'scene': '安全走廊 3 seeds',
            'map': '—',
            'result': 'PASS', 'success': True, 'current': False,
            'metrics': {
                'goal_distance_m': None, 'path_length_m': None,
                'duration_s': num(data.get('duration_s'), 1),
                'command_xy_error_m': None, 'command_z_error_m': None,
                'max_gap_s': None, 'global_path_points': None,
                'pose_samples': data.get('pairs'), 'local_plans': None,
                'clearance_m': None, 'visited_levels': None,
                'ate_xy_mean_m': num(trans.get('xy_m_mean')),
                'ate_xy_max_m': num(trans.get('xy_m_max')),
                'yaw_mean_rad': num(rigid.get('yaw_rad_mean'), 6),
            },
            'checks': {'pairs': data.get('pairs')},
            'notes': '只测 SLAM，不跑导航/运动学；非 formal',
            'evidence': evidence([(path.name, path)]),
        })
    return runs


def mbf_runs():
    runs = []
    for batch in ('R980', 'R981', 'R866'):
        for path in sorted((NOTE / '26-10-29/evidence' / batch).glob('*/driver.json')):
            data = load(path)
            if not data or data.get('run_class') != 'runtime_experimental':
                continue
            verdict = data.get('verdict')
            if verdict not in ('PASS', 'FAIL'):
                continue
            runs.append({
                'id': data.get('run_id') or path.parent.name,
                'type': '2d_mbf', 'type_label': TYPE_LABELS['2d_mbf'],
                'batch': batch, 'date': '2026-10-29',
                'world': 'flat2d（Gazebo 2D MBF）',
                'scene': '%s / seed %s / %s' % (
                    data.get('scenario_id'), data.get('seed'), data.get('track')),
                'map': 'flat2d 场景声明图',
                'result': verdict, 'success': verdict == 'PASS', 'current': False,
                'metrics': {
                    'goal_distance_m': num(data.get('dist_to_goal_m')),
                    'path_length_m': num(data.get('path_length_m')),
                    'duration_s': num(data.get('sim_duration_s'), 1),
                    'command_xy_error_m': None, 'command_z_error_m': None,
                    'max_gap_s': None, 'global_path_points': None,
                    'pose_samples': data.get('odom_samples'), 'local_plans': None,
                    'clearance_m': num(data.get('obstacle_min_clearance_m')),
                    'visited_levels': None,
                    'ate_xy_mean_m': None, 'ate_xy_max_m': None,
                    'yaw_mean_rad': None,
                },
                'checks': {'wall_duration_s': num(data.get('wall_duration_s'), 1),
                           'fail_reason': data.get('fail_reason'),
                           'injection_ok': data.get('injection_ok')},
                'notes': data.get('localization_source') or '',
                'evidence': evidence([('driver.json', path)]),
            })
    return runs


def aggregate(rows):
    keys = ('goal_distance_m', 'path_length_m', 'duration_s', 'command_xy_error_m',
            'command_z_error_m', 'max_gap_s', 'clearance_m', 'visited_levels',
            'ate_xy_mean_m', 'ate_xy_max_m', 'yaw_mean_rad')
    metric_rows = [row for row in rows if '预期拒路' not in str(row.get('result'))]
    out = {'runs': len(rows), 'passed': sum(1 for row in rows if row.get('success')),
           'metric_runs': len(metric_rows)}
    for key in keys:
        vals = [row.get('metrics', {}).get(key) for row in metric_rows]
        vals = [float(value) for value in vals if isinstance(value, (int, float))]
        if not vals:
            continue
        mean = sum(vals) / len(vals)
        var = sum((value - mean) ** 2 for value in vals) / len(vals)
        out[key] = {'n': len(vals), 'mean': round(mean, 4),
                    'std': round(var ** 0.5, 4),
                    'min': round(min(vals), 4), 'max': round(max(vals), 4)}
    return out


def git_head():
    try:
        return subprocess.check_output(
            ['git', '-C', str(REPO), 'rev-parse', '--short', 'HEAD'],
            text=True, stderr=subprocess.DEVNULL).strip()
    except (OSError, subprocess.SubprocessError):
        return None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', default=str(REPO / 'web/assets/data/3d_runs.json'))
    args = parser.parse_args()

    runs = r1127_runs() + evt_runs() + slam_runs() + mbf_runs()
    # driver.json run_id is unique only inside one evidence batch
    seen = set()
    for run in runs:
        run['uid'] = '%s/%s' % (run['batch'], run['id'])
        n = 2
        while run['uid'] in seen:
            run['uid'] = '%s/%s#%d' % (run['batch'], run['id'], n)
            n += 1
        seen.add(run['uid'])
    order = {'3d_chain': 0, '3d_evt': 1, 'slam_demo': 2, '2d_mbf': 3}
    # current two_floor four first, then newest evidence; never mix types
    runs.sort(key=lambda r: (
        order.get(r['type'], 9),
        0 if r.get('current') else 1,
        -int((r.get('date') or '1970-01-01').replace('-', '')),
        r['uid']))

    counts = {}
    for kind in TYPE_LABELS:
        rows = [r for r in runs if r['type'] == kind]
        counts[kind] = {
            'label': TYPE_LABELS[kind],
            'total': len(rows),
            'passed': sum(1 for r in rows if r['success']),
            'expected_refusal': sum(1 for r in rows if '预期拒路' in r['result']),
        }
    current = [r for r in runs if r.get('current')]
    by_type = {}
    for kind in TYPE_LABELS:
        by_type[kind] = aggregate([r for r in runs if r['type'] == kind])
    by_batch = {}
    for batch in sorted({r['batch'] for r in runs}):
        by_batch[batch] = aggregate([r for r in runs if r['batch'] == batch])
    payload = {
        'schema': 'v8_3d_runs/v1',
        'generated_at': dt.datetime.now().astimezone().isoformat(timespec='seconds'),
        'git_head': git_head(),
        'current_3d': {
            'ids': [r['id'] for r in current],
            'passed': sum(1 for r in current if r['success']),
            'total': len(CURRENT_3D),
        },
        'counts': counts,
        'aggregates': {'by_type': by_type, 'by_batch': by_batch},
        'runs': runs,
    }
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(payload, ensure_ascii=False, indent=1) + '\n', encoding='utf-8')
    print('wrote %s' % out)
    print('runs %d ｜ %s' % (len(runs), ' ｜ '.join(
        '%s %d/%d' % (v['label'], v['passed'], v['total']) for v in counts.values())))
    print('current_3d %d/%d' % (payload['current_3d']['passed'], payload['current_3d']['total']))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
