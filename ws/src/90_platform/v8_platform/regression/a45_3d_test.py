#!/usr/bin/env python3
"""3D 导航测试：多层面支撑图全局规划（A4）＋运动学局部执行（A5 设计层）。

范围（勿越读）：
- 纯 Python，无 ROS、无 Gazebo、无真机；载体＝experimental_kinematic_carrier。
- 全局＝support_planner 在 SC02/SC05/SC06 的支撑图上规划，scenarios.evaluate 独立判分。
- 局部＝local_native 在 SC06（层下列，地面穿夹层下方）按 0.5 s 控制周期滚动执行，
  以及 0.15 m 台阶夹具单段到达；逐段过 local_planner/evaluate 独立检查。
- SC02（0.6 m 坡道）/SC05（0.6 m 台阶天桥）局部入口按契约拒绝：默认
  max_step_m=0.2、max_slope_deg=35°，不得据此声称四足跨层或 A5 三维到达。
"""
import argparse
import json
import math
import sys
from dataclasses import replace
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
for sub in ('scenarios', 'calibration', 'planner', 'contracts',
            'local_planner', 'representation', 'validator'):
    sys.path.insert(0, str(ROOT / sub))

import scenarios as scn  # noqa: E402
import support_planner as pln  # noqa: E402
import local_native as native  # noqa: E402
import evaluate as local_eval  # noqa: E402
from test_local_native import layers as step_fixture  # noqa: E402

SCENARIOS = ('SC02', 'SC05', 'SC06')
GOAL_TOL = 0.08


def _xyz(point):
    return [round(float(v), 6) for v in point]


def global_case(sid):
    case = scn.scenario(sid)
    route = pln.plan(case['map'], case['start'], case['goal'])
    path = [tuple(p[:3]) for p in route['path']]
    verdict = scn.evaluate(case, path)
    again = pln.plan(case['map'], case['start'], case['goal'])
    deterministic = json.dumps(again, sort_keys=True) == json.dumps(route, sort_keys=True)
    surfaces = []
    for point in route['path']:
        if point[3] not in surfaces:
            surfaces.append(point[3])
    return {
        'scenario': sid,
        'name': case['name'],
        'ok': bool(verdict['ok'] and deterministic),
        'route': {
            'points': len(route['path']), 'segments': len(route['segments']),
            'transitions': len(route['transitions']), 'length_m': round(route['length'], 4),
            'expansions': route['expansions'], 'surfaces': surfaces,
            'path': [[*_xyz(p[:3]), p[3]] for p in route['path']],
        },
        'evaluation': verdict,
        'deterministic': deterministic,
    }


def cross_layer_stop(sid, max_steps=80):
    """滚动执行到跨层入口：默认上限下应停住并给出拒绝码，而不是声称到达。"""
    case = scn.scenario(sid)
    map_value, goal = case['map'], tuple(case['goal'])
    start = tuple(case['start'])
    state, stamp = start, 0.0
    trace, evaluations, steps = [[_xyz(state), 0.0]], [], 0
    for _ in range(max_steps):
        if math.dist(state, goal) <= GOAL_TOL:
            break
        try:
            result = native.plan(map_value, state, goal, (), now=stamp)
        except native.LocalError as error:
            return {'scenario': sid, 'ok': False, 'refusal': error.code,
                    'detail': error.detail, 'steps': steps,
                    'progress_m': round(math.dist(start, goal) - math.dist(state, goal), 3),
                    'final_state': _xyz(state), 'all_evaluations_ok': all(evaluations),
                    'reason': '默认局部上限 max_step_m=0.2、max_slope_deg=35；'
                              '该场景跨层高差 0.6 m，滚动到入口后按契约拒绝',
                    'trajectory': trace}
        verdict = local_eval.evaluate(map_value, result, [], now=stamp)
        evaluations.append(verdict['ok'])
        if not verdict['ok']:
            return {'scenario': sid, 'ok': False, 'refusal': 'E_LOCAL_EVALUATOR',
                    'detail': verdict['failures'], 'steps': steps,
                    'progress_m': round(math.dist(start, goal) - math.dist(state, goal), 3),
                    'final_state': _xyz(state), 'all_evaluations_ok': False,
                    'reason': '独立评价器拒绝', 'trajectory': trace}
        control = native.command(result, stamp, map_value['map_id'],
                                 map_value['map_version'])
        duration = result['trajectory'][1][3] - stamp
        for point in result['trajectory']:
            trace.append([_xyz(point[:3]), round(point[3], 3)])
        state = tuple(state[i] + duration * control['velocity'][i] for i in range(3))
        stamp += duration
        steps += 1
    arrived = math.dist(state, goal) <= GOAL_TOL
    return {'scenario': sid, 'ok': arrived, 'reason': 'arrived' if arrived else 'max_steps',
            'steps': steps, 'progress_m': round(math.dist(start, goal) - math.dist(state, goal), 3),
            'final_state': _xyz(state), 'all_evaluations_ok': all(evaluations),
            'trajectory': trace}


def roll(sid, max_steps=80):
    """0.5 s 控制周期滚动执行：每周期重规划、只执行一拍、独立检查。"""
    case = scn.scenario(sid)
    map_value, goal = case['map'], tuple(case['goal'])
    state, stamp = tuple(case['start']), 0.0
    trace, evaluations, steps = [], [], []
    for _ in range(max_steps):
        if math.dist(state, goal) <= GOAL_TOL:
            break
        result = native.plan(map_value, state, goal, (), now=stamp)
        verdict = local_eval.evaluate(map_value, result, [], now=stamp)
        evaluations.append(verdict['ok'])
        if not verdict['ok']:
            return {'scenario': sid, 'ok': False, 'reason': 'evaluator',
                    'failures': verdict['failures'], 'steps': len(steps),
                    'trajectory': trace}
        control = native.command(result, stamp, map_value['map_id'],
                                 map_value['map_version'])
        duration = result['trajectory'][1][3] - stamp
        for point in result['trajectory']:
            trace.append([_xyz(point[:3]), round(point[3], 3)])
        state = tuple(state[i] + duration * control['velocity'][i] for i in range(3))
        stamp += duration
        steps.append({'t': round(stamp, 3), 'reason': result['reason'],
                      'expansions': result['expansions'],
                      'valid_until': round(result['valid_until'], 3)})
    arrived = math.dist(state, goal) <= GOAL_TOL
    return {
        'scenario': sid, 'ok': bool(arrived and all(evaluations)),
        'reason': 'arrived' if arrived else 'max_steps',
        'steps': len(steps), 'sim_time_s': round(stamp, 3),
        'final_state': _xyz(state), 'goal': _xyz(goal),
        'final_error_m': round(math.dist(state, goal), 4),
        'all_evaluations_ok': all(evaluations),
        'evaluation_count': len(evaluations),
        'trajectory': trace,
    }


def step_case():
    map_value = step_fixture()
    config = replace(native.Config(), tick_s=1.2, validity_s=1.5)
    result = native.plan(map_value, (-.4, 0., 0.), (.6, 0., .15), config=config)
    verdict = local_eval.evaluate(map_value, result, [], now=0.)
    return {
        'case': 'local_step_0.15m', 'ok': bool(verdict['ok'] and result['reason'] == 'arrived'),
        'reason': result['reason'], 'points': len(result['trajectory']),
        'surfaces': result['surfaces'], 'evaluation': verdict,
        'trajectory': [[_xyz(p[:3]), round(p[3], 3)] for p in result['trajectory']],
        'map': map_value,
    }


# ---- SVG（无第三方依赖的等轴投影） ----

def _project(point, k, offset):
    x, y, z = point
    return offset[0] + (x - y) * 0.866 * k, offset[1] + ((x + y) * 0.5 - z * 1.35) * k


def _fit(points, box):
    raw = [((x - y) * 0.866, (x + y) * 0.5 - z * 1.35) for x, y, z in points]
    xs = [p[0] for p in raw]
    ys = [p[1] for p in raw]
    w, h = max(xs) - min(xs), max(ys) - min(ys)
    k = min(box[2] / max(w, 1e-6), box[3] / max(h, 1e-6)) * 0.88
    offset = (box[0] + (box[2] - w * k) / 2 - min(xs) * k,
              box[1] + (box[3] - h * k) / 2 - min(ys) * k)
    return k, offset, raw


def _surface_polygon(surface, k, offset, color):
    fp = surface['footprint']
    z = surface['z_reference']
    corners = [(fp['min_x'], fp['min_y'], z), (fp['max_x'], fp['min_y'], z),
               (fp['max_x'], fp['max_y'], z), (fp['min_x'], fp['max_y'], z)]
    px = [_project(point, k, offset) for point in corners]
    top = ' '.join('%0.1f,%0.1f' % p for p in px)
    skirt = []
    for i, point in enumerate(corners):
        bottom = (point[0], point[1], max(0.0, z - 0.12))
        skirt.append('<line x1="%0.1f" y1="%0.1f" x2="%0.1f" y2="%0.1f" stroke="%s" '
                     'stroke-opacity=".35" stroke-dasharray="3 3"/>'
                     % (*px[i], *_project(bottom, k, offset), color))
    return ('<polygon points="%s" fill="%s" fill-opacity=".16" stroke="%s" '
            'stroke-opacity=".85" stroke-width="1.2"/>%s' % (top, color, color, ''.join(skirt)))


def _polyline(path, k, offset, color, width, opacity=1.0):
    points = ' '.join('%0.1f,%0.1f' % _project(p[:3], k, offset) for p in path)
    return ('<polyline points="%s" fill="none" stroke="%s" stroke-width="%s" '
            'stroke-opacity="%s" stroke-linejoin="round" stroke-linecap="round"/>'
            % (points, color, width, opacity))


def _marker(point, k, offset, color):
    x, y = _project(point[:3], k, offset)
    return '<circle cx="%0.1f" cy="%0.1f" r="5" fill="%s" stroke="#0b1220" stroke-width="1.5"/>' % (x, y, color)


def _panel(parts, box, title, subtitle, map_value, route, extra, start, goal):
    x, y, w, h = box
    parts.append('<rect x="%d" y="%d" width="%d" height="%d" rx="12" fill="#0e1724" '
                 'stroke="rgba(122,190,255,.14)"/>' % (x, y, w, h))
    parts.append('<text x="%d" y="%d" fill="#dbe7f3" font-size="17" font-weight="600">%s</text>'
                 % (x + 18, y + 30, title))
    parts.append('<text x="%d" y="%d" fill="#8899ab" font-size="12.5">%s</text>'
                 % (x + 18, y + 52, subtitle))
    plot = (x + 16, y + 64, w - 32, h - 82)
    points = []
    for surface in map_value['surfaces']:
        fp = surface['footprint']
        z = surface['z_reference']
        points.extend([(fp['min_x'], fp['min_y'], z), (fp['max_x'], fp['max_y'], z),
                       (fp['min_x'], fp['max_y'], z), (fp['max_x'], fp['min_y'], z)])
    points.extend(p[:3] for p in route)
    points.extend(p[:3] for p in extra)
    k, center, _ = _fit(points, plot)
    order = sorted(map_value['surfaces'], key=lambda s: s['z_reference'])
    palette = {'ground': '#7aa2c8', 'mezz': '#79f2b2', 'bridge': '#ffb703',
               'ground_a': '#7aa2c8', 'ground_b': '#7aa2c8', 'deck': '#79f2b2',
               'roof': '#ffb703'}
    for surface in order:
        color = palette.get(surface['surface_id'], '#a78bfa')
        parts.append(_surface_polygon(surface, k, center, color))
        z = surface['z_reference']
        fp = surface['footprint']
        parts.append('<text x="%0.1f" y="%0.1f" fill="%s" fill-opacity=".8" font-size="11">%s z=%.2f</text>'
                     % (*_project(((fp['min_x'] + fp['max_x']) / 2, fp['max_y'], z + 0.03), k, center),
                        color, surface['surface_id'], z))
    if route:
        parts.append(_polyline(route, k, center, '#4cc9f0', 2.6))
    if extra:
        parts.append(_polyline(extra, k, center, '#ff8ad8', 2.0, .9))
    parts.append(_marker(start, k, center, '#79f2b2'))
    parts.append(_marker(goal, k, center, '#ffb703'))


def render_svg(payload, path):
    parts = ['<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 1400 940" '
             'font-family="ui-monospace, SFMono-Regular, Menlo, monospace">',
             '<rect width="1400" height="940" fill="#060a0f"/>',
             '<text x="40" y="48" fill="#dbe7f3" font-size="24" font-weight="700">'
             '3D 导航测试 · 支撑面 / 全局路线 / 局部执行</text>',
             '<text x="40" y="72" fill="#8899ab" font-size="13">'
             '纯 Python 设计层：support_planner（A4）＋ local_native（A5 运动学）；'
             '无 ROS / Gazebo / 真机，不代表四足跨层</text>']
    cases = {c['scenario']: c for c in payload['global']}
    scroll = {c['scenario']: c for c in payload['local_roll']}
    boxes = [(30, 100), (720, 100), (30, 500), (720, 500)]
    for sid, box in ((SCENARIOS[0], (30, 100, 650, 380)),
                     (SCENARIOS[1], (720, 100, 650, 380)),
                     (SCENARIOS[2], (30, 500, 650, 380))):
        case = cases[sid]
        local = scroll.get(sid)
        route = [p[:3] for p in case['route']['path']]
        extra = [p[0] for p in local['trajectory']] if local and local.get('trajectory') else []
        subtitle = ('全局 %d 点 / %d 转层 · 评价 %s' % (
            case['route']['points'], case['route']['transitions'],
            'PASS' if case['ok'] else 'FAIL'))
        if sid in ('SC02', 'SC05'):
            subtitle += ' · 局部拒绝（0.6 m 跨层 > 0.2 m）'
        else:
            subtitle += ' · 局部滚动 %d 步到达' % (local['steps'] if local else 0)
        _panel(parts, box, '%s %s' % (sid, case['name']), subtitle,
               scn.scenario(sid)['map'], route, extra,
               scn.scenario(sid)['start'], scn.scenario(sid)['goal'])
    step = payload['local_step']
    _panel(parts, (720, 500, 650, 380), 'LOCAL 0.15 m 台阶',
           '局部单段到达 · %d 点 · 检查 %s' % (step['points'], 'ok' if step['ok'] else 'FAIL'),
           step['map'], [], [p[0] for p in step['trajectory']],
           (-.4, 0., 0.), (.6, 0., .15))
    parts.append('<text x="40" y="920" fill="#5c6b7c" font-size="12">'
                 '蓝线＝全局支撑图路线　粉线＝运动学局部执行　绿点＝起点　橙点＝终点　'
                 'z 轴放大 1.35×</text>')
    parts.append('</svg>')
    Path(path).write_text('\n'.join(parts) + '\n', encoding='utf-8')


def run():
    globals_ = [global_case(sid) for sid in SCENARIOS]
    refusals = [cross_layer_stop(sid) for sid in ('SC02', 'SC05')]
    rolls = [roll('SC06')]
    step = step_case()
    summary = {
        'global_pass': sum(1 for c in globals_ if c['ok']),
        'global_total': len(globals_),
        'local_roll_pass': sum(1 for c in rolls if c['ok']),
        'local_roll_total': len(rolls),
        'local_step_pass': int(step['ok']),
        'cross_layer_refusals': sum(1 for c in refusals if c.get('refusal')),
    }
    ok = (summary['global_pass'] == summary['global_total']
          and summary['local_roll_pass'] == summary['local_roll_total']
          and summary['local_step_pass'] == 1
          and summary['cross_layer_refusals'] == 2)
    return {
        'schema': 'v8-3d-nav-test/1',
        'generated': '2026-10-29',
        'scope': {'ros': False, 'gazebo': False, 'hardware': False,
                  'carrier': 'experimental_kinematic_carrier',
                  'cross_layer_execution': False, 'four_leg_contact': False},
        'claims': {
            'global_3d': 'support_planner 在 SC02/SC05/SC06 支撑图全局路线，独立评价 PASS',
            'local_3d': 'SC06 层下列 0.5 s 周期滚动到达；0.15 m 台阶单段到达',
            'refused': 'SC02 坡道 / SC05 天桥 0.6 m 跨层在局部入口按 max_step 0.2 m 拒绝',
            'not_claimed': ['四足跨层', 'Gazebo 运行面', '真机', '#151 停车预算', 'A5 三维到达已验收'],
        },
        'global': globals_,
        'local_roll': rolls,
        'local_step': step,
        'cross_layer_attempts': refusals,
        'summary': summary,
        'ok': bool(ok),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', required=True)
    parser.add_argument('--svg')
    args = parser.parse_args()
    payload = run()
    Path(args.out).write_text(json.dumps(payload, ensure_ascii=False, indent=2) + '\n',
                              encoding='utf-8')
    if args.svg:
        render_svg(payload, args.svg)
    print(json.dumps({'ok': payload['ok'], 'summary': payload['summary']},
                     ensure_ascii=False))
    return 0 if payload['ok'] else 6


if __name__ == '__main__':
    sys.exit(main())
