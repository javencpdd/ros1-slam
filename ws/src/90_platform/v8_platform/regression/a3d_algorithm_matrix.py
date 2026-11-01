#!/usr/bin/env python3
"""3D algorithm-level matrix: global ROS-vs-design and local plan-vs-evaluator.

Global cells call the real ROS planner node and compare its route point-for-point
with the design-layer support_planner.  Local cells call local_native.plan and
score the result with the independent dense evaluator.  This is algorithm
simulation over declared support maps, not Gazebo/real-robot acceptance.
"""
import argparse
import copy
import json
import os
from pathlib import Path
import subprocess
import sys
import time

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
for sub in ('scenarios', 'calibration', 'planner', 'local_planner', 'representation', 'contracts'):
    sys.path.insert(0, str(ROOT / sub))
import support_planner as pln  # noqa: E402
import local_native as native  # noqa: E402
import evaluate as local_eval  # noqa: E402
import support_map  # noqa: E402
import regression as reg  # noqa: E402
import a4_runtime  # noqa: E402

FLAT = str(ROOT / 'contracts' / 'fixtures' / 'map_single_floor.json')
TWO = str(ROOT / 'contracts' / 'fixtures' / 'map_ml_two_surfaces.json')
NIST = '/opt/ros1-slam/data/maps/nist_stairs_120_s020x170_support.json'
BUDGET = 20000
LOCAL_STAIRS = native.Config(resolution=.15, tick_s=1.2, horizon_s=8.0,
                             validity_s=8.0, max_speed_mps=.6, radius_m=.15,
                             height_m=.35, clearance_m=.1)


def load(path):
    return json.loads(Path(path).read_text())


def without_edges(value):
    value = copy.deepcopy(value)
    for surface in value['surfaces']:
        surface['edges'] = []
    return value


def local_layers(bidirectional=True):
    return support_map.build_support_map(
        'local_layers', .2, [-2., -2., 0.],
        [{'surface_id': 'ground', 'z_reference': 0.0, 'data_uri': 'fixture://ground',
          'footprint': {'min_x': -2., 'max_x': 0.2, 'min_y': -1., 'max_y': 1.},
          'edges': [{'to': 'deck', 'kind': 'step', 'max_step_height_m': .2,
                     'bidirectional': bidirectional}]},
         {'surface_id': 'deck', 'z_reference': .15, 'data_uri': 'fixture://deck',
          'footprint': {'min_x': 0., 'max_x': 3., 'min_y': -1., 'max_y': 1.},
          'edges': []}],
        None, {'uri': 'fixture://local_layers', 'sha256': 'a' * 64})


def two_directional():
    value = load(TWO)
    ground, mezz = value['surfaces']
    ground['footprint'] = {'min_x': -5., 'min_y': -5., 'max_x': -1., 'max_y': 5.}
    mezz['footprint'] = {'min_x': 1., 'min_y': 1., 'max_x': 4., 'max_y': 3.}
    for surface in value['surfaces']:
        for edge in surface['edges']:
            edge['bidirectional'] = False
    return value


def global_cases():
    two = load(TWO)
    nist = load(NIST)
    return [
        {'cell': 'two_reach', 'map': two, 'start': (3.0, 2.0, .6),
         'goal': (3.5, 2.0, .6), 'expect': 'route'},
        {'cell': 'two_layer', 'map': two, 'start': (2.5, 2.0, 0.),
         'goal': (2.5, 2.0, .6), 'expect': 'route'},
        {'cell': 'nist_top', 'map': nist, 'start': (0., -2.175, 0.),
         'goal': (0., 1.34, .890358), 'expect': 'route'},
        {'cell': 'nist_mid', 'map': nist, 'start': (0., -2.175, 0.),
         'goal': (0., -.1, .544578), 'expect': 'route'},
        {'cell': 'nist_down', 'map': nist, 'start': (0., 1.34, .890358),
         'goal': (0., -2.175, 0.), 'expect': 'route'},
        {'cell': 'nist_unsupported', 'map': nist, 'start': (0., -2.175, 0.),
         'goal': (10., 10., .7), 'expect': 'E_PLAN_GOAL_INVALID'},
        {'cell': 'nist_budget', 'map': nist, 'start': (0., -2.175, 0.),
         'goal': (0., 1.34, .890358), 'budget': 1,
         'expect': 'E_PLAN_BUDGET_EXCEEDED'},
        {'cell': 'nist_cancel', 'map': nist, 'start': (0., -2.175, 0.),
         'goal': (0., 1.34, .890358), 'cancel_first': True,
         'expect': 'E_PLAN_CANCELLED'},
    ]


def local_cases():
    flat = load(FLAT)
    two = load(TWO)
    nist = load(NIST)
    layers = local_layers()
    layers_oneway = local_layers(bidirectional=False)
    box = {'id': 'box', 'min': [.6, -.2, -.05], 'max': [1.0, .2, .8]}
    moving = dict(box, min=[.6, .8, -.05], max=[1.0, 1.2, .8], velocity=[0., -.8, 0.])
    flat_config = native.Config(resolution=flat['resolution'], tick_s=.5,
                                horizon_s=6.0, validity_s=.75, clearance_m=.2)
    layer_config = native.Config(tick_s=1.2, validity_s=2.0, resolution=.2)
    return [
        {'cell': 'flat_arrive', 'map': flat, 'start': (0., 0., 0.),
         'goal': (1., 0., 0.), 'obstacles': [], 'config': flat_config,
         'expect': 'arrived'},
        {'cell': 'two_layer', 'map': layers, 'start': (-.4, 0., 0.),
         'goal': (.6, 0., .15), 'obstacles': [], 'config': layer_config,
         'expect': 'arrived'},
        {'cell': 'nist_top', 'map': nist, 'start': (0., -2.175, 0.),
         'goal': (0., 1.34, .890358), 'obstacles': [], 'config': LOCAL_STAIRS,
         'expect': 'arrived', 'receding': True},
        {'cell': 'nist_mid', 'map': nist, 'start': (0., -2.175, 0.),
         'goal': (0., -.1, .544578), 'obstacles': [], 'config': LOCAL_STAIRS,
         'expect': 'arrived'},
        {'cell': 'nist_down', 'map': nist, 'start': (0., 1.34, .890358),
         'goal': (0., -2.175, 0.), 'obstacles': [], 'config': LOCAL_STAIRS,
         'expect': 'arrived', 'receding': True},
        {'cell': 'nist_unsupported', 'map': nist, 'start': (0., -2.175, 0.),
         'goal': (0., 0., .7), 'obstacles': [], 'config': LOCAL_STAIRS,
         'expect': 'E_LOCAL_SUPPORT'},
        {'cell': 'nist_no_route', 'map': layers_oneway, 'start': (.6, 0., .15),
         'goal': (-.4, 0., 0.), 'obstacles': [], 'config': layer_config,
         'expect': 'E_LOCAL_NO_ROUTE'},
        {'cell': 'flat_static_box', 'map': flat, 'start': (0., 0., 0.),
         'goal': (1.6, 0., 0.), 'obstacles': [box], 'config': flat_config,
         'expect': 'arrived'},
        {'cell': 'flat_moving_box', 'map': flat, 'start': (0., 0., 0.),
         'goal': (1.6, 0., 0.), 'obstacles': [moving], 'config': flat_config,
         'expect': 'arrived'},
    ]


def global_cell(case, track, call):
    map_value = reg._perturbed(case['map']) if track == 'estimate' else case['map']
    budget = case.get('budget', BUDGET)
    design_route, design_error = None, None
    try:
        kwargs = {}
        if case.get('min_clearance') is not None:
            kwargs['min_clearance_m'] = case['min_clearance']
        planned = pln.plan(map_value, case['start'], case['goal'],
                           budget=budget, **kwargs)
        design_route = [(x, y, z) for x, y, z, _sid in planned['path']]
    except pln.PlannerError as error:
        design_error = error.code
    name = 'a3d_%s_%s' % (case['cell'], track)
    kwargs = {'min_clearance_m': case.get('min_clearance'),
              'cancel_first': bool(case.get('cancel_first'))}
    route, error_code, info = call(map_value, case['start'], case['goal'],
                                   budget, name, **kwargs)
    same = a4_runtime.same_points(route, design_route)
    if case['expect'] == 'route':
        ok = route is not None and design_route is not None and same
    elif case['expect'] == 'E_PLAN_CANCELLED':
        ok = route is None and error_code == case['expect']
    else:
        ok = route is None and design_error == case['expect'] \
            and error_code == case['expect']
    return {'cell': case['cell'], 'track': track, 'kind': 'global',
            'ok': bool(ok), 'expect': case['expect'],
            'design_error': design_error, 'node_error': error_code,
            'same_as_design': same,
            'points': None if route is None else len(route),
            'design_points': None if design_route is None else len(design_route),
            'expansions': None if not info else info.get('expansions')}


def local_cell(case):
    start = case['start']
    now = 0.0
    result, error = None, None
    evaluations = []
    steps = 6 if case.get('receding') else 1
    for _ in range(steps):
        try:
            result = native.plan(case['map'], start, case['goal'],
                                 obstacles=case['obstacles'], now=now,
                                 config=case['config'])
        except native.LocalError as exc:
            error = exc.code
            result = None
            break
        evaluation = local_eval.evaluate(case['map'], result, case['obstacles'], now=now)
        evaluations.append(bool(evaluation.get('ok')))
        if result.get('reason') == 'arrived':
            break
        start = result['trajectory'][-1][:3]
        now = result['trajectory'][-1][3]
    evaluation_ok = bool(evaluations) and all(evaluations)
    if case['expect'] == 'arrived':
        ok = result is not None and result.get('reason') == 'arrived' and evaluation_ok
    else:
        ok = result is None and error == case['expect']
    return {'cell': case['cell'], 'kind': 'local', 'ok': bool(ok),
            'expect': case['expect'], 'error': error,
            'reason': None if result is None else result.get('reason'),
            'points': None if result is None else len(result.get('trajectory', [])),
            'plans': len(evaluations),
            'evaluation_ok': evaluation_ok if result is not None else None}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', required=True)
    parser.add_argument('--port', type=int, default=11680)
    args = parser.parse_args()
    output = Path(args.out).resolve()
    output.mkdir(parents=True, exist_ok=True)
    os.environ.update(ROS_MASTER_URI='http://127.0.0.1:%d' % args.port,
                      ROS_IP='127.0.0.1', ROS_HOSTNAME='127.0.0.1',
                      ROS_HOME='/tmp/a3d_ros', ROS_LOG_DIR='/tmp/a3d_logs',
                      PYTHONDONTWRITEBYTECODE='1')
    Path('/tmp/a3d_ros').mkdir(exist_ok=True)
    Path('/tmp/a3d_logs').mkdir(exist_ok=True)
    master = subprocess.Popen(['roscore', '-p', str(args.port)],
                              stdout=open('/tmp/a3d_master.log', 'ab'),
                              stderr=subprocess.STDOUT, start_new_session=True,
                              env=dict(os.environ))
    try:
        time.sleep(3)
        import rospy
        rospy.init_node('a3d_algorithm_matrix', anonymous=True, disable_signals=True)
        call = a4_runtime.call_node
        globals_ = []
        for case in global_cases():
            for track in ('truth', 'estimate'):
                row = global_cell(case, track, call)
                print('GLOBAL', json.dumps(row, ensure_ascii=False), flush=True)
                globals_.append(row)
        locals_ = []
        for case in local_cases():
            row = local_cell(case)
            print('LOCAL', json.dumps(row, ensure_ascii=False), flush=True)
            locals_.append(row)
        cells = globals_ + locals_
        result = {'scope': '3d_global_ros_vs_design + local_plan_vs_evaluator',
                  'global': globals_, 'local': locals_,
                  'summary': {'cells': len(cells),
                              'pass': sum(1 for c in cells if c['ok']),
                              'fail': sum(1 for c in cells if not c['ok']),
                              'global_cells': len(globals_),
                              'local_cells': len(locals_)},
                  'boundary': 'declared support maps; no Gazebo/contact/real robot; not formal A4/A5'}
        (output / 'results.json').write_text(json.dumps(result, ensure_ascii=False, indent=1) + '\n')
        (output / 'summary.json').write_text(json.dumps(result['summary'], ensure_ascii=False, indent=1) + '\n')
        print(json.dumps(result['summary'], ensure_ascii=False))
        return 0 if result['summary']['fail'] == 0 else 6
    finally:
        try:
            os.killpg(master.pid, 15)
            master.wait(timeout=10)
        except Exception:
            pass


if __name__ == '__main__':
    raise SystemExit(main())
