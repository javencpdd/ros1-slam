"""A3 ROS1 接口证据：同 XY 双层、净空、free/unknown、无支撑与单向不可达。

使用合法连通有向图；不以删边造契约无效图来冒充无路。
这是带独立地图夹具的真实服务查询，非 Gazebo 传感器建图验收。
"""
import argparse
import copy
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
for sub in ('representation', 'planner'):
    sys.path.insert(0, str(ROOT / sub))
import support_map as rep
import a4_runtime as a4


def fixture():
    return rep.build_support_map(
        'a3_multilayer', 0.25, [-2, -2, 0],
        [{'surface_id': 'ground', 'z_reference': 0.0, 'data_uri': 'fixture://ground',
          'footprint': {'min_x': -2, 'max_x': 2, 'min_y': -2, 'max_y': 2},
          'edges': [{'to': 'upper', 'kind': 'slope', 'max_slope_deg': 12.0,
                     'bidirectional': False}]},
         {'surface_id': 'upper', 'z_reference': 0.6, 'data_uri': 'fixture://upper',
          'footprint': {'min_x': -0.5, 'max_x': 0.5, 'min_y': -0.5, 'max_y': 0.5},
          'edges': []}], None, {'uri': 'fixture://a3', 'sha256': 'a' * 64})


QUERIES = [[0.25, 0.25, 0.0], [0.25, 0.25, 0.6], [0.25, 0.25, 0.3],
           [0.75, 0.25, 0.0], [-0.75, 0.25, 0.0]]


def call_snapshots(first, second, evidence1, evidence2, queries):
    import rospy
    from std_srvs.srv import Trigger

    with tempfile.TemporaryDirectory(prefix='a3_map_') as folder:
        map_path = Path(folder) / 'map.json'
        evidence_path = Path(folder) / 'evidence.json'
        map_path.write_text(json.dumps(first))
        evidence_path.write_text(json.dumps(evidence1))
        name = 'a3_map_query'
        command = [sys.executable, str(ROOT / 'representation' / 'map_node.py'),
                   '__name:=' + name, '_map_json:=' + str(map_path),
                   '_evidence_json:=' + str(evidence_path), '_keep_alive_s:=60']
        # stderr to file avoids PIPE backpressure while retaining startup failure details.
        with tempfile.TemporaryFile() as log:
            proc = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
            try:
                rospy.set_param('/' + name + '/query_points', queries)
                rospy.wait_for_service('/' + name + '/query', timeout=20)
                proxy = rospy.ServiceProxy('/' + name + '/query', Trigger)
                observed = []
                for value, evidence in ((first, evidence1), (second, evidence2)):
                    map_path.write_text(json.dumps(value))
                    evidence_path.write_text(json.dumps(evidence))
                    response = proxy()
                    if not response.success:
                        raise RuntimeError(response.message)
                    observed.append(json.loads(response.message))
                return observed
            finally:
                if proc.poll() is None:
                    proc.terminate()
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait(timeout=5)


def run(call_snapshots_fn=None, call_node_fn=None):
    first = fixture()
    second = copy.deepcopy(first)
    second['map_version'] = 2
    second['surfaces'][1]['z_reference'] = 0.8
    evidence1 = [{'xyz': QUERIES[0], 'state': 'free'}]
    evidence2 = [{'xyz': QUERIES[0], 'state': 'occupied'}]
    one, two = (call_snapshots_fn or call_snapshots)(
        first, second, evidence1, evidence2, QUERIES)
    unsupported = copy.deepcopy(first)
    unsupported['surfaces'] = [unsupported['surfaces'][0]]
    unsupported['surfaces'][0]['traversability'] = 'unsupported'
    unsupported['surfaces'][0]['edges'] = []
    unsupported['multiple_surfaces_per_xy'] = False
    unsupported['map_version'] = 3
    unsupported_report, _ = (call_snapshots_fn or call_snapshots)(
        unsupported, unsupported, evidence1, evidence1, QUERIES)
    q = one['queries']
    observations = [
        ('same_xy_layers', [row['z'] for row in q[0]['layers']], [0.0, 0.6]),
        ('positive_free', q[0]['state'], 'free'),
        ('upper_unknown', q[1]['state'], 'unknown'),
        ('unsupported_height', q[2]['supported'], False),
        ('fractional_outside_upper', [row['z'] for row in q[3]['layers']], [0.0]),
        ('negative_outside_upper', [row['z'] for row in q[4]['layers']], [0.0]),
        ('ground_clearance', q[0]['clearance_m'], 0.6),
        ('open_top', q[1]['clearance_m'], None),
        ('updated_version', two['map_version'], 2),
        ('updated_clearance', two['queries'][0]['clearance_m'], 0.8),
        ('updated_occupied', two['queries'][0]['state'], 'occupied'),
        ('digest_changed', one['digest'] != two['digest'], True),
        ('unsupported_surface_not_support', unsupported_report['queries'][0]['supported'], False),
        ('unsupported_layer_traversability',
         unsupported_report['queries'][0]['layers'][0]['traversability'], 'unsupported'),
    ]
    route, reason, info = (call_node_fn or a4.call_node)(
        first, (0.25, 0.25, 0.6), (-1.0, 0.0, 0.0), 20000, 'a3_oneway')
    observations.append(('oneway_planner_refusal', reason if route is None else 'route',
                         'E_PLAN_NO_ROUTE'))
    return [{'case': name, 'observed': got, 'expected': expected,
             'outcome': 'pass' if got == expected else 'unexpected_failure'}
            for name, got, expected in observations]


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--out', default='')
    args = parser.parse_args(argv)
    import rospy
    rospy.init_node('a3_runtime_runner')
    cells = run()
    result = {'scope': 'ROS1 map query and planner services; declared fixture, no sensors',
              'cells': cells, 'passed': sum(c['outcome'] == 'pass' for c in cells),
              'total': len(cells)}
    if args.out:
        Path(args.out).write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps(result, ensure_ascii=False))
    return 0 if result['passed'] == result['total'] else 6


if __name__ == '__main__':
    raise SystemExit(main())
