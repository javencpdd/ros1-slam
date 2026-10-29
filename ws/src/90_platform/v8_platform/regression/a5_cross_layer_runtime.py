"""A5 kinematic staircase cross-layer probe: five support levels, no physics.

This is a chain-level execution cell: the carrier follows the local planner's
declared step edges from ground to a 0.6 m deck. It does not model a four-leg
robot, Gazebo dynamics, or a physical climb capability.
"""
import argparse
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
for sub in ('scenarios', 'planner', 'local_planner', 'representation', 'contracts'):
    sys.path.insert(0, str(ROOT / sub))
import support_map as rep  # noqa: E402

SOURCE = {'uri': 'design://kinematic_staircase', 'sha256': '6' * 64}
EXPERIMENTAL = {'tick_s': 1.2, 'validity_s': 3.0}


def staircase():
    def edge(to):
        return [{'to': to, 'kind': 'step', 'max_step_height_m': .18, 'bidirectional': True}]

    surfaces = [
        {'surface_id': 'ground', 'z_reference': 0.0, 'data_uri': 'g',
         'footprint': {'min_x': -5., 'min_y': -1., 'max_x': -1.6, 'max_y': 1.},
         'edges': edge('step1')},
        {'surface_id': 'step1', 'z_reference': .15, 'data_uri': 's1',
         'footprint': {'min_x': -1.8, 'min_y': -1., 'max_x': -1., 'max_y': 1.},
         'edges': edge('ground') + edge('step2')},
        {'surface_id': 'step2', 'z_reference': .30, 'data_uri': 's2',
         'footprint': {'min_x': -1., 'min_y': -1., 'max_x': -.2, 'max_y': 1.},
         'edges': edge('step1') + edge('step3')},
        {'surface_id': 'step3', 'z_reference': .45, 'data_uri': 's3',
         'footprint': {'min_x': -.2, 'min_y': -1., 'max_x': .6, 'max_y': 1.},
         'edges': edge('step2') + edge('deck')},
        {'surface_id': 'deck', 'z_reference': .60, 'data_uri': 'd',
         'footprint': {'min_x': .4, 'min_y': -1., 'max_x': 5., 'max_y': 1.},
         'edges': edge('step3')},
    ]
    return rep.build_support_map('kinematic_staircase', .2, [-5., -1., 0.], surfaces, None, SOURCE)


def case_args():
    return 'staircase', staircase(), (-4., 0., 0.), (.8, 0., .6)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', required=True)
    parser.add_argument('--wall-deadline-s', type=float, default=90.)
    args = parser.parse_args()
    import rospy
    rospy.init_node('a5_cross_layer_runtime', anonymous=True)
    import runtime_probe
    name, value, start, goal = case_args()
    result = runtime_probe.run_case(name, value, start, goal, [], 'arrival',
                                    deadline_s=args.wall_deadline_s,
                                    experimental=dict(EXPERIMENTAL))
    payload = {'scenario': 'kinematic_staircase', 'config': dict(EXPERIMENTAL),
               'scope': 'kinematic cross-layer chain; no Gazebo; no four-leg contact; no #151',
               'result': result,
               'ok': bool(result.get('ok') and result.get('independent_evaluation', {}).get('ok'))}
    Path(args.out).write_text(json.dumps(payload, ensure_ascii=False, indent=1) + '\n')
    print(json.dumps({'ok': payload['ok'], 'scenario': payload['scenario'],
                      'final': result.get('final_contact_xyz'),
                      'evaluation': result.get('independent_evaluation'),
                      'status': (result.get('last_status') or {}).get('reason')},
                     ensure_ascii=False))
    return 0 if payload['ok'] else 6


if __name__ == '__main__':
    raise SystemExit(main())
