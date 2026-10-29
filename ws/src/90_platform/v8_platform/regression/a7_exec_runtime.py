"""A7 3D execution runtime cell: existing local_native ROS node + kinematic carrier.

This is a scenario-level execution cell, not Gazebo dynamics and not a four-leg
capability claim. It reuses the A5 runtime probe (ROS messages, continuous
receipt-time integration and independent evaluator) with an SC01/SC03/SC04/SC06/
SC08/SC09 map. Long paths use the same experimental 1.2 s tick / 2.0 s validity as
the A5 step cell: the default 0.75 s validity is shorter than planning latency plus
one tick, so such a plan would be expired before it is published.
"""
import argparse
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
for sub in ('scenarios', 'planner', 'local_planner'):
    sys.path.insert(0, str(ROOT / sub))
import scenarios as scn  # noqa: E402

EXPERIMENTAL = {'tick_s': 1.2, 'validity_s': 2.0}


def execution_config():
    return dict(EXPERIMENTAL)


def case_args(sid):
    case = scn.scenario(sid)
    return sid.lower(), case['map'], tuple(case['start']), tuple(case['goal'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario', default='SC03',
                        choices=['SC01', 'SC02', 'SC03', 'SC04', 'SC06', 'SC08', 'SC09'])
    parser.add_argument('--out', required=True)
    parser.add_argument('--wall-deadline-s', type=float, default=90.)
    args = parser.parse_args()
    import rospy
    rospy.init_node('a7_exec_runtime', anonymous=True)
    import runtime_probe
    name, value, start, goal = case_args(args.scenario)
    result = runtime_probe.run_case(name, value, start, goal, [], 'arrival',
                                    deadline_s=args.wall_deadline_s,
                                    experimental=execution_config())
    payload = {'scenario': args.scenario, 'scope': '3D multi-layer kinematic execution; no Gazebo; '
               'no four-leg contact; no #151', 'config': execution_config(), 'result': result,
               'ok': bool(result.get('ok') and result.get('independent_evaluation', {}).get('ok'))}
    Path(args.out).write_text(json.dumps(payload, ensure_ascii=False, indent=1) + '\n')
    print(json.dumps({'ok': payload['ok'], 'scenario': args.scenario,
                      'final': result.get('final_contact_xyz'),
                      'evaluation': result.get('independent_evaluation'),
                      'status': (result.get('last_status') or {}).get('reason')}, ensure_ascii=False))
    return 0 if payload['ok'] else 6


if __name__ == '__main__':
    raise SystemExit(main())
