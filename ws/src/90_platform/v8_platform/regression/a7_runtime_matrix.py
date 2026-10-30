"""A7 planning runtime matrix: 10 scenarios x truth/estimate via support_planner GetPlan.

This is the consolidated runtime summary cell; it does not add scenarios or change
the design denominator. Run inside a container with a ROS master.
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
import a4_runtime  # noqa: E402


def matrix(cells):
    rows = []
    for cell in cells:
        rows.append({'scenario': cell['scenario'], 'track': cell['track'],
                     'outcome': cell['outcome'], 'reason': cell.get('reason'),
                     'points': cell.get('points'), 'design_points': cell.get('design_points'),
                     'same_as_design': cell.get('same_as_design')})
    scenarios = sorted({r['scenario'] for r in rows})
    tracks = sorted({r['track'] for r in rows})
    grid = {sid: {track: next((r for r in rows if r['scenario'] == sid and r['track'] == track), None)
                  for track in tracks} for sid in scenarios}
    summary = {
        'scenarios': scenarios, 'tracks': tracks, 'cells': len(rows),
        'passed': sum(1 for r in rows if r['outcome'] == 'pass'),
        'expected_failures': sum(1 for r in rows if r['outcome'] == 'expected_failure'),
        'unexpected_failures': sum(1 for r in rows if r['outcome'] == 'unexpected_failure'),
        'same_as_design': sum(1 for r in rows if r['same_as_design']),
    }
    return {'summary': summary, 'grid': grid, 'cells': rows}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', required=True)
    args = parser.parse_args()
    import rospy
    rospy.init_node('a7_runtime_matrix')
    ids = tuple(a4_runtime.planning_ids())
    payload = matrix(a4_runtime.run(ids=ids))
    payload['domain'] = 'a7_planning_runtime_matrix'
    payload['planned_scenarios'] = list(ids)
    payload['scope'] = 'support_planner ROS GetPlan; truth/estimate; not four-combo runtime'
    Path(args.out).write_text(json.dumps(payload, ensure_ascii=False, indent=1) + '\n')
    print(json.dumps(payload['summary'], ensure_ascii=False))
    return 0 if payload['summary']['unexpected_failures'] == 0 else 6


if __name__ == '__main__':
    raise SystemExit(main())
