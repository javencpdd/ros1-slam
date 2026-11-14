"""Controlled comparison on an existing step world; no hardware or frozen budgets."""
import argparse
from contextlib import nullcontext
import hashlib
import json
from pathlib import Path

import crawl_gait
from force_crawl import rear_force_bridge


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--xml', required=True)
    parser.add_argument('--out', required=True)
    parser.add_argument('--backend', choices=('pd', 'force', 'mixed'), default='pd')
    parser.add_argument('--paired', action='store_true')
    parser.add_argument('--front-refresh', action='store_true')
    parser.add_argument('--stance-force', action='store_true')
    parser.add_argument('--step-swing', type=float, default=.06)
    parser.add_argument('--shift-attempts', type=int, default=4)
    parser.add_argument('--start-yaw', type=float, default=0.0)
    args = parser.parse_args()
    config = dict(xml=args.xml, start_x=.80, step_x=1., step_h=.06,
                  step_swing=args.step_swing, drive_fwd=.05, swing_fwd=.06,
                  margin_min=.05, climb_x=.10, climb_dz=.03,
                  rear_rounds=4, rear_swing=.08, climb2_x=.06, climb2_dz=.02,
                  kp=150., kd=1., swing_kp=100., swing_kd=1., rear_advance_pair=args.paired,
                  front_refresh=args.front_refresh, shift_attempts=args.shift_attempts,
                  start_yaw=args.start_yaw)
    context = (nullcontext(None) if args.backend == 'pd'
               else rear_force_bridge(swing_pd=args.backend == 'mixed', stance_force=args.stance_force))
    with context as experiment:
        result = crawl_gait.run('stair', **config)
    feet = result['world_feet_end']
    landed = len(feet) == 4 and all(p[0] >= 1.02 and abs(p[2] - .082) <= .03
                                   for p in feet.values())
    result.update(run_class='pilot', hardware=False, backend=args.backend,
                  pilot_config=config, full_stair_completed=landed,
                  force_reports=[] if experiment is None else experiment['reports'],
                  world_xml_sha256=hashlib.sha256(Path(args.xml).read_bytes()).hexdigest())
    result['pass'] = result['pass'] and landed
    if args.backend != 'pd' and not result['force_reports']:
        result['comparison_valid'] = False
    else:
        result['comparison_valid'] = True
    result['verdict'] = ('FAIL' if not result['pass']
                         else 'PASS' if result['comparison_valid'] else 'INVALID')
    Path(args.out).write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps({key: result[key] for key in
                      ('verdict', 'completed_steps', 'fell', 'full_stair_completed')}))
    return 0 if result['verdict'] == 'PASS' else 6


if __name__ == '__main__':
    raise SystemExit(main())
