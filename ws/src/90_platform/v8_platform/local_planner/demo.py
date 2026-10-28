"""Reproducible pure-Python local motion experiments (no ROS/Gazebo claim)."""
import argparse
import json
import time
from dataclasses import replace
from pathlib import Path

import evaluate
import local_native as native
from test_local_native import BOX, flat, layers


def run():
    cases = []
    for name, value, start, goal, obstacles in (
        ('flat', flat(), (0., 0., 0.), (1.6, 0., 0.), []),
        ('new_box', flat(), (0., 0., 0.), (1.6, 0., 0.), [BOX]),
        ('moving_box', flat(), (0., 0., 0.), (1.6, 0., 0.),
         [dict(BOX, min=[.6, .8, -.05], max=[1., 1.2, .8], velocity=[0., -.8, 0.])]),
        ('short_step', layers(), (-.4, 0., 0.), (.6, 0., .15), []),
    ):
        start_wall = time.monotonic()
        # Complete exactly one contact-state segment and replan from measured
        # next support state. This exercises rolling motion, not a single plan
        # whose last point is then mislabeled as executed arrival.
        state, stamp, steps, evaluations = start, 0., [], []
        arrived = False
        for _ in range(60):
            config = replace(native.Config(), tick_s=1.2, validity_s=1.5) if name == 'short_step' else native.Config()
            result = native.plan(value, state, goal, obstacles, now=stamp, config=config)
            evaluations.append(evaluate.evaluate(value, result, obstacles, now=stamp))
            if native._distance(state, goal) <= result['config']['goal_tolerance_m']:
                arrived = True
                break
            control = native.command(result, stamp, value['map_id'], value['map_version'])
            duration = result['trajectory'][1][3] - stamp
            next_state = tuple(state[i] + duration * control['velocity'][i] for i in range(3))
            steps.append({'t': stamp, 'state': state, 'velocity': control['velocity'],
                          'next_state': next_state, 'valid_until': result['valid_until']})
            state, stamp = next_state, stamp + duration
        independent = {'ok': all(e['ok'] for e in evaluations),
                       'failures': sorted({f for e in evaluations for f in e['failures']}),
                       'samples': sum(e['samples'] for e in evaluations)}
        cases.append({'name': name, 'outcome': 'arrived' if arrived else 'not_arrived',
                      'evaluation': independent, 'actual_final_support_state': state,
                      'steps': steps, 'wall_s': time.monotonic() - start_wall, 'result': result})
    # A world update may arrive while an old valid trajectory exists. Replan and
    # show that both old-trajectory collision and a safe new route are detected.
    old = native.plan(flat(), (0., 0., 0.), (1.6, 0., 0.))
    rejected_old = evaluate.evaluate(flat(), old, [BOX])
    blocked = {'min': [-.15, -3., -.1], 'max': [.15, 3., 1.]}
    refusal = None
    try:
        native.plan(flat(), (-.4, 0., 0.), (1.4, 0., 0.), [blocked])
    except native.LocalError as error:
        refusal = error.code
    fail_closed = {
        'no_route_reason': refusal,
        'no_route_command': native.command(None, 0., 'single_floor', 2),
        'expired_command': native.command(old, 1., 'single_floor', 2),
        'stale_map_command': native.command(old, .1, 'single_floor', 3),
        'lost_localization_command': native.command(old, .1, 'single_floor', 2, False)}
    ok = (all(c['evaluation']['ok'] and c['outcome'] == 'arrived' for c in cases)
          and not rejected_old['ok'] and refusal == 'E_LOCAL_NO_ROUTE'
          and all(v['stop'] for k, v in fail_closed.items() if k.endswith('_command')))
    return {'ok': ok, 'domain': 'pure_python_kinematic_local_planner', 'gazebo': False,
            'cases': cases, 'unsafe_old_route_evaluation': rejected_old, 'fail_closed': fail_closed}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--out')
    args = parser.parse_args()
    result = run()
    text = json.dumps(result, ensure_ascii=False, indent=2)
    if args.out:
        Path(args.out).write_text(text + '\n', encoding='utf-8')
    print(json.dumps({'ok': result['ok'], 'cases': len(result['cases']), 'gazebo': False}))
    return 0 if result['ok'] else 6


if __name__ == '__main__':
    raise SystemExit(main())
