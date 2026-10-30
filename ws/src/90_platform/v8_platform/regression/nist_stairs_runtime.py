"""NIST stairs kinematic runtime probe on a mesh-derived support map.

The map is a derived artifact (scaled Collada top faces), not Gazebo geometry
and not a four-leg contact model. This runner only supplies the climb route:
ground surface centre to highest surface centre.
"""
import argparse
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
for sub in ('planner', 'local_planner', 'representation', 'contracts'):
    sys.path.insert(0, str(ROOT / sub))

# A single planning call on this map needs ~1.8 s; validity is counted from the
# request stamp, so 3.0 s left less than one tick of execution and plans expired
# mid-transfer. 6.0 s covers measured planning latency plus a 1.2 s tick.
EXPERIMENTAL = {'tick_s': 1.2, 'validity_s': 6.0, 'clearance_m': .1}


def case_from_map(value):
    supported = [s for s in value['surfaces'] if s['traversability'] == 'supported']
    if len(supported) < 2:
        raise ValueError('map needs at least two supported surfaces')
    ground = min(supported, key=lambda s: s['z_reference'])
    top = max(supported, key=lambda s: s['z_reference'])

    def centre(surface):
        box = surface['footprint']
        return ((box['min_x'] + box['max_x']) / 2., (box['min_y'] + box['max_y']) / 2.,
                surface['z_reference'])

    return value['map_id'], value, centre(ground), centre(top)


def experimental_config(value):
    return dict(EXPERIMENTAL, resolution=value['resolution'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map', required=True)
    parser.add_argument('--out', required=True)
    parser.add_argument('--wall-deadline-s', type=float, default=180.)
    args = parser.parse_args()
    value = json.loads(Path(args.map).read_text(encoding='utf-8'))
    name, value, start, goal = case_from_map(value)
    config = experimental_config(value)
    import rospy
    rospy.init_node('nist_stairs_runtime', anonymous=True)
    import runtime_probe
    result = runtime_probe.run_case(name, value, start, goal, [], 'arrival',
                                    deadline_s=args.wall_deadline_s,
                                    experimental=config)
    payload = {'scenario': name, 'config': config, 'source': value.get('source'),
               'start': list(start), 'goal': list(goal),
               'scope': 'derived support map; kinematic carrier; no Gazebo; no four-leg contact; no #151',
               'result': result,
               'ok': bool(result.get('ok') and result.get('independent_evaluation', {}).get('ok'))}
    Path(args.out).write_text(json.dumps(payload, ensure_ascii=False, indent=1) + '\n')
    print(json.dumps({'ok': payload['ok'], 'scenario': payload['scenario'],
                      'start': payload['start'], 'goal': payload['goal'],
                      'final': result.get('final_contact_xyz'),
                      'evaluation': result.get('independent_evaluation'),
                      'status': (result.get('last_status') or {}).get('reason')},
                     ensure_ascii=False))
    return 0 if payload['ok'] else 6


if __name__ == '__main__':
    raise SystemExit(main())
