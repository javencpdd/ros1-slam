#!/usr/bin/env python3
"""Inflate support_map footprints by a fixed XY margin for stair execution.

The local planner keeps the body center inside a surface shrunk by its radius;
adjacent stair treads with only a few centimetres of overlap therefore leave a
dead band. This tool produces a navigation variant with footprints widened by
the requested margin. It is a map preprocessing step, not a new map claim.
"""
import argparse
import copy
import hashlib
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[4]
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(ROOT / 'ws/src/90_platform/v8_platform/contracts'))
import validate as contract  # noqa: E402
import map_from_pcd  # noqa: E402


def inflate_map(value, map_id, version, margin):
    inflated = copy.deepcopy(value)
    inflated['map_id'] = map_id
    inflated['map_version'] = version
    for surface in inflated['surfaces']:
        box = surface['footprint']
        box['min_x'] -= margin
        box['min_y'] -= margin
        box['max_x'] += margin
        box['max_y'] += margin
        # Cell sets do not grow with the box; drop them so consumers fall back
        # to the inflated footprint instead of the un-inflated real cells.
        surface.pop('cells', None)
    # support_map.source has a closed schema; keep it unchanged and record the
    # inflation in the registry label / evidence, not inside the map payload.
    contract.validate('map', inflated,
                      allow_obstacles='obstacles' in inflated)
    return inflated


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map', required=True)
    parser.add_argument('--out', required=True)
    parser.add_argument('--map-id', required=True)
    parser.add_argument('--version', type=int, default=1)
    parser.add_argument('--margin', type=float, default=0.20)
    parser.add_argument('--register', action='store_true')
    args = parser.parse_args()
    source_path = Path(args.map).resolve()
    value = json.loads(source_path.read_text(encoding='utf-8'))
    inflated = inflate_map(value, args.map_id, args.version, args.margin)
    out = Path(args.out).resolve()
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(inflated, ensure_ascii=False, indent=1) + '\n',
                   encoding='utf-8')
    sha = hashlib.sha256(out.read_bytes()).hexdigest()
    if args.register:
        map_from_pcd.update_registry(args.map_id, args.version, out, sha,
                                     len(inflated['surfaces']),
                                     'inflated %.2f m from %s' % (args.margin, source_path.name))
    print(json.dumps({'ok': True, 'map': str(out), 'sha256': sha,
                      'surfaces': len(inflated['surfaces']),
                      'margin_m': args.margin}, ensure_ascii=False, indent=1))


if __name__ == '__main__':
    main()
