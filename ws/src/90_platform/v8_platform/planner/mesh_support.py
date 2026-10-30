"""Convert horizontal top faces of a Collada mesh into a support_map.

This is a geometry adapter, not a scene generator: only upward-facing polygons
are used; disconnected tops at the same height are merged into one bounding
footprint, which is valid for the NIST stair mesh (one top per level) but not a
general-purpose mesh decomposer.
"""
import argparse
import hashlib
import json
import sys
from pathlib import Path
import xml.etree.ElementTree as ET

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / 'representation'))
sys.path.insert(0, str(HERE.parent / 'contracts'))
import support_map as representation  # noqa: E402

NS = '{http://www.collada.org/2005/11/COLLADASchema}'


def _array(root, suffix):
    for array in root.iter(NS + 'float_array'):
        if array.get('id', '').endswith(suffix):
            values = [float(value) for value in (array.text or '').split()]
            if suffix == 'positions-array':
                return [tuple(values[i:i + 3]) for i in range(0, len(values), 3)]
            return [tuple(values[i:i + 3]) for i in range(0, len(values), 3)]
    raise ValueError('missing %s in DAE' % suffix)


def _polygons(root):
    for tag in ('polylist', 'triangles'):
        for node in root.iter(NS + tag):
            inputs = {item.get('semantic'): int(item.get('offset', 0))
                      for item in node.findall(NS + 'input')}
            if 'VERTEX' not in inputs or 'NORMAL' not in inputs:
                continue
            p = [int(value) for value in (node.find(NS + 'p').text or '').split()]
            if tag == 'triangles':
                counts = [3] * int(node.get('count', '0'))
            else:
                counts = [int(value) for value in (node.find(NS + 'vcount').text or '').split()]
            stride = max(inputs.values()) + 1
            cursor = 0
            for count in counts:
                vertices, normals = [], []
                for _ in range(count):
                    vertices.append(p[cursor + inputs['VERTEX']])
                    normals.append(p[cursor + inputs['NORMAL']])
                    cursor += stride
                yield vertices, normals


def top_faces(dae_path, normal_z=.9, z_tolerance=1e-4):
    root = ET.parse(dae_path).getroot()
    positions = _array(root, 'positions-array')
    normals = _array(root, 'normals-array')
    groups = {}
    for vertices, normal_indices in _polygons(root):
        points = [positions[index] for index in vertices]
        nz = [normals[index][2] for index in normal_indices]
        zs = [point[2] for point in points]
        if max(zs) - min(zs) > z_tolerance or min(nz) <= normal_z:
            continue
        z = round(sum(zs) / len(zs), 4)
        xs = [point[0] for point in points]
        ys = [point[1] for point in points]
        group = groups.setdefault(z, [min(xs), max(xs), min(ys), max(ys)])
        group[0], group[1] = min(group[0], min(xs)), max(group[1], max(xs))
        group[2], group[3] = min(group[2], min(ys)), max(group[3], max(ys))
    if not groups:
        raise ValueError('no upward top faces found')
    return [{'z': z, 'min_x': box[0], 'max_x': box[1],
             'min_y': box[2], 'max_y': box[3]} for z, box in sorted(groups.items())]


def faces_to_surfaces(faces, ground_z=0., margin=1., thickness=.05,
                      clearance=.4, ground_y_max=None):
    if not faces:
        raise ValueError('faces must not be empty')
    min_x = min(face['min_x'] for face in faces) - margin
    max_x = max(face['max_x'] for face in faces) + margin
    min_y = min(face['min_y'] for face in faces) - margin
    max_y = max(face['max_y'] for face in faces) + margin
    if ground_y_max is not None:
        # A line touch is not an overlap; keep a small strip under the first step.
        max_y = ground_y_max + min(0.05, max(0., max_y - ground_y_max))
    levels = [dict(face, surface_id='step%d' % (index + 1))
              for index, face in enumerate(sorted(faces, key=lambda f: f['z']))]

    def edge(to, height):
        return [{'to': to, 'kind': 'step', 'max_step_height_m': height,
                 'bidirectional': True}]

    surfaces = [{'surface_id': 'ground', 'z_reference': ground_z,
                 'footprint': {'min_x': min_x, 'min_y': min_y,
                               'max_x': max_x, 'max_y': max_y},
                 'data_uri': 'mesh://ground',
                 'edges': edge(levels[0]['surface_id'], levels[0]['z'] - ground_z + 1e-9)}]
    previous = ground_z
    for index, level in enumerate(levels):
        step_id = level['surface_id']
        edges = edge('ground', level['z'] - ground_z + 1e-9) if index == 0 else \
            edge(levels[index - 1]['surface_id'], level['z'] - previous + 1e-9)
        if index + 1 < len(levels):
            edges += edge(levels[index + 1]['surface_id'],
                          levels[index + 1]['z'] - level['z'] + 1e-9)
        surfaces.append({'surface_id': step_id, 'z_reference': level['z'],
                         'footprint': {'min_x': level['min_x'], 'min_y': level['min_y'],
                                       'max_x': level['max_x'], 'max_y': level['max_y']},
                         'data_uri': 'mesh://' + step_id, 'edges': edges})
        previous = level['z']
    return surfaces


def build(dae_path, map_id='nist_stairs_120', resolution=.1, margin=1., z_scale=1.,
          xy_scale=1., ground_front_only=False):
    if not (isinstance(z_scale, (int, float)) and z_scale > 0):
        raise ValueError('z_scale must be positive')
    if not (isinstance(xy_scale, (int, float)) and xy_scale > 0):
        raise ValueError('xy_scale must be positive')
    faces = [dict(face, z=face['z'] * z_scale,
                  min_x=face['min_x'] * xy_scale, max_x=face['max_x'] * xy_scale,
                  min_y=face['min_y'] * xy_scale, max_y=face['max_y'] * xy_scale)
             for face in top_faces(dae_path)]
    ground_y_max = min(face['min_y'] for face in faces) if ground_front_only else None
    surfaces = faces_to_surfaces(faces, margin=margin, ground_y_max=ground_y_max)
    raw = Path(dae_path).read_bytes()
    value = representation.build_support_map(
        map_id, resolution,
        [min(s['footprint']['min_x'] for s in surfaces),
         min(s['footprint']['min_y'] for s in surfaces), 0.],
        surfaces, None,
        {'uri': 'fuel://OpenRobotics/NIST stairs 120/' + Path(dae_path).name +
                ('#z_scale=%g&xy_scale=%g' % (z_scale, xy_scale)),
         'sha256': hashlib.sha256(raw).hexdigest()})
    return value, faces


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dae', required=True)
    parser.add_argument('--out', required=True)
    parser.add_argument('--map-id', default='nist_stairs_120')
    parser.add_argument('--resolution', type=float, default=.1)
    parser.add_argument('--z-scale', type=float, default=1.)
    parser.add_argument('--xy-scale', type=float, default=1.)
    parser.add_argument('--ground-front-only', action='store_true')
    args = parser.parse_args()
    value, faces = build(args.dae, args.map_id, args.resolution, z_scale=args.z_scale,
                         xy_scale=args.xy_scale, ground_front_only=args.ground_front_only)
    Path(args.out).write_text(json.dumps(value, ensure_ascii=False, indent=1) + '\n')
    print(json.dumps({'map_id': value['map_id'], 'surfaces': len(value['surfaces']),
                      'faces': faces, 'out': args.out}, ensure_ascii=False))


if __name__ == '__main__':
    main()
