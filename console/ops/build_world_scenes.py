#!/usr/bin/env python3
"""Build console world scenes from Gazebo SDF worlds.

Reads the canonical world registry (scripts/console/world_registry.json),
then reads those world files without touching ROS.  Writes
scripts/console/ui/assets/data/worlds/.  Box visuals are emitted as posed boxes;
DAE meshes are expanded to triangles in world coordinates.  Other geometry and
unresolved includes are reported in the scene stats instead of being guessed.

Usage:
  python3 scripts/ops/build_world_scenes.py [--only m1_world] [--out DIR]
"""
from __future__ import annotations

import argparse
import datetime
import json
import math
import os
from pathlib import Path
import xml.etree.ElementTree as ET

from world_registry import load_registry

CONSOLE = Path(__file__).resolve().parents[1]
ROOT = CONSOLE.parent
_PARENT = ROOT.parent
REPO = Path(os.environ.get('V8_REPO_ROOT') or (_PARENT / 'ros1-slam')).resolve()
SIM = Path(os.environ.get('V8_SIM_ROOT') or (_PARENT / 'ros1-slam-simulation')).resolve()
DEFAULT_OUT = CONSOLE / 'ui/assets/data/worlds'
WORLD_DIRS = [
    SIM / 'ws/src/lite3_gazebo_sim/worlds',
    SIM / 'ws/src/lite3_gazebo_sim/stairstudy/worlds',
    SIM / 'ws/src/lite3_gazebo_sim/scenarios/two_floor_nav/worlds',
]
MODEL_ROOTS = [
    REPO / 'data/maps/worlds',
    SIM / 'ws/src/lite3_gazebo_sim/models',
    Path('/usr/share/gazebo-11/models'),
    Path('/usr/share/gazebo/models'),
]
BUILTIN_INCLUDES = {'sun', 'ground_plane'}
DEFAULT_COLOR = [0.55, 0.60, 0.66]

IDENTITY = [[1.0 if i == j else 0.0 for j in range(4)] for i in range(4)]


def parse_numbers(text):
    try:
        return [float(v) for v in (text or '').split()]
    except ValueError:
        return []


def pose_matrix(text):
    values = parse_numbers(text)
    if len(values) != 6:
        return [row[:] for row in IDENTITY]
    x, y, z, roll, pitch, yaw = values
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, x],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, y],
        [-sp, cp * sr, cp * cr, z],
        [0.0, 0.0, 0.0, 1.0],
    ]


def mat_mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def transform_point(matrix, point):
    x, y, z = point
    return [
        matrix[0][0] * x + matrix[0][1] * y + matrix[0][2] * z + matrix[0][3],
        matrix[1][0] * x + matrix[1][1] * y + matrix[1][2] * z + matrix[1][3],
        matrix[2][0] * x + matrix[2][1] * y + matrix[2][2] * z + matrix[2][3],
    ]


def matrix_pose(matrix):
    x, y, z = matrix[0][3], matrix[1][3], matrix[2][3]
    sp = max(-1.0, min(1.0, -matrix[2][0]))
    pitch = math.asin(sp)
    if abs(math.cos(pitch)) > 1e-8:
        roll = math.atan2(matrix[2][1], matrix[2][2])
        yaw = math.atan2(matrix[1][0], matrix[0][0])
    else:
        roll = 0.0
        yaw = math.atan2(-matrix[0][1], matrix[1][1])
    return [round(v, 6) for v in (x, y, z, roll, pitch, yaw)]


def flatten(matrix):
    return [round(matrix[i][j], 6) for i in range(4) for j in range(4)]


def material_color(visual):
    material = visual.find('material')
    if material is None:
        return list(DEFAULT_COLOR)
    for tag in ('ambient', 'diffuse'):
        node = material.find(tag)
        if node is None or not (node.text or '').strip():
            continue
        values = parse_numbers(node.text)
        if len(values) >= 3:
            return [round(max(0.0, min(1.0, v)), 3) for v in values[:3]]
    return list(DEFAULT_COLOR)


def resolve_model_dir(uri):
    if not uri or not uri.startswith('model://'):
        return None
    name = uri[len('model://'):].split('/')[0]
    if not name:
        return None
    for root in MODEL_ROOTS:
        candidate = root / name
        if (candidate / 'model.sdf').exists() or (candidate / 'model.config').exists():
            return candidate
    return None


def resolve_model_path(uri):
    if not uri.startswith('model://'):
        return None
    parts = uri[len('model://'):].split('/')
    name = parts[0]
    for root in MODEL_ROOTS:
        candidate = root.joinpath(*parts)
        if candidate.exists():
            return candidate
    return None


def load_dae(path, scale=(1.0, 1.0, 1.0)):
    ns = {'c': 'http://www.collada.org/2005/11/COLLADASchema'}
    root = ET.parse(path).getroot()
    vertices = []
    for source in root.findall('.//c:source', ns):
        source_id = source.get('id') or ''
        if not source_id.endswith('-positions'):
            continue
        array = source.find('c:float_array', ns)
        if array is None or not (array.text or '').strip():
            continue
        values = parse_numbers(array.text)
        if len(values) >= 3 and len(values) % 3 == 0:
            vertices = [values[i:i + 3] for i in range(0, len(values), 3)]
            break
    if not vertices:
        return [], []

    faces = []
    for node in root.findall('.//c:polylist', ns):
        inputs = node.findall('c:input', ns)
        if not inputs:
            continue
        vertex_offset = next((int(item.get('offset', '0')) for item in inputs
                              if item.get('semantic') == 'VERTEX'), 0)
        stride = max(int(item.get('offset', '0')) for item in inputs) + 1
        p_node = node.find('c:p', ns)
        v_node = node.find('c:vcount', ns)
        if p_node is None or v_node is None or not (p_node.text or '').strip():
            continue
        indices = [int(v) for v in p_node.text.split()]
        counts = [int(v) for v in v_node.text.split()]
        cursor = 0
        for count in counts:
            if count < 3:
                cursor += count
                continue
            poly = [indices[(cursor + k) * stride + vertex_offset] for k in range(count)]
            cursor += count
            for k in range(1, count - 1):
                faces.append((poly[0], poly[k], poly[k + 1]))

    for node in root.findall('.//c:triangles', ns):
        inputs = node.findall('c:input', ns)
        p_node = node.find('c:p', ns)
        if not inputs or p_node is None or not (p_node.text or '').strip():
            continue
        vertex_offset = next((int(item.get('offset', '0')) for item in inputs
                              if item.get('semantic') == 'VERTEX'), 0)
        stride = max(int(item.get('offset', '0')) for item in inputs) + 1
        indices = [int(v) for v in p_node.text.split()]
        for base in range(0, len(indices) - stride * 2, stride * 3):
            faces.append((indices[base + vertex_offset],
                          indices[base + stride + vertex_offset],
                          indices[base + 2 * stride + vertex_offset]))

    scaled = [[v[0] * scale[0], v[1] * scale[1], v[2] * scale[2]] for v in vertices]
    return scaled, faces


def mesh_color(uri):
    name = uri.lower()
    if 'nist' in name or 'stair' in name:
        return [0.62, 0.46, 0.28]
    return [0.58, 0.58, 0.62]


def parse_visual(visual, parent, ctx, name):
    matrix = mat_mul(parent, pose_matrix(visual.findtext('pose')))
    geometry = visual.find('geometry')
    if geometry is None:
        return
    box = geometry.find('box')
    if box is not None:
        size = parse_numbers(box.findtext('size'))
        if len(size) == 3:
            ctx['boxes'].append({
                'name': name,
                'size': [round(v, 6) for v in size],
                'pose': matrix_pose(matrix),
                'matrix': flatten(matrix),
                'color': material_color(visual),
            })
        return
    mesh = geometry.find('mesh')
    if mesh is not None:
        uri = (mesh.findtext('uri') or '').strip()
        scale_values = parse_numbers(mesh.findtext('scale'))
        scale = tuple(scale_values[:3]) if len(scale_values) >= 3 else (1.0, 1.0, 1.0)
        path = resolve_model_path(uri)
        if path is None or path.suffix.lower() != '.dae':
            ctx['skipped'].append({'name': name, 'geometry': 'mesh', 'uri': uri,
                                   'reason': 'unresolved or unsupported mesh'})
            return
        try:
            vertices, faces = load_dae(path, scale)
        except (ET.ParseError, OSError, ValueError) as error:
            ctx['skipped'].append({'name': name, 'geometry': 'mesh', 'uri': uri,
                                   'reason': 'dae parse failed: %s' % error})
            return
        if not vertices or not faces:
            ctx['skipped'].append({'name': name, 'geometry': 'mesh', 'uri': uri,
                                   'reason': 'dae has no positions/triangles'})
            return
        world_vertices = [transform_point(matrix, vertex) for vertex in vertices]
        ctx['meshes'].append({
            'name': name,
            'source': uri,
            'color': mesh_color(uri),
            'vertices': [[round(v, 5) for v in vertex] for vertex in world_vertices],
            'faces': [list(face) for face in faces],
        })
        return
    for tag in ('cylinder', 'sphere', 'plane', 'heightmap', 'polyline'):
        if geometry.find(tag) is not None:
            ctx['skipped'].append({'name': name, 'geometry': tag, 'reason': 'not expanded'})
            return


def parse_include(include, parent, ctx, depth):
    uri = (include.findtext('uri') or '').strip()
    name = include.findtext('name') or uri.rsplit('/', 1)[-1] or 'include'
    if uri.startswith('model://') and uri[len('model://'):].split('/')[0] in BUILTIN_INCLUDES:
        ctx['includes'].append({'name': name, 'uri': uri, 'status': 'builtin'})
        return
    model_dir = resolve_model_dir(uri)
    if model_dir is None:
        ctx['skipped'].append({'name': name, 'geometry': 'include', 'uri': uri,
                               'reason': 'model library not found'})
        return
    if depth > 8:
        ctx['skipped'].append({'name': name, 'geometry': 'include', 'uri': uri,
                               'reason': 'include depth limit'})
        return
    try:
        model = ET.parse(model_dir / 'model.sdf').getroot()
    except (ET.ParseError, OSError) as error:
        ctx['skipped'].append({'name': name, 'geometry': 'include', 'uri': uri,
                               'reason': 'model.sdf parse failed: %s' % error})
        return
    matrix = mat_mul(parent, pose_matrix(include.findtext('pose')))
    ctx['includes'].append({'name': name, 'uri': uri, 'status': 'expanded'})
    parse_model(model, matrix, ctx, name, depth + 1)


def parse_model(model, parent, ctx, name, depth=0):
    if depth > 8:
        ctx['skipped'].append({'name': name, 'geometry': 'model', 'reason': 'depth limit'})
        return
    matrix = mat_mul(parent, pose_matrix(model.findtext('pose')))
    for link in model.findall('link'):
        link_name = link.get('name') or 'link'
        link_matrix = mat_mul(matrix, pose_matrix(link.findtext('pose')))
        for index, visual in enumerate(link.findall('visual')):
            visual_name = visual.get('name') or ('visual_%d' % index)
            parse_visual(visual, link_matrix, ctx, '%s/%s/%s' % (name, link_name, visual_name))
    for sub in model.findall('model'):
        sub_name = sub.get('name') or 'model'
        parse_model(sub, matrix, ctx, '%s/%s' % (name, sub_name), depth + 1)
    for include in model.findall('include'):
        parse_include(include, matrix, ctx, depth + 1)


def box_bounds(box):
    corners = []
    half = [box['size'][0] / 2, box['size'][1] / 2, box['size'][2] / 2]
    for sx in (-1, 1):
        for sy in (-1, 1):
            for sz in (-1, 1):
                corners.append(transform_point(
                    [box['matrix'][0:4], box['matrix'][4:8],
                     box['matrix'][8:12], box['matrix'][12:16]],
                    [sx * half[0], sy * half[1], sz * half[2]]))
    return corners


def compute_bounds(boxes, meshes):
    points = []
    for box in boxes:
        points.extend(box_bounds(box))
    for mesh in meshes:
        points.extend(mesh['vertices'])
    if not points:
        return [-1, -1, 0, 1, 1, 1]
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    zs = [p[2] for p in points]
    return [round(min(xs), 3), round(min(ys), 3), round(min(zs), 3),
            round(max(xs), 3), round(max(ys), 3), round(max(zs), 3)]


def build_scene(path):
    root = ET.parse(path).getroot()
    if root.tag == 'world':
        world = root
    else:
        world = root.find('world')
    if world is None:
        raise ValueError('no <world> in %s' % path)
    ctx = {'boxes': [], 'meshes': [], 'skipped': [], 'includes': []}
    for model in world.findall('model'):
        parse_model(model, IDENTITY, ctx, model.get('name') or 'model')
    for include in world.findall('include'):
        parse_include(include, IDENTITY, ctx, 0)
    bounds = compute_bounds(ctx['boxes'], ctx['meshes'])
    return {
        'schema': 'v8-world-scene/v1',
        'world': world.get('name') or path.stem,
        'source': str(path.relative_to(ROOT)) if path.is_relative_to(ROOT) else str(path),
        'generated': datetime.datetime.now().astimezone().isoformat(timespec='seconds'),
        'bounds': bounds,
        'ground': {'z': 0.0},
        'boxes': ctx['boxes'],
        'meshes': ctx['meshes'],
        'includes': ctx['includes'],
        'stats': {
            'boxes': len(ctx['boxes']),
            'meshes': len(ctx['meshes']),
            'triangles': sum(len(mesh['faces']) for mesh in ctx['meshes']),
            'skipped': ctx['skipped'],
        },
    }


def world_files(only=None):
    selected = []
    for entry in load_registry().get('worlds', []):
        if only and entry['id'] not in only:
            continue
        path = SIM / entry['path']
        if path.exists():
            selected.append((entry, path))
    return selected


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--only', nargs='*', help='world stems to build')
    parser.add_argument('--out', default=str(DEFAULT_OUT))
    args = parser.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    files = world_files(set(args.only) if args.only else None)
    index = []
    for entry, path in sorted(files, key=lambda item: item[0]['id']):
        stem = entry['id']
        try:
            scene = build_scene(path)
        except (ET.ParseError, OSError, ValueError) as error:
            print('SKIP %-28s %s' % (stem, error))
            continue
        scene['world_id'] = stem
        scene['label'] = entry['label']
        scene['sha256'] = entry['sha256']
        target = out / (stem + '.json')
        target.write_text(json.dumps(scene, ensure_ascii=False, separators=(',', ':')),
                          encoding='utf-8')
        index.append({
            'id': stem,
            'label': entry['label'],
            'file': 'assets/data/worlds/%s.json' % stem,
            'source': scene['source'],
            'sha256': entry['sha256'],
            'bounds': scene['bounds'],
            'boxes': scene['stats']['boxes'],
            'meshes': scene['stats']['meshes'],
            'triangles': scene['stats']['triangles'],
            'skipped': len(scene['stats']['skipped']),
        })
        print('%-30s boxes=%-4d meshes=%-2d tris=%-4d skipped=%d' % (
            stem, scene['stats']['boxes'], scene['stats']['meshes'],
            scene['stats']['triangles'], len(scene['stats']['skipped'])))
    index_path = out / 'worlds.json'
    index_path.write_text(json.dumps({
        'schema': 'v8-world-index/v1',
        'generated': datetime.datetime.now().astimezone().isoformat(timespec='seconds'),
        'worlds': index,
    }, ensure_ascii=False, separators=(',', ':')), encoding='utf-8')
    print('wrote %d scenes + %s' % (len(index), index_path))


if __name__ == '__main__':
    main()
