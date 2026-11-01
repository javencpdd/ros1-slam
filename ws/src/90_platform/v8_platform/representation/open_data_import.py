#!/usr/bin/env python3
"""Import open maps, Gazebo worlds and datasets into a hashed local layout.

map   : PCD -> copied map.pcd + 2D occupancy map.pgm/map.yaml + open_map.json
world : SDF world + referenced model:// directories + open_world.json
dataset: descriptor referencing an imported map/world -> dataset.json (no copy)
verify: re-hash every recorded artifact

No sensor or navigation claim is made; the generated occupancy map is a
projection of the point cloud at the requested resolution.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import shutil
import sys
import xml.etree.ElementTree as ET

SCHEMA = 'open_import/v1'
SYSTEM_MODELS = {'ground_plane', 'sun'}


class ImportError_(Exception):
    def __init__(self, code, detail):
        super().__init__('%s: %s' % (code, detail))
        self.code = code


def sha256(path):
    digest = hashlib.sha256()
    with open(path, 'rb') as stream:
        for chunk in iter(lambda: stream.read(1 << 20), b''):
            digest.update(chunk)
    return digest.hexdigest()


def read_pcd(path):
    import numpy as np
    header = []
    with open(path, 'rb') as stream:
        while True:
            line = stream.readline()
            if not line:
                raise ImportError_('E_IMPORT_PCD', 'header incomplete')
            header.append(line)
            if line.startswith(b'DATA'):
                break
        fields = [v.decode() for v in [line.split()[1:] for line in header
                                       if line.startswith(b'FIELDS')][0]]
        sizes = [int(v) for v in [line.split() for line in header
                                  if line.startswith(b'SIZE')][0][1:]]
        types = [v.decode() for v in [line.split()[1:] for line in header
                                      if line.startswith(b'TYPE')][0]]
        points = int([line.split()[1] for line in header
                      if line.startswith(b'POINTS')][0])
        mode = [line.split()[1].decode() for line in header
                if line.startswith(b'DATA')][0]
        data = stream.read()
    required = {'x', 'y', 'z'}
    if not required.issubset(fields):
        raise ImportError_('E_IMPORT_PCD', 'x/y/z fields required')
    dtype = np.dtype([(fields[i], types[i].lower() + str(sizes[i]))
                      for i in range(len(fields))])
    if mode == 'binary':
        array = np.frombuffer(data, dtype=dtype, count=points)
    elif mode == 'ascii':
        rows = np.loadtxt(path, skiprows=len(header))
        if rows.ndim == 1:
            rows = rows[None, :]
        needed = fields.index('z') + 1
        if rows.shape[1] < needed:
            raise ImportError_('E_IMPORT_PCD', 'ascii columns incomplete')
        array = np.zeros(points, dtype=dtype)
        for i, name in enumerate(fields):
            array[name] = rows[:, i]
    else:
        raise ImportError_('E_IMPORT_PCD', 'binary_compressed unsupported')
    xyz = np.stack([array['x'], array['y'], array['z']], axis=1).astype(float)
    xyz = xyz[np.isfinite(xyz).all(axis=1)]
    if len(xyz) < 16:
        raise ImportError_('E_IMPORT_PCD', 'too few finite xyz points')
    return {'fields': fields, 'points': len(xyz), 'xyz': xyz}


def write_map(points, dest, resolution, z_min, z_max):
    import numpy as np
    xyz = points['xyz']
    if not z_min < z_max:
        raise ImportError_('E_IMPORT_MAP', 'z_min must be below z_max')
    band = xyz[(xyz[:, 2] >= z_min) & (xyz[:, 2] <= z_max)]
    if len(band) < 16:
        raise ImportError_('E_IMPORT_MAP', 'z band has too few points')
    min_x, min_y = float(band[:, 0].min()), float(band[:, 1].min())
    max_x, max_y = float(band[:, 0].max()), float(band[:, 1].max())
    width = max(1, int(math.ceil((max_x - min_x) / resolution)) + 1)
    height = max(1, int(math.ceil((max_y - min_y) / resolution)) + 1)
    if width * height > 20_000_000:
        raise ImportError_('E_IMPORT_MAP', 'occupancy grid too large')
    image = bytearray([205]) * (width * height)  # unknown
    ix = np.floor((band[:, 0] - min_x) / resolution).astype(int)
    iy = np.floor((band[:, 1] - min_y) / resolution).astype(int)
    occupied = np.zeros((height, width), dtype=bool)
    occupied[iy, ix] = True
    for row in range(height):
        start = row * width
        for col in np.flatnonzero(occupied[row]):
            image[start + col] = 0
    (dest / 'map.pgm').write_bytes(b'P5\n%d %d\n255\n' % (width, height) + bytes(image))
    (dest / 'map.yaml').write_text(
        'image: map.pgm\nresolution: %.9g\norigin: [%.9g, %.9g, 0.0]\n'
        'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n'
        % (resolution, min_x, min_y))
    return {'width': width, 'height': height, 'resolution': resolution,
            'origin': [min_x, min_y, 0.0], 'occupied_cells': int(occupied.sum()),
            'z_band': [z_min, z_max]}


def import_map(pcd, dest, map_id, resolution=.1, z_min=None, z_max=None,
               source='', license_text=''):
    import numpy as np
    dest = Path(dest)
    if dest.exists():
        raise ImportError_('E_IMPORT_DEST', 'destination exists')
    dest.mkdir(parents=True)
    points = read_pcd(pcd)
    z = points['xyz'][:, 2]
    z_min = float(np.quantile(z, .01)) if z_min is None else float(z_min)
    z_max = float(np.quantile(z, .99)) if z_max is None else float(z_max)
    shutil.copy2(pcd, dest / 'map.pcd')
    info = write_map(points, dest, resolution, z_min, z_max)
    record = {'schema': SCHEMA, 'kind': 'map', 'id': map_id,
              'source': source or str(Path(pcd).resolve()),
              'license': license_text, 'source_sha256': sha256(pcd),
              'map_pcd': 'map.pcd', 'map_pcd_sha256': sha256(dest / 'map.pcd'),
              'fields': points['fields'], 'points': points['points'],
              'bounds': {'min': [float(v) for v in points['xyz'].min(0)],
                         'max': [float(v) for v in points['xyz'].max(0)]},
              'z_range': [float(z.min()), float(z.max())],
              'map_yaml': 'map.yaml', 'map_yaml_sha256': sha256(dest / 'map.yaml'),
              'map_pgm': 'map.pgm', 'map_pgm_sha256': sha256(dest / 'map.pgm'),
              **info}
    (dest / 'open_map.json').write_text(json.dumps(record, ensure_ascii=False, indent=1) + '\n')
    return record


def import_world(world, dest, world_id, model_paths=(), source='', license_text=''):
    dest = Path(dest)
    if dest.exists():
        raise ImportError_('E_IMPORT_DEST', 'destination exists')
    dest.mkdir(parents=True)
    try:
        root = ET.parse(world).getroot()
    except ET.ParseError as error:
        raise ImportError_('E_IMPORT_WORLD', 'invalid XML: %s' % error)
    world_node = root if root.tag == 'world' else root.find('world')
    if world_node is None:
        raise ImportError_('E_IMPORT_WORLD', 'SDF world element missing')
    includes = [node.text.strip()[len('model://'):] for node in world_node.findall('.//include/uri')
                if node.text and node.text.strip().startswith('model://')]
    models = []
    search = [Path(p) for p in model_paths] + [Path(world).parent]
    for name in sorted(set(includes)):
        if '/' in name or name.startswith('.'):
            raise ImportError_('E_IMPORT_WORLD', 'unsafe model uri')
        if name in SYSTEM_MODELS:
            models.append({'name': name, 'system': True})
            continue
        found = next((parent / name for parent in search if (parent / name).is_dir()), None)
        if found is None:
            raise ImportError_('E_IMPORT_WORLD', 'model not found: ' + name)
        target = dest / 'models' / name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copytree(found, target)
        models.append({'name': name, 'source': str(found), 'sha256': sha256(
            target / 'model.config') if (target / 'model.config').exists() else None})
    shutil.copy2(world, dest / 'world.world')
    record = {'schema': SCHEMA, 'kind': 'world', 'id': world_id,
              'source': source or str(Path(world).resolve()),
              'license': license_text, 'source_sha256': sha256(world),
              'world': 'world.world', 'world_sha256': sha256(dest / 'world.world'),
              'models': models,
              'environment': 'GAZEBO_MODEL_PATH must include %s' % (dest / 'models')}
    (dest / 'open_world.json').write_text(json.dumps(record, ensure_ascii=False, indent=1) + '\n')
    return record


def verify(root):
    root = Path(root)
    records = list(root.glob('open_map.json')) + list(root.glob('open_world.json')) \
        + list(root.glob('dataset.json'))
    if not records:
        raise ImportError_('E_IMPORT_VERIFY', 'no import record in ' + str(root))
    result = []
    for record_path in records:
        value = json.loads(record_path.read_text())
        if value.get('schema') != SCHEMA:
            raise ImportError_('E_IMPORT_VERIFY', 'schema mismatch')
        checked = []
        for key in ('map_pcd', 'world', 'map_yaml', 'map_pgm'):
            if key in value:
                path = record_path.parent / value[key]
                if not path.exists() or sha256(path) != value.get(key + '_sha256', ''):
                    raise ImportError_('E_IMPORT_VERIFY', 'hash mismatch: ' + str(path))
                checked.append(str(path))
        result.append({'record': str(record_path), 'kind': value.get('kind'),
                       'checked': checked})
        if value.get('kind') == 'dataset':
            for artifact in value.get('artifacts', []):
                result.extend(verify(record_path.parent / artifact['path']))
    return result


def import_dataset(descriptor_path, dest, dataset_id):
    dest = Path(dest)
    if dest.exists():
        raise ImportError_('E_IMPORT_DEST', 'destination exists')
    dest.mkdir(parents=True)
    value = json.loads(Path(descriptor_path).read_text())
    artifacts = []
    if 'map' in value:
        record = import_map(value['map'], dest / 'map', dataset_id + '_map',
                            value.get('map_resolution', .1), value.get('z_min'),
                            value.get('z_max'), value.get('source', ''),
                            value.get('license', ''))
        artifacts.append({'role': 'map', 'path': 'map', 'sha256': record['map_pcd_sha256']})
    if 'world' in value:
        record = import_world(value['world'], dest / 'world', dataset_id + '_world',
                              value.get('model_paths', []), value.get('source', ''),
                              value.get('license', ''))
        artifacts.append({'role': 'world', 'path': 'world', 'sha256': record['world_sha256']})
    if not artifacts:
        raise ImportError_('E_IMPORT_DATASET', 'descriptor has no map/world')
    shutil.copy2(descriptor_path, dest / 'dataset_source.json')
    record = {'schema': SCHEMA, 'kind': 'dataset', 'id': dataset_id,
              'source': value.get('source', str(Path(descriptor_path).resolve())),
              'license': value.get('license', ''), 'artifacts': artifacts,
              'dataset_source': 'dataset_source.json'}
    (dest / 'dataset.json').write_text(json.dumps(record, ensure_ascii=False, indent=1) + '\n')
    return record


def main(argv=None):
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest='command', required=True)
    p = sub.add_parser('map')
    p.add_argument('--pcd', required=True)
    p.add_argument('--id', required=True)
    p.add_argument('--out', required=True)
    p.add_argument('--resolution', type=float, default=.1)
    p.add_argument('--z-min', type=float)
    p.add_argument('--z-max', type=float)
    p.add_argument('--source', default='')
    p.add_argument('--license', default='')
    p = sub.add_parser('world')
    p.add_argument('--world', required=True)
    p.add_argument('--id', required=True)
    p.add_argument('--out', required=True)
    p.add_argument('--model-path', action='append', default=[])
    p.add_argument('--source', default='')
    p.add_argument('--license', default='')
    p = sub.add_parser('dataset')
    p.add_argument('--descriptor', required=True)
    p.add_argument('--id', required=True)
    p.add_argument('--out', required=True)
    p = sub.add_parser('verify')
    p.add_argument('--root', required=True)
    args = parser.parse_args(argv)
    try:
        if args.command == 'map':
            result = import_map(args.pcd, args.out, args.id, args.resolution,
                                args.z_min, args.z_max, args.source, args.license)
        elif args.command == 'world':
            result = import_world(args.world, args.out, args.id, args.model_path,
                                  args.source, args.license)
        elif args.command == 'dataset':
            result = import_dataset(args.descriptor, args.out, args.id)
        else:
            result = {'verified': verify(args.root)}
    except ImportError_ as error:
        print(json.dumps({'ok': False, 'code': error.code, 'message': str(error)},
                         ensure_ascii=False))
        return 5
    print(json.dumps({'ok': True, 'result': result}, ensure_ascii=False, indent=1))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
