"""把 Gazebo 世界里的支撑体折算成 map.v1 support_map，并在生成时核对外形。

只认名字以 support_ 开头的静态 box 模型：场景是几何真源，边与净空由场景清单声明，
不在此处猜。生成时逐面核对世界里的尺寸/中心，任一不符即拒绝（不静默取缺省）。
"""
import hashlib
import json
import re

BOX = re.compile(r'<box>\s*<size>([^<]+)</size>\s*</box>')
POSE = re.compile(r'<pose>([^<]+)</pose>')


def world_boxes(text):
    boxes = {}
    for block in re.findall(r'<model\s+name="(support_[a-z0-9_]+)".*?</model>', text, re.S):
        pass
    for match in re.finditer(r'<model\s+name="(support_[a-z0-9_]+)"(.*?)</model>', text, re.S):
        name, body = match.group(1), match.group(2)
        size = BOX.search(body)
        pose = POSE.search(body)
        if not size or not pose:
            raise ValueError('%s missing box size or pose' % name)
        sx, sy, sz = [float(v) for v in size.group(1).split()]
        px, py, pz = [float(v) for v in pose.group(1).split()][:3]
        boxes[name] = {'min_x': px - sx / 2, 'max_x': px + sx / 2,
                       'min_y': py - sy / 2, 'max_y': py + sy / 2, 'z': pz + sz / 2}
    return boxes


def build(world_path, scene_path):
    raw = open(world_path, 'rb').read()
    boxes = world_boxes(raw.decode('utf-8'))
    scene = json.load(open(scene_path, encoding='utf-8'))
    surfaces, seen = [], set()
    for spec in scene['surfaces']:
        name = spec['world_model']
        if name not in boxes:
            raise ValueError('world has no model %s' % name)
        box = boxes[name]
        for key in ('min_x', 'min_y', 'max_x', 'max_y'):
            if abs(box[key] - spec['footprint'][key]) > 1e-6:
                raise ValueError('%s footprint mismatch on %s' % (name, key))
        if abs(box['z'] - spec['z_reference']) > 1e-6:
            raise ValueError('%s z mismatch' % name)
        seen.add(name)
        surfaces.append({'surface_id': spec['surface_id'], 'z_reference': spec['z_reference'],
                         'thickness_m': spec['thickness_m'],
                         'representation': spec['representation'] if 'representation' in spec else 'mesh',
                         'data_uri': 'world://' + name, 'traversability': 'supported',
                         'clearance_min_m': spec['clearance_min_m'],
                         'footprint': dict(spec['footprint']), 'edges': spec['edges']})
    if not surfaces:
        raise ValueError('scene declares no surfaces')
    return {'schema_version': 1, 'map_id': scene['map_id'], 'map_version': scene['map_version'],
            'frame': scene['frame'], 'kind': 'support_map', 'resolution': scene['resolution'],
            'origin': scene['origin'], 'extent': scene['extent'],
            'cell_states': ['free', 'occupied', 'unknown'],
            'unknown_policy': 'unknown_not_traversable', 'multiple_surfaces_per_xy': True,
            'source': {'uri': world_path.split('/')[-1], 'sha256': hashlib.sha256(raw).hexdigest()},
            'surfaces': surfaces}
