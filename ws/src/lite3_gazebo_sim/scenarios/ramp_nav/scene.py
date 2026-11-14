#!/usr/bin/env python3
"""Self-contained ramp Gazebo scene: one geometry source, exact map cells.

斜坡场景的同源生成器（照 two_floor_nav/scene.py 的范式）：
一个 geometry() 函数同时产出
  - Gazebo world（worlds/ramp.world：地面＋10 级坡面切片＋平台＋墙体＋仅视觉载体模型）
  - 支撑图（maps/ramp.json：与每块 world 方盒顶面一一对应的平面＋step 边）
坡面在支撑图语义里＝一串平表面（每片 0.4 m 长、顶面抬 0.06 m），
相邻面之间用 step 边（max_step_height_m 0.06）授权转移——这是本仓
「面＋有向边」架构对坡的表达方式，不是倾斜面网格。

载体＝kinematic 位姿沉（gazebo_3d_carrier.py，SetWorldPose 直推），
不参与接触求解，位姿 z 由载体按支撑面吸附，与地图分格零漂移。
确定性产物由 check() 全量字节校验。
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

sys.dont_write_bytecode = True
HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[5]
PLATFORM = ROOT / 'ws/src/90_platform/v8_platform'
for folder in ('representation', 'planner', 'local_planner', 'contracts', 'framework'):
    sys.path.insert(0, str(PLATFORM / folder))
import support_map

RELATIVE = 'ws/src/95_simulation/lite3_gazebo_sim/scenarios/ramp_nav'
RESOLUTION = .2
RADIUS = .30
HEIGHT = .50
CLEARANCE = .05

# 坡面几何：10 片，每片 0.4 m 长、顶面抬 0.06 m；起点在坡底西侧地面，目标在坡顶平台。
SLICES = 10
SLICE_LEN = .4
SLICE_RISE = .06
RAMP_BASE_X = 1.0
RAMP_Y0, RAMP_Y1 = 0.0, 2.0
PLATFORM_LEN = 2.0
CARRIER_START = (-2.0, 1.0, 0.0)
CARRIER_SIZE = (.45, .30, .20)
CARRIER_CENTER_Z = .10


def json_bytes(value):
    return (json.dumps(value, ensure_ascii=False, sort_keys=True, indent=2, allow_nan=False) + '\n').encode('utf-8')


def box(name, lo, hi, color=(.6, .65, .7, 1)):
    return {'name': name, 'min': list(lo), 'max': list(hi), 'color': list(color)}


def geometry():
    """All lengths are metres. No downloaded models, textures or ROS plugins."""
    top_z = SLICES * SLICE_RISE
    ramp_end = RAMP_BASE_X + SLICES * SLICE_LEN
    boxes = [
        box('ground', [-8, -6, -.2], [8, 6, 0], (.64, .66, .69, 1)),
        box('platform', [ramp_end, RAMP_Y0, 0], [ramp_end + PLATFORM_LEN, RAMP_Y1, top_z], (.45, .54, .62, 1)),
        box('wall_north', [-8.2, 6, -.2], [8.2, 6.2, 3.4]),
        box('wall_south', [-8.2, -6.2, -.2], [8.2, -6, 3.4]),
        box('wall_east', [8, -6, -.2], [8.2, 6, 3.4]),
        box('wall_west', [-8.2, -6, -.2], [-8, 6, 3.4]),
    ]
    for i in range(1, SLICES + 1):
        lo_x = round(RAMP_BASE_X + SLICE_LEN * (i - 1), 6)
        hi_x = round(RAMP_BASE_X + SLICE_LEN * i, 6)
        boxes.append(box('ramp%02d' % i, [lo_x, RAMP_Y0, 0], [hi_x, RAMP_Y1, round(SLICE_RISE * i, 6)],
                         (.65, .46, .24, 1)))
    return {'schema': 'ramp_geometry/v1', 'frame': 'map', 'world_frame': 'world',
            'map_to_world': [0, 0, 0, 0, 0, 0], 'boxes': boxes,
            'planning_envelope': {'radius_m': RADIUS, 'height_m': HEIGHT,
                                  'clearance_m': CLEARANCE, 'kind': 'generic_sim_experimental'},
            'carrier': {'kind': 'kinematic_pose_sink', 'visual_size': list(CARRIER_SIZE),
                        'center_z': CARRIER_CENTER_Z, 'collision_geometry': False,
                        'start': list(CARRIER_START)},
            'slope': {'rise_m': top_z, 'run_m': SLICES * SLICE_LEN,
                      'mean_slope_deg': round(180. * top_z / (SLICES * SLICE_LEN) / 3.141592653589793, 3),
                      'slice_height_m': SLICE_RISE}}


def rectangle_cells(bounds):
    x0, y0, x1, y1 = bounds
    return {(i, j) for i in range(round(x0 / RESOLUTION), round(x1 / RESOLUTION))
            for j in range(round(y0 / RESOLUTION), round(y1 / RESOLUTION))}


def scene_map():
    geo = geometry()
    by_name = {value['name']: value for value in geo['boxes']}
    surfaces = []
    names = ['ground'] + ['ramp%02d' % i for i in range(1, SLICES + 1)] + ['platform']
    for name in names:
        item = by_name[name]
        lo, hi = item['min'], item['max']
        cells = rectangle_cells((lo[0], lo[1], hi[0], hi[1]))
        surfaces.append({'surface_id': name, 'z_reference': hi[2],
                         'thickness_m': hi[2] - lo[2], 'cells': cells,
                         'footprint': {'min_x': lo[0], 'min_y': lo[1], 'max_x': hi[0], 'max_y': hi[1]},
                         'data_uri': 'scene://ramp_nav/' + name,
                         'clearance_min_m': 1.2, 'edges': []})
    for left, right in zip(surfaces, surfaces[1:]):
        step = abs(left['z_reference'] - right['z_reference'])
        edge = {'to': right['surface_id'], 'kind': 'step', 'bidirectional': True,
                'max_step_height_m': round(step + 1e-6, 6)}
        left['edges'].append(edge)
    return support_map.build_support_map('ramp_nav', RESOLUTION, [-8, -6, 0], surfaces, [],
            {'uri': 'scene://ramp_nav/geometry', 'sha256': hashlib.sha256(json_bytes(geo)).hexdigest()},
            map_version=1, obstacles=[])


def world_xml():
    geo = geometry()
    sdf = ET.Element('sdf', version='1.6')
    world = ET.SubElement(sdf, 'world', name='ramp_nav')
    ET.SubElement(world, 'gravity').text = '0 0 -9.81'
    physics = ET.SubElement(world, 'physics', name='default', type='ode')
    ET.SubElement(physics, 'max_step_size').text = '.001'
    ET.SubElement(physics, 'real_time_update_rate').text = '1000'
    light = ET.SubElement(world, 'light', name='sun', type='directional')
    ET.SubElement(light, 'pose').text = '0 0 10 0 0 0'
    ET.SubElement(light, 'diffuse').text = '.9 .9 .9 1'
    ET.SubElement(light, 'direction').text = '-.5 -.2 -.9'
    model = ET.SubElement(world, 'model', name='ramp_geometry')
    ET.SubElement(model, 'static').text = 'true'
    for value in geo['boxes']:
        lo, hi = value['min'], value['max']
        link = ET.SubElement(model, 'link', name=value['name'])
        center = [(a + b) / 2 for a, b in zip(lo, hi)]
        ET.SubElement(link, 'pose').text = ' '.join('%g' % v for v in center) + ' 0 0 0'
        size = ' '.join('%g' % (b - a) for a, b in zip(lo, hi))
        for tag in ('collision', 'visual'):
            element = ET.SubElement(link, tag, name=value['name'] + '_' + tag)
            shape = ET.SubElement(ET.SubElement(element, 'geometry'), 'box')
            ET.SubElement(shape, 'size').text = size
            if tag == 'visual':
                material = ET.SubElement(element, 'material')
                for item in ('ambient', 'diffuse'):
                    ET.SubElement(material, item).text = ' '.join(map(str, value['color']))
    # 载体＝仅视觉模型（kinematic 位姿沉，不参与接触求解）：由 gazebo_3d_carrier
    # 以 SetModelState 直推，故无 collision、无驱动插件。
    carrier = ET.SubElement(world, 'model', name='ramp_carrier')
    ET.SubElement(carrier, 'static').text = 'true'
    clink = ET.SubElement(carrier, 'link', name='body')
    start = geo['carrier']['start']
    cx, cy = start[0], start[1]
    cz = start[2] + geo['carrier']['center_z']
    ET.SubElement(clink, 'pose').text = '%g %g %g 0 0 0' % (cx, cy, cz)
    cvis = ET.SubElement(clink, 'visual', name='carrier_visual')
    cshape = ET.SubElement(ET.SubElement(cvis, 'geometry'), 'box')
    cshape_size = ' '.join('%g' % v for v in geo['carrier']['visual_size'])
    ET.SubElement(cshape, 'size').text = cshape_size
    cmat = ET.SubElement(cvis, 'material')
    for item in ('ambient', 'diffuse'):
        ET.SubElement(cmat, item).text = '.95 .55 .15 1'
    from xml.dom import minidom
    return minidom.parseString(ET.tostring(sdf)).toprettyxml(indent='  ', encoding='UTF-8')


def artifact_bytes():
    """Complete deterministic output set. Check never creates directories/files."""
    geo, value = geometry(), scene_map()
    data, geo_data = json_bytes(value), json_bytes(geo)
    manifest = {'schema': 'ramp_artifacts/v1', 'source': 'scene.py',
                'scope': 'declared known-ramp Gazebo kinematic planning; slope is a '
                         'step-surface chain, not an inclined elevation grid',
                'sha256': {'maps/ramp.json': hashlib.sha256(data).hexdigest(),
                           'worlds/ramp.world': hashlib.sha256(world_xml()).hexdigest(),
                           'geometry/ramp.json': hashlib.sha256(geo_data).hexdigest()}}
    return {'maps/ramp.json': data,
            'worlds/ramp.world': world_xml(),
            'geometry/ramp.json': geo_data,
            'scene_manifest.json': json_bytes(manifest)}


def write_artifacts(destination):
    destination = Path(destination)
    for name, data in artifact_bytes().items():
        path = destination / name
        if not path.parent.exists():
            path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(data)
    return 0


def check_artifacts(destination=HERE):
    expected = artifact_bytes()
    return [name for name, data in expected.items()
            if not (Path(destination) / name).is_file() or (Path(destination) / name).read_bytes() != data]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--write', action='store_true', help='explicitly regenerate this scene only')
    parser.add_argument('--output', type=Path, default=HERE)
    args = parser.parse_args()
    if args.write:
        write_artifacts(args.output)
    different = check_artifacts(args.output)
    if different:
        print('MISMATCH: %s' % ', '.join(sorted(different)))
        return 1
    print('ramp_nav scene OK: %s' % ', '.join(sorted(artifact_bytes())))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
