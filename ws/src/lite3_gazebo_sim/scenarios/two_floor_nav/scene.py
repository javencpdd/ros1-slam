#!/usr/bin/env python3
"""Self-contained two-floor Gazebo scene: one geometry source, exact map cells."""
import argparse
import copy
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

sys.dont_write_bytecode = True
HERE = Path(__file__).resolve().parent
_P2_ROOT = HERE.parents[5]
_P1_ROOT = Path(os.environ.get('V8_REPO_ROOT') or (_P2_ROOT.parent / 'ros1-slam'))
if os.environ.get('V8_REPO_ROOT') is None and Path('/opt/ros1-slam/ws/src/90_platform/v8_platform').is_dir():
    _P1_ROOT = Path('/opt/ros1-slam')
ROOT = _P1_ROOT
PLATFORM = Path(os.environ.get('V8_PLATFORM_ROOT') or (ROOT / 'ws/src/90_platform/v8_platform'))
for folder in ('representation', 'planner', 'local_planner', 'contracts', 'framework'):
    sys.path.insert(0, str(PLATFORM / folder))
import support_map

RELATIVE = os.environ.get('V8_SIM_RELATIVE', 'sim_ws/src/lite3_gazebo_sim/scenarios/two_floor_nav')
RESOLUTION = .2
RADIUS = .30
HEIGHT = .50
CLEARANCE = .05
CASES = ('upstairs', 'downstairs', 'upper_detour', 'blocked')


def json_bytes(value):
    return (json.dumps(value, ensure_ascii=False, sort_keys=True, indent=2, allow_nan=False) + '\n').encode('utf-8')


def box(name, lo, hi, color=(.6, .65, .7, 1)):
    return {'name': name, 'min': list(lo), 'max': list(hi), 'color': list(color)}


def geometry(blocked=False):
    """All lengths are metres. No downloaded models, textures or ROS plugins."""
    boxes = [
        box('ground_floor', [-8, -6, -.2], [8, 6, 0], (.64, .66, .69, 1)),
        box('upper_floor', [-8, -6, 1.4], [-4, 6, 1.6], (.45, .54, .62, 1)),
        box('wall_north', [-8.2, 6, -.2], [8.2, 6.2, 3.4]),
        box('wall_south', [-8.2, -6.2, -.2], [8.2, -6, 3.4]),
        box('wall_east', [8, -6, -.2], [8.2, 6, 3.4]),
        box('wall_west', [-8.2, -6, -.2], [-8, 6, 3.4]),
        box('upper_rail_south', [-4.1, -6, 1.6], [-3.95, -.5, 2.5], (.2, .28, .4, 1)),
        box('upper_rail_north', [-4.1, 2.5, 1.6], [-3.95, 6, 2.5], (.2, .28, .4, 1)),
        box('ground_cabinet', [4.7, -4.8, 0], [6.7, -3.5, 1.2], (.3, .5, .35, 1)),
        box('ground_desk', [-2, -4.6, 0], [0, -3.4, .8], (.55, .35, .2, 1)),
        box('ground_column', [1, 3.7, 0], [1.6, 4.3, 2.8], (.5, .52, .55, 1)),
        box('upper_detour_box', [-6, -3.4, 1.6], [-5.5, -2.6, 2.5], (.85, .3, .15, 1)),
        box('upper_cabinet', [-7.5, 3.5, 1.6], [-6.5, 5, 2.6], (.32, .45, .65, 1)),
    ]
    for i in range(1, 11):
        boxes.append(box('step%02d' % i, [round(4 - .8 * i, 6), 0, 0],
                         [round(4 - .8 * (i - 1), 6), 2, round(.16 * i, 6)],
                         (.65, .46, .24, 1)))
    if blocked:
        # This fills the entire fifth tread and rises beyond the test envelope.
        boxes.append(box('stair_blocker', [0, -.1, .8], [.8, 2.1, 1.8], (.8, .15, .12, 1)))
    return {'schema': 'two_floor_geometry/v1', 'frame': 'map', 'world_frame': 'world',
            'map_to_world': [0, 0, 0, 0, 0, 0], 'boxes': boxes,
            'planning_envelope': {'radius_m': RADIUS, 'height_m': HEIGHT,
                                  'clearance_m': CLEARANCE, 'kind': 'generic_sim_experimental'},
            'carrier': {'kind': 'kinematic_pose_sink', 'visual_size': [.45, .30, .20],
                        'center_z': .30, 'collision_geometry': False}}


def rectangle_cells(bounds):
    x0, y0, x1, y1 = bounds
    return {(i, j) for i in range(round(x0 / RESOLUTION), round(x1 / RESOLUTION))
            for j in range(round(y0 / RESOLUTION), round(y1 / RESOLUTION))}


def scene_map(blocked=False):
    geo = geometry(blocked)
    by_name = {value['name']: value for value in geo['boxes']}
    surfaces = []
    names = ['ground_floor'] + ['step%02d' % i for i in range(1, 11)] + ['upper_floor']
    for name in names:
        item = by_name[name]
        lo, hi = item['min'], item['max']
        cells = rectangle_cells((lo[0], lo[1], hi[0], hi[1]))
        if name == 'ground_floor':
            cells -= rectangle_cells((-4, 0, 4, 2))
        surfaces.append({'surface_id': name, 'z_reference': hi[2],
                         'thickness_m': hi[2] - lo[2], 'cells': cells,
                         'footprint': {'min_x': lo[0], 'min_y': lo[1], 'max_x': hi[0], 'max_y': hi[1]},
                         'data_uri': 'scene://two_floor_nav/' + name,
                         'clearance_min_m': 1.2, 'edges': []})
    for left, right in zip(surfaces, surfaces[1:]):
        kind = 'flat' if abs(left['z_reference'] - right['z_reference']) < 1e-9 else 'step'
        edge = {'to': right['surface_id'], 'kind': kind, 'bidirectional': True}
        if kind == 'step': edge['max_step_height_m'] = .160001
        left['edges'].append(edge)
    obstacles = [{'obstacle_id': b['name'], 'min': b['min'], 'max': b['max']}
                 for b in geo['boxes'] if b['name'] not in names]
    return support_map.build_support_map('two_floor_blocked' if blocked else 'two_floor_nav',
            RESOLUTION, [-8, -6, 0], surfaces, [],
            {'uri': 'scene://two_floor_nav/geometry', 'sha256': hashlib.sha256(json_bytes(geo)).hexdigest()},
            obstacles=obstacles)


def world_xml(blocked=False):
    geo = geometry(blocked)
    sdf = ET.Element('sdf', version='1.6')
    world = ET.SubElement(sdf, 'world', name='two_floor_blocked' if blocked else 'two_floor_nav')
    ET.SubElement(world, 'gravity').text = '0 0 -9.81'
    physics = ET.SubElement(world, 'physics', name='default', type='ode')
    ET.SubElement(physics, 'max_step_size').text = '.001'
    ET.SubElement(physics, 'real_time_update_rate').text = '1000'
    light = ET.SubElement(world, 'light', name='sun', type='directional')
    ET.SubElement(light, 'pose').text = '0 0 10 0 0 0'
    ET.SubElement(light, 'diffuse').text = '.9 .9 .9 1'
    ET.SubElement(light, 'direction').text = '-.5 -.2 -.9'
    model = ET.SubElement(world, 'model', name='two_floor_geometry')
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
    # Stable formatting works on Noetic's Python 3.8 as well.
    from xml.dom import minidom
    return minidom.parseString(ET.tostring(sdf)).toprettyxml(indent='  ', encoding='UTF-8')


def task(case):
    points = {'upstairs': ([5, 1, 0], [-5.9, 1.1, 1.6]),
              'downstairs': ([-6, 1, 1.6], [5.1, 1.1, 0]),
              'upper_detour': ([-4.5, -3, 1.6], [-6.9, -2.9, 1.6]),
              'blocked': ([5, 1, 0], [-5.9, 1.1, 1.6])}
    start, goal = points[case]
    return {'schema_version': 1, 'task_id': 'two_floor_' + case,
            'map': {'map_id': 'two_floor_blocked' if case == 'blocked' else 'two_floor_nav', 'map_version': 1},
            'frame': 'map', 'goal_reference': 'support_surface',
            'start': {'xyz': start, 'yaw': 0},
            'goal': {'xyz': goal, 'yaw': 0,
                     'surface_hint': 'ground_floor' if case == 'downstairs' else 'upper_floor',
                     'surface_disambiguation': 'explicit_hint',
                     'tolerance': {'position_m': .20, 'yaw_rad': .2, 'source': 'operator'}},
            'timeout': {'budget_s': 180, 'harness_deadline_s': 240, 'source': 'operator'}}


def artifact_bytes():
    """Complete deterministic output set. Check never creates directories/files."""
    artifacts = {}
    registry = {'schema': 'map_registry/v1', 'maps': []}
    for blocked in (False, True):
        name = 'blocked' if blocked else 'open'
        value = scene_map(blocked); data = json_bytes(value)
        path = 'maps/' + name + '.json'; artifacts[path] = data
        artifacts['worlds/' + name + '.world'] = world_xml(blocked)
        artifacts['geometry/' + name + '.json'] = json_bytes(geometry(blocked))
        registry['maps'].append({'map_id': value['map_id'], 'version': 1,
                 'path': RELATIVE + '/' + path, 'frame': 'map', 'surfaces': len(value['surfaces']),
                 'sha256': hashlib.sha256(data).hexdigest(), 'label': 'declared exact scene geometry'})
    artifacts['registry.json'] = json_bytes(registry)
    import yaml
    base = yaml.safe_load((PLATFORM / 'framework/fixtures/graphs/graph_3d_gazebo_app_mapped.yaml').read_text())
    graph = copy.deepcopy(base); graph['name'] = 'two_floor_nav'
    for instance in graph['instances']:
        if instance['id'] == 'map': instance['params']['map_json'] = RELATIVE + '/maps/open.json'
        if instance['id'] == 'carrier': instance['params'].update(start_x=5., start_y=1., start_z=0., center_z=.30)
        if instance['id'] == 'local': instance['config'] = '../configs/local.yaml'
        if instance['id'] == 'global':
            instance['params'].update(budget=50000, min_clearance_m=HEIGHT + CLEARANCE,
                                       robot_radius_m=RADIUS + CLEARANCE, robot_height_m=HEIGHT)
    artifacts['graphs/navigation.yaml'] = json_bytes(graph)
    cfg = {'experimental_config': {'resolution': .4, 'tick_s': 1.2, 'horizon_s': 8.4,
             'validity_s': 20., 'max_speed_mps': .6, 'max_step_m': .2, 'max_slope_deg': 35.,
             'radius_m': RADIUS, 'height_m': HEIGHT, 'clearance_m': CLEARANCE,
             'goal_tolerance_m': .20, 'max_expansions': 20000},
           'input_max_age_s': 1., 'wall_watchdog_s': 90., 'sim_pose_clock_wait_s': .01,
           'sim_contact_egress': True,
           'map_topic': '/v8_support_map', 'pose_topic': '/v8/local/pose',
           'goal_topic': '/v8/local/goal', 'obstacles_topic': '/v8/local/obstacles',
           'localization_topic': '/v8/local/localization_ok', 'command_topic': '/v8/local/cmd_vel_3d',
           'trajectory_topic': '/v8/local/trajectory', 'status_topic': '/v8/local/status'}
    artifacts['configs/local.yaml'] = json_bytes(cfg)
    for case in CASES:
        artifacts['tasks/' + case + '.json'] = json_bytes(task(case))
        app = {'schema': 'navigation_app/v1', 'execution_mode': 'simulation', 'robot_id': 'generic_sim',
           'name': 'two_floor_' + case, 'graph': RELATIVE + '/graphs/navigation.yaml',
           'map_registry': RELATIVE + '/registry.json', 'map': {'id': task(case)['map']['map_id'], 'version': 1},
           'task': RELATIVE + '/tasks/' + case + '.json', 'localization': {'source': 'carrier'},
           'carrier': {'backend': 'gazebo', 'model_name': 'app_carrier', 'center_z': .30,
              'snap_m': .03, 'spawn': True,
              'world_launch': 'ws/src/95_simulation/lite3_gazebo_sim/launch/app_world.launch',
              'world': RELATIVE + '/worlds/' + ('blocked' if case == 'blocked' else 'open') + '.world',
              'model_path': RELATIVE,
              'simulation_declaration': 'ws/src/95_simulation/v8_simulation/config/sim_backends.yaml'},
           'visualization': {'markers': True, 'rviz': False},
           'global': {'budget': 50000, 'min_clearance_m': HEIGHT + CLEARANCE}}
        artifacts['apps/' + case + '.yaml'] = json_bytes(app)
    manifest = {'schema': 'two_floor_artifacts/v1', 'source': 'scene.py',
       'scope': 'declared known-map Gazebo kinematic planning; no physical gait or live mapping',
       'sha256': {name: hashlib.sha256(data).hexdigest() for name, data in sorted(artifacts.items())}}
    artifacts['scene_manifest.json'] = json_bytes(manifest)
    return artifacts


def acl(path, directory=False):
    if not __import__('shutil').which('setfacl'): return
    args = ['setfacl', '-m', 'u:ros1-slam:rwx', str(path)]
    subprocess.run(args, check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    if directory:
        subprocess.run(['setfacl', '-d', '-m', 'u:ros1-slam:rwx', str(path)], check=False,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def write_artifacts(destination):
    destination = Path(destination)
    for name, data in artifact_bytes().items():
        path = destination / name
        if not path.parent.exists():
            path.parent.mkdir(parents=True, exist_ok=True); acl(path.parent, True)
        path.write_bytes(data); acl(path)


def check_artifacts(destination=HERE):
    expected = artifact_bytes()
    return [name for name, data in expected.items()
            if not (Path(destination) / name).is_file() or (Path(destination) / name).read_bytes() != data]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--write', action='store_true', help='explicitly regenerate this scene only')
    parser.add_argument('--output', type=Path, default=HERE)
    args = parser.parse_args()
    if args.write: write_artifacts(args.output)
    different = check_artifacts(args.output)
    print(json.dumps({'ok': not different, 'changed_or_missing': different}, ensure_ascii=False))
    return 1 if different else 0


if __name__ == '__main__':
    raise SystemExit(main())
