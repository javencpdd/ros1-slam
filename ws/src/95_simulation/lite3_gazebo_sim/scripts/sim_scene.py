#!/usr/bin/env python3
"""在运行中的 Gazebo 里热插拔 3D 场景模型（不重启世界）。

用法：
  python3 sim_scene.py --list
  python3 sim_scene.py --scene stairs_easy      # 接受 stairs_easy / stair_lab_stairs_easy
  python3 sim_scene.py --scene wall_h045
  python3 sim_scene.py --clear                  # 删除本工具放过场的全部模型
  python3 sim_scene.py --status                 # 世界模型名 + 本工具记账

边界：
- 只动世界件里的 <model>，不动 ground_plane/sun、不动机器人、不重启 gzserver。
- 导航静态地图不会随场景更新；切换后直接跑正式导航判据＝无效。需要正式导航时重启对应世界。
- 场景模型名必须唯一；切换前先删上一套（本工具按记账文件删）。
"""
import argparse
import json
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

WORLDS = Path(__file__).resolve().parent.parent / 'stairstudy' / 'worlds'
PREFIX = 'stair_lab_'
DEFAULT_STATE = Path('/tmp/sim_scene_state.json')


def scene_names():
    names = []
    for path in sorted(WORLDS.glob(PREFIX + '*.world')):
        names.append(path.stem[len(PREFIX):])
    return names


def resolve_scene(name):
    if not isinstance(name, str) or not name.strip():
        raise ValueError('scene name must be a non-empty string')
    raw = name.strip()
    if raw.endswith('.world'):
        path = Path(raw)
        path = path if path.is_absolute() else WORLDS / path
    else:
        short = raw[len(PREFIX):] if raw.startswith(PREFIX) else raw
        path = WORLDS / (PREFIX + short + '.world')
    if not path.is_file():
        raise ValueError('unknown scene %r (have: %s)' % (name, ', '.join(scene_names())))
    return path


def pose_values(text):
    values = [float(v) for v in text.replace(',', ' ').split()]
    if len(values) != 6 or not all(math.isfinite(v) for v in values):
        raise ValueError('pose must be six finite numbers: %r' % text)
    return values


def rpy_to_quaternion(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def models_from_world(path):
    root = ET.parse(str(path)).getroot()
    world = root.find('world')
    if world is None:
        raise ValueError('no <world> in %s' % path)
    models = []
    for model in world.findall('model'):
        name = model.get('name')
        if not name:
            raise ValueError('model without name in %s' % path)
        pose_elem = model.find('pose')
        pose = pose_values(pose_elem.text) if pose_elem is not None else [0.] * 6
        if pose_elem is not None:
            model.remove(pose_elem)
        models.append({'name': name, 'pose': pose,
                       'xml': '<sdf version="1.6">%s</sdf>'
                              % ET.tostring(model, encoding='unicode')})
    return models


def load_state(path):
    try:
        data = json.loads(Path(path).read_text(encoding='utf-8'))
    except (OSError, ValueError):
        return {'scene': None, 'models': []}
    return {'scene': data.get('scene'), 'models': list(data.get('models') or [])}


def save_state(path, scene, models):
    Path(path).write_text(json.dumps({'scene': scene, 'models': models},
                                     ensure_ascii=False, indent=2) + '\n',
                          encoding='utf-8')


def _pose_msg(values):
    from geometry_msgs.msg import Point, Pose, Quaternion
    x, y, z, roll, pitch, yaw = values
    qx, qy, qz, qw = rpy_to_quaternion(roll, pitch, yaw)
    return Pose(position=Point(x, y, z), orientation=Quaternion(qx, qy, qz, qw))


def delete_models(names):
    import rospy
    from gazebo_msgs.srv import DeleteModel
    deleted = []
    if not names:
        return deleted
    rospy.wait_for_service('/gazebo/delete_model', timeout=15)
    proxy = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
    for name in names:
        try:
            response = proxy(name)
            deleted.append({'model': name, 'success': bool(response.success),
                            'status': response.status_message})
        except Exception as error:  # noqa: BLE001 - deletion is best effort
            deleted.append({'model': name, 'success': False, 'status': str(error)})
    return deleted


def spawn_models(models):
    import rospy
    from gazebo_msgs.srv import SpawnModel
    rospy.wait_for_service('/gazebo/spawn_sdf_model', timeout=15)
    proxy = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    spawned = []
    for model in models:
        response = proxy(model['name'], model['xml'], '',
                         _pose_msg(model['pose']), 'world')
        if not response.success:
            raise RuntimeError('spawn %s refused: %s' % (model['name'], response.status_message))
        spawned.append(model['name'])
    return spawned


def world_models():
    import rospy
    from gazebo_msgs.srv import GetWorldProperties
    rospy.wait_for_service('/gazebo/get_world_properties', timeout=15)
    response = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)()
    return list(response.model_names)


def switch(scene, state_path):
    import rospy
    path = resolve_scene(scene)
    before = load_state(state_path)
    deleted = delete_models(before['models'])
    models = models_from_world(path)
    spawned = spawn_models(models)
    save_state(state_path, path.stem[len(PREFIX):], spawned)
    return {'scene': path.stem[len(PREFIX):], 'deleted': deleted,
            'spawned': spawned, 'models': world_models()}


def clear(state_path):
    before = load_state(state_path)
    deleted = delete_models(before['models'])
    save_state(state_path, None, [])
    return {'deleted': deleted, 'models': world_models()}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene')
    parser.add_argument('--clear', action='store_true')
    parser.add_argument('--status', action='store_true')
    parser.add_argument('--list', action='store_true')
    parser.add_argument('--state', default=str(DEFAULT_STATE))
    args = parser.parse_args()
    if args.list:
        print(json.dumps({'scenes': scene_names()}, ensure_ascii=False))
        return 0
    if not (args.scene or args.clear or args.status):
        parser.error('use --scene NAME / --clear / --status / --list')

    import rospy
    rospy.init_node('sim_scene', anonymous=True, disable_signals=True)
    if args.scene:
        result = switch(args.scene, args.state)
    elif args.clear:
        result = clear(args.state)
    else:
        result = {'tracked': load_state(args.state), 'models': world_models()}
    print(json.dumps(result, ensure_ascii=False))
    return 0


if __name__ == '__main__':
    sys.exit(main())
