#!/usr/bin/env python3
"""Publish a support_map JSON file on a latched String topic (app map server)."""
import os as _os
import sys as _sys
_sys.path.insert(0, _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__))))
import hashlib
import json
import time

import rospy
from std_msgs.msg import String


def _repo_root():
    """向上找带 `data/maps` 与 `.git` 的目录作为仓库根。

    不能按 `__file__` 的层数硬数：节点既可能从 `ws/src/...` 运行，也可能从
    `ws/.profiles/<t>/src/...` 的 profile 视图运行（层数不同）。标记法两者都对。
    """
    d = _os.path.dirname(_os.path.abspath(__file__))
    while True:
        if (_os.path.isdir(_os.path.join(d, 'data', 'maps'))
                and _os.path.isdir(_os.path.join(d, '.git'))):
            return d
        parent = _os.path.dirname(d)
        if parent == d:
            return _os.path.dirname(_os.path.dirname(_os.path.dirname(d)))
        d = parent


REPO_ROOT = _repo_root()


def resolve_map_path(value):
    """把 ~map_json 解析成真实文件路径；相对路径按**仓库根**解析，不依赖 cwd。

    2026-11-23（F5）：graph 里写的是 `data/maps/generated/nist_sensor_v2.json`（仓库相对），
    而节点 cwd 是 `.ros`，`open()` 直接 FileNotFoundError。该资产属仓库 data/，不在任何
    catkin 包内，因此不能用 `$(find pkg)`；也不写死开发机绝对路径。
    缺失时抛 FileNotFoundError 并带上原始值与尝试过的绝对路径（明确失败，不静默兜底）。
    """
    path = str(value or '')
    if _os.path.isabs(path):
        if _os.path.isfile(path):
            return path
        raise FileNotFoundError('map_json not found: %s' % path)
    tried = _os.path.join(REPO_ROOT, path)
    if _os.path.isfile(tried):
        return tried
    if _os.path.isfile(path):
        return path
    raise FileNotFoundError('map_json not found: %r (tried %r)' % (path, tried))


def main():
    rospy.init_node('v8_map_file')
    map_path = resolve_map_path(rospy.get_param('~map_json'))
    map_topic = rospy.get_param('~map_topic', '/v8_support_map')
    status_topic = rospy.get_param('~status_topic', '/v8_map_status')
    with open(map_path, encoding='utf-8') as stream:
        value = json.load(stream)
    if value.get('kind') != 'support_map' or not value.get('surfaces'):
        raise SystemExit('map_json is not a support_map: %s' % map_path)
    payload = json.dumps(value, ensure_ascii=False, separators=(',', ':'))
    sha = hashlib.sha256(payload.encode()).hexdigest()
    publisher = rospy.Publisher(map_topic, String, queue_size=1, latch=True)
    status = rospy.Publisher(status_topic, String, queue_size=1, latch=True)
    time.sleep(.3)
    publisher.publish(String(data=payload))
    status.publish(String(data=json.dumps({
        'ok': True, 'map_id': value.get('map_id'), 'map_version': value.get('map_version'),
        'surfaces': len(value['surfaces']), 'sha256': sha, 'map_json': map_path,
        'topic': map_topic}, ensure_ascii=False)))
    print('READY', flush=True)
    rate = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        status.publish(String(data=json.dumps({
            'ok': True, 'map_id': value.get('map_id'), 'map_version': value.get('map_version'),
            'surfaces': len(value['surfaces']), 'sha256': sha, 'topic': map_topic},
            ensure_ascii=False)))
        rate.sleep()


if __name__ == '__main__':
    main()
