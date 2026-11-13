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


def main():
    rospy.init_node('v8_map_file')
    map_path = rospy.get_param('~map_json')
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
