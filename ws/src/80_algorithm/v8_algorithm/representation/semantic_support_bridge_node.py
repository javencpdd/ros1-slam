#!/usr/bin/env python3
"""ROS publisher that turns semantic support surfaces into support_map/v1."""
import json
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import semantic_support_bridge  # noqa: E402


def main():
    import rospy
    from std_msgs.msg import String

    rospy.init_node('v8_semantic_bridge')
    semantic_topic = rospy.get_param('~semantic_topic', '')
    if not semantic_topic:
        raise ValueError('semantic_topic is required')
    support_topic = rospy.get_param('~support_map_topic', '/v8/support_map_sem')
    status_topic = rospy.get_param('~status_topic', support_topic + '/status')
    resolution = float(rospy.get_param('~resolution', 0.1))
    map_pub = rospy.Publisher(support_topic, String, queue_size=1, latch=True)
    status_pub = rospy.Publisher(status_topic, String, queue_size=10, latch=True)

    def publish_status(value):
        status_pub.publish(String(data=json.dumps(value, ensure_ascii=False)))

    def on_semantic(message):
        try:
            semantic = json.loads(message.data)
            converted = semantic_support_bridge.convert(semantic, resolution=resolution)
        except (ValueError, semantic_support_bridge.SemanticBridgeError) as error:
            publish_status({'ok': False,
                            'error': getattr(error, 'code', 'E_SEMANTIC_BRIDGE_JSON'),
                            'detail': str(error)})
            return
        map_pub.publish(String(data=json.dumps(converted, ensure_ascii=False)))
        publish_status({'ok': True, 'map_id': converted['map_id'],
                        'map_version': converted['map_version'],
                        'resolution': converted['resolution'],
                        'surfaces': [item['surface_id'] for item in converted['surfaces']]})

    rospy.Subscriber(semantic_topic, String, on_semantic, queue_size=1)
    publish_status({'ok': True, 'waiting': semantic_topic})
    keep_alive_s = float(rospy.get_param('~keep_alive_s', 0.0))
    deadline = time.monotonic() + keep_alive_s
    while not rospy.is_shutdown():
        if keep_alive_s > 0 and time.monotonic() >= deadline:
            break
        time.sleep(.05)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
