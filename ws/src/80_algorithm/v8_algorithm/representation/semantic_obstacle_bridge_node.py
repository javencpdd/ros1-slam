#!/usr/bin/env python3
"""ROS publisher that turns semantic obstacle boxes into planner obstacle JSON."""
import json
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import semantic_obstacle_bridge  # noqa: E402


def main():
    import rospy
    from std_msgs.msg import String

    rospy.init_node('v8_semantic_obstacles')
    semantic_topic = rospy.get_param('~semantic_topic', '')
    if not semantic_topic:
        raise ValueError('semantic_topic is required')
    obstacles_topic = rospy.get_param('~obstacles_topic', '/v8/local/obstacles')
    status_topic = rospy.get_param('~status_topic', obstacles_topic + '/status')
    publisher = rospy.Publisher(obstacles_topic, String, queue_size=1, latch=True)
    status_pub = rospy.Publisher(status_topic, String, queue_size=10, latch=True)

    def publish_status(value):
        status_pub.publish(String(data=json.dumps(value, ensure_ascii=False)))

    def on_semantic(message):
        try:
            value = json.loads(message.data)
            obstacles = semantic_obstacle_bridge.convert(value)
        except (ValueError, semantic_obstacle_bridge.SemanticObstacleError) as error:
            publish_status({'ok': False,
                            'error': getattr(error, 'code', 'E_SEMANTIC_OBSTACLE_JSON'),
                            'detail': str(error)})
            return
        publisher.publish(String(data=json.dumps(obstacles, ensure_ascii=False)))
        publish_status({'ok': True, 'count': len(obstacles),
                        'ids': [item['id'] for item in obstacles],
                        'map_version': value.get('map_version')})

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
