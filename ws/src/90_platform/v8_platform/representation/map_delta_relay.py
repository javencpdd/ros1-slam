#!/usr/bin/env python3
"""Relay v8_msgs/MapDelta to the JSON map_delta/v1 string bus."""
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import ros_map_codec  # noqa: E402


def main():
    import rospy
    from std_msgs.msg import String
    from v8_msgs.msg import MapDelta

    rospy.init_node('v8_map_delta_relay')
    delta_topic = rospy.get_param('~delta_topic', '/v8/cache/delta')
    json_topic = rospy.get_param('~json_topic', '/map_delta')
    publisher = rospy.Publisher(json_topic, String, queue_size=10)
    count = {'value': 0}

    def on_delta(message):
        value = ros_map_codec.delta_value(message)
        publisher.publish(String(data=json.dumps(value, ensure_ascii=False)))
        count['value'] += 1

    rospy.Subscriber(delta_topic, MapDelta, on_delta, queue_size=20)
    rospy.spin()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
