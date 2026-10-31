#!/usr/bin/env python3
"""ROS map merger: combine two typed MapDelta streams with later-wins semantics."""
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import map_cache  # noqa: E402
import ros_map_codec  # noqa: E402


def main():
    import rospy
    from std_msgs.msg import String
    from v8_msgs.msg import MapDelta

    rospy.init_node('v8_map_merge')
    topic_a = rospy.get_param('~delta_a', '/v8/delta_a')
    topic_b = rospy.get_param('~delta_b', '/v8/delta_b')
    out_topic = rospy.get_param('~delta_topic', '/map_delta')
    status_topic = rospy.get_param('~status_topic', '/v8/merge/status')
    latest = {'a': None, 'b': None, 'keys': None}
    publisher = rospy.Publisher(out_topic, MapDelta, queue_size=10)
    status_pub = rospy.Publisher(status_topic, String, queue_size=10, latch=True)

    def publish_status(error=None):
        status_pub.publish(String(data=json.dumps(
            {'a': None if latest['a'] is None else latest['a']['delta_id'],
             'b': None if latest['b'] is None else latest['b']['delta_id'],
             'error': error}, ensure_ascii=False)))

    def merge():
        if latest['a'] is None or latest['b'] is None:
            return
        keys = (latest['a']['delta_id'], latest['b']['delta_id'])
        if latest['keys'] == keys:
            return
        try:
            value = map_cache.merge_deltas([latest['a'], latest['b']])
            message = ros_map_codec.delta_msg(value)
            message.header.stamp = rospy.Time.now()
            publisher.publish(message)
            latest['keys'] = keys
            publish_status()
        except Exception as error:
            publish_status(getattr(error, 'code', 'E_MERGE'))

    def on_a(message):
        latest['a'] = ros_map_codec.delta_value(message)
        merge()

    def on_b(message):
        latest['b'] = ros_map_codec.delta_value(message)
        merge()

    rospy.Subscriber(topic_a, MapDelta, on_a, queue_size=10)
    rospy.Subscriber(topic_b, MapDelta, on_b, queue_size=10)
    publish_status()
    rospy.spin()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
