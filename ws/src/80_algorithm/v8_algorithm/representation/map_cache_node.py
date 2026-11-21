#!/usr/bin/env python3
"""ROS map cache: buffer JSON surface ops -> typed MapDelta, explicit flush/ack."""
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import map_cache  # noqa: E402
import ros_map_codec  # noqa: E402


def main():
    import rospy
    from std_msgs.msg import String, UInt32
    from std_srvs.srv import Trigger, TriggerResponse
    from v8_msgs.msg import MapDelta, SupportMap

    rospy.init_node('v8_map_cache')
    map_id = rospy.get_param('~map_id')
    base_version = int(rospy.get_param('~base_version', 1))
    ops_topic = rospy.get_param('~ops_topic', '/v8/cache/ops')
    delta_topic = rospy.get_param('~delta_topic', '/map_delta')
    ack_topic = rospy.get_param('~ack_topic', '/v8/cache/ack')
    status_topic = rospy.get_param('~status_topic', '/v8/cache/status')
    source_uri = rospy.get_param('~source_uri', 'cache://node')
    base_map_topic = rospy.get_param('~base_map_topic', '')
    cache = map_cache.MapCache(map_id, base_version)
    state = {'errors': [], 'last_delta': None, 'sequence': 0}
    delta_pub = rospy.Publisher(delta_topic, MapDelta, queue_size=10)
    status_pub = rospy.Publisher(status_topic, String, queue_size=10, latch=True)

    def publish_status():
        status_pub.publish(String(data=json.dumps(
            {'map_id': map_id, 'base_version': cache.base_version,
             'pending': len(cache.pending()), 'last_delta': state['last_delta'],
             'errors': state['errors'][-10:]}, ensure_ascii=False)))

    def on_ops(message):
        try:
            payload = json.loads(message.data)
            if payload.get('op') == 'upsert':
                cache.upsert(payload['surface'])
            elif payload.get('op') == 'remove':
                cache.remove(payload['surface_id'])
            else:
                raise map_cache.MapCacheError('E_CACHE_OP', payload.get('op'))
        except Exception as error:
            state['errors'].append(getattr(error, 'code', 'E_CACHE_JSON'))
            state['errors'] = state['errors'][-10:]
            publish_status()

    def flush(_request):
        try:
            state['sequence'] += 1
            delta_id = 'cache-%s-%d-%d' % (map_id, cache.base_version,
                                           state['sequence'])
            value = cache.make_delta(delta_id, source_uri)
            message = ros_map_codec.delta_msg(value)
            message.header.stamp = rospy.Time.now()
            delta_pub.publish(message)
            state['last_delta'] = delta_id
            publish_status()
            return TriggerResponse(success=True,
                                   message=json.dumps(value, ensure_ascii=False))
        except Exception as error:
            return TriggerResponse(success=False, message=str(error))

    def on_ack(message):
        try:
            cache.ack(int(message.data))
            publish_status()
        except Exception as error:
            state['errors'].append(getattr(error, 'code', 'E_CACHE_ACK'))
            state['errors'] = state['errors'][-10:]
            publish_status()

    def on_base_map(message):
        try:
            for surface in ros_map_codec.surfaces_from_support_map(message):
                cache.upsert(surface)
            publish_status()
        except Exception as error:
            state['errors'].append(getattr(error, 'code', 'E_CACHE_BASE_MAP'))
            state['errors'] = state['errors'][-10:]
            publish_status()

    rospy.Subscriber(ops_topic, String, on_ops, queue_size=20)
    rospy.Subscriber(ack_topic, UInt32, on_ack, queue_size=10)
    if base_map_topic:
        rospy.Subscriber(base_map_topic, SupportMap, on_base_map, queue_size=5)
    rospy.Service('~flush', Trigger, flush)
    publish_status()
    rospy.spin()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
