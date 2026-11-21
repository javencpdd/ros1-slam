#!/usr/bin/env python3
"""ROS1 read-only semantic map provider (semantic_map/v1).

Publishes one latched topic and one Trigger query service per map. The node is
read-only: it validates files at startup and never mutates the catalog.
"""
import json
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import semantic_map  # noqa: E402


def _paths(value):
    if isinstance(value, str):
        return [item.strip() for item in value.split(',') if item.strip()]
    return list(value)


def main():
    import rospy
    from std_msgs.msg import String
    from std_srvs.srv import Trigger, TriggerResponse

    rospy.init_node('v8_semantic_map')
    paths = _paths(rospy.get_param('~semantic_files', []))
    if not paths:
        raise ValueError('semantic_files must contain at least one map')
    catalog = semantic_map.SemanticCatalog().load_files(paths)
    prefix = rospy.get_param('~topic_prefix', '/v8/semantics')
    status_topic = rospy.get_param('~status_topic', prefix + '/status')
    publishers = {map_id: rospy.Publisher('%s/%s' % (prefix, map_id), String,
                                          queue_size=1, latch=True)
                  for map_id in catalog.ids()}
    status_pub = rospy.Publisher(status_topic, String, queue_size=10, latch=True)

    def publish_all():
        for map_id, publisher in publishers.items():
            publisher.publish(String(data=json.dumps(catalog.get(map_id),
                                                     ensure_ascii=False)))
        status_pub.publish(String(data=json.dumps(catalog.status(), ensure_ascii=False)))

    for map_id in catalog.ids():
        def query(_request, map_id=map_id):
            try:
                return TriggerResponse(success=True,
                                       message=json.dumps(catalog.get(map_id),
                                                          ensure_ascii=False))
            except semantic_map.SemanticMapError as error:
                return TriggerResponse(success=False, message=str(error))
        rospy.Service('%s/%s/query' % (prefix, map_id), Trigger, query)

    publish_all()
    keep_alive_s = float(rospy.get_param('~keep_alive_s', 0.0))
    deadline = time.monotonic() + keep_alive_s
    while not rospy.is_shutdown():
        if keep_alive_s > 0 and time.monotonic() >= deadline:
            break
        time.sleep(.05)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
