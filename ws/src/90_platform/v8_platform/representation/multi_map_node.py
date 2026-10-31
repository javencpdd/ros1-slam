"""ROS1 multi-map provider: route /map_delta by map_id.

Each map gets its own latched topic and Trigger query service. State is persisted
through MapRouter/MapStore; the node does not change map or delta contracts.
"""
import json
import sys
import threading
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import map_router  # noqa: E402


class MultiMapCore:
    def __init__(self, state_dir, maps):
        self.router = map_router.MapRouter(state_dir, maps)
        self.errors = []
        self.lock = threading.Lock()

    def apply_text(self, text):
        with self.lock:
            try:
                version = self.router.apply_text(text)
            except Exception as error:  # map_service errors are fail-closed
                code = getattr(error, 'code', 'E_DELTA_JSON')
                self.errors.append(code)
                self.errors = self.errors[-10:]
                return {'ok': False, 'reason': code, 'detail': str(error)}
            return {'ok': True, 'map_version': version}

    def snapshot(self, map_id):
        with self.lock:
            return self.router.snapshot(map_id)

    def status(self):
        with self.lock:
            return {'maps': self.router.status(), 'errors': list(self.errors)}


def _load_files(value):
    if isinstance(value, str):
        paths = [item.strip() for item in value.split(',') if item.strip()]
    else:
        paths = list(value)
    return [json.loads(Path(path).read_text(encoding='utf-8')) for path in paths]


def main():
    import rospy
    from std_msgs.msg import String
    from std_srvs.srv import Trigger, TriggerResponse

    rospy.init_node('v8_multi_map')
    state_dir = rospy.get_param('~state_dir')
    maps = _load_files(rospy.get_param('~map_files', []))
    if not maps:
        raise ValueError('map_files must contain at least one map')
    core = MultiMapCore(state_dir, maps)
    prefix = rospy.get_param('~topic_prefix', '/v8/maps')
    status_topic = rospy.get_param('~status_topic', prefix + '/status')
    delta_topic = rospy.get_param('~delta_topic', '/map_delta')
    publishers = {map_id: rospy.Publisher('%s/%s' % (prefix, map_id), String,
                                         queue_size=1, latch=True)
                  for map_id in core.router.ids()}
    status_pub = rospy.Publisher(status_topic, String, queue_size=10, latch=True)

    def publish_all():
        for map_id, publisher in publishers.items():
            publisher.publish(String(data=json.dumps(core.snapshot(map_id),
                                                      ensure_ascii=False)))
        status_pub.publish(String(data=json.dumps(core.status(), ensure_ascii=False)))

    def on_delta(message):
        result = core.apply_text(message.data)
        if result['ok']:
            publish_all()
        else:
            status_pub.publish(String(data=json.dumps(
                dict(core.status(), last_error=result), ensure_ascii=False)))

    rospy.Subscriber(delta_topic, String, on_delta, queue_size=20)

    for map_id in core.router.ids():
        def query(_request, map_id=map_id):
            try:
                return TriggerResponse(success=True,
                                       message=json.dumps(core.snapshot(map_id),
                                                          ensure_ascii=False))
            except Exception as error:
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
