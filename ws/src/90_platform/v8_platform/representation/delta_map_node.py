#!/usr/bin/env python3
"""ROS1 map provider consuming map_delta/v1 (runtime update stream).

The pure core owns a MapService; the node subscribes to std_msgs/String deltas,
applies them atomically, and publishes the versioned map/status. No ROS import
in the core, so delta semantics stay testable without a master.
"""
import json
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / 'contracts'))
import map_service as svc  # noqa: E402


class DeltaNodeCore:
    def __init__(self, map_value, state_path=None):
        self.state_path = Path(state_path) if state_path else None
        self.restored = bool(self.state_path and self.state_path.is_file())
        if self.restored:
            self.service = svc.MapService.load(self.state_path,
                                               map_id=map_value['map_id'])
        else:
            self.service = svc.MapService(map_value['map_id'])
            self.service.publish(map_value)
        self.errors = []
        self.persist_errors = []
        if not self.restored:
            self._persist()

    def _persist(self):
        if self.state_path is None:
            return {'ok': True, 'skipped': True}
        try:
            saved = self.service.save(self.state_path)
        except OSError as error:
            self.persist_errors.append(str(error))
            self.persist_errors = self.persist_errors[-10:]
            return {'ok': False, 'reason': 'E_MAPSERVICE_PERSIST',
                    'detail': str(error)}
        return {'ok': True, 'path': saved['path'], 'sha256': saved['sha256'],
                'map_version': saved['map_version']}

    def apply_text(self, text):
        try:
            delta = json.loads(text)
            version = self.service.propose_delta(delta)
        except (ValueError, svc.MapServiceError) as error:
            self.errors.append(getattr(error, 'code', 'E_DELTA_JSON'))
            return {'ok': False, 'reason': getattr(error, 'code', 'E_DELTA_JSON'),
                    'detail': str(error)}
        persisted = self._persist()
        return {'ok': True, 'map_id': self.service.map_id, 'map_version': version,
                'digest': svc.content_digest(self.service.current, self.service.evidence),
                'persisted': persisted['ok'], 'persist': persisted}

    def snapshot(self):
        return {'ok': True, 'map_id': self.service.map_id,
                'map_version': self.service.current['map_version'],
                'digest': svc.content_digest(self.service.current, self.service.evidence),
                'map': self.service.current,
                'evidence': self.service.evidence_summary(),
                'restored': self.restored,
                'state_path': None if self.state_path is None else str(self.state_path)}

    def status(self):
        return {'map_id': self.service.map_id,
                'map_version': self.service.current['map_version'],
                'history': [list(row) for row in self.service.history],
                'applied_deltas': len(self.service.applied_deltas),
                'errors': self.errors[-10:],
                'persist_errors': self.persist_errors[-10:],
                'restored': self.restored,
                'state_path': None if self.state_path is None else str(self.state_path),
                'evidence': self.service.evidence_summary()}


def main():
    import rospy
    from std_msgs.msg import String
    from std_srvs.srv import Trigger, TriggerResponse

    rospy.init_node('v8_delta_map')
    with open(rospy.get_param('~map_json'), encoding='utf-8') as stream:
        map_value = json.load(stream)
    state_path = rospy.get_param('~state_path', '')
    core = DeltaNodeCore(map_value, state_path or None)
    rospy.loginfo('v8_delta_map version=%s restored=%s state=%s',
                  core.service.current['map_version'], core.restored,
                  core.state_path or '-')
    map_topic = rospy.get_param('~map_topic', '/v8_support_map')
    status_topic = rospy.get_param('~status_topic', '/v8_map_status')
    delta_topic = rospy.get_param('~delta_topic', '/map_delta')
    map_pub = rospy.Publisher(map_topic, String, queue_size=1, latch=True)
    status_pub = rospy.Publisher(status_topic, String, queue_size=10, latch=True)

    def publish():
        map_pub.publish(String(data=json.dumps(core.snapshot(), ensure_ascii=False)))
        status_pub.publish(String(data=json.dumps(core.status(), ensure_ascii=False)))

    def on_delta(message):
        result = core.apply_text(message.data)
        if result['ok']:
            publish()
        else:
            status_pub.publish(String(data=json.dumps(
                dict(core.status(), last_error=result), ensure_ascii=False)))

    rospy.Subscriber(rospy.get_param('~delta_topic', delta_topic), String,
                     on_delta, queue_size=20)

    def query(_request):
        return TriggerResponse(success=True,
                               message=json.dumps(core.snapshot(), ensure_ascii=False))

    rospy.Service(rospy.get_param('~query_service', rospy.get_name() + '/query'),
                  Trigger, query)
    publish()
    deadline = time.monotonic() + float(rospy.get_param('~keep_alive_s', 0.0))
    while not rospy.is_shutdown():
        if rospy.get_param('~keep_alive_s', 0.0) > 0 and time.monotonic() >= deadline:
            break
        time.sleep(.05)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
