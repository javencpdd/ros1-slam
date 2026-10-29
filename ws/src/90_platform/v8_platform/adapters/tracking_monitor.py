"""SC11 tracking monitor: reference vs actual pose -> deviation -> rejection chain.

The monitor is a pure state machine plus a thin ROS1 wrapper. A lateral,
longitudinal or vertical deviation over tolerance rejects the sample; two
consecutive rejections latch a stop. The stop is published as a Bool hold and a
status record, never as a fabricated planar command.
"""
import json
import math
import sys
from pathlib import Path

BASE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(BASE / 'validator'))
import enforcement as enf  # noqa: E402
import tracking as trk  # noqa: E402

REASON = 'E_TRACK_DEVIATION'


class TrackingMonitor:
    def __init__(self, reference, limit=2, tol_lateral=.15, tol_longitudinal=.20,
                 tol_vertical=.10):
        if not reference:
            raise ValueError('reference must not be empty')
        self.reference = [tuple(point) for point in reference]
        self.tolerances = dict(tol_lateral=tol_lateral, tol_longitudinal=tol_longitudinal,
                               tol_vertical=tol_vertical)
        self.tracker = enf.RejectionTracker(limit=limit)
        self.stopped = False

    def observe(self, pose):
        if self.stopped:
            return self._result('E_TRACK_STOP_LATCHED', [], 'stop', True)
        violations = trk.check_tracking(self.reference, pose, **self.tolerances)
        if not violations:
            self.tracker.record({'ok': True})
            return self._result('track', [], 'accept', False)
        verdict = {'ok': False, 'violations': violations}
        action = enf.on_rejection(verdict)
        outcome = self.tracker.record(verdict)
        stop = action['action'] == 'stop_chain' and outcome == 'stop'
        self.stopped = stop
        return self._result(REASON if stop else 'track_replan', violations, outcome, stop,
                            action=action['action'])

    def _result(self, reason, violations, tracker, stop, action=None):
        return {'ok': not stop and reason not in ('track_replan', 'E_TRACK_STOP_LATCHED'),
                'stop': stop, 'reason': reason, 'tracker': tracker,
                'consecutive': self.tracker.consecutive,
                'violations': [list(v) for v in violations], 'action': action}


def main():
    import rospy
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import Bool, String

    rospy.init_node('v8_tracking_monitor')
    monitor = {'value': None}
    hold = rospy.Publisher(rospy.get_param('~hold_topic', '/a7_tracking/hold'),
                           Bool, queue_size=1, latch=True)
    status = rospy.Publisher(rospy.get_param('~status_topic', '/a7_tracking/status'),
                             String, queue_size=20, latch=True)

    def on_reference(message):
        try:
            points = json.loads(message.data)
            monitor['value'] = TrackingMonitor(
                points, limit=int(rospy.get_param('~rejection_limit', 2)))
            hold.publish(Bool(data=False))
            status.publish(String(data=json.dumps({'ok': True, 'stop': False, 'reason': 'reference_ready',
                                                   'points': len(points)})))
        except (ValueError, TypeError, json.JSONDecodeError) as error:
            status.publish(String(data=json.dumps({'ok': False, 'stop': True,
                                                   'reason': 'E_TRACK_REFERENCE', 'detail': str(error)})))
            hold.publish(Bool(data=True))

    def on_pose(message):
        current = monitor['value']
        if current is None:
            status.publish(String(data=json.dumps({'ok': False, 'stop': True,
                                                   'reason': 'E_TRACK_NO_REFERENCE'})))
            return
        pose = (message.pose.position.x, message.pose.position.y, message.pose.position.z)
        result = current.observe(pose)
        hold.publish(Bool(data=result['stop']))
        status.publish(String(data=json.dumps(dict(result, pose=list(pose)))))
        if result['stop']:
            rospy.logwarn('tracking stop latched: %s', result['reason'])

    rospy.Subscriber(rospy.get_param('~reference_topic', '/a7_tracking/reference'),
                     String, on_reference, queue_size=1)
    rospy.Subscriber(rospy.get_param('~pose_topic', '/a7_tracking/pose'),
                     PoseStamped, on_pose, queue_size=20)
    rospy.spin()
    hold.publish(Bool(data=False))


if __name__ == '__main__':
    main()
