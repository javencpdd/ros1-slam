"""Directed tests for the shadow localization-initialization readiness node.

Uses a minimal ROS stub: no roscore, no real robot, no topic writes outside
the single diagnostic topic.
"""
import sys as _sys
from pathlib import Path as _Path

_BASE = _Path(__file__).resolve().parent.parent
for _entry in ('', 'nodes', 'tools', 'smokes'):
    _sys.path.insert(0, str(_BASE / _entry) if _entry else str(_BASE))

import json
import unittest

import nav_init_readiness_node as n
import vendor_nav_chain as v


class _Msg:
    def __init__(self, data=None):
        self.data = data


class _Pub:
    def __init__(self):
        self.sent = []

    def publish(self, payload):
        self.sent.append(payload)


class _Sub:
    def __init__(self, topic, msg_type, cb):
        self.topic = topic
        self.msg_type = msg_type
        self.cb = cb


class _RosStub:
    String = str
    Bool = _Msg
    PoseWithCovarianceStamped = object
    OccupancyGrid = object
    Odometry = object

    Publisher = staticmethod(lambda *a, **k: _Pub())
    Subscriber = staticmethod(lambda *a, **k: _Sub(*a))
    Timer = staticmethod(lambda *a, **k: None)
    Duration = staticmethod(lambda x: x)


class NavInitReadinessTests(unittest.TestCase):

    def test_default_is_off_and_start_is_inert(self):
        node = n.NavInitReadinessNode(ros=_RosStub())
        self.assertEqual(node.mode, "off")
        node.start()
        self.assertEqual(node.subscribers, [])
        self.assertIsNone(node.status_pub)
        self.assertEqual(node.written_topics(), [n.DIAG_TOPIC])
        self.assertEqual(node.topic_writes, [])

    def test_off_mode_never_publishes(self):
        node = n.NavInitReadinessNode(ros=_RosStub())
        self.assertFalse(node.publish(node.evaluate()))

    def test_shadow_subscribes_only_to_declared_inputs(self):
        node = n.NavInitReadinessNode(ros=_RosStub(),
                                     params={"readiness_mode": "shadow"})
        node.start()
        self.assertEqual(len(node.subscribers), 4)
        self.assertEqual(set(node.subscribed_topics()), {
            n.INIT_POSE_TOPIC, n.MAP_TOPIC, n.ODOM_TOPIC, n.CONVERGED_TOPIC})
        # no motion topic is ever among inputs/outputs
        for t in node.subscribed_topics() + node.written_topics():
            self.assertNotIn("cmd_vel", t)
            self.assertNotIn("chassis", t)
            self.assertNotIn("43893", t)

    def test_empty_signals_refuse_not_green(self):
        node = n.NavInitReadinessNode(ros=_RosStub(),
                                     params={"readiness_mode": "assist"})
        r = node.evaluate()
        self.assertEqual(r["state"], "insufficient_data")
        self.assertEqual(r["decision"], "refuse")
        self.assertFalse(r["ready"])

    def test_callbacks_fill_signals_and_gate_becomes_ready(self):
        node = n.NavInitReadinessNode(ros=_RosStub(),
                                     params={"readiness_mode": "assist"})
        node.on_initial_pose(object())
        node.on_map(object())
        node.on_odom(object())
        node.on_converged(_Msg(True))
        r = node.evaluate()
        self.assertEqual(r["state"], "ready")
        self.assertEqual(r["decision"], "allow")
        self.assertTrue(r["ready"])

    def test_missing_converged_producer_stays_fail_closed(self):
        node = n.NavInitReadinessNode(ros=_RosStub(),
                                     params={"readiness_mode": "assist"})
        node.on_initial_pose(object())
        node.on_map(object())
        node.on_odom(object())
        # no convergence producer exists in this repository
        r = node.evaluate()
        self.assertEqual(r["state"], "not_ready")
        self.assertEqual(r["decision"], "veto")
        self.assertIn("localization_converged", r["missing"])

    def test_veto_is_recommendation_only_and_self_reports_no_hardware(self):
        node = n.NavInitReadinessNode(ros=_RosStub(),
                                     params={"readiness_mode": "assist"})
        r = node.evaluate()
        self.assertTrue(r["veto_is_recommendation_only"])
        self.assertFalse(r["hardware"])
        self.assertEqual(r["live_topic_writes"], 0)
        self.assertEqual(r["motion_topic_writes"], 0)
        self.assertFalse(r["gate_thresholds_modified"])
        self.assertFalse(r["produces_pass_fail"])
        self.assertFalse(r["converged_producer_registered"])

    def test_shadow_publishes_json_diagnostic_only(self):
        node = n.NavInitReadinessNode(ros=_RosStub(),
                                     params={"readiness_mode": "shadow"})
        node.start()
        self.assertTrue(node.publish(node.evaluate()))
        self.assertEqual(node.topic_writes, [n.DIAG_TOPIC])
        # empty signal set must be refused, never printed green
        payload = json.loads(node.status_pub.sent[0])
        self.assertEqual(payload["state"], "insufficient_data")
        self.assertEqual(payload["decision"], "refuse")
        # injection effectiveness: with all signals present shadow really
        # becomes observe_only (proves the assertion above is not an empty control)
        node.on_initial_pose(object())
        node.on_map(object())
        node.on_odom(object())
        node.on_converged(_Msg(True))
        node.publish(node.evaluate())
        payload_ready = json.loads(node.status_pub.sent[1])
        self.assertEqual(payload_ready["state"], "ready")
        self.assertEqual(payload_ready["decision"], "observe_only")
        self.assertEqual(node.topic_writes, [n.DIAG_TOPIC, n.DIAG_TOPIC])

    def test_injection_and_reverse_control_on_node(self):
        node = n.NavInitReadinessNode(ros=_RosStub(),
                                     params={"readiness_mode": "assist"})
        node.on_initial_pose(object())
        node.on_map(object())
        node.on_odom(object())
        node.on_converged(_Msg(True))
        self.assertEqual(node.evaluate()["decision"], "allow")
        node.on_converged(_Msg(False))       # injection
        self.assertEqual(node.evaluate()["decision"], "veto")
        node.on_converged(_Msg(True))        # reverse control
        self.assertEqual(node.evaluate()["decision"], "allow")

    def test_converged_none_is_ignored_not_treated_as_false(self):
        node = n.NavInitReadinessNode(ros=_RosStub())
        node.on_converged(_Msg(None))
        self.assertIsNone(node.signals["localization_converged"])

    def test_start_without_ros_refuses_off_mode_silently_but_other_modes_raise(self):
        off = n.NavInitReadinessNode()
        off.start()  # off returns before touching ros
        with self.assertRaises(RuntimeError):
            n.NavInitReadinessNode(params={"readiness_mode": "shadow"}).start()

    def test_projected_settings_refuses_backfill(self):
        clean = {k: "own" for k in v.REQUIRED_SIGNALS}
        clean["max_vel_x"] = 0.5
        self.assertEqual(n.projected_settings_check(clean)["state"], "clean")
        dirty = dict(clean, max_vel_x=v.VENDOR_INTERFACES["max_vel_x"]["value"])
        self.assertEqual(n.projected_settings_check(dirty)["state"],
                         "backfill_detected")


if __name__ == "__main__":
    unittest.main()
