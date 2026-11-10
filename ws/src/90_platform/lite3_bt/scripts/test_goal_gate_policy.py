"""Offline tests for the demo goal-gate publisher allow-list.

No rospy, no ROS master, no topics.  Besides the decision-function tests this
file checks the wiring: a green pure-function test is worthless if on_request
never calls the policy (pit 88 family: verdict visible, defect outside it).
"""
from __future__ import annotations

import ast
import importlib.util
import os
import sys
import time
import types
import unittest
from unittest import mock

import goal_gate_policy as p

HERE = os.path.dirname(os.path.abspath(__file__))
GATE = os.path.join(HERE, "goal_gate.py")
LAUNCH = os.path.join(HERE, "..", "launch", "bt_console.launch")


class PolicyTests(unittest.TestCase):

    def test_main_check_refuses_unlisted_publisher(self):
        ok, why = p.accept_request("/some_other_node", True, True)
        self.assertFalse(ok)
        self.assertTrue(why.startswith("callerid_not_allowed"))

    def test_injection_effectiveness_listed_publisher_passes(self):
        ok, why = p.accept_request("/bt_console", True, True)
        self.assertTrue(ok)
        self.assertEqual(why, "ok")

    def test_reverse_control_disabling_acl_lets_unlisted_through(self):
        ok, _ = p.accept_request("/some_other_node", True, True,
                                 require_acl=False)
        self.assertTrue(ok)

    def test_missing_or_empty_callerid_is_refused(self):
        for bad in (None, "", "   ", 123, [], b"/bt_console"):
            self.assertFalse(p.accept_request(bad, True, True)[0])
        self.assertEqual(p.accept_request(None, True, True)[1],
                         "callerid_missing")

    def test_empty_allow_list_refuses_everything_not_green(self):
        self.assertEqual(p.parse_allowed(""), ())
        ok, why = p.accept_request("/bt_console", True, True, ())
        self.assertFalse(ok)
        self.assertTrue(why.startswith("callerid_not_allowed"))

    def test_lease_check_is_not_weakened_by_acl(self):
        self.assertFalse(p.accept_request("/bt_console", False, True)[0])
        self.assertFalse(p.accept_request("/bt_console", True, False)[0])
        self.assertEqual(p.accept_request("/bt_console", False, True)[1],
                         "lease_invalid")
        self.assertTrue(p.accept_request("/bt_console", False, True,
                                        require_lease=False)[0])

    def test_identity_refusal_wins_over_stale_lease(self):
        self.assertEqual(p.accept_request("/impostor", False, False)[1],
                         "callerid_not_allowed:/impostor")

    def test_parse_allowed_variants(self):
        self.assertEqual(p.parse_allowed("/bt_console,/other"),
                         ("/bt_console", "/other"))
        self.assertEqual(p.parse_allowed("/a  /b"), ("/a", "/b"))
        self.assertEqual(p.parse_allowed(["/a", "/b"]), ("/a", "/b"))
        self.assertEqual(p.parse_allowed(None), p.DEFAULT_ALLOWED_CALLERS)

    def test_default_allow_list_is_not_a_wildcard(self):
        self.assertEqual(len(p.DEFAULT_ALLOWED_CALLERS), 1)
        self.assertIn("/bt_console", p.DEFAULT_ALLOWED_CALLERS)


class WiringTests(unittest.TestCase):
    """The policy must actually be called by the ROS node, before publish."""

    def _on_request(self):
        with open(GATE, "r", encoding="utf-8") as fh:
            tree = ast.parse(fh.read())
        fn = next((n for n in ast.walk(tree)
                   if isinstance(n, ast.FunctionDef) and n.name == "on_request"),
                  None)
        self.assertIsNotNone(fn, "on_request not found in goal_gate.py")
        calls = sorted((n.lineno, ast.unparse(n.func))
                       for n in ast.walk(fn) if isinstance(n, ast.Call))
        return tree, fn, calls

    def test_policy_is_imported(self):
        tree, _, _ = self._on_request()
        imported = any(isinstance(n, ast.Import)
                       and any(a.name == "goal_gate_policy" and a.asname == "policy"
                               for a in n.names)
                       for n in ast.walk(tree))
        self.assertTrue(imported, "goal_gate.py must import goal_gate_policy as policy")

    def test_on_request_calls_policy_before_publish(self):
        _, fn, calls = self._on_request()
        accept = [ln for ln, name in calls if "accept_request" in name]
        publish = [ln for ln, name in calls if "self.output.publish" in name]
        self.assertTrue(accept, "on_request never calls accept_request")
        self.assertTrue(publish, "on_request never publishes")
        self.assertLess(min(accept), min(publish),
                        "publish happens before (or without) the ACL decision")

    def test_denied_path_returns_without_publishing(self):
        _, fn, _ = self._on_request()
        self.assertTrue(any(isinstance(n, ast.Return) for n in ast.walk(fn)),
                        "no early return on a denied request")

    def test_launch_declares_the_allow_list(self):
        with open(LAUNCH, "r", encoding="utf-8") as fh:
            text = fh.read()
        self.assertIn('name="allowed_callers"', text)
        self.assertIn('name="require_acl"', text)
        self.assertIn('/bt_console', text)
        self.assertIn('value="true"', text)


def _load_gate_module():
    """Import goal_gate.py against a stubbed rospy; no ROS master required."""
    fake_rospy = types.ModuleType("rospy")
    fake_rospy.get_param = lambda name, default=None: default
    fake_rospy.Publisher = lambda *a, **k: mock.MagicMock()
    fake_rospy.Subscriber = lambda *a, **k: None
    fake_rospy.loginfo = lambda *a, **k: None
    fake_rospy.logwarn = lambda *a, **k: None
    fake_rospy.init_node = lambda *a, **k: None
    fake_rospy.spin = lambda *a, **k: None

    geom_pkg = types.ModuleType("geometry_msgs")
    geom = types.ModuleType("geometry_msgs.msg")
    geom.PoseStamped = object
    geom_pkg.msg = geom
    std_pkg = types.ModuleType("std_msgs")
    std = types.ModuleType("std_msgs.msg")
    std.Bool = object
    std_pkg.msg = std

    names = ("rospy", "geometry_msgs", "geometry_msgs.msg",
             "std_msgs", "std_msgs.msg")
    saved = {n: sys.modules.get(n) for n in names}
    sys.modules.update({"rospy": fake_rospy, "geometry_msgs": geom_pkg,
                        "geometry_msgs.msg": geom, "std_msgs": std_pkg,
                        "std_msgs.msg": std})
    if HERE not in sys.path:
        sys.path.insert(0, HERE)
    try:
        spec = importlib.util.spec_from_file_location("goal_gate_under_test", GATE)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
    finally:
        for n, old in saved.items():
            if old is None:
                sys.modules.pop(n, None)
            else:
                sys.modules[n] = old
    return mod


class RuntimeWiringTests(unittest.TestCase):
    """Drive the real on_request, not just the pure decision function."""

    @staticmethod
    def _msg(callerid):
        class Msg:
            def __init__(self):
                self.pose = mock.MagicMock()
                if callerid is not None:
                    self._connection_header = {"callerid": callerid}
        return Msg()

    def setUp(self):
        self.gate = _load_gate_module().GoalGate()
        self.gate.lease = True
        self.gate.lease_at = time.monotonic()

    def test_listed_publisher_actually_publishes(self):
        self.gate.on_request(self._msg("/bt_console"))
        self.assertEqual(self.gate.output.publish.call_count, 1)

    def test_unlisted_publisher_is_dropped(self):
        self.gate.on_request(self._msg("/impostor"))
        self.assertEqual(self.gate.output.publish.call_count, 0)

    def test_missing_connection_header_is_dropped(self):
        self.gate.on_request(self._msg(None))
        self.assertEqual(self.gate.output.publish.call_count, 0)

    def test_reverse_control_runtime_acl_off_lets_impostor_through(self):
        self.gate.require_acl = False
        self.gate.on_request(self._msg("/impostor"))
        self.assertEqual(self.gate.output.publish.call_count, 1)


if __name__ == "__main__":
    unittest.main()
