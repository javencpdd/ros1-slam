#!/usr/bin/env python3
"""Host-only safety/protocol tests; no ROS installation or master required."""

import json
import math
import sys
import threading
import unittest
from types import ModuleType, SimpleNamespace as NS
from unittest.mock import Mock, patch

import navigation_executor as nav


def message(data, caller="/bt_console", latched=False):
    return NS(data=data, _connection_header={"callerid": caller, "latching": "1" if latched else "0"})


def pose(seq=41, caller="/bt_console"):
    return NS(header=NS(seq=seq, frame_id="map", stamp=123),
              pose=NS(position=NS(x=1.0, y=2.0, z=0.0),
                      orientation=NS(x=0.0, y=0.0, z=0.0, w=1.0)),
              _connection_header={"callerid": caller, "latching": "0"})


def twist(x=0.3, yaw=0.2):
    return NS(linear=NS(x=x, y=0.0, z=0.0), angular=NS(x=0.0, y=0.0, z=yaw))


class Clock:
    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now

    def advance(self, dt):
        self.now += dt


class Client:
    def __init__(self):
        self.sent = []
        self.cancelled = []
        self.state = nav.PENDING
        self.send_hook = None

    def send_goal(self, goal, done_cb=None, active_cb=None):
        self.sent.append((goal, done_cb, active_cb))
        self.state = nav.PENDING
        if self.send_hook:
            self.send_hook()

    def cancel_goal(self):
        self.cancelled.append("current")

    def cancel_all_goals(self):
        self.cancelled.append("all")

    def get_state(self):
        return self.state

    def active(self, index=-1):
        self.state = nav.ACTIVE
        self.sent[index][2]()

    def done(self, state, index=-1):
        self.state = state
        self.sent[index][1](state, None)


class ExecutorTest(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.client = Client()
        self.ready = True
        self.results, self.acks, self.statuses, self.commands = [], [], [], []
        self.node = nav.NavigationExecutor(
            self.client, lambda: self.ready, self.results.append, self.acks.append,
            lambda text: self.statuses.append(json.loads(text)), self.commands.append,
            clock=self.clock)

    def lease(self, value=True):
        self.node.on_lease(message(value))

    def start(self, seq=41, activate=True):
        self.lease()
        self.assertTrue(self.node.on_goal(pose(seq)))
        self.node.service_backend()
        if activate:
            self.client.active()

    def moving(self):
        self.node.on_candidate(twist())
        self.node.tick()
        self.assertEqual(self.commands[-1], (0.3, 0, 0, 0, 0, 0.2))

    def cancel(self, seq=900, mode="current"):
        self.node.on_cancel(message("{}:{}:BT stop:requested".format(seq, mode)))

    def test_success_preserves_exact_bt_seq_and_pose(self):
        request = pose(2345)
        request.pose.orientation.z = math.sin(0.3)
        request.pose.orientation.w = math.cos(0.3)
        self.lease()
        self.assertTrue(self.node.on_goal(request))
        self.node.service_backend()
        sent = self.client.sent[0][0]
        self.assertEqual(sent, request)
        self.assertIsNot(sent, request)
        self.client.active()
        self.moving()
        self.client.done(nav.SUCCEEDED)
        self.assertEqual(self.results, ["2345:ok"])
        self.assertEqual(self.commands[-1], nav.ZERO)
        self.assertIsNone(self.node.current)
        self.assertEqual(self.statuses[-1]["backend_state"], nav.SUCCEEDED)

    def test_backend_failures_and_preemptions_are_scoped_failures(self):
        self.lease()
        for seq, state in enumerate((nav.ABORTED, nav.REJECTED, nav.PREEMPTED, nav.RECALLED), 1):
            with self.subTest(state=state):
                self.assertTrue(self.node.on_goal(pose(seq)))
                self.node.service_backend()
                self.client.done(state)
                self.assertEqual(self.results[-1], "{}:fail:backend_terminal".format(seq))
                self.assertIsNone(self.node.current)
                self.assertFalse(self.node.latch)

    def test_cancel_waits_for_terminal_and_uses_request_not_goal_seq(self):
        self.start()
        self.moving()
        self.cancel(901)
        self.assertEqual(self.acks, [])
        self.assertEqual(self.commands[-1], nav.ZERO)
        self.node.service_backend()
        self.assertEqual(self.client.cancelled, ["current"])
        self.node.service_backend()
        self.node.on_candidate(twist())
        self.node.tick()
        self.assertEqual(self.commands[-1], nav.ZERO)
        self.assertEqual(self.acks, [])
        self.client.done(nav.PREEMPTED)
        self.assertEqual(self.acks, ["901:cancelled"])
        self.assertEqual(self.results, ["41:fail:cancelled"])

    def test_recalled_confirms_cancel_of_pending_goal(self):
        self.start(activate=False)
        self.cancel()
        self.node.service_backend()
        self.client.done(nav.RECALLED)
        self.assertEqual(self.acks, ["900:cancelled"])

    def test_success_racing_cancel_is_failure_but_confirms_termination(self):
        self.start()
        self.cancel()
        self.client.done(nav.SUCCEEDED)
        self.assertEqual(self.acks, ["900:cancelled"])
        self.assertEqual(self.results, ["41:fail:cancelled"])

    def test_all_cancel_uses_sdk_all_once(self):
        self.start()
        self.cancel(mode="all")
        self.node.service_backend()
        self.cancel(mode="all")
        self.node.service_backend()
        self.assertEqual(self.client.cancelled, ["all"])
        self.assertEqual(self.acks, [])
        self.client.done(nav.ABORTED)
        self.assertEqual(self.acks, ["900:cancelled"])

    def test_multiple_cancel_ids_wait_for_same_terminal_without_resending(self):
        self.start()
        self.cancel(900)
        self.node.service_backend()
        self.cancel(901)
        self.node.service_backend()
        self.assertEqual(self.client.cancelled, ["current"])
        self.client.done(nav.PREEMPTED)
        self.assertEqual(self.acks, ["900:cancelled", "901:cancelled"])

    def test_idle_and_late_cancel_ack_without_backend_calls(self):
        self.cancel(1, "all")
        self.assertEqual(self.acks, ["1:cancelled"])
        self.start()
        self.client.done(nav.SUCCEEDED)
        self.cancel(2)
        self.node.service_backend()
        self.assertEqual(self.acks[-1], "2:cancelled")
        self.assertEqual(self.results, ["41:ok"])
        self.assertEqual(self.client.cancelled, [])

    def test_old_cancel_request_is_not_applied_to_new_goal(self):
        self.cancel(900)
        self.start()
        self.cancel(900)
        self.cancel(899)
        self.node.service_backend()
        self.assertIsNone(self.node.current.cancel_at)
        self.assertEqual(self.client.cancelled, [])
        self.assertEqual(self.acks, ["900:cancelled", "900:cancelled"])

    def test_old_done_and_active_callbacks_cannot_overwrite_new_goal(self):
        self.start(41)
        old_done, old_active = self.client.sent[0][1:]
        self.client.done(nav.SUCCEEDED)
        self.assertTrue(self.node.on_goal(pose(42)))
        self.node.service_backend()
        self.client.active()
        self.moving()
        old_done(nav.LOST, None)
        old_active()
        self.assertEqual(self.node.current.seq, 42)
        self.assertFalse(self.node.latch)
        self.assertEqual(self.results, ["41:ok"])
        self.node.tick()
        self.assertNotEqual(self.commands[-1], nav.ZERO)
        self.client.done(nav.SUCCEEDED)
        self.assertEqual(self.results, ["41:ok", "42:ok"])

    def test_no_lease_rejects_scoped_goal_without_backend(self):
        self.assertFalse(self.node.on_goal(pose()))
        self.node.service_backend()
        self.assertEqual(self.results, ["41:fail:no_lease"])
        self.assertEqual(self.client.sent, [])
        self.assertEqual(self.statuses[-1]["reason"], "no_lease")

    def test_initial_false_lease_does_not_authorize_goal(self):
        self.lease(False)
        self.assertFalse(self.node.on_goal(pose()))
        self.assertFalse(self.node.latch)
        self.lease()
        self.assertFalse(self.node.on_goal(pose()))
        self.assertTrue(self.node.on_goal(pose(42)))

    def test_false_lease_immediately_zeroes_cancels_and_requires_restart(self):
        self.start()
        self.moving()
        self.lease(False)
        self.assertEqual(self.commands[-1], nav.ZERO)
        self.node.service_backend()
        self.assertEqual(self.client.cancelled, ["current"])
        self.client.done(nav.PREEMPTED)
        self.lease()
        self.assertFalse(self.node.on_goal(pose(42)))
        self.assertEqual(self.node.latch, "lease_lost")
        self.assertEqual(len(self.client.sent), 1)

    def test_expired_lease_triggers_wall_watchdog_without_ros_clock(self):
        self.start()
        self.moving()
        self.clock.advance(1.0001)
        self.node.tick()
        self.assertEqual(self.commands[-1], nav.ZERO)
        self.node.service_backend()
        self.assertEqual(self.client.cancelled, ["current"])
        self.assertEqual(self.results, ["41:fail:lease_lost"])
        self.assertEqual(self.node.latch, "lease_lost")

    def test_lease_boundary_is_inclusive(self):
        self.lease()
        self.clock.advance(1.0)
        self.assertTrue(self.node.on_goal(pose()))
        self.node.service_backend()
        self.assertEqual(len(self.client.sent), 1)
        self.clock.advance(0.0001)
        self.node.tick()
        self.assertEqual(self.node.latch, "lease_lost")

    def test_late_true_heartbeat_cannot_heal_expiration_between_ticks(self):
        self.start()
        self.clock.advance(1.01)
        self.lease()
        self.assertEqual(self.node.latch, "lease_lost")
        self.node.service_backend()
        self.assertEqual(self.client.cancelled, ["current"])

    def test_lease_loss_when_idle_still_blocks_counter_restart(self):
        self.start()
        self.client.done(nav.SUCCEEDED)
        self.lease(False)
        self.lease()
        self.assertFalse(self.node.on_goal(pose(1)))
        self.assertFalse(self.node.on_goal(pose(42)))
        self.assertEqual(self.node.latch, "lease_lost")

    def test_expired_initial_lease_rejects_even_before_first_goal(self):
        self.lease()
        self.clock.advance(1.1)
        self.assertFalse(self.node.on_goal(pose()))
        self.assertEqual(self.client.sent, [])
        self.assertEqual(self.results, ["41:fail:restart_required"])

    def test_candidates_are_stored_not_directly_forwarded_and_expire(self):
        self.start()
        count = len(self.commands)
        self.node.on_candidate(twist())
        self.assertEqual(len(self.commands), count)
        self.clock.advance(0.5)
        self.node.tick()
        self.assertNotEqual(self.commands[-1], nav.ZERO)
        self.clock.advance(0.0001)
        self.node.tick()
        self.assertEqual(self.commands[-1], nav.ZERO)

    def test_candidates_require_backend_active_and_are_not_carried_over(self):
        self.node.on_candidate(twist())
        self.start(activate=False)
        self.node.on_candidate(twist())
        self.node.tick()
        self.assertEqual(self.commands[-1], nav.ZERO)
        self.client.active()
        self.node.tick()
        self.assertEqual(self.commands[-1], nav.ZERO)
        self.moving()
        self.client.done(nav.SUCCEEDED)
        self.assertTrue(self.node.on_goal(pose(42)))
        self.node.service_backend()
        self.client.active()
        self.node.tick()
        self.assertEqual(self.commands[-1], nav.ZERO)

    def test_invalid_candidates_clear_last_valid_sample(self):
        self.start()
        for path, value in (("linear.x", float("nan")), ("angular.z", float("inf")),
                            ("linear.z", 0.1), ("angular.x", 0.1), ("angular.y", 0.1)):
            with self.subTest(path=path):
                self.moving()
                bad = twist()
                group, axis = path.split(".")
                setattr(getattr(bad, group), axis, value)
                self.node.on_candidate(bad)
                self.assertEqual(self.commands[-1], nav.ZERO)
                self.node.tick()
                self.assertEqual(self.commands[-1], nav.ZERO)

    def test_busy_rejects_new_goal_without_orphaning_old(self):
        self.start()
        self.assertFalse(self.node.on_goal(pose(42)))
        self.node.service_backend()
        self.assertEqual(len(self.client.sent), 1)
        self.assertEqual(self.node.current.seq, 41)
        self.assertEqual(self.results, ["42:fail:busy"])
        self.client.done(nav.SUCCEEDED)
        self.assertEqual(self.results, ["42:fail:busy", "41:ok"])
        self.assertFalse(self.node.on_goal(pose(42)))

    def test_duplicate_and_stale_goal_ids_do_not_emit_conflicting_results(self):
        self.start()
        for seq in (41, 40, 1):
            self.assertFalse(self.node.on_goal(pose(seq)))
        self.assertEqual(self.results, [])
        self.client.done(nav.SUCCEEDED)
        self.assertFalse(self.node.on_goal(pose(41)))
        self.assertEqual(self.results, ["41:ok"])
        self.assertEqual(len(self.client.sent), 1)

    def test_missing_server_is_scoped_rejection_not_deferred_execution(self):
        self.lease()
        self.ready = False
        self.assertFalse(self.node.on_goal(pose()))
        self.assertEqual(self.results, ["41:fail:backend_unavailable"])
        self.ready = True
        self.node.service_backend()
        self.assertEqual(self.client.sent, [])
        self.assertFalse(self.node.on_goal(pose()))
        self.assertTrue(self.node.on_goal(pose(42)))

    def test_server_disappears_before_dispatch(self):
        self.lease()
        self.assertTrue(self.node.on_goal(pose()))
        self.ready = False
        self.node.service_backend()
        self.assertEqual(self.results, ["41:fail:backend_unavailable"])
        self.assertEqual(self.client.sent, [])
        self.assertIsNone(self.node.current)

    def test_invalid_goal_identity_never_starts_action(self):
        self.lease()
        for seq in (0, -1, 0x100000000, True, 1.5, "41", None):
            with self.subTest(seq=seq):
                self.assertFalse(self.node.on_goal(pose(seq)))
        self.assertEqual(self.results, [])
        self.assertEqual(self.client.sent, [])

    def test_invalid_pose_is_rejected_without_projection(self):
        self.lease()
        cases = [("position", "x", float("nan")), ("position", "y", float("inf")),
                 ("position", "z", 0.001), ("orientation", "x", 0.1),
                 ("orientation", "y", 0.1), ("orientation", "w", 0.0),
                 ("orientation", "z", float("nan")), ("orientation", "w", 2.0),
                 ("orientation", "w", 1e300)]
        for seq, (group, axis, value) in enumerate(cases, 1):
            with self.subTest(group=group, axis=axis, value=value):
                msg = pose(seq)
                setattr(getattr(msg.pose, group), axis, value)
                self.assertFalse(self.node.on_goal(msg))
                self.assertEqual(self.results[-1], "{}:fail:invalid_pose".format(seq))
                self.assertEqual(self.statuses[-1]["reason"], "invalid_pose")
        msg = pose(100)
        msg.header.frame_id = " "
        self.assertFalse(self.node.on_goal(msg))
        self.assertEqual(self.client.sent, [])

    def test_wrong_caller_is_ignored_on_all_three_bt_inputs(self):
        self.node.on_lease(message(True, "/other"))
        self.assertFalse(self.node.lease_seen)
        self.start()
        self.node.on_lease(message(False, "/other"))
        self.node.on_cancel(message("900:all:spoof", "/other"))
        self.assertFalse(self.node.on_goal(pose(42, "/other")))
        self.assertIsNone(self.node.current.cancel_at)
        self.assertFalse(self.node.latch)
        self.assertEqual(self.node.last_goal_seq, 41)
        self.assertEqual(self.acks, [])

    def test_missing_connection_header_is_supported_for_local_injection(self):
        self.node.on_lease(NS(data=True))
        msg = pose()
        del msg._connection_header
        self.assertTrue(self.node.on_goal(msg))
        self.node.service_backend()
        self.node.on_cancel(NS(data="8:current:stop"))
        self.client.done(nav.RECALLED)
        self.assertEqual(self.acks, ["8:cancelled"])

    def test_latched_goal_lease_and_cancel_are_not_replayed(self):
        self.node.on_lease(message(True, latched=True))
        self.assertFalse(self.node.lease_seen)
        self.lease()
        msg = pose()
        msg._connection_header["latching"] = "1"
        self.assertFalse(self.node.on_goal(msg))
        self.assertEqual(self.node.last_goal_seq, 0)
        self.start()
        self.node.on_cancel(message("900:all:stale", latched=True))
        self.assertIsNone(self.node.current.cancel_at)
        self.assertEqual(self.acks, [])

    def test_invalid_cancel_payload_does_not_cancel_or_ack(self):
        self.start()
        for payload in ("1", "1:current", "0:current:x", "-1:all:x", "1:some:x",
                        " 1:all:x", "01:all:x", "9" * 21 + ":all:x", None, 1):
            with self.subTest(payload=payload):
                self.node.on_cancel(message(payload))
        self.assertEqual(self.acks, [])
        self.assertIsNone(self.node.current.cancel_at)

    def test_lost_is_not_confirmed_cancel_and_permanently_latches(self):
        self.start()
        self.cancel()
        self.client.done(nav.LOST)
        self.assertEqual(self.results, ["41:fail:backend_lost"])
        self.assertEqual(self.acks, ["900:ack_fail"])
        self.assertIsNotNone(self.node.current)
        self.assertEqual(self.commands[-1], nav.ZERO)
        self.cancel(901)
        self.assertEqual(self.acks[-1], "901:ack_fail")
        self.assertFalse(self.node.on_goal(pose(42)))
        self.client.done(nav.PREEMPTED)
        self.assertIsNone(self.node.current)
        self.assertFalse(self.node.on_goal(pose(43)))
        self.assertNotIn("900:cancelled", self.acks)

    def test_lost_without_cancel_is_detected_by_state_poll(self):
        self.start()
        self.client.state = nav.LOST
        self.node.service_backend()
        self.assertEqual(self.node.latch, "backend_lost")
        self.assertEqual(self.results, ["41:fail:backend_lost"])
        self.assertEqual(self.commands[-1], nav.ZERO)

    def test_polled_terminal_can_confirm_cancel_without_done_callback(self):
        self.start()
        self.cancel()
        self.node.service_backend()
        self.client.state = nav.PREEMPTED
        self.node.service_backend()
        self.assertEqual(self.acks, ["900:cancelled"])
        self.assertIsNone(self.node.current)

    def test_server_disconnect_revokes_output_and_never_fakes_ack(self):
        self.start()
        self.moving()
        self.cancel()
        self.ready = False
        self.node.tick()
        self.assertEqual(self.commands[-1], nav.ZERO)
        self.assertEqual(self.acks, [])
        self.assertEqual(self.node.latch, "backend_lost")
        self.assertEqual(self.results, ["41:fail:backend_lost"])
        self.assertEqual(self.statuses[-1]["detail"], "backend_disconnected")
        self.node.service_backend()
        self.assertEqual(self.client.cancelled, ["current"])

    def test_cancel_deadline_ack_fail_and_latch_survive_late_terminal(self):
        self.start()
        self.cancel()
        self.node.service_backend()
        for dt in (0.9, 0.9, 0.199):
            self.clock.advance(dt)
            self.lease()
            self.node.tick()
        self.assertEqual(self.acks, [])
        self.clock.advance(0.002)
        self.lease()
        self.node.tick()
        self.assertEqual(self.acks, ["900:ack_fail"])
        self.assertEqual(self.node.latch, "cancel_timeout")
        self.assertEqual(self.results, ["41:fail:cancel_timeout"])
        self.assertEqual(self.commands[-1], nav.ZERO)
        self.assertFalse(self.node.on_goal(pose(42)))
        self.client.done(nav.PREEMPTED)
        self.assertEqual(self.acks, ["900:ack_fail"])
        self.assertFalse(self.node.on_goal(pose(43)))
        self.cancel(901)
        self.assertEqual(self.acks[-1], "901:cancelled")
        self.assertEqual(self.node.latch, "cancel_timeout")

    def test_terminal_callback_itself_enforces_cancel_deadline(self):
        self.start()
        self.cancel()
        self.clock.advance(2.0)
        self.client.done(nav.PREEMPTED)
        self.assertEqual(self.acks, ["900:ack_fail"])
        self.assertTrue(self.node.latch)

    def test_cancel_before_dispatch_never_sends_goal(self):
        self.lease()
        self.assertTrue(self.node.on_goal(pose()))
        self.cancel()
        self.node.service_backend()
        self.assertEqual(self.client.sent, [])
        self.assertEqual(self.client.cancelled, [])
        self.assertEqual(self.acks, ["900:cancelled"])
        self.assertEqual(self.results, ["41:fail:cancelled"])

    def test_blocked_send_does_not_block_watchdog_or_lose_cancellation(self):
        entered, release = threading.Event(), threading.Event()

        def blocked():
            entered.set()
            self.assertTrue(release.wait(2))

        self.client.send_hook = blocked
        self.lease()
        self.assertTrue(self.node.on_goal(pose()))
        worker = threading.Thread(target=self.node.service_backend)
        worker.start()
        try:
            self.assertTrue(entered.wait(1))
            self.cancel()
            self.node.tick()
            self.assertEqual(self.commands[-1], nav.ZERO)
            self.assertFalse(self.node.on_goal(pose(42)))
            self.assertEqual(self.client.cancelled, [])
            self.assertEqual(self.acks, [])
        finally:
            release.set()
            worker.join(2)
        self.assertFalse(worker.is_alive())
        self.node.service_backend()
        self.assertEqual(self.client.cancelled, ["current"])
        self.client.done(nav.PREEMPTED)
        self.assertEqual(self.acks, ["900:cancelled"])

    def test_blocked_poll_cannot_apply_old_state_to_new_goal(self):
        self.start()
        entered, release = threading.Event(), threading.Event()

        def poll():
            entered.set()
            release.wait(2)
            return nav.LOST

        self.client.get_state = poll
        worker = threading.Thread(target=self.node.service_backend)
        worker.start()
        try:
            self.assertTrue(entered.wait(1))
            self.client.done(nav.SUCCEEDED)
            self.assertTrue(self.node.on_goal(pose(42)))
            self.node.tick()
        finally:
            release.set()
            worker.join(2)
        self.assertFalse(self.node.latch)
        self.assertEqual(self.node.current.seq, 42)
        self.assertEqual(self.results, ["41:ok"])

    def test_send_exception_fails_closed_and_keeps_unknown_action(self):
        self.client.send_hook = Mock(side_effect=RuntimeError("transport failed"))
        self.start(activate=False)
        self.assertEqual(self.node.latch, "backend_lost")
        self.assertIsNotNone(self.node.current)
        self.assertEqual(self.results, ["41:fail:backend_lost"])
        self.node.service_backend()
        self.assertEqual(self.client.cancelled, ["current"])
        self.assertFalse(self.node.on_goal(pose(42)))

    def test_cancel_exception_waits_for_terminal_or_deadline(self):
        self.start()
        self.client.cancel_goal = Mock(side_effect=RuntimeError("disconnected"))
        self.cancel()
        self.node.service_backend()
        self.assertEqual(self.acks, [])
        self.assertEqual(self.node.latch, "backend_lost")
        self.node.service_backend()
        self.client.cancel_goal.assert_called_once()
        self.client.done(nav.PREEMPTED)
        self.assertEqual(self.acks, ["900:cancelled"])

    def test_shutdown_revokes_and_cancels_best_effort(self):
        self.start()
        self.moving()
        self.node.shutdown()
        self.assertEqual(self.commands[-1], nav.ZERO)
        self.node.service_backend()
        self.assertEqual(self.client.cancelled, ["current"])
        self.node.shutdown()
        self.node.service_backend()
        self.assertEqual(self.client.cancelled, ["current"])
        self.assertFalse(self.node.on_goal(pose(42)))

    def test_diagnostics_always_include_required_fields(self):
        self.start()
        self.cancel()
        self.client.done(nav.RECALLED)
        for status in self.statuses:
            self.assertEqual(set(status), {"seq", "state", "reason", "detail", "backend_state"})

    def test_idle_cancel_fences_delayed_goal_even_after_ack(self):
        self.lease()
        self.node.on_cancel(message("901:current:"), target_seq=41, session="100")
        self.assertEqual(self.node.session, "100")
        self.assertEqual(self.node.cancelled_goal_seq, 41)
        self.assertEqual(self.acks, ["901:cancelled"])
        for seq in (40, 41):
            self.assertFalse(self.node.on_goal(pose(seq), session="100"))
        self.node.service_backend()
        self.assertEqual(self.results, ["40:fail:cancelled", "41:fail:cancelled"])
        self.assertEqual(self.client.sent, [])
        self.assertTrue(self.node.on_goal(pose(42), session="100"))
        self.node.service_backend()
        self.assertEqual(len(self.client.sent), 1)

    def test_reasonless_cancel_waits_for_own_terminal(self):
        self.start()
        self.node.on_cancel(message("901:current:"), target_seq=41)
        self.node.service_backend()
        self.assertEqual(self.client.cancelled, ["current"])
        self.assertEqual(self.acks, [])
        self.client.done(nav.PREEMPTED)
        self.assertEqual(self.results, ["41:fail:cancelled"])
        self.assertEqual(self.acks, ["901:cancelled"])

    def test_stale_target_does_not_cancel_or_zero_newer_goal_even_scope_all(self):
        self.start(41)
        self.client.done(nav.SUCCEEDED)
        self.start(42)
        self.moving()
        self.node.on_cancel(message("901:all:"), target_seq=41)
        self.node.service_backend()
        self.assertEqual(self.acks, ["901:cancelled"])
        self.assertEqual(self.results, ["41:ok"])
        self.assertIsNone(self.node.current.cancel_at)
        self.assertEqual(self.client.cancelled, [])
        self.node.tick()
        self.assertEqual(self.commands[-1], (0.3, 0, 0, 0, 0, 0.2))
        self.client.done(nav.SUCCEEDED)
        self.assertEqual(self.results, ["41:ok", "42:ok"])

    def test_future_target_cancels_current_and_fences_unsent_target(self):
        self.start(41)
        self.node.on_cancel(message("901:all:"), target_seq=43)
        self.node.service_backend()
        self.assertEqual(self.client.cancelled, ["all"])
        self.assertEqual(self.acks, [])
        self.client.done(nav.PREEMPTED)
        self.assertEqual(self.acks, ["901:cancelled"])
        for seq in (42, 43):
            self.assertFalse(self.node.on_goal(pose(seq)))
        self.assertEqual(self.results, ["41:fail:cancelled", "42:fail:cancelled", "43:fail:cancelled"])
        self.assertTrue(self.node.on_goal(pose(44)))

    def test_rapid_goal_session_change_latches_before_sequence_check(self):
        self.lease()
        self.assertTrue(self.node.on_goal(pose(41), session="100"))
        self.node.service_backend()
        self.client.active()
        self.moving()
        self.assertFalse(self.node.on_goal(pose(1), session="101"))
        self.assertEqual(self.node.latch, "restart_required")
        self.assertEqual(self.node.session, "100")
        self.assertEqual(self.results, ["41:fail:restart_required", "1:fail:restart_required"])
        self.assertEqual(self.commands[-1], nav.ZERO)
        self.node.service_backend()
        self.assertEqual(self.client.cancelled, ["current"])
        self.client.done(nav.PREEMPTED)
        self.lease()  # Never expired: session, not the heartbeat, detected restart.
        self.assertFalse(self.node.on_goal(pose(42), session="100"))
        self.assertFalse(self.node.on_goal(pose(43), session="101"))
        self.assertEqual(self.results[-2:], ["42:fail:restart_required", "43:fail:restart_required"])
        self.assertEqual(len(self.client.sent), 1)

    def test_session_change_on_duplicate_goal_emits_only_one_failure(self):
        self.lease()
        self.assertTrue(self.node.on_goal(pose(41), session="100"))
        self.assertFalse(self.node.on_goal(pose(41), session="101"))
        self.node.service_backend()
        self.assertEqual(self.node.latch, "restart_required")
        self.assertEqual(self.results, ["41:fail:restart_required"])
        self.assertEqual(self.client.sent, [])

    def test_changed_cancel_session_is_checked_before_cached_ack(self):
        self.lease()
        self.node.on_cancel(message("900:current:"), target_seq=40, session="100")
        self.assertTrue(self.node.on_goal(pose(41), session="100"))
        self.node.service_backend()
        self.client.active()
        self.node.on_cancel(message("900:current:"), target_seq=40, session="101")
        self.assertEqual(self.node.latch, "restart_required")
        self.assertEqual(self.acks, ["900:cancelled"])
        self.assertEqual(self.results, ["41:fail:restart_required"])
        self.assertEqual(self.commands[-1], nav.ZERO)
        self.node.service_backend()
        self.assertEqual(self.client.cancelled, ["current"])

    def test_rapid_idle_session_change_also_permanently_latches(self):
        self.lease()
        self.node.on_cancel(message("900:all:"), target_seq=41, session="100")
        self.node.on_cancel(message("1:all:"), target_seq=0, session="101")
        self.assertEqual(self.node.latch, "restart_required")
        self.assertFalse(self.node.on_goal(pose(42), session="100"))
        self.assertEqual(self.results, ["42:fail:restart_required"])
        self.assertEqual(self.acks, ["900:cancelled"])
        self.assertEqual(self.client.sent, [])

    def test_untrusted_input_cannot_pin_or_change_session(self):
        self.lease()
        self.node.on_cancel(message("900:all:", "/other"), target_seq=41, session="999")
        self.assertFalse(self.node.on_goal(pose(41, "/other"), session="999"))
        self.assertEqual(self.node.session, "")
        self.assertTrue(self.node.on_goal(pose(41), session="100"))
        self.assertFalse(self.node.on_goal(pose(42, "/other"), session="101"))
        self.assertFalse(self.node.latch)
        self.assertEqual(self.results, [])

    def test_missing_session_cannot_bypass_pinned_session(self):
        self.lease()
        self.node.on_cancel(message("900:all:"), target_seq=40, session="100")
        self.assertFalse(self.node.on_goal(pose(41)))
        self.assertEqual(self.results, ["41:fail:invalid_goal"])
        self.assertEqual(self.node.session, "100")
        self.assertTrue(self.node.on_goal(pose(41), session="100"))

    def test_missing_session_duplicate_cannot_fail_a_live_goal(self):
        self.lease()
        self.assertTrue(self.node.on_goal(pose(41), session="100"))
        self.node.service_backend()
        self.assertFalse(self.node.on_goal(pose(41)))
        self.assertEqual(self.results, [])
        self.assertFalse(self.node.latch)
        self.node.client.done(nav.SUCCEEDED)
        self.assertEqual(self.results, ["41:ok"])

    def test_cancel_ack_cache_is_bounded_and_evicted_request_is_not_reapplied(self):
        for seq in range(1, 131):
            self.node.on_cancel(message("{}:all:".format(seq)), target_seq=0)
        self.assertEqual(len(self.node.ack_cache), 128)
        self.assertEqual(list(self.node.ack_cache), list(range(3, 131)))
        self.start()
        self.node.on_cancel(message("1:all:"), target_seq=0)
        self.node.on_cancel(message("130:all:"), target_seq=0)
        self.node.service_backend()
        self.assertEqual(self.client.cancelled, [])
        self.assertEqual(self.acks, ["{}:cancelled".format(seq) for seq in range(1, 131)] +
                         ["130:cancelled"])

    def test_startup_without_terminal_confirmation_never_fakes_idle_ack(self):
        self.node.startup_ready = lambda: False
        self.lease()
        self.node.on_cancel(message("900:all:"), target_seq=41)
        self.assertEqual(self.acks, ["900:ack_fail"])
        self.assertFalse(self.node.on_goal(pose(42)))
        self.assertEqual(self.results, ["42:fail:backend_unavailable"])
        self.assertEqual(self.client.sent, [])
        self.assertEqual(self.commands[-1], nav.ZERO)

    def test_goal_factory_error_has_atomic_reason(self):
        self.lease()
        self.node.make_goal = Mock(side_effect=RuntimeError("cannot construct: goal"))
        self.assertFalse(self.node.on_goal(pose(41)))
        self.assertEqual(self.results, ["41:fail:invalid_goal"])
        self.assertEqual(self.client.sent, [])

    def test_invalid_timeouts_rejected(self):
        for value in (0, -1, float("nan"), float("inf")):
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    nav.NavigationExecutor(self.client, lambda: True, Mock(), Mock(), Mock(),
                                           Mock(), lease_timeout=value)


class RosWiringTest(unittest.TestCase):
    def setUp(self):
        rospy = self.rospy = ModuleType("rospy")
        rospy.get_param = Mock(side_effect=lambda name, default: default)
        rospy.Publisher = Mock(side_effect=lambda *args, **kwargs: Mock())
        rospy.Subscriber = Mock(side_effect=lambda *args, **kwargs: Mock())
        rospy.on_shutdown = Mock()
        rospy.is_shutdown = Mock(return_value=False)
        rospy.Duration = Mock(side_effect=lambda seconds: seconds)
        actionlib = self.actionlib = ModuleType("actionlib")
        client = Client()
        client.wait_for_server = Mock(return_value=True)
        actionlib.SimpleActionClient = Mock(return_value=client)
        geometry = ModuleType("geometry_msgs.msg")
        geometry.PoseStamped = lambda: pose(1)
        rospy.Time = NS(now=lambda: 123)
        rospy.logwarn = Mock()
        geometry.Twist = lambda: twist(0, 0)
        move_base = self.move_base = ModuleType("move_base_msgs.msg")
        move_base.MoveBaseAction = object
        move_base.MoveBaseGoal = NS
        std = ModuleType("std_msgs.msg")
        std.Bool, std.String = NS, NS
        statuses = ModuleType("actionlib_msgs.msg")
        statuses.GoalStatusArray = NS
        modules = {"rospy": rospy, "actionlib": actionlib, "geometry_msgs.msg": geometry,
                   "move_base_msgs.msg": move_base, "std_msgs.msg": std, "actionlib_msgs.msg": statuses}
        module_patch = patch.dict(sys.modules, modules)
        module_patch.start()
        self.addCleanup(module_patch.stop)
        # No ROS or live backend threads: callbacks are driven explicitly below.
        with patch.object(nav.threading, "Thread"):
            self.node = nav.RosNavigationExecutor()
        self.addCleanup(self.node.shutdown)
        self.goal_callback = rospy.Subscriber.call_args_list[0].args[2]
        self.cancel_callback = rospy.Subscriber.call_args_list[2].args[2]
        self.status_callback = rospy.Subscriber.call_args_list[4].args[2]

    def backend_status(self, *states):
        self.status_callback(NS(status_list=[NS(status=state) for state in states]))

    def ready(self):
        self.node._connect_once()
        self.backend_status(nav.PREEMPTED)
        self.assertTrue(self.node._server_ready())
        self.node.executor.on_lease(message(True))

    @staticmethod
    def goal_data(seq="72", session="100", **kwargs):
        data = dict(seq=seq, session=session, frame="map", x="3", y="0",
                    z="0", qx="0", qy="0", qz="0", qw="1")
        data.update(kwargs)
        return data

    @staticmethod
    def cancel_data(seq="900", target="72", session="100", **kwargs):
        data = dict(seq=seq, goal_seq=target, session=session, scope="current", reason="")
        data.update(kwargs)
        return data

    def results(self):
        return [call.args[0].data for call in self.node.result_pub.publish.call_args_list]

    def acks(self):
        return [call.args[0].data for call in self.node.ack_pub.publish.call_args_list]

    def test_default_topics_action_types_and_message_conversion(self):
        node = self.node
        self.actionlib.SimpleActionClient.assert_called_once_with(
            "/navigation_executor/move_base", self.move_base.MoveBaseAction)
        self.assertEqual([c.args[0] for c in self.rospy.Publisher.call_args_list],
                         ["/v8/navigation/result", "/v8/navigation/cancel_ack",
                          "/v8/navigation/status", "/cmd_vel_raw"])
        self.assertEqual([c.args[0] for c in self.rospy.Subscriber.call_args_list],
                         ["/v8/navigation/goal", "/v8/bt/lease",
                          "/v8/navigation/cancel", "/v8/navigation/cmd_vel_raw_candidate",
                          "/navigation_executor/move_base/status"])
        self.assertIs(self.rospy.Subscriber.call_args_list[0].args[1], NS)
        self.assertIs(self.rospy.Subscriber.call_args_list[2].args[1], NS)
        self.assertEqual(node.executor.lease_timeout, 1.0)
        self.assertEqual(node.executor.command_timeout, 0.5)
        self.assertEqual(node.executor.cancel_timeout, 2.0)
        self.assertFalse(node._server_ready())
        self.ready()
        self.goal_callback(message(json.dumps(self.goal_data())))
        node.executor.service_backend()
        sent_pose = node.client.sent[0][0].target_pose
        self.assertEqual(sent_pose.header.seq, 72)
        self.assertEqual(sent_pose.pose.position.x, 3.0)
        self.assertEqual(sent_pose.header.frame_id, "map")
        self.assertEqual(node.executor.session, "100")
        node.client.active()
        node.executor.on_candidate(twist())
        node.executor.tick()
        sent = node.command_pub.publish.call_args.args[0]
        self.assertEqual(sent.linear.x, 0.3)
        self.assertEqual(sent.angular.z, 0.2)
        node.client.done(nav.SUCCEEDED)
        self.assertEqual(self.results(), ["72:ok"])
        node.shutdown()
        self.assertEqual(node.command_pub.publish.call_args.args[0].linear.x, 0.0)

    def test_numeric_json_values_and_explicit_seq_ignore_any_transport_header(self):
        self.ready()
        msg = message(json.dumps(self.goal_data(seq=73, x=3, y=0.0)))
        msg.header = NS(seq=999)
        self.goal_callback(msg)
        self.node.executor.service_backend()
        self.assertEqual(self.node.executor.current.seq, 73)
        self.node.client.done(nav.SUCCEEDED)
        self.assertEqual(self.results(), ["73:ok"])

    def test_json_cancel_empty_reason_uses_its_target_not_request_id(self):
        self.ready()
        self.goal_callback(message(json.dumps(self.goal_data())))
        self.node.executor.service_backend()
        self.node.client.active()
        self.cancel_callback(message(json.dumps(self.cancel_data())))
        self.node.executor.service_backend()
        self.assertEqual(self.node.client.cancelled, ["all", "current"])
        self.assertEqual(self.acks(), [])
        self.node.client.done(nav.PREEMPTED)
        self.assertEqual(self.acks(), ["900:cancelled"])
        self.assertEqual(self.results(), ["72:fail:cancelled"])

    def test_json_cancel_before_goal_blocks_later_delivery(self):
        self.ready()
        self.cancel_callback(message(json.dumps(self.cancel_data())))
        self.assertEqual(self.acks(), ["900:cancelled"])
        self.goal_callback(message(json.dumps(self.goal_data())))
        self.node.executor.service_backend()
        self.assertEqual(self.results(), ["72:fail:cancelled"])
        self.assertEqual(self.node.client.sent, [])

    def test_json_session_change_is_forwarded_on_both_inputs(self):
        self.ready()
        self.goal_callback(message(json.dumps(self.goal_data())))
        self.cancel_callback(message(json.dumps(self.cancel_data(session="101"))))
        self.assertEqual(self.node.executor.latch, "restart_required")
        self.node.executor.service_backend()
        self.assertEqual(self.node.client.sent, [])
        self.assertEqual(self.results(), ["72:fail:restart_required"])
        self.assertEqual(self.acks(), [])

    def test_wire_rejects_legacy_missing_extra_and_duplicate_fields(self):
        self.ready()
        bad_goal = self.goal_data()
        del bad_goal["session"]
        for payload in (json.dumps(bad_goal), json.dumps(self.goal_data(extra=1)), "[]",
                        "null", "72:map:3:0", json.dumps(self.goal_data())[:-1] + ',"seq":"73"}'):
            with self.subTest(payload=payload):
                self.goal_callback(message(payload))
        bad_cancel = self.cancel_data()
        del bad_cancel["goal_seq"]
        for payload in ("900:all:", json.dumps(bad_cancel), json.dumps(self.cancel_data(extra=1))):
            with self.subTest(payload=payload):
                self.cancel_callback(message(payload))
        self.assertIsNone(self.node.executor.current)
        self.assertEqual(self.node.executor.session, "")
        self.assertEqual(self.results(), [])
        self.assertEqual(self.acks(), [])

    def test_wire_identity_and_session_validation(self):
        self.ready()
        for value in ("", "-1", "1.0", " 1", True, None, [], {}):
            with self.subTest(value=value):
                self.goal_callback(message(json.dumps(self.goal_data(seq=value))))
                self.cancel_callback(message(json.dumps(self.cancel_data(target=value))))
        for session in ("", "abc", "-1", "1.0", 123, None):
            with self.subTest(session=session):
                self.goal_callback(message(json.dumps(self.goal_data(session=session))))
                self.cancel_callback(message(json.dumps(self.cancel_data(session=session))))
        self.goal_callback(message(json.dumps(self.goal_data(seq=0x100000000))))
        self.cancel_callback(message(json.dumps(self.cancel_data(seq=0x10000000000000000))))
        self.assertEqual(self.node.executor.session, "")
        self.assertEqual(self.results(), [])
        self.assertEqual(self.acks(), [])
        self.assertIsNone(self.node.executor.current)

    def test_wire_invalid_pose_has_atomic_scoped_failure(self):
        self.ready()
        for seq, value in enumerate(("not a number", None, True, {}, "nan", "inf"), 1):
            self.goal_callback(message(json.dumps(self.goal_data(seq=seq, x=value))))
        self.assertEqual(self.results(), ["{}:fail:invalid_pose".format(seq) for seq in range(1, 7)])
        self.assertEqual(self.node.client.sent, [])

    def test_startup_cancels_all_before_readiness_and_waits_for_terminal_status(self):
        node = self.node
        self.assertEqual(node.command_pub.publish.call_args.args[0].linear.x, 0.0)
        self.backend_status()  # Earlier/latched observations do not open the gate.
        self.assertFalse(node.startup_confirmed)
        node._connect_once()
        self.assertEqual(node.client.cancelled, ["all"])
        self.assertFalse(node._server_ready())
        node.executor.on_lease(message(True))
        self.goal_callback(message(json.dumps(self.goal_data())))
        self.assertEqual(self.results(), ["72:fail:backend_unavailable"])
        for state in (nav.PENDING, nav.ACTIVE, nav.PREEMPTING, nav.RECALLING, nav.LOST, 99):
            self.backend_status(nav.SUCCEEDED, state)
            node.executor.tick()
            self.assertFalse(node._server_ready())
            self.assertEqual(node.command_pub.publish.call_args.args[0].linear.x, 0.0)
        self.backend_status(nav.PREEMPTED, nav.RECALLED, nav.SUCCEEDED, nav.ABORTED, nav.REJECTED)
        self.assertTrue(node._server_ready())
        node._connect_once()
        self.assertEqual(node.client.cancelled, ["all"])
        self.goal_callback(message(json.dumps(self.goal_data(seq="73"))))
        node.executor.service_backend()
        self.assertEqual(len(node.client.sent), 1)

    def test_startup_empty_status_is_quiescent_but_not_an_external_cancel_ack(self):
        self.node._connect_once()
        self.cancel_callback(message(json.dumps(self.cancel_data())))
        self.assertEqual(self.acks(), ["900:ack_fail"])
        self.backend_status()
        self.assertTrue(self.node._server_ready())
        self.assertEqual(self.acks(), ["900:ack_fail"])
        self.assertEqual(self.results(), [])

    def test_startup_cancel_exception_keeps_gate_closed(self):
        self.node.client.cancel_all_goals = Mock(side_effect=RuntimeError("transport failed"))
        self.node._connect_once()
        self.backend_status()
        self.assertFalse(self.node._server_ready())
        self.assertFalse(self.node.startup_cancel_sent)
        self.assertFalse(self.node.startup_confirmed)
        self.assertEqual(self.acks(), [])

    def test_blocked_startup_cancel_does_not_stop_zero_loop(self):
        entered, release = threading.Event(), threading.Event()

        def blocked_cancel():
            entered.set()
            release.wait(2)

        self.node.client.cancel_all_goals = blocked_cancel
        worker = threading.Thread(target=self.node._connect_once)
        worker.start()
        try:
            self.assertTrue(entered.wait(1))
            self.backend_status()
            self.node.executor.tick()
            self.assertFalse(self.node._server_ready())
            self.assertEqual(self.node.command_pub.publish.call_args.args[0].linear.x, 0.0)
            self.assertEqual(self.acks(), [])
        finally:
            release.set()
            worker.join(2)
        self.assertFalse(worker.is_alive())
        self.assertFalse(self.node._server_ready())
        self.backend_status()
        self.assertTrue(self.node._server_ready())


if __name__ == "__main__":
    unittest.main()
