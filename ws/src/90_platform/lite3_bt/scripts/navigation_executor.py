#!/usr/bin/env python3
"""BT-owned, fail-closed MoveBaseAction adapter; all safety clocks are wall clocks."""

import copy
import json
import math
import re
import threading
import time
from collections import OrderedDict
from dataclasses import dataclass, field


PENDING, ACTIVE, PREEMPTED, SUCCEEDED, ABORTED = range(5)
REJECTED, PREEMPTING, RECALLING, RECALLED, LOST = range(5, 10)
TERMINAL = frozenset((PREEMPTED, SUCCEEDED, ABORTED, REJECTED, RECALLED))
ZERO = (0.0,) * 6
# Legacy text is an internal policy-test representation, never an external wire.
CANCEL_PATTERN = re.compile(r"([1-9][0-9]*):(current|all):(.*)", re.DOTALL)
SESSION_PATTERN = re.compile(r"[0-9]+")


def _wire_uint(value, maximum, allow_zero=False):
    if type(value) not in (int, str) or not re.fullmatch(r"0|[1-9][0-9]{0,19}", str(value)):
        raise ValueError("invalid request identity")
    number = int(value)
    if not (0 if allow_zero else 1) <= number <= maximum:
        raise ValueError("request identity out of range")
    return number


def _wire_object(text, fields):
    def unique(pairs):
        result = {}
        for key, value in pairs:
            if key in result:
                raise ValueError("duplicate request field")
            result[key] = value
        return result

    data = json.loads(text, object_pairs_hook=unique)
    if not isinstance(data, dict) or set(data) != fields:
        raise ValueError("unexpected request fields")
    if not isinstance(data["session"], str) or not SESSION_PATTERN.fullmatch(data["session"]):
        raise ValueError("invalid session")
    return data


@dataclass
class _Action:
    seq: int
    generation: int
    goal: object
    phase: str = "queued"
    backend_state: int = PENDING
    cancel_at: object = None
    cancel_mode: str = "current"
    cancel_sent: bool = False
    cancel_ids: set = field(default_factory=set)
    result_sent: bool = False
    timed_out: bool = False


class NavigationExecutor:
    """ROS-free policy. Publish callbacks and server_ready must be nonblocking.

    One backend worker calls service_backend(); an independent wall loop calls
    tick(). Neither a blocked actionlib call nor paused /clock can stop zeroing.
    Lease loss after the first TRUE or a changed BT session permanently requires
    a coupled backend/adapter restart. Cancellation fences its target goal seq,
    independently of the cancel request id and cross-topic delivery order.
    """

    def __init__(self, client, server_ready, publish_result, publish_ack,
                 publish_status, publish_command, make_goal=lambda pose: pose,
                 clock=time.monotonic, expected_caller="/bt_console",
                 lease_timeout=1.0, command_timeout=0.5, cancel_timeout=2.0,
                 startup_ready=lambda: True):
        for value in (lease_timeout, command_timeout, cancel_timeout):
            if not math.isfinite(value) or value <= 0:
                raise ValueError("timeouts must be finite and positive")
        self.client = client
        self.server_ready = server_ready
        self.publish_result = publish_result
        self.publish_ack = publish_ack
        self.publish_status = publish_status
        self.publish_command = publish_command
        self.make_goal = make_goal
        self.clock = clock
        self.expected_caller = expected_caller
        self.lease_timeout = lease_timeout
        self.command_timeout = command_timeout
        self.cancel_timeout = cancel_timeout
        self.lock = threading.RLock()
        self.backend_lock = threading.Lock()
        self.current = None
        self.generation = 0
        self.last_goal_seq = 0
        self.last_cancel_seq = 0
        self.cancelled_goal_seq = 0
        self.session = ""
        self.startup_ready = startup_ready
        self.ack_cache = OrderedDict()
        self.lease = False
        self.lease_at = None
        self.lease_seen = False
        self.latch = ""
        self.stopped = False
        self.candidate = None
        self.candidate_at = None

    def _status(self, seq, state, reason, detail="", backend_state=None):
        self.publish_status(json.dumps(dict(seq=seq, state=state, reason=reason,
                                            detail=detail, backend_state=backend_state),
                                       allow_nan=False, sort_keys=True))

    def _trusted(self, msg, channel):
        header = getattr(msg, "_connection_header", None) or {}
        # ROS callerid is a spoofable ownership check, not authentication.
        caller = header.get("callerid")
        if caller is not None and caller != self.expected_caller:
            self._status(0, "ignored", "unexpected_caller", channel + ":" + str(caller))
            return False
        if str(header.get("latching", "0")) == "1":
            self._status(0, "ignored", "latched_input", channel)
            return False
        return True

    def _fresh_lease(self, now):
        return (self.lease and self.lease_at is not None and
                0 <= now - self.lease_at <= self.lease_timeout)

    def _zero(self):
        self.candidate = self.candidate_at = None
        self.publish_command(ZERO)

    def _session_matches(self, session):
        # Empty sessions are supported only by the pre-session, ROS-free test API.
        if not session and not self.session:
            return True
        if not isinstance(session, str) or not SESSION_PATTERN.fullmatch(session):
            return False
        if self.session and session != self.session:
            self.latch = "restart_required"
            self._fault("restart_required", "BT session changed; restart backend and adapter together")
            return False
        self.session = session
        return True

    def _failure(self, seq, reason):
        self.publish_result("{}:fail:{}".format(seq, reason))

    def _result(self, action, ok, reason):
        if not action.result_sent:
            action.result_sent = True
            if ok:
                self.publish_result("{}:ok".format(action.seq))
            else:
                self._failure(action.seq, reason)

    def _ack(self, seq, outcome):
        self.ack_cache[seq] = outcome
        while len(self.ack_cache) > 128:
            self.ack_cache.popitem(last=False)
        self.publish_ack("{}:{}".format(seq, outcome))

    def _cancel(self, action, now, mode="current"):
        if action.cancel_at is None:
            action.cancel_at = now
            action.cancel_mode = mode
        elif mode == "all" and not action.cancel_sent:
            action.cancel_mode = mode
        self._zero()

    def _fault(self, reason, detail=""):
        self.latch = self.latch or reason
        self._zero()
        action = self.current
        if action is not None:
            self._cancel(action, self.clock())
            self._result(action, False, self.latch)
        self._status(action.seq if action else self.last_goal_seq, "latched",
                     reason, detail, action.backend_state if action else None)

    def _guard(self, now):
        if self.lease_seen and not self._fresh_lease(now) and not self.latch:
            self._fault("lease_lost", "restart executor before a new BT session")
        action = self.current
        if (action and action.cancel_at is not None and not action.timed_out and
                now - action.cancel_at >= self.cancel_timeout):
            action.timed_out = True
            self._fault("cancel_timeout", "backend termination is unconfirmed")
            for seq in sorted(action.cancel_ids):
                self._ack(seq, "ack_fail")
            action.cancel_ids.clear()

    def on_lease(self, msg):
        with self.lock:
            if not self._trusted(msg, "lease"):
                return
            now = self.clock()
            self._guard(now)  # A late TRUE must not heal a missed heartbeat.
            self.lease = msg.data is True
            self.lease_at = now
            self.lease_seen = self.lease_seen or self.lease
            self._guard(now)
            if not self.lease:
                self._zero()

    @staticmethod
    def _pose_error(msg):
        try:
            p, q = msg.pose.position, msg.pose.orientation
            values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
            if not all(math.isfinite(v) for v in values):
                return "pose contains nonfinite values"
            if not isinstance(msg.header.frame_id, str) or not msg.header.frame_id.strip():
                return "frame_id is empty"
            if p.z != 0.0 or q.x != 0.0 or q.y != 0.0:
                return "planar backend rejects nonzero z/roll/pitch"
            norm = sum(v * v for v in (q.x, q.y, q.z, q.w))
            if not math.isclose(norm, 1.0, rel_tol=0.0, abs_tol=1e-6):
                return "orientation must be a normalized quaternion"
        except (AttributeError, TypeError, ValueError, OverflowError):
            return "malformed pose"
        return ""

    def on_goal(self, msg, session=""):
        with self.lock:
            if not self._trusted(msg, "goal"):
                return False
            seq = getattr(getattr(msg, "header", None), "seq", None)
            # Check session before sequence ordering: a restarted BT may reset seq.
            if not self._session_matches(session):
                reason = "restart_required" if self.latch else "invalid_goal"
                if type(seq) is int and 0 < seq <= 0xffffffff:
                    if ((self.latch or seq > self.last_goal_seq) and
                            not (self.current and self.current.seq == seq)):
                        self._failure(seq, reason)
                self._status(seq if type(seq) is int else 0, "rejected", reason, "invalid BT session")
                return False
            now = self.clock()
            self._guard(now)
            if type(seq) is not int or not 0 < seq <= 0xffffffff:
                self._status(0, "rejected", "invalid_seq")
                return False
            if seq <= self.last_goal_seq:
                # No duplicate result: a failure for a live duplicate could kill BT's goal.
                self._status(seq, "ignored", "stale_or_duplicate_seq")
                return False
            self.last_goal_seq = seq
            error = self._pose_error(msg)
            reason = ("shutdown" if self.stopped else "restart_required" if self.latch else
                      "cancelled" if seq <= self.cancelled_goal_seq else
                      "busy" if self.current is not None else
                      "no_lease" if not self._fresh_lease(now) else
                      "invalid_pose" if error else
                      "backend_unavailable" if not self.startup_ready() or not self.server_ready() else "")
            if reason:
                self._failure(seq, reason)
                self._status(seq, "rejected", reason, error or self.latch)
                return False
            try:
                goal = self.make_goal(copy.deepcopy(msg))
            except Exception as exc:
                self._failure(seq, "invalid_goal")
                self._status(seq, "rejected", "invalid_goal", str(exc))
                return False
            self.generation += 1
            self.current = _Action(seq, self.generation, goal)
            self._zero()
            self._status(seq, "pending", "accepted", backend_state=PENDING)
            return True

    def on_cancel(self, msg, target_seq=None, session=""):
        with self.lock:
            if not self._trusted(msg, "cancel"):
                return
            if not self._session_matches(session):
                self._status(0, "ignored", "restart_required" if self.latch else "invalid_cancel")
                return
            payload = getattr(msg, "data", "")
            match = CANCEL_PATTERN.fullmatch(payload) if isinstance(payload, str) else None
            if match is None or len(match[1]) > 20 or int(match[1]) > 0xffffffffffffffff:
                self._status(0, "ignored", "invalid_cancel")
                return
            seq, mode, reason = int(match[1]), match[2], match[3]
            action = self.current
            # Omitted target is only for internal legacy tests, never the JSON wire.
            if target_seq is None:
                target_seq = action.seq if action else self.last_goal_seq
            if type(target_seq) is not int or not 0 <= target_seq <= 0xffffffff:
                self._status(seq, "ignored", "invalid_cancel")
                return
            now = self.clock()
            self._guard(now)
            if seq in self.ack_cache:
                self.publish_ack("{}:{}".format(seq, self.ack_cache[seq]))
                return
            if seq <= self.last_cancel_seq:
                self._status(seq, "ignored", "stale_or_pending_cancel")
                return
            self.last_cancel_seq = seq
            # Persist even after idle ACK: a goal delayed on the other topic must
            # never start after its cancellation has already been confirmed.
            self.cancelled_goal_seq = max(self.cancelled_goal_seq, target_seq)
            if action is None or target_seq < action.seq:
                if action is None:
                    self._zero()
                # Single-flight means older targets are terminal (or never sent).
                # Startup is different: an inherited backend goal is unconfirmed.
                outcome = "cancelled" if self.startup_ready() else "ack_fail"
                self._ack(seq, outcome)
                self._status(seq, "idle", "already_terminated" if outcome == "cancelled" else
                             "backend_unavailable", reason)
                return
            # A future target is fenced above; cancel the current action as well.
            self._cancel(action, now, mode)
            if action.timed_out or action.backend_state == LOST:
                self._ack(seq, "ack_fail")
            else:
                action.cancel_ids.add(seq)
            self._status(action.seq, "cancelling", "cancel_requested", reason,
                         action.backend_state)

    def on_candidate(self, msg):
        with self.lock:
            now = self.clock()
            self._guard(now)
            action = self.current
            if not self._can_move(action, now):
                self._zero()
                return
            try:
                values = tuple(float(v) for v in (msg.linear.x, msg.linear.y, msg.linear.z,
                                                 msg.angular.x, msg.angular.y, msg.angular.z))
                valid = all(math.isfinite(v) for v in values) and values[2:5] == ZERO[2:5]
            except (AttributeError, TypeError, ValueError, OverflowError):
                valid = False
            if not valid:
                self._zero()
                self._status(action.seq, "active", "invalid_candidate",
                             backend_state=action.backend_state)
                return
            self.candidate, self.candidate_at = values, now

    def _can_move(self, action, now):
        return (not self.stopped and not self.latch and self._fresh_lease(now) and
                action is not None and action.phase == "active" and
                action.backend_state == ACTIVE and action.cancel_at is None)

    def tick(self):
        with self.lock:
            now = self.clock()
            self._guard(now)
            action = self.current
            if (action and action.phase != "queued" and not self.latch and
                    not self.server_ready()):
                self._fault("backend_lost", "backend_disconnected")
            valid = (self._can_move(action, now) and self.candidate is not None and
                     0 <= now - self.candidate_at <= self.command_timeout)
            self.publish_command(self.candidate if valid else ZERO)

    def _active(self, generation):
        with self.lock:
            action = self.current
            if action is None or action.generation != generation:
                return
            self._guard(self.clock())
            action.phase, action.backend_state = "active", ACTIVE
            self.candidate = self.candidate_at = None
            self._status(action.seq, "cancelling" if action.cancel_at is not None else "active",
                         "backend_active", backend_state=ACTIVE)

    def _done(self, generation, state, result=None):
        with self.lock:
            action = self.current
            if action is None or action.generation != generation:
                return
            self._guard(self.clock())
            action.backend_state = state
            self._zero()
            if state not in TERMINAL:
                action.backend_state = LOST
                self._fault("backend_lost", "terminal state is not confirmed: {}".format(state))
                for seq in sorted(action.cancel_ids):
                    self._ack(seq, "ack_fail")
                action.cancel_ids.clear()
                return
            ok = state == SUCCEEDED and action.cancel_at is None and not self.latch
            reason = self.latch or ("cancelled" if action.cancel_at is not None else "backend_terminal")
            self._result(action, ok, reason)
            for seq in sorted(action.cancel_ids):
                self._ack(seq, "cancelled")
            action.cancel_ids.clear()
            self.current = None
            self._status(action.seq, "latched" if self.latch else "succeeded" if ok else "failed",
                         self.latch or ("cancelled" if action.cancel_at is not None else
                                        "backend_terminal"), backend_state=state)

    def service_backend(self):
        # Serialize client operations without taking the watchdog's state lock across IO.
        if not self.backend_lock.acquire(blocking=False):
            return
        try:
            with self.lock:
                self._guard(self.clock())
                action = self.current
                if action is None:
                    return
                generation = action.generation
                if action.phase == "queued":
                    if action.cancel_at is not None:
                        # Never sent: no backend action exists to terminate.
                        self._done(generation, RECALLED)
                        return
                    if not self.server_ready():
                        self._result(action, False, "backend_unavailable")
                        self.current = None
                        self._zero()
                        self._status(action.seq, "rejected", "backend_unavailable")
                        return
                    action.phase = "sending"
                    operation = "send"
                elif action.cancel_at is not None and not action.cancel_sent:
                    action.cancel_sent = True
                    operation = "cancel"
                else:
                    operation = "poll"
            try:
                if operation == "send":
                    self.client.send_goal(
                        action.goal,
                        done_cb=lambda state, result: self._done(generation, state, result),
                        active_cb=lambda: self._active(generation))
                    with self.lock:
                        if self.current is action and action.phase == "sending":
                            action.phase = "pending"
                elif operation == "cancel":
                    if action.cancel_mode == "all":
                        self.client.cancel_all_goals()
                    else:
                        self.client.cancel_goal()
                else:
                    state = self.client.get_state()
                    if state in TERMINAL or state == LOST:
                        self._done(generation, state)
                    elif state in (PREEMPTING, RECALLING):
                        with self.lock:
                            if self.current is action:
                                self._cancel(action, self.clock())
            except Exception as exc:
                with self.lock:
                    if self.current is action:
                        self._fault("backend_lost", "backend_io_error: " + str(exc))
        finally:
            self.backend_lock.release()

    def shutdown(self):
        with self.lock:
            if self.stopped:
                return
            self.stopped = True
            self._fault("shutdown")


class RosNavigationExecutor:
    """Startup cancellation is a zero-output gate, not autonomous reconciliation.

    Backend and adapter must both be required=true, with no individual respawn;
    restart the whole launch together. GoalStatusArray has no cancel-request
    correlation or server-incarnation identity, so a post-cancel terminal/empty
    observation alone cannot prove safety across independent process restarts.
    """

    def __init__(self):
        import actionlib
        import rospy
        from actionlib_msgs.msg import GoalStatusArray
        from geometry_msgs.msg import PoseStamped, Twist
        from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
        from std_msgs.msg import Bool, String

        self.rospy = rospy
        self.stop = threading.Event()
        self.connection = (False, 0.0)
        self.startup_lock = threading.Lock()
        self.startup_cancel_sent = False
        self.startup_confirmed = False
        backend_action = rospy.get_param("~backend_action", "/navigation_executor/move_base")
        self.client = actionlib.SimpleActionClient(backend_action, MoveBaseAction)
        self.result_pub = rospy.Publisher(rospy.get_param("~result_topic", "/v8/navigation/result"),
                                          String, queue_size=20)
        self.ack_pub = rospy.Publisher(rospy.get_param("~cancel_ack_topic", "/v8/navigation/cancel_ack"),
                                       String, queue_size=20)
        self.status_pub = rospy.Publisher(rospy.get_param("~status_topic", "/v8/navigation/status"),
                                          String, queue_size=20)
        self.command_pub = rospy.Publisher(rospy.get_param("~output_topic", "/cmd_vel_raw"),
                                           Twist, queue_size=1)

        def command(values):
            msg = Twist()
            msg.linear.x, msg.linear.y, msg.linear.z = values[:3]
            msg.angular.x, msg.angular.y, msg.angular.z = values[3:]
            self.command_pub.publish(msg)

        def goal(pose):
            msg = MoveBaseGoal()
            msg.target_pose = pose
            return msg

        self.executor = NavigationExecutor(
            self.client, self._server_ready,
            lambda text: self.result_pub.publish(String(data=text)),
            lambda text: self.ack_pub.publish(String(data=text)),
            lambda text: self.status_pub.publish(String(data=text)), command, goal,
            expected_caller=rospy.get_param("~expected_caller", "/bt_console"),
            lease_timeout=float(rospy.get_param("~lease_timeout", 1.0)),
            command_timeout=float(rospy.get_param("~command_timeout", 0.5)),
            cancel_timeout=float(rospy.get_param("~cancel_timeout", 2.0)),
            startup_ready=lambda: self.startup_confirmed)
        self.executor.tick()  # Zero before subscribing or attempting any backend IO.

        def request(message):
            if not self.executor._trusted(message, "goal"):
                return
            try:
                data = _wire_object(message.data,
                                    {"seq", "session", "frame", "x", "y", "z", "qx", "qy", "qz", "qw"})
                seq = _wire_uint(data["seq"], 0xffffffff)
                pose = PoseStamped()
                pose._connection_header = getattr(message, "_connection_header", None)
                # This header is strictly internal: ROS never transports the goal
                # pose before policy consumes the explicit JSON application seq.
                pose.header.seq = seq
                pose.header.frame_id = data["frame"]
                pose.header.stamp = rospy.Time.now()
                try:
                    values = [data[key] for key in ("x", "y", "z", "qx", "qy", "qz", "qw")]
                    if any(type(value) not in (int, float, str) for value in values):
                        raise ValueError("invalid pose number")
                    (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z,
                     pose.pose.orientation.x, pose.pose.orientation.y,
                     pose.pose.orientation.z, pose.pose.orientation.w) = map(float, values)
                except (TypeError, ValueError, OverflowError):
                    # Let policy reject the identified goal atomically, including
                    # its duplicate/session checks, rather than just logging it.
                    pose.pose.position.x = float("nan")
                self.executor.on_goal(pose, session=data["session"])
            except (TypeError, ValueError, KeyError, AttributeError, OverflowError) as error:
                rospy.logwarn("Rejected navigation request: %s", error)

        def cancel(message):
            if not self.executor._trusted(message, "cancel"):
                return
            try:
                data = _wire_object(message.data, {"seq", "goal_seq", "session", "scope", "reason"})
                seq = _wire_uint(data["seq"], 0xffffffffffffffff)
                target = _wire_uint(data["goal_seq"], 0xffffffff, allow_zero=True)
                if data["scope"] not in ("current", "all") or not isinstance(data["reason"], str):
                    raise ValueError("invalid cancellation")
                internal = String(data="{}:{}:{}".format(seq, data["scope"], data["reason"]))
                internal._connection_header = getattr(message, "_connection_header", None)
                self.executor.on_cancel(internal, target_seq=target, session=data["session"])
            except (TypeError, ValueError, KeyError, AttributeError) as error:
                rospy.logwarn("Rejected navigation cancellation: %s", error)

        self.subscribers = [
            rospy.Subscriber(rospy.get_param("~goal_topic", "/v8/navigation/goal"), String,
                             request, queue_size=1),
            rospy.Subscriber(rospy.get_param("~lease_topic", "/v8/bt/lease"), Bool,
                             self.executor.on_lease, queue_size=1),
            rospy.Subscriber(rospy.get_param("~cancel_topic", "/v8/navigation/cancel"), String,
                             cancel, queue_size=20),
            rospy.Subscriber(rospy.get_param("~candidate_topic", "/v8/navigation/cmd_vel_raw_candidate"),
                             Twist, self.executor.on_candidate, queue_size=1),
            rospy.Subscriber(backend_action.rstrip("/") + "/status", GoalStatusArray,
                             self._startup_status, queue_size=1)]
        self.connection_thread = threading.Thread(target=self._connection_loop, daemon=True)
        self.backend_thread = threading.Thread(target=self._backend_loop, daemon=True)
        rospy.on_shutdown(self.shutdown)
        self.connection_thread.start()
        self.backend_thread.start()

    def _server_ready(self):
        ready, at = self.connection
        return self.startup_confirmed and ready and 0 <= time.monotonic() - at <= 0.5

    def _startup_status(self, message):
        with self.startup_lock:
            if self.startup_cancel_sent and not self.startup_confirmed:
                # LOST/unknown is not terminal evidence; no timeout opens this gate.
                self.startup_confirmed = all(status.status in TERMINAL for status in message.status_list)

    def _connect_once(self):
        try:
            ready = self.client.wait_for_server(self.rospy.Duration(0.1))
            if ready and not self.startup_cancel_sent:
                # No policy goal can pass readiness yet. Do not hold the state lock
                # across IO: a stuck cancel must not stop the wall-clock zero loop.
                self.client.cancel_all_goals()
                with self.startup_lock:
                    self.startup_cancel_sent = True
            self.connection = (ready, time.monotonic())
        except Exception:
            self.connection = (False, time.monotonic())

    def _connection_loop(self):
        # wait_for_server uses ROS time; isolate it from the wall-clock watchdog.
        while not self.stop.is_set() and not self.rospy.is_shutdown():
            self._connect_once()
            self.stop.wait(0.05)

    def _backend_loop(self):
        while not self.stop.is_set():
            self.executor.service_backend()
            self.stop.wait(0.05)
        self.executor.service_backend()

    def spin(self):
        try:
            while not self.rospy.is_shutdown() and not self.stop.is_set():
                self.executor.tick()
                self.stop.wait(0.05)
        finally:
            self.shutdown()

    def shutdown(self):
        if self.stop.is_set():
            return
        self.executor.shutdown()
        self.stop.set()
        if threading.current_thread() is not self.backend_thread:
            self.backend_thread.join(timeout=0.2)
        self.executor.tick()


def main():
    import rospy
    rospy.init_node("navigation_executor")
    RosNavigationExecutor().spin()


if __name__ == "__main__":
    main()
