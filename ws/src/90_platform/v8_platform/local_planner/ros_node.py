#!/usr/bin/env python3
"""ROS1 adapter for the native space-time planner; explicit map-frame 3D IO.

Separate wall-clock controller and planning loops ensure a slow/failed planner
cannot keep an old nonzero command alive. The optional plain-Twist output is a
validated yaw-plane body adapter; the stamped 3D stream remains in map coordinates.
"""
import json
import math
import multiprocessing as mp
import queue
from pathlib import Path
import sys
import threading
import time

import local_native as native

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from common.pose_safety import (PoseError, PoseFreshness, finite_number, finite_vector,
                                map_to_body_planar, normalized_quaternion)


def freshness(now, pose_received, obstacles_received, localization_received,
              localization_ok, max_age_s):
    if not localization_ok or now - localization_received >= max_age_s:
        return 'E_LOCAL_LOCALIZATION'
    if now - pose_received >= max_age_s:
        return 'E_LOCAL_POSE_STALE'
    if now - obstacles_received >= max_age_s:
        return 'E_LOCAL_OBSTACLES_STALE'
    return None


def feedback_at(pose, observed_at, now, commands):
    """Predict to controller time from timestamped feedback and issued commands.

    This changes the controller estimate only. Received/raw poses are never
    moved or snapped. Piecewise integration avoids braking against an old pose
    while the previous nonzero command is still moving the experimental carrier.
    """
    if observed_at <= 0:
        return tuple(pose)
    if observed_at > now:
        raise ValueError('pose timestamp is in the future')
    position, cursor, velocity = tuple(pose), observed_at, (0., 0., 0.)
    for stamp, issued in commands:
        if stamp <= observed_at:
            velocity = tuple(issued)
        elif stamp <= now:
            position = tuple(position[i] + (stamp - cursor) * velocity[i] for i in range(3))
            cursor, velocity = stamp, tuple(issued)
        else:
            break
    return tuple(position[i] + (now - cursor) * velocity[i] for i in range(3))


def feedback_uncertainty(observed_at, now, commands, latency_bound_s):
    """Distance bound from issued-vs-applied timing and one pending command."""
    velocity, variation = (0., 0., 0.), 0.
    for stamp, issued in commands:
        if stamp > now:
            break
        if stamp >= observed_at:
            variation += math.sqrt(sum((issued[i] - velocity[i]) ** 2 for i in range(3)))
        velocity = tuple(issued)
    return latency_bound_s * (variation + math.sqrt(sum(v * v for v in velocity)))


NON_LATCHING = {'E_LOCAL_WORLD_UPDATED', 'E_LOCAL_EXPIRED', 'E_LOCAL_NO_TRAJECTORY',
                'E_LOCAL_NOT_READY', 'E_LOCAL_INPUT'}


def stop_reason(fresh, latched, latched_reason, result, result_at, wall_now,
                wall_watchdog_s, sim_now, generation, result_generation, last_error):
    """Ordered fail-closed policy; plan validity is sim time, wall is a fallback.

    World updates precede plan expiry so a sensed change is not relabelled as
    an expired plan. A terminal reason latches until a new goal arrives.
    """
    if fresh is not None:
        return fresh
    if latched:
        return latched_reason or 'E_LOCAL_STOP_LATCHED'
    if result is None:
        return last_error if last_error and last_error != 'ok' else 'E_LOCAL_NO_TRAJECTORY'
    if generation != result_generation:
        return 'E_LOCAL_WORLD_UPDATED'
    if not isinstance(result, dict) or not finite_number(result.get('valid_until')):
        return 'E_LOCAL_INPUT'
    valid_until = result.get('valid_until')
    if sim_now and sim_now > 0 and valid_until is not None and sim_now >= valid_until:
        return 'E_LOCAL_EXPIRED'
    if wall_now - result_at >= wall_watchdog_s:
        return 'E_LOCAL_WALL_EXPIRED'
    return None


def map_from_message(text):
    """Accept a raw support_map or a delta_map_node snapshot envelope."""
    value = json.loads(text)
    if isinstance(value, dict) and 'map' in value:
        value = value['map']
    if not isinstance(value, dict):
        raise ValueError('map message must be an object')
    return value


def _planner_worker(requests, responses):
    """Run pure-Python planning away from ROS callbacks and the control loop."""
    while True:
        item = requests.get()
        if item is None:
            return
        started = time.perf_counter()
        try:
            result = native.plan(item['map'], item['pose'], item['goal'], item['obstacles'],
                                 now=item['now'], config=item['config'],
                                 robot_profile=item['profile'])
            responses.put({'seq': item['seq'], 'generation': item['generation'],
                           'ok': True, 'result': result,
                           'plan_ms': round((time.perf_counter() - started) * 1e3, 3)})
        except native.LocalError as error:
            responses.put({'seq': item['seq'], 'generation': item['generation'],
                           'ok': False, 'error': getattr(error, 'code', 'E_LOCAL_INPUT'),
                           'message': str(error),
                           'plan_ms': round((time.perf_counter() - started) * 1e3, 3)})
        except Exception as error:
            responses.put({'seq': item['seq'], 'generation': item['generation'],
                           'ok': False, 'error': 'E_LOCAL_INPUT', 'message': repr(error),
                           'plan_ms': round((time.perf_counter() - started) * 1e3, 3)})


class PlannerWorker:
    """Single fork worker; the parent keeps ROS callbacks and control responsive."""

    def __init__(self, config, profile):
        ctx = mp.get_context('fork')
        self.requests = ctx.Queue(maxsize=1)
        self.responses = ctx.Queue(maxsize=1)
        self.process = ctx.Process(target=_planner_worker, args=(self.requests, self.responses),
                                   daemon=True)
        self.process.start()
        self.config = config
        self.profile = profile
        self.seq = 0
        self.pending = None

    def submit(self, map_value, pose, goal, obstacles, now, generation):
        if self.pending is not None or not self.process.is_alive():
            return False
        self.seq += 1
        try:
            self.requests.put_nowait({'seq': self.seq, 'map': map_value, 'pose': pose,
                                      'goal': goal, 'obstacles': obstacles, 'now': now,
                                      'generation': generation, 'config': self.config,
                                      'profile': self.profile})
        except queue.Full:
            return False
        self.pending = self.seq
        return True

    def poll(self):
        try:
            payload = self.responses.get_nowait()
        except queue.Empty:
            return None
        if payload.get('seq') != self.pending:
            return None
        self.pending = None
        return payload

    def close(self):
        try:
            self.requests.put_nowait(None)
        except queue.Full:
            pass
        self.process.join(timeout=1.)
        if self.process.is_alive():
            self.process.terminate()


def _local_pose_reason(reason):
    if reason.startswith('E_LOCAL_'):
        return reason
    if reason in ('E_POSE_STALE', 'E_POSE_NO_PROGRESS'):
        return 'E_LOCAL_POSE_STALE'
    if reason == 'E_POSE_ORIENTATION':
        return 'E_LOCAL_ORIENTATION'
    if reason == 'E_POSE_FRAME':
        return 'E_LOCAL_FRAME'
    if reason == 'E_POSE_POSITION':
        return 'E_LOCAL_POSE_INVALID'
    if reason == 'E_POSE_MISSING':
        return 'E_LOCAL_NOT_READY'
    return 'E_LOCAL_CLOCK'


class LocalPoseInput:
    """Pose contract over the node's shared state; callers hold its one lock."""
    def __init__(self, shared, max_age_s, require_orientation=False):
        self.shared = shared
        self.guard = PoseFreshness(max_age_s)
        self.require_orientation = require_orientation
        self.revocation = 0
        shared.update(pose_orientation=None, pose_error='E_LOCAL_NOT_READY',
                      pose_detail='E_POSE_MISSING')

    def invalidate(self, reason, detail=None):
        reason = _local_pose_reason(reason)
        value = self.shared
        if value.get('pose_error') != reason or value.get('pose') is not None or value.get('result') is not None:
            value['generation'] += 1
            self.revocation += 1
        self.guard.invalidate(detail or reason)
        value.update(pose=None, pose_orientation=None, pose_at=-1e100,
                     result=None, result_at=-1e100, result_generation=-1,
                     pose_error=reason, pose_detail=detail or reason,
                     last_error=reason)
        return reason

    def accept(self, message, ros_now, wall_now, rejection=None):
        try:
            source_stamp = message.header.stamp.to_sec()
            if message.header.frame_id != 'map':
                raise PoseError('E_POSE_FRAME')
            p = message.pose.position
            position = finite_vector((p.x, p.y, p.z), 3)
            orientation = None
            if self.require_orientation:
                q = message.pose.orientation
                orientation = normalized_quaternion((q.x, q.y, q.z, q.w))
            status, advanced = self.guard.observe(source_stamp, ros_now, wall_now, rejection=rejection)
            if not status['ok']:
                return self.invalidate(status['reason'], status['reason'])
            if advanced:
                self.shared.update(pose=position, pose_orientation=orientation,
                                   pose_at=wall_now, pose_ros_stamp=message.header.stamp.to_sec(),
                                   pose_error=None, pose_detail=None)
            return None
        except (PoseError, AttributeError, TypeError, ValueError) as error:
            reason = getattr(error, 'code', 'E_LOCAL_POSE_INVALID')
            # Account for rejected samples without granting them a new lease.
            try:
                self.guard.observe(message.header.stamp.to_sec(), ros_now, wall_now,
                                   rejection=reason)
            except (AttributeError, TypeError, ValueError):
                pass
            return self.invalidate(reason, reason)

    def check(self, ros_now, wall_now):
        status = self.guard.check(ros_now, wall_now)
        if not status['ok']:
            self.invalidate(status['reason'], status['reason'])
        return status




def simulation_pose_clock_wait(value, config, sim_clock, profile=None,
                               body_enabled=False, lockstep_command=False, max_age_s=1.):
    """Opt-in experiment only; do not extend timing or source freshness bounds."""
    if not finite_number(value) or value < 0.:
        raise ValueError('sim_pose_clock_wait_s must be finite and non-negative')
    if value == 0.:
        return 0.
    if sim_clock is not True or profile is not None or body_enabled or lockstep_command:
        raise ValueError('pose clock wait requires unprofiled simulation without body mirror or lockstep')
    if value > config.command_latency_bound_s:
        raise ValueError('pose clock wait exceeds command_latency_bound_s')
    if not finite_number(max_age_s) or value >= max_age_s:
        raise ValueError('pose clock wait must be strictly less than input_max_age_s')
    return float(value)


def accept_pose_with_clock_wait(pose_input, message, ros_clock, wall_clock, wait_s=0.,
                                sleep=time.sleep, is_shutdown=lambda: False):
    """Caller holds the existing state lock; rospy's clock updates independently.

    Hold the raw source stamp and first receipt monotonic time. Only a real
    in-budget clock catch-up may enter the unchanged accept guard; late wake,
    shutdown and rollback explicitly reject without granting a new lease.
    """
    if not finite_number(wait_s) or wait_s < 0.:
        raise ValueError('invalid pose clock wait')
    received_wall, ros_now = wall_clock(), ros_clock()
    if wait_s == 0. or not finite_number(received_wall):
        return pose_input.accept(message, ros_now, received_wall)
    try:
        source_stamp = message.header.stamp.to_sec()
        if message.header.frame_id != 'map':
            return pose_input.accept(message, ros_now, received_wall)
        p = message.pose.position
        finite_vector((p.x, p.y, p.z), 3)
        if pose_input.require_orientation:
            q = message.pose.orientation
            normalized_quaternion((q.x, q.y, q.z, q.w))
    except (PoseError, AttributeError, TypeError, ValueError):
        return pose_input.accept(message, ros_now, received_wall)
    if (not finite_number(source_stamp) or source_stamp <= 0. or
            not finite_number(ros_now) or ros_now <= 0. or source_stamp <= ros_now):
        return pose_input.accept(message, ros_now, received_wall)
    initial_ros, deadline = ros_now, received_wall + wait_s
    if not finite_number(deadline):
        return pose_input.accept(message, ros_now, received_wall, rejection='E_POSE_CLOCK')
    previous_ros, previous_wall = ros_now, received_wall

    def finish(error, latest_wall, outcome):
        old = pose_input.shared.get('pose_clock_ordering') or {}
        pose_input.shared['pose_clock_ordering'] = {
            'initial_ros': initial_ros, 'source_stamp': source_stamp,
            'received_wall': received_wall,
            'waited_wall_s': max(0., latest_wall - received_wall) if finite_number(latest_wall) else None,
            'outcome': outcome, 'wait_count': old.get('wait_count', 0) + 1,
            'accepted_count': old.get('accepted_count', 0) + int(error is None),
            'timeout_count': old.get('timeout_count', 0) + int(outcome == 'E_POSE_CLOCK_WAIT_TIMEOUT')}
        return error

    def rollback(before_ros, after_ros, at_wall):
        # Both values are actual observations. Explicitly reject the pending
        # old-epoch sample BEFORE feeding the real decrease to the old guard.
        # Its existing epoch mechanism then quarantines this pending stamp too.
        pose_input.accept(message, before_ros, received_wall, rejection='E_POSE_CLOCK_WAIT_ABORT')
        status = pose_input.check(after_ros, received_wall)
        return finish(pose_input.shared.get('pose_error'), at_wall, status['reason'])

    cached_ros = pose_input.guard.last_ros
    if cached_ros is not None and ros_now < cached_ros:
        return rollback(cached_ros, ros_now, received_wall)
    while True:
        latest_ros, latest_wall = ros_clock(), wall_clock()
        if finite_number(latest_ros) and latest_ros > 0. and latest_ros < previous_ros:
            return rollback(previous_ros, latest_ros, latest_wall)
        if not finite_number(latest_wall) or latest_wall < previous_wall:
            # Preserve the observed wall decrease as well, without retiming the
            # source or accepting a sample from a broken monotonic sequence.
            pose_input.check(previous_ros, previous_wall)
            error = pose_input.accept(message, latest_ros, latest_wall, rejection='E_POSE_WALL_BACKWARD')
            return finish(error, latest_wall, 'E_POSE_WALL_BACKWARD')
        if not finite_number(latest_ros) or latest_ros <= 0.:
            error = pose_input.accept(message, latest_ros, received_wall)
            return finish(error, latest_wall, 'E_POSE_CLOCK')
        # Check deadline before eligibility: scheduler oversleep is a rejection,
        # even when the clock has caught up by the time this callback resumes.
        rejection = ('E_POSE_CLOCK_WAIT_TIMEOUT' if latest_wall >= deadline else
                     'E_POSE_CLOCK_WAIT_SHUTDOWN' if is_shutdown() else None)
        if rejection:
            error = pose_input.accept(message, latest_ros, received_wall, rejection=rejection)
            return finish(error, latest_wall, rejection)
        if latest_ros >= source_stamp:
            error = pose_input.accept(message, latest_ros, received_wall)
            return finish(error, latest_wall, 'accepted' if error is None else pose_input.shared.get('pose_detail'))
        previous_ros, previous_wall = latest_ros, latest_wall
        sleep(min(.001, deadline - latest_wall))


def finalize_output(out, orientation, body_enabled):
    """Decide both command streams before either is published."""
    try:
        if not isinstance(out, dict) or not isinstance(out.get('reason'), str):
            raise PoseError('E_LOCAL_OUTPUT')
        velocity = finite_vector(out['velocity'], 3, 'E_LOCAL_VELOCITY')
        if out.get('stop'):
            return dict(out, velocity=[0., 0., 0.]), (0., 0., 0.) if body_enabled else None
        body = map_to_body_planar(velocity, orientation) if body_enabled else None
        if body_enabled:
            velocity = (velocity[0], velocity[1], 0.)
        return dict(out, velocity=list(velocity)), body
    except (PoseError, ArithmeticError, KeyError, TypeError, ValueError) as error:
        reason = _local_pose_reason(getattr(error, 'code', 'E_LOCAL_VELOCITY'))
        return dict(velocity=[0., 0., 0.], stop=True, complete=False, reason=reason), (0., 0., 0.)


def publish_output(out, orientation, stamp, sequence, command_pub, twist_pub, messages):
    """Publish the same final decision; caller holds the callback/state lock."""
    stamped_type, twist_type, _ = messages
    out, body = finalize_output(out, orientation, twist_pub is not None)
    message = stamped_type()
    message.header.frame_id, message.header.stamp, message.header.seq = 'map', stamp, sequence
    message.twist.linear.x, message.twist.linear.y, message.twist.linear.z = out['velocity']
    command_pub.publish(message)
    if twist_pub is not None:
        flat = twist_type()
        flat.linear.x, flat.linear.y, flat.linear.z = body
        twist_pub.publish(flat)
    return out


def accept_goal(shared, message):
    """Only an actual new goal clears an already latched terminal stop."""
    if message.header.frame_id != 'map':
        shared.update(goal=None, last_error='E_LOCAL_FRAME')
        return False
    p = message.pose.position
    shared.update(goal=(p.x, p.y, p.z), generation=shared['generation'] + 1, result=None,
                  stop_latched=False, stop_reason=None, last_error='ok')
    return True


def accept_worker_result(shared, payload, wall_now):
    """Old in-flight results cannot resurrect an invalidated pose/clock epoch."""
    if (payload['generation'] != shared['generation'] or shared.get('pose_error')
            or shared['stop_latched']):
        return False
    if payload['ok']:
        result = payload.get('result')
        if not isinstance(result, dict) or not finite_number(result.get('valid_until')):
            shared.update(result=None, last_error='E_LOCAL_INPUT')
            return False
        plan_ms = payload.get('plan_ms')
        shared.update(result=payload['result'], result_at=wall_now,
                      result_generation=payload['generation'], last_error='ok',
                      plan_ms=plan_ms,
                      plan_ms_max=max(shared.get('plan_ms_max', 0.), plan_ms or 0.))
        return True
    shared.update(result=None, last_error=payload['error'], last_error_detail=payload.get('message'))
    return False


def publish_worker_result(shared, payload, pose_input, lock, ros_clock, wall_clock,
                          path_pub, string_type):
    """Serialize outside the lock, then accept/publish atomically with revocation."""
    encoded = None
    try:
        if payload['ok']:
            encoded = json.dumps(payload['result'], allow_nan=False)
    except (KeyError, TypeError, ValueError, OverflowError):
        payload = dict(payload, ok=False, error='E_LOCAL_INPUT', message='invalid planner serialization')
    with lock:
        pose_input.check(ros_clock(), wall_clock())
        if accept_worker_result(shared, payload, wall_clock()):
            path_pub.publish(string_type(data=encoded))
            return True
    return False


class LocalControl:
    """One control cycle, with a final atomic recheck against pose callbacks."""
    def __init__(self, shared, pose_input, lock, config, max_age, wall_watchdog_s,
                 command_pub, twist_pub, status_pub, messages, profile=None):
        self.shared, self.pose_input, self.lock = shared, pose_input, lock
        self.config, self.max_age, self.wall_watchdog_s = config, max_age, wall_watchdog_s
        self.command_pub, self.twist_pub, self.status_pub = command_pub, twist_pub, status_pub
        self.messages, self.profile = messages, profile
        self.history, self.previous_stamp, self.previous_command = [], None, (0., 0., 0.)
        self.epoch, self.sequence = pose_input.guard.epoch, 0
        self.revocation = pose_input.revocation

    def _reset_history(self):
        self.history.clear()
        self.previous_stamp, self.previous_command = None, (0., 0., 0.)
        self.epoch = self.pose_input.guard.epoch
        self.revocation = self.pose_input.revocation

    def tick(self, ros_clock, wall_clock, stamp_factory, clock_ready=True, clock_stamp=None):
        with self.lock:
            ros_now, wall_now = ros_clock(), wall_clock()
            self.pose_input.check(ros_now, wall_now)
            stamp_sec = ros_now if clock_stamp is None else clock_stamp
            if (not clock_ready or not finite_number(stamp_sec) or stamp_sec <= 0.
                    or stamp_sec > ros_now
                    or (self.shared.get('pose') is not None and stamp_sec < self.shared['pose_ros_stamp'])
                    or (self.previous_stamp is not None and stamp_sec < self.previous_stamp)):
                self.pose_input.invalidate('E_LOCAL_CLOCK')
                self._reset_history()
                stamp_sec = ros_now if finite_number(ros_now) and ros_now > 0. else 0.
            if (self.epoch != self.pose_input.guard.epoch
                    or self.revocation != self.pose_input.revocation):
                self._reset_history()
            snapshot = dict(self.shared)
        fresh = snapshot.get('pose_error') or freshness(
            wall_now, snapshot['pose_at'], snapshot['obstacles_at'],
            snapshot['localization_at'], snapshot['localization_ok'], self.max_age)
        reason = stop_reason(fresh, snapshot['stop_latched'], snapshot['stop_reason'],
                             snapshot['result'], snapshot['result_at'], wall_now,
                             self.wall_watchdog_s, stamp_sec, snapshot['generation'],
                             snapshot['result_generation'], snapshot['last_error'])
        if not reason and snapshot['map'] is not None:
            try:
                estimated = feedback_at(snapshot['pose'], snapshot['pose_ros_stamp'], stamp_sec, self.history)
                period = (.05 if self.previous_stamp is None else max(.05, min(.1, stamp_sec - self.previous_stamp)))
                uncertainty = feedback_uncertainty(snapshot['pose_ros_stamp'], stamp_sec, self.history,
                                                   self.config.command_latency_bound_s)
                out = native.track_segment(snapshot['map'], snapshot['result'], estimated, stamp_sec,
                                           snapshot['obstacles'], config=self.config, control_period_s=period,
                                           feedback_pose=snapshot['pose'], uncertainty_m=uncertainty,
                                           previous_velocity=self.previous_command)
            except (ValueError, TypeError, KeyError, OverflowError) as error:
                out = dict(velocity=[0., 0., 0.], stop=True, reason='E_LOCAL_INPUT', input_error=str(error))
        else:
            out = dict(velocity=[0., 0., 0.], stop=True, reason=reason or snapshot['last_error'])
        # A pose/clock callback may invalidate the snapshot while tracking runs.
        # Recheck and publish under the SAME lock used by all those callbacks.
        with self.lock:
            latest_ros, latest_wall = ros_clock(), wall_clock()
            latest_status = self.pose_input.check(latest_ros, latest_wall)
            latest_fresh = self.shared.get('pose_error') or freshness(
                latest_wall, self.shared['pose_at'], self.shared['obstacles_at'],
                self.shared['localization_at'], self.shared['localization_ok'], self.max_age)
            if (self.epoch != self.pose_input.guard.epoch
                    or self.revocation != self.pose_input.revocation):
                self._reset_history()
                stamp_sec = latest_ros
            final_reason = stop_reason(
                latest_fresh, self.shared['stop_latched'], self.shared['stop_reason'],
                self.shared['result'], self.shared['result_at'], latest_wall,
                self.wall_watchdog_s, latest_ros, self.shared['generation'],
                self.shared['result_generation'], self.shared['last_error'])
            if (self.shared['generation'] != snapshot['generation']
                    or self.shared['result'] is not snapshot['result']):
                final_reason = latest_fresh or 'E_LOCAL_WORLD_UPDATED'
            if final_reason:
                out = dict(velocity=[0., 0., 0.], stop=True, reason=final_reason)
            # Header/history describe publication time, not the older tracking
            # sample. Lockstep's input tick only selects the calculation above.
            stamp_sec = latest_ros if finite_number(latest_ros) and latest_ros > 0. else 0.
            out = publish_output(out, self.shared.get('pose_orientation'), stamp_factory(stamp_sec),
                                 self.sequence, self.command_pub, self.twist_pub, self.messages)
            self.sequence += 1
            self.history.append((stamp_sec, tuple(out['velocity'])))
            self.history[:] = self.history[-200:]
            self.previous_stamp, self.previous_command = stamp_sec, tuple(out['velocity'])
            latched = (self.shared['stop_latched'] or (out['reason'].startswith('E_LOCAL_')
                       and out['reason'] not in NON_LATCHING and snapshot['result_at'] > 0))
            status = dict(out, clock='ros_source+monotonic_progress', pose_clock=latest_status,
                          wall_watchdog_s=self.wall_watchdog_s,
                          plan_ms=snapshot.get('plan_ms'), detail=snapshot.get('last_error_detail'),
                          map_version=snapshot.get('map_version'), plan_ms_max=snapshot.get('plan_ms_max'),
                          latched=bool(latched), plain_twist_frame='base_link_yaw_plane' if self.twist_pub else None,
                          capability_domain='experimental_kinematic_carrier' if self.profile is None else 'validated_robot_profile')
            if isinstance(snapshot['result'], dict) and finite_number(snapshot['result'].get('valid_until')):
                status['plan_valid_until'] = snapshot['result']['valid_until']
            if self.shared.get('pose_clock_ordering') is not None:
                status['pose_clock_ordering'] = dict(self.shared['pose_clock_ordering'])
            self.status_pub.publish(self.messages[2](data=json.dumps(status)))
            if out.get('complete') or out['reason'].startswith('E_LOCAL_'):
                if self.shared['result'] is snapshot['result']:
                    terminal = self.shared['stop_latched'] or (latched and snapshot['result_at'] > 0)
                    terminal_reason = (self.shared['stop_reason'] if self.shared['stop_latched']
                                       else out['reason'])
                    self.shared.update(result=None, last_error=out['reason'], stop_latched=terminal,
                                       stop_reason=terminal_reason if terminal else None)
        return out


def main():
    import rospy
    from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
    from std_msgs.msg import Bool, String

    rospy.init_node('v8_native_local_planner')
    sim_clock_value = rospy.get_param('/use_sim_time', False)
    sim_clock = bool(sim_clock_value)
    lockstep_command = bool(rospy.get_param('~lockstep_command', False))
    clock_event = threading.Event()
    clock_sec = [None]
    plan_sim_tick_s = float(rospy.get_param('~plan_sim_tick_s', 0.))
    if plan_sim_tick_s < 0:
        raise ValueError('plan_sim_tick_s must be non-negative')
    config = native.Config(**rospy.get_param('~experimental_config', {}))
    config.validate()
    max_age = float(rospy.get_param('~input_max_age_s', 1.0))
    if not math.isfinite(max_age) or max_age <= 0:
        raise ValueError('input_max_age_s must be positive')
    wall_watchdog_s = float(rospy.get_param('~wall_watchdog_s', max(5., 4. * config.validity_s)))
    if not math.isfinite(wall_watchdog_s) or wall_watchdog_s <= 0:
        raise ValueError('wall_watchdog_s must be positive')
    map_path = rospy.get_param('~map_json', '')
    map_topic = rospy.get_param('~map_topic', '')
    if not map_path and not map_topic:
        raise ValueError('map_json or map_topic is required')
    profile_path = rospy.get_param('~robot_profile_json', '')
    profile = None
    if profile_path:
        with open(profile_path, encoding='utf-8') as stream:
            profile = json.load(stream)
    # Missing localization or obstacle heartbeat is unsafe, including at startup.
    shared = {'pose': None, 'goal': None, 'pose_at': -1e100, 'obstacles': [],
              'obstacles_at': -1e100, 'localization_at': -1e100, 'localization_ok': False,
              'result': None, 'result_at': -1e100, 'map': None, 'generation': 0,
              'result_generation': -1, 'last_error': 'E_LOCAL_NOT_READY', 'last_error_detail': None,
              'obstacle_key': None,
              'pose_ros_stamp': 0., 'stop_latched': False, 'stop_reason': None,
              'plan_ms': None, 'plan_ms_max': 0., 'map_version': None}
    lock = threading.Lock()
    plan_wake = threading.Event()
    command_pub = rospy.Publisher(rospy.get_param('~command_topic', '/v8/local/cmd_vel_3d'),
                                  TwistStamped, queue_size=1)
    # Optional plain-Twist mirror for the real-robot safety mux, whose sources are
    # geometry_msgs/Twist. Empty (default) keeps the sim behaviour unchanged.
    twist_topic = rospy.get_param('~twist_topic', '')
    twist_pub = (rospy.Publisher(twist_topic, Twist, queue_size=1)
                 if twist_topic else None)
    pose_clock_wait_s = simulation_pose_clock_wait(
        rospy.get_param('~sim_pose_clock_wait_s', 0.), config, sim_clock_value,
        profile=profile, body_enabled=twist_pub is not None, lockstep_command=lockstep_command,
        max_age_s=max_age)
    pose_input = LocalPoseInput(shared, max_age, require_orientation=twist_pub is not None)
    path_pub = rospy.Publisher(rospy.get_param('~trajectory_topic', '/v8/local/trajectory'),
                               String, queue_size=1, latch=True)
    status_pub = rospy.Publisher(rospy.get_param('~status_topic', '/v8/local/status'),
                                 String, queue_size=1, latch=True)

    def on_clock(msg):
        with lock:
            clock_sec[0] = msg.clock.to_sec()
        clock_event.set()

    if lockstep_command:
        from rosgraph_msgs.msg import Clock
        rospy.Subscriber('/clock', Clock, on_clock, queue_size=1)

    def on_pose(msg):
        with lock:
            error = accept_pose_with_clock_wait(
                pose_input, msg, lambda: rospy.Time.now().to_sec(), time.monotonic,
                wait_s=pose_clock_wait_s, is_shutdown=rospy.is_shutdown)
            if error:
                plan_wake.set()

    def on_goal(msg):
        with lock:
            if accept_goal(shared, msg):
                plan_wake.set()

    def on_obstacles(msg):
        try:
            payload = json.loads(msg.data)
            values = payload['obstacles'] if isinstance(payload, dict) else payload
            native._obstacles(values)
            # Repeated heartbeat with identical prediction does not discard valid motion.
            key = json.dumps(payload, sort_keys=True, separators=(',', ':'))
            with lock:
                if key != shared['obstacle_key']:
                    shared.update(generation=shared['generation'] + 1, result=None)
                    plan_wake.set()
                shared.update(obstacles=values, obstacles_at=time.monotonic(), obstacle_key=key)
        except (ValueError, TypeError, KeyError, native.LocalError):
            with lock:
                shared.update(obstacles_at=-1e100, result=None, last_error='E_LOCAL_OBSTACLES_INVALID')

    def on_localization(msg):
        with lock:
            shared.update(localization_ok=msg.data, localization_at=time.monotonic())
            if not msg.data:
                shared.update(result=None, generation=shared['generation'] + 1)
                plan_wake.set()

    def on_map(msg):
        try:
            value = map_from_message(msg.data)
        except ValueError:
            with lock:
                shared.update(last_error='E_LOCAL_MAP_INVALID', result=None)
                plan_wake.set()
            return
        with lock:
            shared.update(map=value, map_version=value.get('map_version'),
                          generation=shared['generation'] + 1, result=None,
                          last_error='ok')
            plan_wake.set()

    if map_topic:
        rospy.Subscriber(map_topic, String, on_map, queue_size=1)
    rospy.Subscriber(rospy.get_param('~pose_topic', '/v8/local/pose'), PoseStamped, on_pose, queue_size=1)
    rospy.Subscriber(rospy.get_param('~goal_topic', '/v8/local/goal'), PoseStamped, on_goal, queue_size=1)
    rospy.Subscriber(rospy.get_param('~obstacles_topic', '/v8/local/obstacles'), String, on_obstacles, queue_size=1)
    rospy.Subscriber(rospy.get_param('~localization_topic', '/v8/local/localization_ok'),
                     Bool, on_localization, queue_size=1)

    controller = LocalControl(shared, pose_input, lock, config, max_age, wall_watchdog_s,
                              command_pub, twist_pub, status_pub, (TwistStamped, Twist, String), profile)

    def control_loop():
        while not rospy.is_shutdown():
            clock_ready = not lockstep_command or clock_event.wait(1.0)
            if lockstep_command:
                clock_event.clear()
            controller.tick(lambda: rospy.Time.now().to_sec(), time.monotonic, rospy.Time.from_sec,
                            clock_ready=clock_ready,
                            clock_stamp=clock_sec[0] if lockstep_command else None)
            # A missing /clock still reaches the zero-speed publish above.
            time.sleep(.02 if sim_clock else .05)

    worker = PlannerWorker(config, profile)
    last_plan_tick = -1
    last_plan_epoch = pose_input.guard.epoch

    def pace(timeout):
        plan_wake.wait(timeout)
        plan_wake.clear()
    thread = threading.Thread(target=control_loop, daemon=True)
    thread.start()
    try:
        while not rospy.is_shutdown():
            started = time.monotonic()
            # Simulation fault injection: the independent control/watchdog loop
            # keeps running while planner refresh is deliberately suspended.
            if rospy.get_param('~pause_planning', False):
                pace(.05)
                continue
            with lock:
                stop_latched = shared['stop_latched']
            if stop_latched:
                pace(.05)
                continue
            payload = worker.poll()
            if payload is not None:
                publish_worker_result(shared, payload, pose_input, lock,
                                      lambda: rospy.Time.now().to_sec(), time.monotonic,
                                      path_pub, String)
            with lock:
                pose_input.check(rospy.Time.now().to_sec(), time.monotonic())
                snapshot = dict(shared)
                if pose_input.guard.epoch != last_plan_epoch:
                    last_plan_tick = -1
                    last_plan_epoch = pose_input.guard.epoch
            try:
                if map_topic:
                    with lock:
                        value = shared['map']
                    if value is None:
                        pace(max(.01, .2 - (time.monotonic() - started)))
                        continue
                else:
                    with open(map_path, encoding='utf-8') as stream:
                        value = json.load(stream)
                    with lock:
                        shared['map'] = value
                # Replanning from a held contact pose while its command is
                # still being executed changes direction mid-phase and loses
                # the state/time relation used by collision prediction.
                if snapshot['result'] is not None:
                    pace(max(.01, .2 - (time.monotonic() - started)))
                    continue
                reason = snapshot.get('pose_error') or freshness(started, snapshot['pose_at'], snapshot['obstacles_at'],
                                   snapshot['localization_at'], snapshot['localization_ok'], max_age)
                stamp = rospy.Time.now().to_sec()
                if reason is None and snapshot['pose'] is not None and snapshot['goal'] is not None and stamp > 0:
                    if sim_clock and plan_sim_tick_s > 0:
                        tick = int(math.floor(stamp / plan_sim_tick_s + 1e-9))
                        if tick == last_plan_tick:
                            pace(max(.01, .2 - (time.monotonic() - started)))
                            continue
                        last_plan_tick = tick
                    if not worker.submit(value, snapshot['pose'], snapshot['goal'],
                                         snapshot['obstacles'], stamp, snapshot['generation']):
                        if not worker.process.is_alive():
                            with lock:
                                shared.update(result=None, last_error='E_LOCAL_PLANNER_DEAD')
                else:
                    with lock:
                        shared.update(result=None, last_error=reason or 'E_LOCAL_NOT_READY')
            except (OSError, ValueError, KeyError) as error:
                with lock:
                    shared.update(result=None, last_error=getattr(error, 'code', 'E_LOCAL_INPUT'))
                rospy.logwarn_throttle(2., 'local planner input error: %s', error)
            pending = worker.pending is not None
            pace(.02 if pending else max(.01, .2 - (time.monotonic() - started)))
    finally:
        command_pub.publish(TwistStamped())
        if twist_pub is not None:
            twist_pub.publish(Twist())
        thread.join(timeout=1.)
        worker.close()


if __name__ == '__main__':
    main()
