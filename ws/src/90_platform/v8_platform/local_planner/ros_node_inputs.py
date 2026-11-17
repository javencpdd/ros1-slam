#!/usr/bin/env python3
"""local_planner 的输入侧：规划线程、位姿输入、时钟等待（自 ros_node.py 拆出，M9）。

这里回答「输入到了没有、新鲜不新鲜、什么时候该等时钟」。
不写控制指令——那是 ros_node_control 的职责。
"""
import math
import multiprocessing as mp
import queue
import sys
import threading
import time
from contextlib import contextmanager
from pathlib import Path

import local_native as native
from ros_node_signals import _local_pose_reason

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from common.pose_safety import (PoseError, PoseFreshness, finite_number,
                                finite_vector, normalized_quaternion)

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
            # 诊断：失败输入快照随 detail 透出（status.last_error_detail），
            # 用于事后复现「仿真里一次瞬时失败锁死整个执行段」的具体输入。
            responses.put({'seq': item['seq'], 'generation': item['generation'],
                           'ok': False, 'error': getattr(error, 'code', 'E_LOCAL_INPUT'),
                           'message': '%s | start=%s goal=%s now=%.3f obs=%d' % (
                               str(error), tuple(item['pose']), tuple(item['goal']),
                               item['now'], len(item['obstacles'])),
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

class ClockWaiter:
    def __init__(self):
        self.condition = threading.Condition()
        self.generation = 0
        self.armed_generation = 0

    def arm(self):
        with self.condition:
            self.armed_generation = self.generation

    def publish(self):
        with self.condition:
            self.generation += 1
            self.condition.notify_all()

    def __call__(self, seconds):
        with self.condition:
            if self.generation == self.armed_generation:
                self.condition.wait(seconds)

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

@contextmanager
def simulation_thread_switch(wait_s):
    """Reduce interpreter contention only for the validated simulation wait.

    This is a process-local scheduling target, not a deadline guarantee. Keep
    an already shorter interval and restore the original even on cleanup errors.
    The default path does not inspect or change interpreter scheduling.
    """
    if not finite_number(wait_s) or wait_s < 0.:
        raise ValueError('invalid pose clock wait')
    if wait_s == 0.:
        yield
        return
    original = sys.getswitchinterval()
    target = min(original, .001)
    changed = target < original
    try:
        if changed:
            sys.setswitchinterval(target)
        yield
    finally:
        if changed:
            sys.setswitchinterval(original)

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
    arm = getattr(sleep, 'arm', None)
    while True:
        if arm is not None:
            arm()
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
