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
from contextlib import contextmanager
from pathlib import Path
import sys
import threading
import time

# 2026-11-18（M9）：实现已拆到 ros_node_{signals,inputs,control}，本文件是 façade。
# 下面这些 re-export 让 9 处外部导入（import ros_node / from ros_node import X）零改动。
import local_native as native
import cancel_policy
import gait_switch
from contact_egress import ContactEgress, simulation_contact_egress

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / '90_platform/v8_platform'))
from common.pose_safety import (PoseError, PoseFreshness, finite_number, finite_vector,
                                map_to_body_planar, normalized_quaternion)



# ── re-export（façade 契约：外部按旧名取用，导入路径与符号名都不变）──────────────
from ros_node_signals import (  # noqa: F401
    NON_LATCHING, freshness, feedback_at, feedback_uncertainty, stop_reason,
    map_from_message, _local_pose_reason)
from ros_node_inputs import (  # noqa: F401
    PlannerWorker, LocalPoseInput, ClockWaiter, simulation_pose_clock_wait,
    simulation_thread_switch, accept_pose_with_clock_wait, _planner_worker)
from ros_node_control import (  # noqa: F401
    LocalControl, finalize_output, publish_output)


def accept_goal(shared, message):
    """Only an actual new goal clears an already latched terminal stop."""
    if message.header.frame_id != 'map':
        shared.update(goal=None, last_error='E_LOCAL_FRAME')
        return False
    # 任务身份：goal 的 header.seq 由上游写成"任务代次"。开启 require_goal_seq 后，
    # 代次必须严格大于已接受的代次 —— 这样即便旧任务带着更新的时间戳回来，也进不来。
    if shared.get('require_goal_seq'):
        seq = int(getattr(message.header, 'seq', 0) or 0)
        last = int(shared.get('last_goal_seq') or 0)
        if seq != 0 and seq <= last:
            shared.update(last_error='E_LOCAL_STALE_GOAL')
            import rospy as _r
            _r.logwarn('[local] goal REJECTED stale: seq=%d last=%d stop_latched=%s',
                       seq, last, shared.get('stop_latched'))
            return False
        if seq > last:
            shared['last_goal_seq'] = seq
    p = message.pose.position
    try:
        import rospy as _r
        _r.loginfo('[local] goal ACCEPTED seq=%d (last=%d)', int(getattr(message.header, 'seq', 0) or 0),
                   int(shared.get('last_goal_seq') or 0))
    except Exception:
        pass
    shared.update(goal=(p.x, p.y, p.z), generation=shared['generation'] + 1, result=None,
                  stop_latched=False, stop_reason=None, last_error='ok',
                  cancel_requested=False, cancel_seq=0, cancel_reason=None)
    return True

def drop_task_on_pose_invalid(shared, now_wall, threshold_s):
    """★定位持续失效 ⇒ 当前任务作废★（真人裁决 2026-11-26）。

    语义：定位失效后当前任务作废，**定位恢复不等于运动授权恢复**——恢复后必须收到
    合法的新任务（`header.seq` 代次更高的 goal）才能重新武装。

    只由"**持续**失效"触发（`threshold_s` 内一直有 pose_error）：
    单帧抖动就清任务会破坏正常连续步进，所以这里用超时而不是即时。
    """
    if not shared.get('pose_error'):
        return False
    since = shared.get('pose_invalid_since')
    if since is None or (now_wall - since) < threshold_s:
        return False
    if shared.get('goal') is None and shared.get('stop_latched'):
        return False                      # 已经作废过，别反复推代次
    shared.update(goal=None, result=None, result_generation=-1,
                  stop_latched=True, stop_reason='E_LOCAL_LOCALIZATION_LOST',
                  generation=shared['generation'] + 1,
                  cancel_requested=True, last_error='E_LOCAL_LOCALIZATION_LOST')
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

def initialize_local_node(rospy):
    """Request low-latency simulated time before its first TCPROS connection.

    rospy shares one subscriber implementation per resolved topic. Its later
    default-False subscribers do not clear an already enabled TCP_NODELAY.
    Keep the returned handle alive until the node stops, including reconnects.
    """
    clock_transport = None
    if rospy.get_param('/use_sim_time', False) is True:
        if rospy.core.is_initialized():
            raise ValueError('simulation clock transport requires a fresh node process')
        from rosgraph_msgs.msg import Clock
        clock_transport = rospy.Subscriber('/clock', Clock, None,
                                           queue_size=1, tcp_nodelay=True)
    try:
        rospy.init_node('v8_native_local_planner')
        if (clock_transport is not None and
                clock_transport.resolved_name != rospy.resolve_name('/clock')):
            raise ValueError('clock remapping changed during node initialization')
    except BaseException:
        if clock_transport is not None:
            clock_transport.unregister()
        raise
    return clock_transport

def main():
    import rospy
    from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
    from std_msgs.msg import Bool, String

    clock_transport = initialize_local_node(rospy)
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
    require_goal_seq = bool(rospy.get_param('~require_goal_seq', False))
    # 定位持续失效多久才作废任务（秒）。必须 > 正常帧间隔（~0.1 s），又与"位姿最大龄期"同一量级。
    task_drop_after_s = float(rospy.get_param('~task_drop_after_s', 0.8))
    if not math.isfinite(max_age) or max_age <= 0:
        raise ValueError('input_max_age_s must be positive')
    wall_watchdog_s = float(rospy.get_param('~wall_watchdog_s', max(5., 4. * config.validity_s)))
    if not math.isfinite(wall_watchdog_s) or wall_watchdog_s <= 0:
        raise ValueError('wall_watchdog_s must be positive')
    cancel_topic = rospy.get_param('~cancel_topic', '/v8/local/cancel')
    cancel_ack_topic = rospy.get_param('~cancel_ack_topic', '/v8/local/cancel_ack')
    gait_hold = None
    gait_desired_topic = chassis_status_topic = None
    if bool(rospy.get_param('~gait_hold_enable', False)):
        # Stop-before-switch: the planner owns velocity, the chassis bridge owns
        # the gait. Hold zero until the bridge reports the requested gait active.
        gait_hold = gait_switch.GaitSwitchHold(
            settle_s=rospy.get_param('~gait_hold_settle_s', 0.5),
            timeout_s=rospy.get_param('~gait_hold_timeout_s', 5.0),
            status_stale_s=rospy.get_param('~chassis_status_stale_s', 1.5))
        gait_desired_topic = rospy.get_param('~gait_desired_topic', '/v8/chassis/gait')
        chassis_status_topic = rospy.get_param('~chassis_status_topic', '/v8/chassis/status')
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
              'require_goal_seq': require_goal_seq, 'last_goal_seq': 0,
              'obstacles_at': -1e100, 'localization_at': -1e100, 'localization_ok': False,
              'result': None, 'result_at': -1e100, 'map': None, 'generation': 0,
              'result_generation': -1, 'last_error': 'E_LOCAL_NOT_READY', 'last_error_detail': None,
              'obstacle_key': None,
              'cancel_requested': False, 'cancel_seq': 0, 'cancel_reason': None,
              'pose_ros_stamp': 0., 'stop_latched': False, 'stop_reason': None,
              'plan_ms': None, 'plan_ms_max': 0., 'map_version': None,
              'gait_desired': None, 'chassis_gait': None, 'chassis_status_at': -1e100,
              'chassis_speed': None, 'chassis_refusal': None}
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
    contact_egress_enabled = simulation_contact_egress(
        rospy.get_param('~sim_contact_egress', False), sim_clock_value,
        profile=profile, body_enabled=twist_pub is not None)
    pose_input = LocalPoseInput(shared, max_age, require_orientation=twist_pub is not None)
    path_pub = rospy.Publisher(rospy.get_param('~trajectory_topic', '/v8/local/trajectory'),
                               String, queue_size=1, latch=True)
    status_pub = rospy.Publisher(rospy.get_param('~status_topic', '/v8/local/status'),
                                 String, queue_size=1, latch=True)
    cancel_ack_pub = rospy.Publisher(cancel_ack_topic, String, queue_size=1)

    def on_clock(msg):
        with lock:
            clock_sec[0] = msg.clock.to_sec()
        clock_event.set()

    if lockstep_command:
        from rosgraph_msgs.msg import Clock
        rospy.Subscriber('/clock', Clock, on_clock, queue_size=1)

    pose_clock_waiter = ClockWaiter()

    def on_pose_clock(msg):
        pose_clock_waiter.publish()

    if pose_clock_wait_s > 0.:
        from rosgraph_msgs.msg import Clock
        rospy.Subscriber('/clock', Clock, on_pose_clock, queue_size=1)

    def on_pose(msg):
        with lock:
            error = accept_pose_with_clock_wait(
                pose_input, msg, lambda: rospy.Time.now().to_sec(), time.monotonic,
                wait_s=pose_clock_wait_s,
                sleep=pose_clock_waiter if pose_clock_wait_s > 0. else time.sleep,
                is_shutdown=rospy.is_shutdown)
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
                shared['obstacles_cb'] = shared.get('obstacles_cb', 0) + 1
                shared.update(obstacles=values, obstacles_at=time.monotonic(), obstacle_key=key)
            rospy.loginfo_throttle(5.0, '[local] obstacles ok: %d',
                                   len(values) if isinstance(values, (list, tuple)) else -1)
        except (ValueError, TypeError, KeyError, native.LocalError) as error:
            # 诊断：区分「回调没进」与「进了但解析失败」——两者都表现为
            # E_LOCAL_OBSTACLES_STALE，只靠 status 分不出来。
            rospy.logwarn_throttle(5.0, '[local] obstacles rejected: %s: %s',
                                   type(error).__name__, error)
            with lock:
                shared.update(obstacles_at=-1e100, result=None, last_error='E_LOCAL_OBSTACLES_INVALID')

    def on_localization(msg):
        with lock:
            shared.update(localization_ok=msg.data, localization_at=time.monotonic())
            if not msg.data:
                shared.update(result=None, generation=shared['generation'] + 1)
                plan_wake.set()

    def on_cancel(msg):
        parsed = cancel_policy.parse_cancel(msg.data)
        if parsed is None:
            rospy.logwarn_throttle(5.0, '[local] malformed cancel ignored: %s', msg.data[:80])
            return
        seq, scope, reason = parsed
        with lock:
            shared.update(cancel_requested=True, cancel_seq=seq, cancel_reason=reason,
                          goal=None, result=None,
                          generation=shared['generation'] + 1,
                          stop_latched=True, stop_reason='E_LOCAL_CANCELLED')
            plan_wake.set()
        cancel_ack_pub.publish(String(data=cancel_policy.ack_text(seq, 'cancelled')))
        rospy.logwarn('[local] cancel accepted seq=%d scope=%s', seq, scope)

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
    rospy.Subscriber(cancel_topic, String, on_cancel, queue_size=1)

    if gait_hold is not None:
        def on_gait_desired(msg):
            if isinstance(msg.data, str) and msg.data:
                with lock:
                    shared.update(gait_desired=msg.data)

        def on_chassis_status(msg):
            try:
                payload = json.loads(msg.data)
                gait = payload.get('gait')
                speed = payload.get('speed')
                if not isinstance(gait, str) or not gait:
                    raise ValueError('gait missing')
                if speed is not None and not finite_number(speed):
                    raise ValueError('speed invalid')
            except (ValueError, TypeError, AttributeError):
                with lock:
                    shared.update(chassis_status_at=-1e100, chassis_gait=None,
                                  chassis_speed=None, chassis_refusal=None)
                return
            refusal = payload.get('refusal')
            with lock:
                shared.update(chassis_gait=gait, chassis_speed=speed,
                              chassis_refusal=refusal if isinstance(refusal, str) else None,
                              chassis_status_at=time.monotonic())

        rospy.Subscriber(gait_desired_topic, String, on_gait_desired, queue_size=1)
        rospy.Subscriber(chassis_status_topic, String, on_chassis_status, queue_size=1)

    controller = LocalControl(shared, pose_input, lock, config, max_age, wall_watchdog_s,
                              command_pub, twist_pub, status_pub, (TwistStamped, Twist, String), profile,
                              sim_contact_egress=contact_egress_enabled, gait_hold=gait_hold)

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

    with simulation_thread_switch(pose_clock_wait_s):
        if pose_clock_wait_s > 0.:
            rospy.loginfo('simulation callback thread switch target: %.6f s', sys.getswitchinterval())
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
                    # ★定位持续失效 ⇒ 作废当前任务并锁存停止★（恢复后不会自己续走）
                    if drop_task_on_pose_invalid(shared, time.monotonic(), task_drop_after_s):
                        rospy.logwarn('[local] localization lost > %.2fs: task dropped '
                                      '(goal cleared, stop latched)', task_drop_after_s)
                        plan_wake.set()
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
            if clock_transport is not None:
                clock_transport.unregister()
