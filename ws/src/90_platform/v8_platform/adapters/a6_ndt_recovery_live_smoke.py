#!/usr/bin/env python3
"""Live A6 NDT recovery: loss -> bounded NDT relocalize -> candidate -> seed
-> new lease, with the localization lease gate rejecting commands throughout.

The NDT monitor is fed through a pausable relay so loss is a real absence of
observations, not an injected flag.  Backend application is the NDT native
`~initialpose` seed; the A6 supervisor never receives Gazebo truth.  Gazebo
truth is used only for the post-recovery XY gate and to report the candidate's
distance from truth.  The certified slice is the static-loss case; `--move`
is exploratory because a nearest-prior candidate may stay at the prior.
"""
import argparse
import json
import math
import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

import rospy

HERE = Path(__file__).resolve().parent
SIM = Path('/opt/ros1-slam/ws/src/95_simulation/lite3_gazebo_sim')
PCD = '/opt/ros1-slam/data/maps/m1_world_synth_v1.pcd'
PORT, GAZEBO_PORT = 11950, 11951
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:%d' % PORT,
                  GAZEBO_MASTER_URI='http://127.0.0.1:%d' % GAZEBO_PORT,
                  ROS_IP='127.0.0.1', ROS_HOME='/tmp/a6_ndt_ros',
                  ROS_LOG_DIR='/tmp/a6_ndt_logs', PYTHONDONTWRITEBYTECODE='1')
Path('/tmp/a6_ndt_ros').mkdir(exist_ok=True)
Path('/tmp/a6_ndt_logs').mkdir(exist_ok=True)


def stop(process):
    if process is not None and process.poll() is None:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5)


def wait_until(predicate, timeout, label, diagnostic=None):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(.05)
    if diagnostic is not None:
        print('DIAG ' + json.dumps(diagnostic(), ensure_ascii=False, default=str)[:6000],
              flush=True)
    raise RuntimeError('timeout waiting for ' + label)


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--spawn', default='1.5,0')
    parser.add_argument('--move', default='0,0')
    parser.add_argument('--cycles', type=int, default=1)
    parser.add_argument('--gate-m', type=float, default=0.3)
    parser.add_argument('--deadline-s', type=float, default=25.0)
    parser.add_argument('--relocalize-budget-s', type=float, default=8.0)
    parser.add_argument('--search-resume-delay-s', type=float, default=.5)
    parser.add_argument('--negative', action='store_true')
    parser.add_argument('--out')
    args = parser.parse_args(argv)
    from gazebo_msgs.msg import ModelState, ModelStates
    from gazebo_msgs.srv import SetModelState
    from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, Vector3
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import Bool, String, UInt64

    spawn_x, spawn_y = (float(v) for v in args.spawn.split(','))
    move_x, move_y = (float(v) for v in args.move.split(','))
    master = gazebo = ndt = bridge = a6 = gate = None
    relay = {'paused': False, 'clouds': 0}
    status = {'phase': None, 'allowed': False, 'lease': None, 'backend': None,
              'calls': 0, 'report': None}
    observations = {'odom': None, 'truth': None, 'gate_status': None,
                    'gate_cmd': None, 'gate_cmds': 0, 'health': None,
                    'relocalized': 0, 'backend': None, 'resets': []}
    lock = threading.RLock()
    callbacks = []

    def diagnostic():
        return {'status': status, 'observations': observations,
                'relay': relay}

    try:
        master = subprocess.Popen(['roscore', '-p', str(PORT)], start_new_session=True,
                                  stdout=open('/tmp/a6_ndt_master.log', 'ab'),
                                  stderr=subprocess.STDOUT, env=dict(os.environ))
        time.sleep(3)
        subprocess.run(['rosparam', 'set', '/use_sim_time', 'true'], check=True,
                       env=dict(os.environ))
        gazebo = subprocess.Popen(['roslaunch', str(SIM / 'launch/m1_gazebo.launch'),
                                   'payload_dir:=' + str(SIM), 'gui:=false'],
                                  start_new_session=True,
                                  stdout=open('/tmp/a6_ndt_gazebo.log', 'ab'),
                                  stderr=subprocess.STDOUT, env=dict(os.environ))
        rospy.init_node('a6_ndt_recovery_live_smoke', anonymous=True, disable_signals=True)
        rospy.wait_for_message('/clock', Clock, timeout=90)
        rospy.wait_for_message('/gazebo/model_states', ModelStates, timeout=90)
        rospy.wait_for_message('/rslidar_points', PointCloud2, timeout=90)
        rospy.wait_for_service('/gazebo/set_model_state', timeout=20)
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        def place(x, y):
            state = ModelState(model_name='lite3_planar', reference_frame='world')
            state.pose.position.x, state.pose.position.y, state.pose.position.z = x, y, .112
            state.pose.orientation.w = 1.
            set_state(state)

        place(spawn_x, spawn_y)
        time.sleep(2.0)

        ndt = subprocess.Popen(['rosrun', 'v8_ndt_odom', 'ndt_odom_node', '__name:=a6_ndt',
                                '_map_path:=' + PCD, '_cloud_topic:=/a6_ndt/cloud',
                                '_resolution:=0.5', '_score_max:=30.0',
                                '_ext_z:=0.34', '_ext_pitch:=0.2617993878',
                                '_init_x:=%s' % spawn_x, '_init_y:=%s' % spawn_y,
                                '_init_yaw_deg:=0', '_global_search:=false',
                                '_relocalize_radius_m:=0.4', '_relocalize_xy_step_m:=0.2',
                                '_relocalize_yaw_span_deg:=10',
                                '_relocalize_yaw_step_deg:=5',
                                '_relocalize_budget_s:=%s' % args.relocalize_budget_s,
                                '_relocalize_score_max:=15.0',
                                '_relocalize_max_displacement_m:=0.6'],
                               start_new_session=True,
                               stdout=open('/tmp/a6_ndt_node.log', 'ab'),
                               stderr=subprocess.STDOUT, env=dict(os.environ))
        def service_ready(name):
            try:
                rospy.wait_for_service(name, timeout=.5)
                return True
            except rospy.ROSException:
                return False

        wait_until(lambda: service_ready('/a6_ndt/relocalize'), 30, 'NDT relocalize service')
        wait_until(lambda: service_ready('/a6_ndt/reset'), 30, 'NDT reset service')

        bridge = subprocess.Popen([sys.executable, '-B', str(HERE / 'ndt_recovery_bridge.py'),
                                   '--odom-topic', '/a6_ndt/odom',
                                   '--health-topic', '/a6_ndt/healthy',
                                   '--score-topic', '/a6_ndt/score',
                                   '--relocalized-topic', '/a6_ndt/relocalized',
                                   '--candidate-topic', '/a6/recovery/candidate',
                                   '--fitness-max', '15.0'],
                                  start_new_session=True,
                                  stdout=open('/tmp/a6_ndt_bridge.log', 'ab'),
                                  stderr=subprocess.STDOUT, env=dict(os.environ))
        a6 = subprocess.Popen([sys.executable, '-B', str(HERE / 'a6_recovery_node.py'),
                               '--namespace', '/a6/recovery',
                               '--odom-topic', '/a6_ndt/odom',
                               '--candidate-topic', '/a6/recovery/candidate',
                               '--relocalize-service', '/a6_ndt/relocalize',
                               '--relocalize-service-type', 'trigger',
                               '--backend-name', 'ndt',
                               '--initial-lease', '1',
                               '--deadline-s', str(args.deadline_s),
                               '--timeout-s', '1.0',
                               '--score-min', '0.5',
                               '--bounds=-3,8,-5,5'],
                              start_new_session=True,
                              stdout=open('/tmp/a6_ndt_recovery.log', 'ab'),
                              stderr=subprocess.STDOUT, env=dict(os.environ))
        gate = subprocess.Popen([sys.executable, '-B', str(HERE / 'localization_state.py'),
                                 '--namespace', '/a6/gate',
                                 '--odom-topic', '/a6_ndt/odom',
                                 '--lease-topic', '/a6/recovery/lease',
                                 '--candidate-topic', '/a6/gate/candidate',
                                 '--output-topic', '/a6/gate/cmd_vel',
                                 '--timeout-s', '1.0',
                                 '--command-timeout-s', '0.3',
                                 '--keep-alive-s', '180'],
                                start_new_session=True,
                                stdout=open('/tmp/a6_ndt_gate.log', 'ab'),
                                stderr=subprocess.STDOUT, env=dict(os.environ))

        def on_cloud(message):
            if not relay['paused']:
                relay['clouds'] += 1
                cloud_pub.publish(message)

        def on_truth(message):
            names = list(message.name)
            if 'lite3_planar' not in names:
                return
            pose = message.pose[names.index('lite3_planar')]
            q = pose.orientation
            with lock:
                observations['truth'] = [pose.position.x, pose.position.y, pose.position.z,
                                         math.atan2(2 * (q.w * q.z + q.x * q.y),
                                                    1 - 2 * (q.y * q.y + q.z * q.z))]

        def on_odom(message):
            q = message.pose.pose.orientation
            with lock:
                observations['odom'] = [message.pose.pose.position.x,
                                        message.pose.pose.position.y,
                                        message.pose.pose.position.z,
                                        math.atan2(2 * (q.w * q.z + q.x * q.y),
                                                   1 - 2 * (q.y * q.y + q.z * q.z))]

        def on_health(message):
            with lock:
                observations['health'] = bool(message.data)

        def on_relocalized(message):
            with lock:
                if message.data:
                    observations['relocalized'] += 1

        def on_gate_status(message):
            with lock:
                observations['gate_status'] = json.loads(message.data)

        def on_gate_cmd(message):
            with lock:
                observations['gate_cmd'] = [message.linear.x, message.angular.z]
                observations['gate_cmds'] += 1

        def on_a6_status(message):
            report = json.loads(message.data)
            status.update(report=report, phase=report.get('phase'),
                          calls=report.get('relocalize_calls', 0))

        def on_a6_allowed(message):
            status['allowed'] = bool(message.data)

        def on_a6_lease(message):
            status['lease'] = int(message.data)

        def on_backend(message):
            payload = json.loads(message.data)
            with lock:
                observations['backend'] = payload
            if payload.get('lease') in [item['lease'] for item in observations['resets']]:
                return
            pose = payload['pose']
            yaw = payload.get('yaw')
            yaw = 0.0 if yaw is None else float(yaw)
            reset = PoseWithCovarianceStamped()
            reset.header.stamp, reset.header.frame_id = rospy.Time.now(), 'map'
            reset.pose.pose.position.x, reset.pose.pose.position.y = float(pose[0]), float(pose[1])
            reset.pose.pose.position.z = 0.0
            reset.pose.pose.orientation.z = math.sin(yaw / 2.0)
            reset.pose.pose.orientation.w = math.cos(yaw / 2.0)
            initialpose_pub.publish(reset)
            with lock:
                observations['resets'].append({'lease': int(payload['lease']),
                                               'pose': list(pose), 'yaw': yaw})

        cloud_pub = rospy.Publisher('/a6_ndt/cloud', PointCloud2, queue_size=2)
        initialpose_pub = rospy.Publisher('/a6_ndt/initialpose',
                                          PoseWithCovarianceStamped, queue_size=5)
        gate_candidate = rospy.Publisher('/a6/gate/candidate', Twist, queue_size=10)
        callbacks.extend([
            rospy.Subscriber('/rslidar_points', PointCloud2, on_cloud, queue_size=2),
            rospy.Subscriber('/gazebo/model_states', ModelStates, on_truth, queue_size=2),
            rospy.Subscriber('/a6_ndt/odom', Odometry, on_odom, queue_size=50),
            rospy.Subscriber('/a6_ndt/healthy', Bool, on_health, queue_size=20),
            rospy.Subscriber('/a6_ndt/relocalized', Bool, on_relocalized, queue_size=20),
            rospy.Subscriber('/a6/recovery/status', String, on_a6_status, queue_size=20),
            rospy.Subscriber('/a6/recovery/command_allowed', Bool, on_a6_allowed, queue_size=20),
            rospy.Subscriber('/a6/recovery/lease', UInt64, on_a6_lease, queue_size=10),
            rospy.Subscriber('/a6/recovery/backend', String, on_backend, queue_size=10),
            rospy.Subscriber('/a6/gate/status', String, on_gate_status, queue_size=20),
            rospy.Subscriber('/a6/gate/cmd_vel', Twist, on_gate_cmd, queue_size=20),
        ])
        stop_feed = threading.Event()

        def feed_candidate():
            # Keep a non-zero command offered even while lost: the gate, not the
            # command timeout, must be the reason the output drops to zero.
            while not stop_feed.is_set():
                gate_candidate.publish(Twist(linear=Vector3(x=.1)))
                time.sleep(.05)

        threading.Thread(target=feed_candidate, daemon=True).start()

        wait_until(lambda: observations['health'] is True and observations['odom'] is not None,
                   90, 'NDT initial tracking', diagnostic)
        wait_until(lambda: status['phase'] == 'TRACKING' and status['allowed']
                   and status['lease'] == 1, 30, 'A6 initial lease 1', diagnostic)
        wait_until(lambda: observations['gate_status'] is not None
                   and observations['gate_status'].get('active_lease') == 1,
                   20, 'gate lease 1', diagnostic)
        with lock:
            initial = {'odom': list(observations['odom']), 'truth': list(observations['truth']),
                       'health': observations['health'], 'lease': status['lease'],
                       'phase': status['phase'], 'allowed': status['allowed'],
                       'gate': dict(observations['gate_status'])}
        # The gate must pass the offered non-zero command only while healthy + leased.
        time.sleep(.4)
        with lock:
            gate_open = observations['gate_cmd']
        initial['gate_command'] = gate_open
        initial['ok'] = (gate_open is not None and abs(gate_open[0] - .1) < .01)

        cycles = []
        for index in range(args.cycles):
            expected = index + 2
            with lock:
                relay['paused'] = True
                calls_before = status['calls']
                resets_before = len(observations['resets'])
                gate_before = observations['gate_cmd']
            wait_until(lambda: status['phase'] == 'RELOCALIZING', 6, 'RELOCALIZING', diagnostic)
            wait_until(lambda: status['calls'] > calls_before, 6, 'relocalize service call',
                       diagnostic)
            time.sleep(.7)
            with lock:
                gate_during_loss = observations['gate_cmd']
            time.sleep(max(0.0, args.search_resume_delay_s - .7))
            if move_x or move_y:
                place(spawn_x + move_x, spawn_y + move_y)
            with lock:
                relay['paused'] = False
            wait_until(lambda: len(observations['resets']) > resets_before,
                       args.deadline_s + 5, 'backend seed lease %d' % expected, diagnostic)
            wait_until(lambda: status['phase'] == 'TRACKING' and status['allowed']
                       and status['lease'] == expected, 20, 'TRACKING lease %d' % expected,
                       diagnostic)
            wait_until(lambda: observations['gate_status'] is not None
                       and observations['gate_status'].get('active_lease') == expected,
                       20, 'gate lease %d' % expected)
            time.sleep(1.5)
            with lock:
                odom = list(observations['odom'])
                truth = list(observations['truth'])
                reset = observations['resets'][-1]
                backend = observations['backend']
                relocalized = observations['relocalized']
                gate_now = observations['gate_status']
            error = math.hypot(odom[0] - truth[0], odom[1] - truth[1])
            candidate_error = math.hypot(reset['pose'][0] - truth[0],
                                         reset['pose'][1] - truth[1])
            yaw_error = wrap(odom[3] - truth[3])
            cycles.append({'lease': expected, 'truth_xy': truth[:2], 'odom_xy': odom[:2],
                           'error_m': error, 'candidate_error_m': candidate_error,
                           'yaw_error_rad': yaw_error, 'reset': reset,
                           'backend': backend, 'relocalized_events': relocalized,
                           'gate': gate_now, 'gate_before_loss': gate_before,
                           'gate_during_loss': gate_during_loss,
                           'phase': status['phase'], 'allowed': status['allowed']})

        negative = None
        if args.negative:
            with lock:
                relay['paused'] = True
                calls_before = status['calls']
                lease_before = status['lease']
                resets_before = len(observations['resets'])
            wait_until(lambda: status['phase'] == 'RELOCALIZING', 6, 'negative RELOCALIZING')
            wait_until(lambda: status['calls'] > calls_before, 6, 'negative relocalize call')
            wait_until(lambda: status['phase'] == 'FAILED', args.deadline_s + 3, 'negative FAILED')
            time.sleep(1.0)
            with lock:
                negative = {'phase': status['phase'], 'allowed': status['allowed'],
                            'lease_before': lease_before, 'lease_after': status['lease'],
                            'new_reset': len(observations['resets']) > resets_before,
                            'gate': observations['gate_status'],
                            'gate_cmd': observations['gate_cmd']}

        result = {
            'domain': 'a6_ndt_recovery_live_gazebo',
            'hardware': False,
            'spawn_xy': [spawn_x, spawn_y], 'move_xy': [move_x, move_y],
            'initial': initial, 'cycles': cycles, 'negative': negative,
            'relocalize_budget_s': args.relocalize_budget_s,
            'deadline_s': args.deadline_s, 'gate_m': args.gate_m,
            'gate_rejection': {'gate_before_loss': initial.get('gate_command'),
                               'gate_during_loss': ([cycle['gate_during_loss']
                                                    for cycle in cycles]
                                                   if cycles else None)},
            'input_semantics': 'live Gazebo /rslidar_points through a pausable relay; '
                               'Gazebo truth used only for the post-recovery XY gate',
            'reset_path': 'A6 backend message -> NDT native /initialpose',
            'relocalize_path': 'A6 /relocalize trigger -> NDT bounded local search',
        }
        # The callbacks keep the latest gate output; sample it after loss above.
        result['ok'] = bool(
            initial['ok']
            and all(item['error_m'] <= args.gate_m and item['phase'] == 'TRACKING'
                    and item['allowed'] and item['gate_during_loss'] is not None
                    and abs(item['gate_during_loss'][0]) < 1e-9 for item in cycles)
            and (negative is None or (negative['phase'] == 'FAILED'
                                      and not negative['allowed']
                                      and negative['lease_after'] == negative['lease_before']
                                      and not negative['new_reset'])))
        print(json.dumps(result, ensure_ascii=False, indent=1))
        if args.out:
            Path(args.out).write_text(json.dumps(result, ensure_ascii=False, indent=1) + '\n')
        return 0 if result['ok'] else 6
    finally:
        stop_feed.set() if 'stop_feed' in locals() else None
        for subscription in callbacks:
            subscription.unregister()
        for process in (gate, a6, bridge, ndt, gazebo, master):
            stop(process)


if __name__ == '__main__':
    raise SystemExit(main())
