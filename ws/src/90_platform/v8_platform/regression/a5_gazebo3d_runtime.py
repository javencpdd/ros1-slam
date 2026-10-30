#!/usr/bin/env python3
"""A5 ROS local planner -> Gazebo kinematic 3D carrier on the scaled NIST world.

The carrier applies map-frame 3D velocity to a static Gazebo model, so vertical
transfers are executed instead of rejected by the planar carrier. Kinematic pose
sink only: no Gazebo contact dynamics, no four-leg climb, no real robot.
"""
import argparse
import json
import math
import os
import signal
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
LOCAL = HERE.parent / 'local_planner'
SCRIPTS = HERE.parents[2] / '95_simulation' / 'lite3_gazebo_sim' / 'scripts'
EXPERIMENTAL = {'tick_s': 1.2, 'horizon_s': 6.0, 'validity_s': 6.0, 'clearance_m': .1}


def support_sequence(trace, levels, tolerance=1e-4):
    found, times = [], []
    for level in levels:
        for sample in trace:
            if abs(sample[3] - level) <= tolerance:
                found.append(level)
                times.append(sample[0])
                break
    return found, times


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map', required=True)
    parser.add_argument('--terrain', default='nist_stairs_120_s020x170')
    parser.add_argument('--out', required=True)
    parser.add_argument('--wall-deadline-s', type=float, default=120.)
    parser.add_argument('--goal-map', default='0,1.34,0.890358')
    args = parser.parse_args()
    value = json.loads(Path(args.map).read_text(encoding='utf-8'))
    goal = tuple(float(v) for v in args.goal_map.split(','))

    import rospy
    from gazebo_msgs.msg import ModelState
    from gazebo_msgs.srv import DeleteModel, GetLinkState, GetWorldProperties, SetModelState, SpawnModel
    from geometry_msgs.msg import Pose, PoseStamped
    from std_msgs.msg import Bool, String
    rospy.init_node('a5_gazebo3d_runtime', anonymous=True)
    carrier_name = 'a5_carrier_%d' % os.getpid()
    rospy.wait_for_service('/gazebo/spawn_sdf_model', timeout=20.)
    rospy.wait_for_service('/gazebo/get_link_state', timeout=20.)
    rospy.wait_for_service('/gazebo/delete_model', timeout=20.)
    spawn = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    delete = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
    get_link = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
    world = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
    sdf = ('<sdf version="1.6"><model name="%s"><static>true</static>'
           '<link name="link"><visual name="visual"><geometry><box><size>0.45 0.30 0.20</size></box>'
           '</geometry><material><ambient>0.2 0.5 0.9 1</ambient></material></visual>'
           '</link></model></sdf>' % carrier_name)
    result = spawn(carrier_name, sdf, '', Pose(), 'world')
    if not result.success:
        raise RuntimeError('spawn failed: ' + result.status_message)
    world_models = list(world().model_names)

    node_name = 'a5gz3d_%d' % os.getpid()
    prefix = '/' + node_name
    shared = {'pose': None, 'body': None, 'status': None, 'plans': 0, 'last_plan': None}
    pose_trace, status_log = [], []
    lock = threading.Lock()

    def on_pose(message):
        p = message.pose.position
        with lock:
            shared['pose'] = (p.x, p.y, p.z)
            pose_trace.append([message.header.stamp.to_sec(), p.x, p.y, p.z])

    def on_body(message):
        p = message.pose.position
        with lock:
            shared['body'] = (p.x, p.y, p.z)

    def on_status(message):
        try:
            payload = json.loads(message.data)
        except ValueError:
            return
        with lock:
            shared['status'] = payload
            status_log.append([rospy.Time.now().to_sec(), payload.get('reason')])

    def on_trajectory(message):
        try:
            payload = json.loads(message.data)
        except ValueError:
            return
        with lock:
            shared['plans'] += 1
            shared['last_plan'] = payload

    callbacks = [
        rospy.Subscriber(prefix + '/pose', PoseStamped, on_pose, queue_size=200),
        rospy.Subscriber(prefix + '/body', PoseStamped, on_body, queue_size=200),
        rospy.Subscriber(prefix + '/status', String, on_status, queue_size=100),
        rospy.Subscriber(prefix + '/trajectory', String, on_trajectory, queue_size=10)]
    goal_pub = rospy.Publisher(prefix + '/goal', PoseStamped, queue_size=1, latch=True)
    obstacle_pub = rospy.Publisher(prefix + '/obstacles', String, queue_size=1)
    loc_pub = rospy.Publisher(prefix + '/localization', Bool, queue_size=1)
    pubs = [goal_pub, obstacle_pub, loc_pub]
    config = dict(EXPERIMENTAL, resolution=value['resolution'])

    with tempfile.TemporaryDirectory(prefix='a5gz3d_') as directory:
        map_path, node_log, carrier_log = (Path(directory) / 'map.json',
                                           Path(directory) / 'node.log',
                                           Path(directory) / 'carrier.log')
        map_path.write_text(json.dumps(value), encoding='utf-8')
        node_argv = [sys.executable, '-B', str(LOCAL / 'ros_node.py'),
                     '__name:=' + node_name, '_map_json:=' + str(map_path),
                     '_pose_topic:=' + prefix + '/pose', '_goal_topic:=' + prefix + '/goal',
                     '_obstacles_topic:=' + prefix + '/obstacles',
                     '_localization_topic:=' + prefix + '/localization',
                     '_command_topic:=' + prefix + '/cmd', '_status_topic:=' + prefix + '/status',
                     '_trajectory_topic:=' + prefix + '/trajectory',
                     '_experimental_config:=' + json.dumps(config)]
        carrier_argv = [sys.executable, '-B', str(SCRIPTS / 'gazebo_3d_carrier.py'),
                        '--prefix', prefix, '--model', carrier_name, '--start', '0,-2.175,0']
        node_stream, carrier_stream = node_log.open('w'), carrier_log.open('w')
        node = subprocess.Popen(node_argv, stdout=node_stream, stderr=subprocess.STDOUT)
        carrier = subprocess.Popen(carrier_argv, stdout=carrier_stream, stderr=subprocess.STDOUT)
        link_pose = None
        try:
            deadline = time.monotonic() + args.wall_deadline_s
            while time.monotonic() < deadline and not all(p.get_num_connections() for p in pubs):
                if node.poll() is not None or carrier.poll() is not None:
                    raise RuntimeError('child exited: ' + node_log.read_text() + carrier_log.read_text())
                time.sleep(.05)
            if not all(p.get_num_connections() for p in pubs):
                raise RuntimeError('subscription readiness timeout')
            message = PoseStamped()
            message.header.frame_id, message.pose.orientation.w = 'map', 1.
            message.pose.position.x, message.pose.position.y, message.pose.position.z = goal
            goal_pub.publish(message)
            arrived = False
            while time.monotonic() < deadline and not rospy.is_shutdown():
                obstacle_pub.publish(String(data=json.dumps({'obstacles': []})))
                loc_pub.publish(Bool(data=True))
                time.sleep(.05)
                with lock:
                    pose = shared['pose']
                if pose is None:
                    continue
                distance = math.sqrt((pose[0] - goal[0]) ** 2 + (pose[1] - goal[1]) ** 2)
                if distance <= .05 and abs(pose[2] - goal[2]) <= 1e-4:
                    arrived = True
                    break
            link = get_link(carrier_name + '::link', 'world').link_state.pose.position
            link_pose = [link.x, link.y, link.z]
        finally:
            for proc in (carrier, node):
                if proc.poll() is None:
                    proc.send_signal(signal.SIGTERM)
            for proc in (carrier, node):
                try:
                    proc.wait(timeout=5.)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait(timeout=5.)
            node_stream.close()
            carrier_stream.close()
            for obj in callbacks + pubs:
                obj.unregister()
            try:
                delete(carrier_name)
            except rospy.ServiceException:
                pass
        node_text, carrier_text = node_log.read_text(), carrier_log.read_text()

    with lock:
        pose_view = list(pose_trace)
        status_view = list(status_log)
        plans = shared['plans']
        last_plan = shared['last_plan']
        status = shared['status']
    levels = sorted({surface['z_reference'] for surface in value['surfaces']
                     if surface.get('traversability') == 'supported'})
    sequence, sequence_times = support_sequence(pose_view, levels)
    climbed = bool(sequence == levels and all(a < b for a, b in zip(sequence_times, sequence_times[1:])))
    final = pose_view[-1] if pose_view else None
    rejected = any(row[1] == 'E_CARRIER_VERTICAL_UNSUPPORTED' for row in status_view)
    checks = {
        'terrain_loaded': args.terrain in world_models,
        'climbed': climbed,
        'arrived_goal': bool(arrived),
        'no_vertical_rejection': not rejected,
        'a5_active': bool(plans),
        'gazebo_pose_matches': bool(link_pose and final and
                                    abs(link_pose[0] - final[1]) <= .02 and
                                    abs(link_pose[1] - final[2]) <= .02 and
                                    abs(link_pose[2] - (final[3] + .1)) <= .02),
    }
    ok = all(checks.values())
    payload = {
        'scenario': 'a5_ros_local_to_gazebo_3d_carrier',
        'config': config, 'goal_map': list(goal), 'terrain': args.terrain,
        'carrier_model': carrier_name, 'carrier_scope': 'static box moved by SetModelState',
        'scope': 'Gazebo kinematic pose sink; no contact dynamics/no four-leg/no real robot',
        'ok': ok, 'checks': checks, 'support_sequence': sequence, 'support_levels': levels,
        'final_pose': final, 'gazebo_link_pose': link_pose, 'plans': plans,
        'last_status': status, 'last_plan': last_plan,
        'pose_trace': pose_view[::5], 'status_log': status_view[::5],
        'world_models': world_models, 'node_log': node_text, 'carrier_log': carrier_text,
    }
    Path(args.out).write_text(json.dumps(payload, ensure_ascii=False, indent=1) + '\n')
    print(json.dumps({'ok': ok, 'checks': checks, 'final': final, 'gazebo_link': link_pose,
                      'plans': plans}, ensure_ascii=False))
    return 0 if ok else 6


if __name__ == '__main__':
    raise SystemExit(main())
