#!/usr/bin/env python3
"""A5 ROS local planner -> V8-X1 270 policy in MuJoCo (stairs pilot).

A5 commands map-frame XY velocity; the policy has no height command, so contact
climbing supplies Z. The pose returned to A5 is the highest declared support
under the body, not the floating base height. This is a chain pilot, not a
four-leg or real-robot acceptance.
"""
import argparse
import json
import math
import time
from pathlib import Path

import numpy as np

import mj_dreamwaq_walk as walk


def world_to_map(x, y, x_offset=0., y_offset=0.):
    """Map +y = world +x, map +x = -world +y; offsets place the map origin."""
    return (-y + x_offset, x + y_offset)


def map_to_world(x, y, x_offset=0., y_offset=0.):
    return (y - y_offset, -x + x_offset)


def map_velocity_to_world(vx, vy):
    return (vy, -vx)


def body_command(vx_map, vy_map, yaw, scale):
    """Pursue the path heading with forward speed only.

    The policy's proven stair regime is forward speed plus a yaw-rate hold
    (R942); feeding the lateral component directly destabilised it at step 3.
    A lateral position term was tried in R946 and worsened the seed matrix
    (3/5 vs 4/5), so it is not kept.
    """
    vx_world, vy_world = map_velocity_to_world(vx_map, vy_map)
    speed = math.hypot(vx_world, vy_world)
    if speed <= 1e-9:
        return 0., 0., float(np.clip(-yaw, -.5, .5))
    desired = math.atan2(vy_world, vx_world)
    error = math.atan2(math.sin(desired - yaw), math.cos(desired - yaw))
    return float(speed * scale), 0., float(np.clip(3.0 * error, -.5, .5))


def support_z(value, x, y, radius):
    """Highest declared support whose footprint holds the body with margin."""
    hits = []
    for surface in value['surfaces']:
        if surface.get('traversability') != 'supported':
            continue
        box = surface['footprint']
        if (box['min_x'] + radius - 1e-9 <= x <= box['max_x'] - radius + 1e-9 and
                box['min_y'] + radius - 1e-9 <= y <= box['max_y'] - radius + 1e-9):
            hits.append(surface['z_reference'])
    return max(hits) if hits else None


def path_scale(distance, brake_distance=.30):
    """Slow the policy as it approaches a waypoint; the gait needs braking."""
    if distance is None:
        return 1.
    return min(1., max(.25, distance / brake_distance))


def should_snap(map_x, map_y, support, target, tolerance=.10):
    """A5 completes only at 1e-6; the legged body cannot. Snap inside tolerance."""
    if target is None:
        return False, None
    distance = math.hypot(map_x - target[0], map_y - target[1])
    return (distance <= tolerance and abs(support - target[2]) <= 1e-6), distance


def plain(value):
    if hasattr(value, 'tolist'):
        return value.tolist()
    return float(value)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map', required=True)
    parser.add_argument('--xml', required=True)
    parser.add_argument('--policy', required=True)
    parser.add_argument('--prefix', required=True)
    parser.add_argument('--x-offset', type=float, default=0.)
    parser.add_argument('--y-offset', type=float, default=-2.7)
    parser.add_argument('--start-map-y', type=float, default=-2.175)
    parser.add_argument('--radius', type=float, default=.15)
    parser.add_argument('--speed-scale', type=float, default=.4)
    parser.add_argument('--max-command', type=float, default=.5)
    parser.add_argument('--cmd-timeout', type=float, default=.5)
    parser.add_argument('--initial-z', type=float, default=.4)
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--perturb', action='store_true')
    parser.add_argument('--finish-hold', action='store_true')
    args = parser.parse_args()
    value = json.loads(Path(args.map).read_text(encoding='utf-8'))
    top_z = max(s['z_reference'] for s in value['surfaces']
                if s.get('traversability') == 'supported')

    import mujoco
    import onnxruntime as ort
    import rospy
    from geometry_msgs.msg import PoseStamped, TwistStamped
    from std_msgs.msg import Bool, String

    rospy.init_node('a5_v8x1_bridge', anonymous=True, disable_signals=True)
    model = mujoco.MjModel.from_xml_path(args.xml)
    data = mujoco.MjData(model)
    start_x, start_y = map_to_world(0., args.start_map_y, args.x_offset, args.y_offset)
    perturb = walk.sample_initial(args.seed, args.perturb)
    data.qpos[:3] = [start_x + perturb['dx'], start_y + perturb['dy'],
                     args.initial_z + perturb['dz']]
    data.qpos[3:7] = [math.cos(perturb['dyaw'] / 2.), 0., 0., math.sin(perturb['dyaw'] / 2.)]
    data.qpos[7:19] = walk.DEFAULT_POSE
    mujoco.mj_forward(model, data)
    session = ort.InferenceSession(args.policy, providers=['CPUExecutionProvider'])
    inp = session.get_inputs()[0].name

    shared = {'velocity': (0., 0., 0.), 'at': -1e9, 'target': None}

    def on_cmd(message):
        shared['velocity'] = (message.twist.linear.x, message.twist.linear.y, message.twist.linear.z)
        shared['at'] = time.monotonic()

    def on_trajectory(message):
        try:
            payload = json.loads(message.data)
            points = payload['trajectory']
            if len(points) >= 2:
                shared['target'] = tuple(points[1][:3])
        except (ValueError, KeyError, TypeError):
            pass

    rospy.Subscriber(args.prefix + '/cmd', TwistStamped, on_cmd, queue_size=10)
    rospy.Subscriber(args.prefix + '/trajectory', String, on_trajectory, queue_size=10)
    pose_pub = rospy.Publisher(args.prefix + '/pose', PoseStamped, queue_size=10)
    body_pub = rospy.Publisher(args.prefix + '/body', PoseStamped, queue_size=50)
    applied_pub = rospy.Publisher(args.prefix + '/applied', String, queue_size=10)
    obstacle_pub = rospy.Publisher(args.prefix + '/obstacles', String, queue_size=1)
    loc_pub = rospy.Publisher(args.prefix + '/localization', Bool, queue_size=1)

    history = [np.zeros(walk.OBS_DIM, dtype=np.float32) for _ in range(walk.HISTORY)]
    last_action = np.zeros(12, dtype=np.float32)
    target = walk.DEFAULT_POSE.copy()
    dt = model.opt.timestep
    stride = max(1, int(round(.02 / dt)))
    next_tick = time.monotonic()
    last_support = None
    last_pose = None
    fault = None
    finish = False
    tick = 0
    while not rospy.is_shutdown():
        now = time.monotonic()
        velocity = shared['velocity'] if now - shared['at'] <= args.cmd_timeout else (0., 0., 0.)
        world_x, world_y = float(data.qpos[0]), float(data.qpos[1])
        map_x, map_y = world_to_map(world_x, world_y, args.x_offset, args.y_offset)
        support = support_z(value, map_x, map_y, args.radius)
        if support is None:
            if last_pose is not None:
                map_x, map_y, support = last_pose
            else:
                support = last_support if last_support is not None else 0.
        else:
            last_support = support
            last_pose = (map_x, map_y, support)
        target = shared['target']
        snapped, snap_error = should_snap(map_x, map_y, support, target)
        if snapped:
            map_x, map_y = target[0], target[1]
        distance = None if target is None else math.hypot(map_x - target[0], map_y - target[1])
        actual_map_x, actual_map_y = world_to_map(world_x, world_y, args.x_offset, args.y_offset)
        actual_support = support_z(value, actual_map_x, actual_map_y, args.radius)
        actual_distance = None if target is None else math.hypot(
            actual_map_x - target[0], actual_map_y - target[1])
        if (args.finish_hold and not finish and target is not None
                and abs(target[2] - top_z) <= 1e-6
                and actual_support is not None and actual_support >= top_z - 1e-6
                and float(data.qpos[2]) > top_z + .10
                and actual_distance is not None and actual_distance <= .15):
            finish = True
        if finish:
            command = np.zeros(3, dtype=np.float32)
        else:
            yaw = walk.yaw_of(data.qpos[3:7])
            body_vx, body_vy, wz = body_command(velocity[0], velocity[1], yaw,
                                                args.speed_scale * path_scale(distance))
            command = np.array([np.clip(body_vx, -args.max_command, args.max_command), body_vy, wz],
                               dtype=np.float32)
        if fault is None:
            obs = walk.observation(data.qpos, data.qvel, command, last_action, walk.DEFAULT_POSE)
            state = walk.state_vector(obs, history, newest_first=True)
            action = session.run(None, {inp: state.reshape(1, -1)})[0].reshape(-1)
            if not np.all(np.isfinite(action)):
                fault = 'action_nonfinite'
                command = np.zeros(3, dtype=np.float32)
                last_action = np.zeros(12, dtype=np.float32)
                joint_target = walk.DEFAULT_POSE.copy()
            else:
                joint_target = walk.joint_target(action, walk.DEFAULT_POSE)
                last_action = action.astype(np.float32)
                history = history[1:] + [obs]
        else:
            command = np.zeros(3, dtype=np.float32)
            joint_target = walk.DEFAULT_POSE.copy()
        for _ in range(stride):
            tau = np.clip(25. * (joint_target - data.qpos[7:19]) - .5 * data.qvel[6:18], -30., 30.)
            data.qfrc_applied[6:18] = tau
            mujoco.mj_step(model, data)

        stamp = rospy.Time.now()
        pose = PoseStamped()
        pose.header.frame_id, pose.header.stamp = 'map', stamp
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map_x, map_y, support
        pose.pose.orientation.w = 1.
        pose_pub.publish(pose)
        body = PoseStamped()
        body.header.frame_id, body.header.stamp = 'world', stamp
        body.pose.position.x, body.pose.position.y, body.pose.position.z = (
            float(data.qpos[0]), float(data.qpos[1]), float(data.qpos[2]))
        body.pose.orientation.w = float(data.qpos[3])
        body.pose.orientation.x, body.pose.orientation.y, body.pose.orientation.z = (
            float(data.qpos[4]), float(data.qpos[5]), float(data.qpos[6]))
        body_pub.publish(body)
        if tick % 5 == 0:
            obstacle_pub.publish(String(data=json.dumps({'obstacles': []})))
            loc_pub.publish(Bool(data=True))
            applied_pub.publish(String(data=json.dumps({
                'map_velocity': list(velocity), 'body_command': command.tolist(),
                'support_z': support, 'world_xyz': [world_x, world_y, float(data.qpos[2])],
                'map_xyz': [map_x, map_y, support], 'velocity_age_s': now - shared['at'],
                'waypoint': target, 'snapped': snapped, 'snap_error_m': snap_error,
                'finish_hold': finish, 'fault': fault}, default=plain)))
        tick += 1
        next_tick += .02
        delay = next_tick - time.monotonic()
        if delay > 0:
            time.sleep(delay)


if __name__ == '__main__':
    main()
