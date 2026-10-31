#!/usr/bin/env python3
"""ROS map-velocity -> 45-dim Lite3 policy in MuJoCo, flat stop pilot.

The local node speaks map-frame 3D velocity.  This bridge maps it to body
forward speed plus yaw hold (the proven flat policy regime), runs the 45-dim
observer/action contract, and publishes map/support pose plus world feedback.
It is a pilot bridge: no Gazebo, no real robot, not formal #151.
"""
import argparse
import json
import math
import time
from pathlib import Path

import numpy as np

import a5_v8x1_bridge as a5

DEFAULT_POSE = np.array([0., -.65, 1.3] * 4)
SCALE = np.array([.125, .25, .25] * 4)


def rot(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


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
    parser.add_argument('--speed-scale', type=float, default=1.)
    parser.add_argument('--max-command', type=float, default=.4)
    parser.add_argument('--cmd-timeout', type=float, default=.5)
    parser.add_argument('--initial-z', type=float, default=.35)
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--perturb', action='store_true')
    parser.add_argument('--sense', action='store_true')
    parser.add_argument('--sensor-range', type=float, default=2.0)
    args = parser.parse_args()
    value = json.loads(Path(args.map).read_text())

    import mujoco
    import onnxruntime as ort
    import rospy
    from geometry_msgs.msg import PoseStamped, TwistStamped
    from std_msgs.msg import Bool, String

    rospy.init_node('a5_v8x1_stop_bridge45', anonymous=True, disable_signals=True)
    model = mujoco.MjModel.from_xml_path(args.xml)
    data = mujoco.MjData(model)
    start_x, start_y = a5.map_to_world(0., args.start_map_y, args.x_offset, args.y_offset)
    perturb = a5.walk.sample_initial(args.seed, args.perturb)
    data.qpos[:3] = [start_x + perturb['dx'], start_y + perturb['dy'],
                     args.initial_z + perturb['dz']]
    data.qpos[3:7] = [math.cos(perturb['dyaw'] / 2.), 0., 0.,
                      math.sin(perturb['dyaw'] / 2.)]
    data.qpos[7:19] = DEFAULT_POSE
    mujoco.mj_forward(model, data)
    session = ort.InferenceSession(args.policy, providers=['CPUExecutionProvider'])
    inp = session.get_inputs()[0].name
    shared = {'velocity': (0., 0., 0.), 'at': -1e9, 'target': None}
    lock = __import__('threading').Lock()

    def on_cmd(message):
        with lock:
            shared['velocity'] = (message.twist.linear.x, message.twist.linear.y,
                                  message.twist.linear.z)
            shared['at'] = time.monotonic()

    def on_trajectory(message):
        try:
            payload = json.loads(message.data)
            points = payload['trajectory']
            if len(points) >= 2:
                with lock:
                    shared['target'] = tuple(points[1][:3])
        except (ValueError, KeyError, TypeError):
            pass

    rospy.Subscriber(args.prefix + '/cmd', TwistStamped, on_cmd, queue_size=10)
    rospy.Subscriber(args.prefix + '/trajectory', String, on_trajectory, queue_size=10)
    pose_pub = rospy.Publisher(args.prefix + '/pose', PoseStamped, queue_size=20)
    body_pub = rospy.Publisher(args.prefix + '/body', PoseStamped, queue_size=50)
    applied_pub = rospy.Publisher(args.prefix + '/applied', String, queue_size=20)
    loc_pub = rospy.Publisher(args.prefix + '/localization', Bool, queue_size=1)
    obstacle_pub = rospy.Publisher(args.prefix + '/stop_obstacles', String, queue_size=1)

    def ray_sense(model, data, sim_time):
        """Real MuJoCo raycast against the physical barrier; map-frame box."""
        if not args.sense:
            return None
        points = []
        for lateral in np.linspace(-.4, .4, 9):
            origin = np.array([data.qpos[0] + .45, data.qpos[1] + lateral, .35])
            geom = np.array([-1], dtype=np.int32)
            distance = float(mujoco.mj_ray(model, data, origin,
                                           np.array([1., 0., 0.]), None, 1, -1, geom))
            valid = math.isfinite(distance) and (distance < 0 or geom[0] >= 0)
            if valid and 0 <= distance <= args.sensor_range:
                points.append(origin + distance * np.array([1., 0., 0.]))
        if not points:
            return {'sensor_hit': False, 'stamp': sim_time, 'box': None}
        mapped = np.array([[-p[1], p[0] - 2.7] for p in points])
        lo, hi = mapped.min(axis=0), mapped.max(axis=0)
        box = {'min': [float(lo[0] - .12), float(lo[1] - .12), 0.],
               'max': [float(hi[0] + .12), float(hi[1] + .12), .5],
               'stamp': 0., 'velocity': [0., 0., 0.]}
        return {'sensor_hit': True, 'stamp': float(sim_time), 'box': box,
                'points': [p.tolist() for p in points]}
    last_action = np.zeros(12, dtype=np.float32)
    target = DEFAULT_POSE.copy()
    sensed_latched = {'box': None}
    dt = model.opt.timestep
    stride = max(1, int(round(.02 / dt)))
    next_tick = time.monotonic()
    tick = 0
    while not rospy.is_shutdown():
        now = time.monotonic()
        with lock:
            velocity = shared['velocity'] if now - shared['at'] <= args.cmd_timeout else (0., 0., 0.)
            target_point = shared['target']
        world_x, world_y = float(data.qpos[0]), float(data.qpos[1])
        map_x, map_y = a5.world_to_map(world_x, world_y, args.x_offset, args.y_offset)
        support = a5.support_z(value, map_x, map_y, args.radius)
        if support is None:
            support = 0.
        distance = None if target_point is None else math.hypot(map_x - target_point[0],
                                                              map_y - target_point[1])
        yaw = math.atan2(2. * (data.qpos[3] * data.qpos[6] + data.qpos[4] * data.qpos[5]),
                         1. - 2. * (data.qpos[5] ** 2 + data.qpos[6] ** 2))
        world_vx, world_vy = a5.map_velocity_to_world(velocity[0], velocity[1])
        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
        command = np.array([
            np.clip(cos_yaw * world_vx + sin_yaw * world_vy,
                    -args.max_command, args.max_command),
            np.clip(-sin_yaw * world_vx + cos_yaw * world_vy,
                    -args.max_command, args.max_command),
            float(np.clip(-yaw, -.15, .15))], dtype=np.float32)
        obs = np.concatenate([
            data.qvel[3:6] * .25,
            rot(data.qpos[3:7]).T @ np.array([0., 0., -1.]),
            command / .8,
            data.qpos[7:19] - DEFAULT_POSE,
            data.qvel[6:18] * .05,
            last_action]).astype(np.float32)
        action = session.run(None, {inp: obs.reshape(1, -1)})[0].reshape(-1)
        if np.all(np.isfinite(action)):
            last_action = action.astype(np.float32)
            target = action * SCALE + DEFAULT_POSE
        else:
            last_action = np.zeros(12, dtype=np.float32)
            target = DEFAULT_POSE.copy()
        for _ in range(stride):
            tau = np.clip(30. * (target - data.qpos[7:19]) - data.qvel[6:18], -30., 30.)
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
        body.pose.orientation.w, body.pose.orientation.x, body.pose.orientation.y, body.pose.orientation.z = (
            float(data.qpos[3]), float(data.qpos[4]), float(data.qpos[5]), float(data.qpos[6]))
        body_pub.publish(body)
        sensed = ray_sense(model, data, float(data.time))
        if args.sense:
            if sensed and sensed.get('box') and sensed_latched['box'] is None:
                sensed_latched['box'] = sensed['box']
            box = sensed_latched['box']
            if box is not None:
                obstacle_pub.publish(String(data=json.dumps({'obstacles': [box]})))
            else:
                obstacle_pub.publish(String(data=json.dumps({'obstacles': []})))
        if tick % 5 == 0:
            payload = {
                'map_velocity': list(velocity), 'body_command': command.tolist(),
                'support_z': support, 'world_xyz': [world_x, world_y, float(data.qpos[2])],
                'base_vxy': [float(data.qvel[0]), float(data.qvel[1])],
                'map_xyz': [map_x, map_y, support], 'sim_time': float(data.time),
                'target': target_point, 'waypoint': target_point,
                'sensor_hit': sensed_latched['box'] is not None,
                'sensed_box': sensed_latched['box'],
                'scenario': 'a5_v8x1_stop_bridge45_sensed' if args.sense else 'a5_v8x1_stop_bridge45'}
            applied_pub.publish(String(data=json.dumps(payload)))
        loc_pub.publish(Bool(data=True))
        tick += 1
        next_tick += .02
        delay = next_tick - time.monotonic()
        if delay > 0:
            time.sleep(delay)


if __name__ == '__main__':
    main()
