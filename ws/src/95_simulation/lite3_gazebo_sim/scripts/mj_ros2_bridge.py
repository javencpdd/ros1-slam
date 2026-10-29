#!/usr/bin/env python3
"""ROS2 (Foxy) topic bridge for a MuJoCo Lite3 rollout.

Mirrors mj_ros_bridge.py on ROS2: /cmd_vel in; /odom, /sim/odom_gt,
/joint_states, /imu, /scan, /clock and TF odom->base_link out. Test harness
only; the PPO policy is the same flat-locomotion checkpoint.
"""
import argparse
import math
import time

import numpy as np

DEFAULT_POSE = np.array([0, -0.65, 1.3] * 4)
SCALE = np.array([0.125, 0.25, 0.25] * 4)


def rot(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def yaw_of(q):
    w, x, y, z = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--xml', required=True)
    parser.add_argument('--policy', required=True)
    parser.add_argument('--initial-z', type=float, default=0.35)
    parser.add_argument('--realtime-factor', type=float, default=1.0)
    parser.add_argument('--cmd-timeout', type=float, default=0.5)
    parser.add_argument('--scan-rate', type=float, default=10.0)
    parser.add_argument('--scan-rays', type=int, default=360)
    parser.add_argument('--scan-range', type=float, default=8.0)
    parser.add_argument('--max-seconds', type=float, default=0.0)
    args = parser.parse_args()
    if args.realtime_factor <= 0 or args.scan_rate <= 0 or args.scan_rays < 8:
        raise SystemExit('positive realtime/scan rate and >=8 rays required')

    import mujoco
    import onnxruntime as ort
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist, TransformStamped
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import Imu, JointState, LaserScan
    from tf2_ros import TransformBroadcaster

    rclpy.init()
    node = Node('mj_ros2_bridge')
    model = mujoco.MjModel.from_xml_path(args.xml)
    data = mujoco.MjData(model)
    data.qpos[:3] = [0., 0., args.initial_z]
    data.qpos[7:19] = DEFAULT_POSE
    mujoco.mj_forward(model, data)
    for index in range(model.ngeom):
        if model.geom_bodyid[index] != 0:
            model.geom_group[index] = 1
    scan_group = np.array([1, 0, 0, 0, 0, 0], dtype=np.uint8)

    session = ort.InferenceSession(args.policy, providers=['CPUExecutionProvider'])
    inp = session.get_inputs()[0].name
    last_action = np.zeros(12)
    state = {'command': np.zeros(3), 'stamp': time.monotonic()}

    def on_cmd(message):
        state['command'] = np.array([message.linear.x, message.linear.y, message.angular.z])
        state['stamp'] = time.monotonic()

    node.create_subscription(Twist, '/cmd_vel', on_cmd, 10)
    odom_pub = node.create_publisher(Odometry, '/odom', 20)
    gt_pub = node.create_publisher(Odometry, '/sim/odom_gt', 20)
    joint_pub = node.create_publisher(JointState, '/joint_states', 20)
    imu_pub = node.create_publisher(Imu, '/imu', 20)
    scan_pub = node.create_publisher(LaserScan, '/scan', 5)
    clock_pub = node.create_publisher(Clock, '/clock', 20)
    broadcaster = TransformBroadcaster(node)

    joint_names = []
    for leg in ('FL', 'FR', 'HL', 'HR'):
        joint_names += [leg + '_HipX_joint', leg + '_HipY_joint', leg + '_Knee_joint']
    joint_indices = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in joint_names]
    if any(index < 0 for index in joint_indices):
        raise SystemExit('cannot resolve the 12 leg joints by name')
    qpos_adr = np.array([model.jnt_qposadr[index] for index in joint_indices])
    qvel_adr = np.array([model.jnt_dofadr[index] for index in joint_indices])

    dt = model.opt.timestep
    stride = max(1, int(round(0.02 / dt)))
    scan_every = max(1, int(round(1.0 / (args.scan_rate * 0.02))))
    tick = 0
    wall_start = time.monotonic()
    sim_start = float(data.time)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            if time.monotonic() - state['stamp'] > args.cmd_timeout:
                state['command'] = np.zeros(3)
            command = state['command']
            obs = np.concatenate([
                data.qvel[3:6] * 0.25,
                rot(data.qpos[3:7]).T @ np.array([0, 0, -1.0]),
                command / 0.8,
                data.qpos[7:19] - DEFAULT_POSE,
                data.qvel[6:18] * 0.05,
                last_action]).astype(np.float32)
            out = session.run(None, {inp: obs.reshape(1, -1)})[0].reshape(-1)
            last_action = out.astype(np.float32)
            target = out * SCALE + DEFAULT_POSE
            for _ in range(stride):
                tau = np.clip(30.0 * (target - data.qpos[7:19]) - 1.0 * data.qvel[6:18], -30, 30)
                data.qfrc_applied[6:18] = tau
                mujoco.mj_step(model, data)

            stamp = node.get_clock().now().to_msg()
            stamp.sec = int(data.time)
            stamp.nanosec = int(round((data.time - int(data.time)) * 1e9))
            clock_pub.publish(Clock(clock=stamp))
            quat = data.qpos[3:7]
            pose = Odometry()
            pose.header.stamp = stamp
            pose.header.frame_id = 'odom'
            pose.child_frame_id = 'base_link'
            pose.pose.pose.position.x, pose.pose.pose.position.y, pose.pose.pose.position.z = data.qpos[:3]
            pose.pose.pose.orientation.w, pose.pose.pose.orientation.x = quat[0], quat[1]
            pose.pose.pose.orientation.y, pose.pose.pose.orientation.z = quat[2], quat[3]
            body_velocity = rot(quat).T @ data.qvel[:3]
            pose.twist.twist.linear.x, pose.twist.twist.linear.y, pose.twist.twist.linear.z = body_velocity
            pose.twist.twist.angular.x, pose.twist.twist.angular.y, pose.twist.twist.angular.z = data.qvel[3:6]
            odom_pub.publish(pose)
            gt_pub.publish(pose)

            joint = JointState()
            joint.header.stamp = stamp
            joint.name = joint_names
            joint.position = np.asarray(data.qpos)[qpos_adr].tolist()
            joint.velocity = np.asarray(data.qvel)[qvel_adr].tolist()
            joint_pub.publish(joint)

            imu = Imu()
            imu.header.stamp = stamp
            imu.header.frame_id = 'base_link'
            imu.orientation.w, imu.orientation.x = quat[0], quat[1]
            imu.orientation.y, imu.orientation.z = quat[2], quat[3]
            imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = data.qvel[3:6]
            imu_pub.publish(imu)

            if tick % scan_every == 0:
                lidar = np.array([data.qpos[0], data.qpos[1], data.qpos[2] + 0.34])
                yaw = yaw_of(quat)
                scan = LaserScan()
                scan.header.stamp = stamp
                scan.header.frame_id = 'base_link'
                scan.angle_min = -math.pi
                scan.angle_max = math.pi
                scan.angle_increment = 2 * math.pi / args.scan_rays
                scan.range_min, scan.range_max = 0.1, args.scan_range
                geom_id = np.zeros(1, dtype=np.int32)
                ranges = []
                for ray in range(args.scan_rays):
                    angle = scan.angle_min + ray * scan.angle_increment + yaw
                    vector = np.array([math.cos(angle), math.sin(angle), 0.])
                    distance = mujoco.mj_ray(model, data, lidar, vector, scan_group, 1, -1, geom_id)
                    ranges.append(args.scan_range if distance < 0 else min(distance, args.scan_range))
                scan.ranges = ranges
                scan_pub.publish(scan)

            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = 'odom'
            transform.child_frame_id = 'base_link'
            transform.transform.translation.x = float(data.qpos[0])
            transform.transform.translation.y = float(data.qpos[1])
            transform.transform.translation.z = float(data.qpos[2])
            transform.transform.rotation.w, transform.transform.rotation.x = quat[0], quat[1]
            transform.transform.rotation.y, transform.transform.rotation.z = quat[2], quat[3]
            broadcaster.sendTransform(transform)

            tick += 1
            if args.max_seconds > 0 and data.time - sim_start >= args.max_seconds:
                break
            deadline = wall_start + (data.time - sim_start) / args.realtime_factor
            sleep = deadline - time.monotonic()
            if sleep > 0:
                time.sleep(sleep)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
