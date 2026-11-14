#!/usr/bin/env python3
"""Kinematic 3D carrier inside Gazebo for the A5 local planner.

Applies the map-frame 3D velocity directly to a static Gazebo model with
SetModelState and returns the same pose to the planner. This removes the planar
carrier's vertical rejection by construction; it is a kinematic pose sink, not
Gazebo contact dynamics and not a four-leg climb.
"""
import argparse
import json
import math
import time

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import String

# 支撑高跟随的可达门槛，与 local_native.Config.max_step_m 默认值对齐：
# 只允许登上比当前 z 高不超过一步的面（坡面切片层差 0.06 << 0.2）。
MAX_STEP_M = 0.2


def advance(position, velocity, dt):
    return tuple(position[i] + velocity[i] * dt for i in range(3))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prefix', required=True)
    parser.add_argument('--model', required=True)
    parser.add_argument('--start', default='0,-2.175,0')
    parser.add_argument('--center-z', type=float, default=0.1)
    parser.add_argument('--cmd-timeout', type=float, default=0.5)
    parser.add_argument('--snap-map', default='')
    parser.add_argument('--snap-m', type=float, default=0.05)
    # rospy.myargv() 剥离 roslaunch 注入的 __name:= / __log:= 重映射参数，
    # 否则 parse_args() 视其为未识别参数、以 exit 2 秒退（坡面栈首次起栈实证）。
    args = parser.parse_args(rospy.myargv()[1:])
    start = tuple(float(v) for v in args.start.split(','))
    snap_surfaces = []
    snap_resolution = .2
    if args.snap_map:
        with open(args.snap_map, encoding='utf-8') as stream:
            document = json.load(stream)
        snap_resolution = float(document.get('resolution', .2))
        snap_surfaces = document.get('surfaces', [])

    def decode_cells(surface):
        cells = surface.get('cells')
        if not isinstance(cells, dict) or cells.get('encoding') != 'rle':
            return None
        decoded = set()
        for row, first, count in cells.get('runs', []):
            for column in range(first, first + count):
                decoded.add((column, row))
        return decoded

    snap_cells = {index: decode_cells(surface) for index, surface in enumerate(snap_surfaces)}

    def snap_contains(index, x, y):
        """格语义 XY 归属，与规划层 contains_xy 同款（floor(v/res+1e-9)∈cells）。

        用 bbox 判定会在格边界上与规划层分叉：载体积分停在 x=1.4-1.7e-10 时，
        bbox 把它归左片、规划层 +1e-9 容差归右片，z 吸附与支撑判定互相矛盾，
        即卡死为 E_LOCAL_SUPPORT（坡面栈第九次起栈实证）。无格集时退回 bbox。
        """
        surface = snap_surfaces[index]
        cells = snap_cells[index]
        if cells is None:
            box = surface['footprint']
            return (box['min_x'] <= x <= box['max_x'] and box['min_y'] <= y <= box['max_y'])
        return (math.floor(x / snap_resolution + 1e-9),
                math.floor(y / snap_resolution + 1e-9)) in cells
    rospy.init_node('gazebo_3d_carrier', anonymous=True, disable_signals=True)
    rospy.wait_for_service('/gazebo/set_model_state', timeout=20.)
    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    shared = {'velocity': (0., 0., 0.), 'at': -1e9}

    def on_cmd(message):
        shared['velocity'] = (message.twist.linear.x, message.twist.linear.y, message.twist.linear.z)
        shared['at'] = time.monotonic()

    rospy.Subscriber(args.prefix + '/cmd', TwistStamped, on_cmd, queue_size=50)
    pose_pub = rospy.Publisher(args.prefix + '/pose', PoseStamped, queue_size=50)
    body_pub = rospy.Publisher(args.prefix + '/body', PoseStamped, queue_size=50)
    status_pub = rospy.Publisher(args.prefix + '/carrier', String, queue_size=20)

    position = start
    last = time.monotonic()
    tick = 0
    while not rospy.is_shutdown():
        now = time.monotonic()
        velocity = shared['velocity'] if now - shared['at'] <= args.cmd_timeout else (0., 0., 0.)
        position = advance(position, velocity, max(0., now - last))
        # z 完全由支撑面语义决定（kinematic 位姿沉＝支撑点跟随）：取 XY 格
        # 归属的面中 z_reference <= 当前 z + max_step 的最高者为支撑高。台阶
        # 近似坡的 z 在格界处阶梯跳变，与规划层/地图分格零漂移。vz 指令不
        # 参与积分——第九、十次起栈实证：vz 积分＋双向 snap 在面间平手处
        # 反复回零（ground/ramp01 差相等时 min 取低面），载体上不了第一片。
        best_z = None
        for index, surface in enumerate(snap_surfaces):
            if (snap_contains(index, position[0], position[1])
                    and surface['z_reference'] <= position[2] + MAX_STEP_M):
                if best_z is None or surface['z_reference'] > best_z:
                    best_z = surface['z_reference']
        if best_z is not None:
            position = (position[0], position[1], best_z)
        last = now
        stamp = rospy.Time.now()
        pose = PoseStamped()
        pose.header.frame_id, pose.header.stamp = 'map', stamp
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = position
        pose.pose.orientation.w = 1.
        pose_pub.publish(pose)
        body = PoseStamped()
        body.header.frame_id, body.header.stamp = 'map', stamp
        body.pose.position.x, body.pose.position.y = position[0], position[1]
        body.pose.position.z = position[2]
        body.pose.orientation.w = 1.
        body_pub.publish(body)
        state = ModelState()
        state.model_name, state.reference_frame = args.model, 'world'
        state.pose.position.x, state.pose.position.y = position[0], position[1]
        state.pose.position.z = position[2] + args.center_z
        state.pose.orientation.w = 1.
        try:
            set_state(state)
        except rospy.ServiceException:
            pass
        if tick % 5 == 0:
            status_pub.publish(String(data=json.dumps({
                'position': position, 'velocity': velocity,
                'model_center_z': position[2] + args.center_z,
                'velocity_age_s': now - shared['at']})))
        tick += 1
        time.sleep(.02)


if __name__ == '__main__':
    main()
