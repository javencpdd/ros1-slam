#!/usr/bin/env python3
"""Graph-compatible kinematic 3D carrier backed by a Gazebo static box.

Subscribes the mux output (geometry_msgs/Twist), integrates map-frame velocity
and applies it with SetModelState. Publishes the same pose/localization contract
as the pure-ROS carrier. This is a kinematic pose sink: no contact dynamics.
"""
import json
import math
import time

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import DeleteModel, SetModelState, SpawnModel
from geometry_msgs.msg import Pose, PoseStamped, Twist
from std_msgs.msg import Bool, String

import carrier_kinematics


def main():
    rospy.init_node('gazebo_carrier3d')
    model = rospy.get_param('~model_name', 'app_carrier')
    start = (float(rospy.get_param('~start_x', 0.)),
             float(rospy.get_param('~start_y', 0.)),
             float(rospy.get_param('~start_z', 0.)))
    center_z = float(rospy.get_param('~center_z', .1))
    snap_m = float(rospy.get_param('~snap_m', .03))
    rate_hz = float(rospy.get_param('~rate_hz', 50.))
    cmd_topic = rospy.get_param('~cmd_topic', '/cmd_vel')
    map_topic = rospy.get_param('~map_topic', '/v8_support_map')
    pose_topic = rospy.get_param('~pose_topic', '/v8/local/pose')
    loc_topic = rospy.get_param('~localization_topic', '/v8/local/localization_ok')
    status_topic = rospy.get_param('~status_topic', '/v8/3d/carrier/status')
    spawn = bool(rospy.get_param('~spawn', True))
    reference = rospy.get_param('~reference_frame', 'world')

    rospy.wait_for_service('/gazebo/set_model_state', timeout=20.)
    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    delete = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
    if spawn:
        rospy.wait_for_service('/gazebo/spawn_sdf_model', timeout=20.)
        spawner = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        sdf = ('<sdf version="1.6"><model name="%s"><static>true</static>'
               '<link name="link"><visual name="visual"><geometry>'
               '<box><size>0.45 0.30 0.20</size></box></geometry>'
               '<material><ambient>0.2 0.5 0.9 1</ambient></material>'
               '</visual></link></model></sdf>' % model)
        result = spawner(model, sdf, '', Pose(), reference)
        if not result.success:
            raise SystemExit('spawn failed: ' + result.status_message)

    shared = {'velocity': (0., 0., 0.), 'at': -1e9, 'map': {'surfaces': []}}
    lock = __import__('threading').Lock()

    def on_cmd(message):
        with lock:
            shared['velocity'] = (message.linear.x, message.linear.y, message.linear.z)
            shared['at'] = time.monotonic()

    def on_map(message):
        try:
            value = json.loads(message.data)
        except ValueError:
            return
        if isinstance(value, dict) and isinstance(value.get('map'), dict):
            value = value['map']
        with lock:
            shared['map'] = value

    rospy.Subscriber(cmd_topic, Twist, on_cmd, queue_size=50)
    rospy.Subscriber(map_topic, String, on_map, queue_size=1)
    pose_pub = rospy.Publisher(pose_topic, PoseStamped, queue_size=50)
    loc_pub = rospy.Publisher(loc_topic, Bool, queue_size=1)
    status_pub = rospy.Publisher(status_topic, String, queue_size=20)

    position = start
    last = time.monotonic()
    tick = 0
    print('READY', flush=True)
    rate = rospy.Rate(rate_hz)
    try:
        while not rospy.is_shutdown():
            now = time.monotonic()
            with lock:
                velocity = (shared['velocity'] if now - shared['at'] <= .5
                            else (0., 0., 0.))
                map_value = dict(shared['map'])
                map_value['surfaces'] = list(map_value.get('surfaces', []))
            position = carrier_kinematics.piecewise_advance(
                position, velocity, max(0., now - last), map_value['surfaces'],
                resolution=map_value.get('resolution'), snap_m=snap_m)
            position = carrier_kinematics.snap_position(position, velocity,
                                                        map_value['surfaces'], snap_m)
            last = now
            stamp = rospy.Time.now()
            pose = PoseStamped()
            pose.header.frame_id, pose.header.stamp = 'map', stamp
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = position
            pose.pose.orientation.w = 1.
            pose_pub.publish(pose)
            loc_pub.publish(Bool(data=True))
            state = ModelState()
            state.model_name, state.reference_frame = model, reference
            state.pose.position.x, state.pose.position.y = position[0], position[1]
            state.pose.position.z = position[2] + center_z
            state.pose.orientation.w = 1.
            try:
                set_state(state)
            except rospy.ServiceException:
                pass
            if tick % 10 == 0:
                status_pub.publish(String(data=json.dumps({
                    'ok': True, 'position': list(position), 'velocity': list(velocity),
                    'surfaces': len(surfaces), 'model': model}, ensure_ascii=False)))
            tick += 1
            rate.sleep()
    finally:
        if spawn:
            try:
                delete(model)
            except rospy.ServiceException:
                pass


if __name__ == '__main__':
    main()
