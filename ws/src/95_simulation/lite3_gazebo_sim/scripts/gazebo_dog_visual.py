#!/usr/bin/env python3
"""Visual-only Lite3 URDF follower for Gazebo scenes.

The official planar carrier remains the physics body and keeps all criteria.
This node spawns a static, collision-free copy of Lite3.urdf and follows
/sim/odom_gt so screenshots show the dog instead of only the white box.
"""
import argparse
import subprocess
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import DeleteModel, SpawnModel, SetModelState
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry

# STL has no texture; tint the visual copy so the dog reads against white stairs.
LINK_COLORS = {
    'TORSO': ('0.10 0.12 0.15 1', '0.16 0.19 0.23 1'),
    'HIP': ('0.85 0.35 0.05 1', '0.95 0.45 0.08 1'),
    'THIGH': ('0.18 0.21 0.25 1', '0.28 0.32 0.37 1'),
    'SHANK': ('0.55 0.58 0.62 1', '0.72 0.75 0.78 1'),
}


def link_color(name):
    for key, value in LINK_COLORS.items():
        if key in name:
            return value
    return ('0.20 0.23 0.27 1', '0.30 0.34 0.39 1')


def visual_sdf(urdf, name, mesh_root):
    raw = subprocess.check_output(['gz', 'sdf', '-p', str(urdf)],
                                  text=True, stderr=subprocess.DEVNULL)
    root = ET.fromstring(raw)
    model = root.find('model')
    if model is None:
        raise RuntimeError('converted SDF has no model')
    model.set('name', name)
    if model.find('static') is None:
        ET.SubElement(model, 'static').text = 'true'
    else:
        model.find('static').text = 'true'
    for parent in list(root.iter()):
        for collision in list(parent.findall('collision')):
            parent.remove(collision)
    prefixes = ('package://lite3_description/meshes/',
                'model://lite3_description/meshes/')
    for uri in root.iter('uri'):
        text = uri.text or ''
        for prefix in prefixes:
            if text.startswith(prefix):
                uri.text = 'file://' + str(Path(mesh_root) / text[len(prefix):])
                break
    for link in root.iter('link'):
        ambient, diffuse = link_color(link.get('name', ''))
        for visual in link.findall('visual'):
            for material in visual.findall('material'):
                visual.remove(material)
            material = ET.SubElement(visual, 'material')
            ET.SubElement(material, 'ambient').text = ambient
            ET.SubElement(material, 'diffuse').text = diffuse
    return ET.tostring(root, encoding='unicode')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--urdf', required=True)
    parser.add_argument('--mesh-root', default='')
    parser.add_argument('--model-name', default='lite3_visual')
    parser.add_argument('--odom-topic', default='/sim/odom_gt')
    parser.add_argument('--z-offset', type=float, default=.30)
    parser.add_argument('--rate', type=float, default=20.)
    args, _ = parser.parse_known_args()
    mesh_root = args.mesh_root or str(Path(args.urdf).resolve().parent.parent / 'meshes')

    rospy.init_node('gazebo_dog_visual')
    sdf = visual_sdf(args.urdf, args.model_name, mesh_root)
    rospy.wait_for_service('/gazebo/spawn_sdf_model', timeout=60.)
    odom = rospy.wait_for_message(args.odom_topic, Odometry, timeout=60.)
    spawn = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    delete = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
    pose = Pose()
    pose.position.x, pose.position.y = odom.pose.pose.position.x, odom.pose.pose.position.y
    pose.position.z = odom.pose.pose.position.z + args.z_offset
    pose.orientation = odom.pose.pose.orientation
    response = spawn(args.model_name, sdf, '', pose, 'world')
    if not response.success:
        raise RuntimeError(response.status_message)
    rospy.loginfo('spawned visual %s; collision-free, static', args.model_name)

    period = 1. / max(args.rate, 1.)
    last = [0.]

    def follow(message):
        now = time.monotonic()
        if now - last[0] < period:
            return
        last[0] = now
        target = ModelState()
        target.model_name = args.model_name
        target.reference_frame = 'world'
        target.pose = message.pose.pose
        target.pose.position.z = message.pose.pose.position.z + args.z_offset
        try:
            set_state(target)
        except rospy.ServiceException as error:
            rospy.logwarn_throttle(5., 'set_model_state failed: %s', error)

    rospy.Subscriber(args.odom_topic, Odometry, follow, queue_size=1)
    rospy.on_shutdown(lambda: delete(args.model_name))
    rospy.spin()


if __name__ == '__main__':
    main()
