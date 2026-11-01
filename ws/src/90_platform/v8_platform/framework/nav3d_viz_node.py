#!/usr/bin/env python3
"""RViz MarkerArray/Path bridge for support_map, global/local paths and poses."""
import json

import rospy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

import support_map_viz


def _marker(kind, ns, marker_id, frame, stamp, specs):
    marker = Marker()
    marker.header.frame_id, marker.header.stamp = frame, stamp
    marker.ns, marker.id, marker.action = ns, marker_id, Marker.ADD
    if kind == 'surface':
        marker.type, marker.scale.x, marker.scale.y, marker.scale.z = (
            Marker.CUBE, specs['sx'], specs['sy'], specs['sz'])
        marker.pose.position.x, marker.pose.position.y = specs['x'], specs['y']
        marker.pose.position.z, marker.pose.orientation.w = specs['z'], 1.
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = specs['color']
        marker.lifetime = rospy.Duration(0.5)
    elif kind == 'obstacle':
        marker.type, marker.scale.x, marker.scale.y, marker.scale.z = (
            Marker.CUBE, specs['sx'], specs['sy'], specs['sz'])
        marker.pose.position.x, marker.pose.position.y = specs['x'], specs['y']
        marker.pose.position.z, marker.pose.orientation.w = specs['z'], 1.
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = specs['color']
    return marker


def _text(ns, marker_id, frame, stamp, text, x, y, z, color=(1, 1, 1, 1)):
    marker = Marker()
    marker.header.frame_id, marker.header.stamp = frame, stamp
    marker.ns, marker.id, marker.action, marker.type = ns, marker_id, Marker.ADD, Marker.TEXT_VIEW_FACING
    marker.pose.position = Point(x, y, z + .05)
    marker.pose.orientation.w = 1.
    marker.scale.z = .08
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
    marker.text = text
    marker.lifetime = rospy.Duration(0.5)
    return marker


def _line(ns, marker_id, frame, stamp, points, color=(0.1, 0.9, 0.2, 1.0), width=.02):
    marker = Marker()
    marker.header.frame_id, marker.header.stamp = frame, stamp
    marker.ns, marker.id, marker.action, marker.type = ns, marker_id, Marker.ADD, Marker.LINE_STRIP
    marker.scale.x, marker.pose.orientation.w = width, 1.
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
    marker.points = [Point(x, y, z) for x, y, z in points]
    marker.lifetime = rospy.Duration(0.5)
    return marker


def _sphere(ns, marker_id, frame, stamp, point, color, scale=.12):
    marker = Marker()
    marker.header.frame_id, marker.header.stamp = frame, stamp
    marker.ns, marker.id, marker.action, marker.type = ns, marker_id, Marker.ADD, Marker.SPHERE
    marker.pose.position = Point(*point)
    marker.pose.orientation.w = 1.
    marker.scale.x = marker.scale.y = marker.scale.z = scale
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
    marker.lifetime = rospy.Duration(0.5)
    return marker


def main():
    rospy.init_node('nav3d_viz')
    frame = rospy.get_param('~frame', 'map')
    map_topic = rospy.get_param('~map_topic', '/v8_support_map')
    trajectory_topic = rospy.get_param('~trajectory_topic', '/v8/local/trajectory')
    path_topic = rospy.get_param('~path_topic', '/v8/3d/global/path')
    pose_topic = rospy.get_param('~pose_topic', '/v8/local/pose')
    goal_topic = rospy.get_param('~goal_topic', '/v8/3d/global/goal')
    start_topic = rospy.get_param('~start_topic', '/v8/3d/global/start')
    marker_topic = rospy.get_param('~marker_topic', '/v8/3d/viz/markers')
    local_path_topic = rospy.get_param('~local_path_topic', '/v8/3d/viz/local_path')
    publisher = rospy.Publisher(marker_topic, MarkerArray, queue_size=2)
    local_path_pub = rospy.Publisher(local_path_topic, Path, queue_size=2, latch=True)
    state = {'map': None, 'trajectory': None, 'path': None, 'pose': None,
             'goal': None, 'start': None}
    lock = __import__('threading').Lock()

    def publish():
        stamp = rospy.Time.now()
        with lock:
            snapshot = dict(state)
        markers = MarkerArray()
        index = 0
        if snapshot['map'] is not None:
            for spec in support_map_viz.surface_specs(snapshot['map']):
                markers.markers.append(_marker('surface', 'surfaces', index, frame, stamp, spec))
                markers.markers.append(_text('surface_labels', index, frame, stamp,
                                             spec['label'], spec['x'], spec['y'], spec['z'],
                                             spec['color'][:3] + (1.,)))
                index += 1
            for spec in support_map_viz.obstacle_specs(snapshot['map']):
                markers.markers.append(_marker('obstacle', 'obstacles', index, frame, stamp, spec))
                index += 1
        if snapshot['path'] is not None:
            points = [(p.pose.position.x, p.pose.position.y, p.pose.position.z)
                      for p in snapshot['path'].poses]
            if points:
                markers.markers.append(_line('global_path', 0, frame, stamp, points,
                                             (0.1, 0.8, 0.95, 1.0)))
        if snapshot['trajectory'] is not None:
            points = support_map_viz.trajectory_points(snapshot['trajectory'])
            if points:
                markers.markers.append(_line('local_trajectory', 0, frame, stamp, points,
                                             (0.95, 0.55, 0.1, 1.0)))
                path = Path()
                path.header.frame_id, path.header.stamp = frame, stamp
                for x, y, z in points:
                    pose = PoseStamped()
                    pose.header = path.header
                    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = x, y, z
                    pose.pose.orientation.w = 1.
                    path.poses.append(pose)
                local_path_pub.publish(path)
        if snapshot['pose'] is not None:
            p = snapshot['pose'].pose.position
            markers.markers.append(_sphere('pose', 0, frame, stamp, (p.x, p.y, p.z + .06),
                                           (0.1, 1.0, 0.2, 1.0)))
        for key, color in (('start', (0.2, 1.0, 0.4, 1.0)), ('goal', (1.0, 0.3, 0.2, 1.0))):
            if snapshot[key] is not None:
                p = snapshot[key].pose.position
                markers.markers.append(_sphere(key, 0, frame, stamp, (p.x, p.y, p.z + .06), color))
        publisher.publish(markers)

    def on_map(message):
        try:
            with lock:
                state['map'] = json.loads(message.data)
        except ValueError:
            return
        publish()

    def on_trajectory(message):
        try:
            with lock:
                state['trajectory'] = json.loads(message.data)
        except ValueError:
            return
        publish()

    def on_path(message):
        with lock:
            state['path'] = message
        publish()

    def on_pose(key):
        def callback(message):
            with lock:
                state[key] = message
            publish()
        return callback

    rospy.Subscriber(map_topic, String, on_map, queue_size=1)
    rospy.Subscriber(trajectory_topic, String, on_trajectory, queue_size=2)
    rospy.Subscriber(path_topic, Path, on_path, queue_size=2)
    rospy.Subscriber(pose_topic, PoseStamped, on_pose('pose'), queue_size=20)
    rospy.Subscriber(goal_topic, PoseStamped, on_pose('goal'), queue_size=1)
    rospy.Subscriber(start_topic, PoseStamped, on_pose('start'), queue_size=1)
    rospy.loginfo('nav3d_viz ready: %s -> %s', map_topic, marker_topic)
    rospy.spin()


if __name__ == '__main__':
    main()
