#!/usr/bin/env python3
"""BT 编排阶段 → 3D 文本标记（Foxglove / RViz 只读可视化）。

订阅 /v8/bt/status（JSON）与 /v8/local/pose，在机器人上方发布一条
TEXT_VIEW_FACING 标记显示当前编排阶段（WAIT_GOAL / REQUEST_PATH / ...），
并附一行 seq/event/outcome 摘要。只读：不发目标、不发速度。
"""
import json

try:
    import rospy
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import String
    from visualization_msgs.msg import Marker, MarkerArray
except ImportError:  # 宿主机跑 stage_of 单测时没有 ROS
    rospy = None

STAGE_NAMES = (
    'wait_goal', 'request_path', 'wait_path', 'follow_path',
    'wait_arrival', 'dwell_at_goal', 'stop_after_success',
    'recovery_on_failure',
)


def stage_of(text):
    """status JSON -> (stage, detail)；不可解析时返回 ('', '')。"""
    try:
        data = json.loads(text)
    except (TypeError, ValueError):
        return '', ''
    if not isinstance(data, dict):
        return '', ''
    stage = 'idle'
    for node in data.get('nodes') or []:
        if not isinstance(node, dict):
            continue
        if node.get('name') in STAGE_NAMES and node.get('status') == 'RUNNING':
            stage = node['name']
            break
    detail = ['seq %s/%s' % (data.get('current_seq', 0), data.get('goal_seq', 0))]
    if data.get('queue_depth'):
        detail.append('queue %s' % data['queue_depth'])
    if data.get('path_state'):
        detail.append(str(data['path_state']))
    if data.get('outcome'):
        detail.append(str(data['outcome']))
    elif data.get('event'):
        detail.append(str(data['event']))
    return stage.upper(), ' '.join(detail)


class StageMarker(object):
    def __init__(self):
        self.pose = None
        self.pub = rospy.Publisher(
            rospy.get_param('~marker_topic', '/v8/bt/stage_markers'),
            MarkerArray, queue_size=5)
        self.height = float(rospy.get_param('~height', 1.6))
        self.frame_fallback = rospy.get_param('~frame', 'camera_init')
        rospy.Subscriber(rospy.get_param('~status_topic', '/v8/bt/status'),
                         String, self.on_status, queue_size=5)
        rospy.Subscriber(rospy.get_param('~pose_topic', '/v8/local/pose'),
                         PoseStamped, self.on_pose, queue_size=20)

    def on_pose(self, msg):
        self.pose = msg

    def on_status(self, msg):
        if self.pose is None:
            rospy.logwarn_throttle(10.0, '[bt_stage_marker] wait /v8/local/pose')
            return
        stage, detail = stage_of(msg.data)
        if not stage:
            return
        array = MarkerArray()
        array.markers.append(self.text_marker(0, stage, 0.55, self.height))
        array.markers.append(self.text_marker(1, detail, 0.30, self.height - 0.42))
        self.pub.publish(array)

    def text_marker(self, marker_id, text, scale, dz):
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = self.pose.header.frame_id or self.frame_fallback
        marker.ns = 'bt_stage'
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = self.pose.pose.position.x
        marker.pose.position.y = self.pose.pose.position.y
        marker.pose.position.z = self.pose.pose.position.z + dz
        marker.pose.orientation.w = 1.0
        marker.scale.z = scale
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = (
            0.30, 0.95, 0.65, 1.0) if marker_id == 0 else (0.75, 0.85, 1.0, 0.9)
        marker.lifetime = rospy.Duration(1.5)
        marker.text = text
        return marker


def main():
    if rospy is None:
        raise SystemExit('rospy unavailable: run inside the ROS workspace')
    rospy.init_node('bt_stage_marker')
    StageMarker()
    rospy.spin()


if __name__ == '__main__':
    main()
