#!/usr/bin/env python3
"""GT 评测域中继：raw GT -> eval GT topic/TF，可分别关断。

背景（A3）：
  Gazebo 插件在隔离模式下把真值发到 raw 域（如 /sim/odom_gt_raw +
  gt_raw_odom->gt_raw_base），本节点把它转成评测域（/sim/odom_gt +
  gt_odom->gt_base）。这样 `set_tf(false)` / `set_topic(false)` 可以独立
  关掉评测 GT 的 TF 或话题，而生产链（map->camera_init->body->base_link）
  仍由 LIO/闸门提供。

只用于 GT 隔离负控，不进入生产图；默认两个开关都开。
"""
import copy
import json
import rospy
import tf
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import SetBool, SetBoolResponse


def retarget(msg, frame, child):
    out = copy.deepcopy(msg)
    out.header.frame_id = frame
    out.child_frame_id = child
    return out


class GtEvalRelay(object):
    def __init__(self):
        self.in_topic = rospy.get_param('~in_topic', '/sim/odom_gt_raw')
        self.out_topic = rospy.get_param('~out_topic', '/sim/odom_gt')
        self.frame = rospy.get_param('~tf_parent', 'gt_odom')
        self.child = rospy.get_param('~tf_child', 'gt_base')
        self.state = {
            'topic': bool(rospy.get_param('~enable_topic', True)),
            'tf': bool(rospy.get_param('~enable_tf', True)),
            'received': 0, 'topic_sent': 0, 'tf_sent': 0,
        }
        self.pub = rospy.Publisher(self.out_topic, Odometry, queue_size=50)
        self.br = tf.TransformBroadcaster()
        self.status_pub = rospy.Publisher('~status', String, queue_size=10, latch=True)
        rospy.Subscriber(self.in_topic, Odometry, self.on_odom, queue_size=50)
        rospy.Service('~set_topic', SetBool, self.on_set_topic)
        rospy.Service('~set_tf', SetBool, self.on_set_tf)
        rospy.Timer(rospy.Duration(1.0), lambda _: self.publish_status())
        self.publish_status()
        rospy.loginfo('[gt_relay] %s -> %s | %s->%s topic=%s tf=%s',
                      self.in_topic, self.out_topic, self.frame, self.child,
                      self.state['topic'], self.state['tf'])

    def publish_status(self):
        self.status_pub.publish(String(data=json.dumps(self.state, sort_keys=True)))

    def on_set_topic(self, req):
        self.state['topic'] = bool(req.data)
        self.publish_status()
        return SetBoolResponse(success=True, message='topic=%s' % self.state['topic'])

    def on_set_tf(self, req):
        self.state['tf'] = bool(req.data)
        self.publish_status()
        return SetBoolResponse(success=True, message='tf=%s' % self.state['tf'])

    def on_odom(self, msg):
        self.state['received'] += 1
        if self.state['topic']:
            self.pub.publish(retarget(msg, self.frame, self.child))
            self.state['topic_sent'] += 1
        if self.state['tf']:
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            self.br.sendTransform((p.x, p.y, p.z), (q.x, q.y, q.z, q.w),
                                  msg.header.stamp, self.child, self.frame)
            self.state['tf_sent'] += 1


def main():
    rospy.init_node('gt_eval_relay')
    GtEvalRelay()
    rospy.spin()


if __name__ == '__main__':
    main()
