#!/usr/bin/env python3
"""LIO 数据闸门（装配层组件）：把定位数据从 in_topic 转到 out_topic，可被服务启停。

为什么把它做成**图里的组件**而不是验收脚本里的 `rosrun relay`：
真实导航需要一条稳定的定位输入链；只有"故障注入验收"才需要能把这根线掐断/接回。
把这根线交给验收脚本临时 `rosrun`，等于**在装配之外维护导航输入**——那个临时进程一死，
导航就静默失去定位（实测 `/Odometry_gated` 0 帧 ⇒ `/v8/local/pose` 0 帧 ⇒ 整轮 INVALID）。
做成图内组件后：装配负责启动、退出由 roslaunch 回收，验收只调 `set_enabled` 注入/恢复。

契约：
  服务 `~set_enabled` (std_srvs/SetBool)：false = 停止转发并计数丢弃；true = 恢复
  状态 `~status` (String JSON)：{"enabled":..,"forwarded":..,"dropped":..}
"""
import json

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import SetBool, SetBoolResponse


def main():
    rospy.init_node('lio_gate')
    in_topic = rospy.get_param('~in_topic', '/Odometry')
    out_topic = rospy.get_param('~out_topic', '/Odometry_gated')
    state = {'enabled': bool(rospy.get_param('~enabled', True)),
             'forwarded': 0, 'dropped': 0}
    pub = rospy.Publisher(out_topic, Odometry, queue_size=5)
    status_pub = rospy.Publisher('~status', String, queue_size=5, latch=True)

    def publish_status():
        status_pub.publish(String(data=json.dumps(state)))

    def on_data(message):
        if not state['enabled']:
            state['dropped'] += 1
            return
        state['forwarded'] += 1
        pub.publish(message)

    def on_set(request):
        state['enabled'] = bool(request.data)
        publish_status()
        rospy.logwarn('[lio_gate] enabled=%s (forwarded=%d dropped=%d)',
                      state['enabled'], state['forwarded'], state['dropped'])
        return SetBoolResponse(success=True, message='enabled=%s' % state['enabled'])

    rospy.Service('~set_enabled', SetBool, on_set)
    rospy.Subscriber(in_topic, Odometry, on_data, queue_size=5)
    publish_status()
    rospy.loginfo('[lio_gate] %s -> %s enabled=%s', in_topic, out_topic, state['enabled'])
    rospy.spin()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
