#!/usr/bin/env python3
"""HDL `/status` → 通用健康信号的适配器。

为什么要它：`v8_platform` 的 carrier 只认通用契约（`std_msgs/Bool` 收敛 + `std_msgs/Float64`
质量），不该为了一个指标去依赖 `hdl_localization` 的 msg 包——那会把平台层钉死在某个
定位后端上。本节点属于定位包（本来就依赖 hdl），负责把后端特有的消息折成通用信号。

★解码失败按"不发布"处理，而不是发布一个"健康"★：下游 carrier 对"没有新鲜信号"本来
就是 fail-closed，所以不发布＝不健康，方向正确。
"""
import os
import sys

import rospy
from std_msgs.msg import Bool, Float64

from hdl_localization.msg import ScanMatchingStatus

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import loc_status_codec as codec  # noqa: E402


def main():
    rospy.init_node('v8_hdl_status_adapter')
    status_topic = rospy.get_param('~status_topic', '/hdl_localization/status')
    converged_topic = rospy.get_param('~converged_topic', '/v8/local/loc_converged')
    quality_topic = rospy.get_param('~quality_topic', '/v8/local/loc_quality')
    converged_pub = rospy.Publisher(converged_topic, Bool, queue_size=10)
    quality_pub = rospy.Publisher(quality_topic, Float64, queue_size=10)
    stats = {'ok': 0, 'bad': 0}

    def on_status(message):
        try:
            out = codec.decode(message)
        except codec.StatusDecodeError as exc:
            stats['bad'] += 1
            rospy.logwarn_throttle(30.0, '[loc_status] 解码失败，不发布健康信号：%s', exc)
            return
        stats['ok'] += 1
        converged_pub.publish(Bool(data=out['converged']))
        quality_pub.publish(Float64(data=out['quality']))

    rospy.Subscriber(status_topic, ScanMatchingStatus, on_status, queue_size=10)
    rospy.loginfo('[loc_status] %s → (%s, %s)', status_topic, converged_topic, quality_topic)
    rospy.spin()


if __name__ == '__main__':
    main()
