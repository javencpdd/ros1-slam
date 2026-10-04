#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# stub_tf_broadcaster.py —— mode_nav_stub 的 stub TF 节点（P7 变更批新增，A4/N-1 主理人修正裁决）
#
# 职责：广播 map→base_link 恒等位姿 @ ~10 Hz，header stamp 每拍取 rospy.Time.now()（fresh-stamp）。
# 依据：P6 修正批终态同款 python TransformBroadcaster 逻辑（修正批 tfbr.py，实测 GetPath
#       outcome=0 / "Plan found" / 末点精确命中）——
#       · stamp=0 的 static_transform_publisher 恒等位姿被 mbf 保鲜校验拒收
#         （"Most recent robot pose is ~1.79e9s old" = now - stamp0）；
#       · /tf_static 旧 latch 在 tf2 静态缓存压制动态新戳（修正批 r4，需重启 M4 才能清）。
#       动态新戳广播根治上述缺陷路径：GetPath 冒烟零重启、可重复。
# 红线：本节点仅发布 TF，不发布/订阅任何速度类话题（速度唯一写者 = M6 safe_cmd_vel_mux，
#       §2.6-a）；base_link→rslidar 恒等由驱动 base2livox 提供，此处不得重复广播；
#       二期真实定位接管 map→base_link（map→odom 链）时本节点退役（先停本节点再起定位，
#       避免 base_link 双父）。
# 话题：/tf（TransformBroadcaster 内建）；参数：无；帧：parent=map，child=base_link。

import rospy
import tf


def main():
    rospy.init_node('stub_tf_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        br.sendTransform((0.0, 0.0, 0.0),
                         (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         'base_link',
                         'map')
        rate.sleep()


if __name__ == '__main__':
    main()
