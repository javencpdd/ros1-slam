#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tf_frame_assert.py —— TF 帧「父边唯一」断言（R572／批224 新建）

为什么有这件：R572 把仿真台的帧口径钉死为 map，由 launch 里一条 map→odom 恒等桥供给
（先例 m2_teb.launch:31-32）。而**双发布者是必须明文防的坑**：一旦台上另外引入
hdl_localization / faster_lio 这类自己发 map 系的节点，就会与恒等桥抢发同一条父边
（map→odom 与 map→base_link 同在 map 系下），下游 tf buffer 按到达顺序取用 ⇒
读数随机跑在两个口径之间，正是「名=口径」要防的漂移。
⇒ 本节点把「台上 map 系到底有几个发布者」从「靠人记得核验」变成「链路自己拦」。

★判据取法：第一版写错过两次，此处把两种错法记下来，防止下一个人再走★
  ✗ 按「变换数值」判：恒等桥与真定位节点在同一时刻可以发出数值完全相同的 map→odom，
    光看数值分不出来。
  ✗ 按「/tf 的 topic 注册表」判：/tf 是全台共用的一个话题，注册表列的是**所有** tf 广播者
    （robot_state_publisher、lite3_planar_drive、桥……），全算进来会得到一枚**恒假的红**
    ——这与 world_load_assert 里「sun 不进 model_names」同类：拿了一个不承载该语义的代理当判据。
  ✗ 按 tf2 Buffer 的 _Buffer__get_parent 判：那是 Boost.Python 私有实现，C 扩展版里不存在。

  ✓ 本件取法（可判、可测、不产生假红）＝**直接读 /tf 与 /tf_static 的消息本体**：
    对每条 transform，若 child_frame_id == protected（缺省 odom），就记下它的 parent_frame_id。
    窗口内出现 >1 个不同 parent ⇒ 双发布者（两个来源在交替改写同一条边），rc=1；
    出现 0 个 parent 且 expect_parent 查不到 ⇒ 桥没起，rc=1；
    恰 1 个 parent 但 ≠ expect_parent ⇒ 父帧不符，rc=1。
    ★为什么这样是完备的★：两个发布者若发同一条边（同为 map→odom），本判据看不出来——但那种
    情况下**读数口径是一致的**，不构成口径漂移，本件不需要拦；本件要拦的是「口径被换掉」
    与「压根没起」，这两者都会被上面的取值集合直接命中。

出口：除 rc 外，结论写成全局参数
      /tf_frame_assert/ok (bool)、/tf_frame_assert/{protected,expect_parent,parents_seen,n_msgs}，
      供采集脚本直接断言——理由同 world_load_assert：roslaunch 的 required="true" 是
      「节点一死就整链关」，与本节点「PASS 即退出 0」的正常收尾冲突，不能靠 required 当闸。

边界：只读。不写文件、不发变换、不改任何参数（除上述四枚结论参数）、不参与判据计算。
"""
import sys

import rospy
import tf2_ros
from tf2_msgs.msg import TFMessage


def main():
    rospy.init_node("tf_frame_assert", anonymous=False)
    rospy.set_param("/tf_frame_assert/ok", False)

    protected = rospy.get_param("~protected_child", "odom")
    expect_parent = rospy.get_param("~expect_parent", "map")
    timeout = float(rospy.get_param("~timeout", 30.0))
    observe_sec = float(rospy.get_param("~observe_sec", 3.0))

    rospy.set_param("/tf_frame_assert/protected", protected)
    rospy.set_param("/tf_frame_assert/expect_parent", expect_parent)
    rospy.loginfo("[tf_frame_assert] 保护子帧=%s ‖ 期望父帧=%s ‖ 观察窗=%.1fs",
                  protected, expect_parent, observe_sec)

    parents_seen = []          # 去重保序
    n_msgs = [0]

    def _on_tf(msg):
        n_msgs[0] += 1
        for tr in msg.transforms:
            if tr.child_frame_id.lstrip("/") != protected:
                continue
            p = tr.header.frame_id.lstrip("/")
            if p not in parents_seen:
                parents_seen.append(p)

    sub_tf = rospy.Subscriber("/tf", TFMessage, _on_tf, queue_size=200)
    sub_static = rospy.Subscriber("/tf_static", TFMessage, _on_tf, queue_size=200)

    # ---- 观察窗（同时进行 tf2 可达性轮询，避免纯 sleep 浪费时间）----
    buf = tf2_ros.Buffer(cache_time=rospy.Duration(15.0))
    tf2_ros.TransformListener(buf)
    deadline = rospy.Time.now() + rospy.Duration(max(observe_sec, 1.0))
    while rospy.Time.now() < deadline and not rospy.is_shutdown():
        rospy.sleep(0.1)

    reachable = False
    try:
        buf.lookup_transform(expect_parent, expect_parent, rospy.Time(0), rospy.Duration(timeout))
        reachable = True
    except Exception:
        reachable = False

    sub_tf.unregister()
    sub_static.unregister()

    rospy.set_param("/tf_frame_assert/parents_seen", parents_seen)
    rospy.set_param("/tf_frame_assert/n_msgs", n_msgs[0])
    rospy.loginfo("[tf_frame_assert] 窗口内 %d 条 TFMessage；%s 的父帧取值集合=%s",
                  n_msgs[0], protected, parents_seen)

    # ---- 判据 ----
    if len(parents_seen) > 1:
        rospy.logfatal(
            "[tf_frame_assert] ★双发布者／口径跳变：%s 的父帧在窗口内出现 %s★ —— "
            "两个来源在交替改写同一条边。按 R572 口径『凡台上引入发 map 系的节点，桥不起』择一保留。",
            protected, parents_seen)
        sys.exit(1)

    if not reachable:
        rospy.logfatal(
            "[tf_frame_assert] ★%ss 内 tf 查不到 %s 帧★ —— 台上没有 %s 帧在发（父边集合=%s）。"
            "本 launch 应带 %s→%s 恒等桥；若被有意移除（真定位接入时），须同时撤掉本断言节点。",
            timeout, expect_parent, expect_parent, parents_seen,
            expect_parent, protected)
        sys.exit(1)

    if not parents_seen:
        rospy.logfatal(
            "[tf_frame_assert] ★窗口内一条 %s 的父边都没采到★（%s 虽在 tf buffer 里可达，"
            "但那多半是缓存里残留的旧变换）—— 台上没有节点在发 %s→%s。"
            "本 launch 应带 %s→%s 恒等桥；若被有意移除（真定位接入时），须同时撤掉本断言节点。",
            protected, expect_parent, expect_parent, protected, expect_parent, protected)
        sys.exit(1)

    if parents_seen[0] != expect_parent:
        rospy.logfatal("[tf_frame_assert] ★父帧不符：实测 %s→%s，期望 %s→%s★",
                       parents_seen[0], protected, expect_parent, protected)
        sys.exit(1)

    rospy.set_param("/tf_frame_assert/ok", True)
    rospy.loginfo("[tf_frame_assert] PASS —— %s→%s 父帧唯一且符合、%s 可达",
                  expect_parent, protected, expect_parent)


if __name__ == "__main__":
    main()
