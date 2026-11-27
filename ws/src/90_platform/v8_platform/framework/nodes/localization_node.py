#!/usr/bin/env python3
"""Localization source adapter and GT/LIO error monitor.

Sources:
  sim_gt    - /sim/odom_gt (Gazebo WorldPose)
  faster_lio- /Odometry (faster-lio)
  ndt       - /ndt_pose (monitor-only by default; not a navigation source)
The node publishes the selected pose on the local planner contract and a
rolling GT-vs-selected error for later comparison. NDT is deliberately marked
not navigation-ready until a rejection/lease gate is wired.
"""
import os as _os
import sys as _sys
_sys.path.insert(0, _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__))))
import json
import math
import time

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String


def yaw_of(q):
    # q 有两种来源：rospy 的四元数消息（有 .x/.y/.z/.w 属性），
    # 以及 pose_from_odom 产出的 (x, y, z, qx, qy, qz, qw) 元组 —— error_between 传的是
    # estimate[3:]，即 (qx, qy, qz, qw) 四元组。原实现只认属性形式，
    # 于是 /sim/odom_gt 一到，on_gt -> select -> error_between 就抛
    # AttributeError: 'tuple' object has no attribute 'w'，loc 节点直接被打死。
    if hasattr(q, 'w'):
        x, y, z, w = q.x, q.y, q.z, q.w
    else:
        x, y, z, w = q[0], q[1], q[2], q[3]
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def pose_from_odom(message):
    p, q = message.pose.pose.position, message.pose.pose.orientation
    return (p.x, p.y, p.z, q.x, q.y, q.z, q.w)


def error_between(gt, estimate):
    dx, dy, dz = (estimate[i] - gt[i] for i in range(3))
    dyaw = math.atan2(math.sin(yaw_of(estimate[3:]) - yaw_of(gt[3:])),
                      math.cos(yaw_of(estimate[3:]) - yaw_of(gt[3:])))
    return {'xy_m': math.hypot(dx, dy), 'z_m': abs(dz),
            'yaw_rad': abs(dyaw), 'distance_m': math.sqrt(dx * dx + dy * dy + dz * dz)}


def rotate_to_map(pose, origin_x, origin_y, origin_z, origin_yaw):
    """T1：LIO 系 → map 系，**位置与姿态一起转**（纯函数，便于零节点测试）。

    p_map = R(yaw)·p_lio + (x, y, z)
    q_map = q_z(yaw) ⊗ q_lio   （左乘绕 Z 的四元数）

    修复背景（2026-11-27）：原实现只转位置、四元数原样返回，origin_yaw≠0 时
    位置在 map、朝向还在 LIO，下游按朝向取支撑面会整体偏。
    """
    x, y, z, qx, qy, qz, qw = pose
    c, s_ = math.cos(origin_yaw), math.sin(origin_yaw)
    hc, hs = math.cos(origin_yaw / 2.0), math.sin(origin_yaw / 2.0)
    # Hamilton 积 a⊗b（a = q_z = (0,0,s,c)）：
    #   x = aw*bx + ax*bw + ay*bz - az*by = c*qx - s*qy
    #   y = aw*by - ax*bz + ay*bw + az*bx = c*qy + s*qx
    # 2026-11-27 真人指出上一版把这两个符号写反了——只测纯 yaw 时 qx=qy=0，
    # 符号错了也照样过，所以必须补 roll/pitch/yaw 组合用例。
    nx = hc * qx - hs * qy
    ny = hc * qy + hs * qx
    nz = hc * qz + hs * qw
    nw = hc * qw - hs * qz
    return (c * x - s_ * y + origin_x,
            s_ * x + c * y + origin_y,
            z + origin_z, nx, ny, nz, nw)


def freshness(selected, accepted_stamp, now, max_age, wall_now=None, wall_at_accept=None):
    """新鲜度判定：**回调与健康循环必须共用这一处**。

    2026-11-27：原实现两处各写一遍，回调那份因为"先赋值再比较"恒真，
    循环那份却引用了已被改名的 `state['selected_at']`，首个样本到达后直接 KeyError
    把节点打死（loc 一死，p4_nav_hdl 的 gate_fail 清掉整个进程组，于是 local/BT
    "没有状态"看起来像自己崩了，其实是连带清理）。

    返回 (fresh, age)：无样本时 fresh=False、age=None。
    """
    if selected is None or accepted_stamp is None:
        return False, None
    age = max(0.0, now - accepted_stamp)
    fresh = age <= max_age
    # ★墙钟旁路：ROS 时钟冻结时 age 不会涨，判 fresh 会一直放行★（2026-11-27 真人指出，
    # 「停钟仍判 fresh」不能当作安全闭合）。所以再要一条墙钟条件：
    # 源戳在**墙钟**上停滞超过 max_age，同样判不新鲜。回跳同理——墙钟只进不退。
    if fresh and wall_now is not None and wall_at_accept is not None:
        if (wall_now - wall_at_accept) > max_age:
            fresh = False
    return fresh, age


def stamp_verdict(stamp, now, prev_accepted, future_tol):
    """源戳体检：返回 'ok' | 'bad' | 'future' | 'back'。

    这是把"新鲜度恒真"那类缺陷挡在门外的唯一入口：
    · 非正/非有限 ⇒ 'bad'；· 超前 now+future_tol ⇒ 'future'；
    · 早于上一次被接受的戳 ⇒ 'back'。
    """
    if not isinstance(stamp, float) and not isinstance(stamp, int):
        return 'bad'
    if not math.isfinite(stamp) or stamp <= 0.0:
        return 'bad'
    if stamp > now + future_tol:
        return 'future'
    if prev_accepted is not None and stamp < prev_accepted - 1e-9:
        return 'back'
    return 'ok'


def main():
    rospy.init_node('v8_localization_source')
    source = rospy.get_param('~source', 'sim_gt')
    if source not in ('sim_gt', 'faster_lio', 'ndt'):
        raise SystemExit('unsupported localization source: %s' % source)
    frame = rospy.get_param('~frame', 'map')
    max_age = float(rospy.get_param('~max_age_s', 1.0))
    navigation_allowed = bool(rospy.get_param('~navigation_allowed', source != 'ndt'))
    gt_topic = rospy.get_param('~gt_topic', '/sim/odom_gt')
    lio_topic = rospy.get_param('~lio_topic', '/Odometry')
    ndt_topic = rospy.get_param('~ndt_topic', '/ndt_pose')
    pose_topic = rospy.get_param('~pose_topic', '/v8/local/pose')
    ok_topic = rospy.get_param('~localization_topic', '/v8/local/localization_ok')
    status_topic = rospy.get_param('~status_topic', '/v8/local/localization_status')
    error_topic = rospy.get_param('~error_topic', '/v8/local/localization_error')
    # ★LIO→map 的真实刚体变换（真人裁决 2026-11-26 第 2 条）★
    # 原实现只把 header.frame_id 写成 map，坐标仍是 LIO 自己原点的（等于撒谎）。
    # 这里按**已知初始位姿**建立常量刚体变换（平移 + 绕 Z 旋转），不使用 Gazebo 真值实时驱动。
    lio_frame = rospy.get_param('~lio_frame', 'odom')
    origin_x = float(rospy.get_param('~lio_origin_x', 0.0))
    origin_y = float(rospy.get_param('~lio_origin_y', 0.0))
    origin_yaw = float(rospy.get_param('~lio_origin_yaw', 0.0))
    need_transform = bool(rospy.get_param('~lio_to_map', source != 'sim_gt'))
    # ★三层转换必须分开表达、各配置一次★（2026-11-27 真人裁决）：
    #   T1 世界系  ：LIO 世界 → map，平移 (origin_x, origin_y, origin_z) + 绕 Z 的 origin_yaw；
    #   T2 传感器→机体：rslidar→base_link 外参，由 URDF 唯一给出（本节点不参与，也不许在这里再补）；
    #   T3 机体→规划参考点：机体原点 → 支撑参考点（足底/接触面语义），只在这里做一次刚体偏移。
    # 把三件事揉成一个"z 偏移"会让下游分不清"世界没对齐"和"参考点不同"，历史上就是这么丢的。
    origin_z = float(rospy.get_param('~lio_origin_z', 0.0))
    plan_ref_topic = rospy.get_param('~pose_plan_topic', '')
    plan_ref_z = float(rospy.get_param('~plan_ref_z', 0.0))
    future_tol = float(rospy.get_param('~stamp_future_tol_s', 0.5))
    pose_pub = rospy.Publisher(pose_topic, PoseStamped, queue_size=20)
    plan_pub = rospy.Publisher(plan_ref_topic, PoseStamped, queue_size=20) if plan_ref_topic else None
    ok_pub = rospy.Publisher(ok_topic, Bool, queue_size=1)
    status_pub = rospy.Publisher(status_topic, String, queue_size=10)
    error_pub = rospy.Publisher(error_topic, String, queue_size=20)
    state = {'gt': None, 'gt_at': -1e100, 'selected': None, 'accepted_stamp': None,
             'accepted_wall': None,
             'errors': [], 'count': 0, 'last_error': None}

    def publish_ok(ok):
        ok_pub.publish(Bool(data=bool(ok)))

    def to_map(pose):
        return rotate_to_map(pose, origin_x, origin_y, origin_z, origin_yaw)

    def select(pose, stamp, label):
        """发布位姿并维护时效。**stamp 是源戳**（输入消息的 header.stamp），不是本地时间。

        2026-11-27 真人指出的两处缺陷在这里一并收口：
        ① 原实现 `message.header.stamp = rospy.Time.now()` —— 输出戳是现造的，
           下游无法按源戳配对 GT/EST，也没法判断数据新旧；
        ② 原实现先 `state['selected_at'] = now` 再算 `now - selected_at <= max_age`，
           差值恒为 0 ⇒ fresh 恒真，localization_ok 永远不会因过期而落。

        现在：拒绝 非正/未来/回退 三类戳；新鲜度用"最近一次被接受样本的源戳"算。
        """
        now = rospy.get_time()
        verdict = stamp_verdict(stamp, now, state['accepted_stamp'], future_tol)
        if verdict != 'ok':
            state['rej_' + verdict] = state.get('rej_' + verdict, 0) + 1
            rospy.logwarn_throttle(10, '[loc] 丢弃%s戳 %.3f（now=%.3f 上次=%.3f 源=%s）',
                                   {'bad': '非法', 'future': '未来', 'back': '回退'}[verdict],
                                   stamp, now, state['accepted_stamp'] or -1.0, label)
            return
        if need_transform and label != 'sim_gt':
            pose = to_map(pose)
        state['selected'], state['accepted_stamp'] = pose, stamp
        state['accepted_wall'] = time.monotonic()
        edited_at = rospy.Time.now()
        message = PoseStamped()
        message.header.frame_id, message.header.stamp = frame, rospy.Time.from_sec(stamp)
        message.pose.position.x, message.pose.position.y, message.pose.position.z = pose[:3]
        message.pose.orientation.x, message.pose.orientation.y = pose[3], pose[4]
        message.pose.orientation.z, message.pose.orientation.w = pose[5], pose[6]
        pose_pub.publish(message)
        # T3：同一时刻、同一次刚体偏移，另发一条规划参考点（真实机体位姿仍走上面那条）
        if plan_pub is not None:
            # ★独立对象★：直接 `plan_msg.pose = message.pose` 会让两条消息共享同一个
            # Pose（Python 对象语义），改 z 就把已发布的机体位姿也改了。
            plan_msg = PoseStamped()
            plan_msg.header.frame_id, plan_msg.header.stamp = frame, rospy.Time.from_sec(stamp)
            plan_msg.pose.position.x = pose[0]
            plan_msg.pose.position.y = pose[1]
            plan_msg.pose.position.z = pose[2] + plan_ref_z
            plan_msg.pose.orientation.x = pose[3]
            plan_msg.pose.orientation.y = pose[4]
            plan_msg.pose.orientation.z = pose[5]
            plan_msg.pose.orientation.w = pose[6]
            plan_pub.publish(plan_msg)
        if state['gt'] is not None:
            error = error_between(state['gt'], pose)
            state['last_error'] = error
            state['errors'].append(error)
            state['count'] += 1
            error_pub.publish(String(data=json.dumps(dict(error, source=label),
                                                     ensure_ascii=False)))
        fresh, age = freshness(state['selected'], state['accepted_stamp'], now, max_age,
                               time.monotonic(), state['accepted_wall'])
        age = age or 0.0
        publish_ok(navigation_allowed and fresh)
        status_pub.publish(String(data=json.dumps({
            'ok': navigation_allowed and fresh, 'source': label,
            'navigation_allowed': navigation_allowed,
            'age_s': age, 'src_stamp': state['accepted_stamp'],
            'samples': state['count'], 'last_error': state['last_error'],
            'rejected': {k: state.get(k, 0) for k in ('rej_bad', 'rej_future', 'rej_back')}},
            ensure_ascii=False)))

    def on_gt(message):
        clock = message.header.stamp.to_sec()
        state['gt'], state['gt_at'] = pose_from_odom(message), clock
        if source == 'sim_gt':
            select(state['gt'], clock, 'sim_gt')

    def on_lio(message):
        if source == 'faster_lio':
            select(pose_from_odom(message), message.header.stamp.to_sec(), 'faster_lio')

    def on_ndt(message):
        if source != 'ndt':
            return
        p, q = message.pose.position, message.pose.orientation
        # 源戳无效就让 select 去拒，不要用 monotonic 顶替——那等于凭空造一个"未来戳"。
        select((p.x, p.y, p.z, q.x, q.y, q.z, q.w), message.header.stamp.to_sec(), 'ndt')

    rospy.Subscriber(gt_topic, Odometry, on_gt, queue_size=20)
    if source == 'faster_lio':
        rospy.Subscriber(lio_topic, Odometry, on_lio, queue_size=20)
    if source == 'ndt':
        rospy.Subscriber(ndt_topic, PoseStamped, on_ndt, queue_size=10)
    rospy.loginfo('localization source=%s navigation_allowed=%s', source, navigation_allowed)
    print('READY', flush=True)
    # 20 Hz 而非 5 Hz：本循环同时承担"定位失效宣告"的职责 —— 失效一旦成立，
    # 下游要等到下一次 publish_ok(False) 才会停。5 Hz 意味着最多多等 0.2 s，
    # 20 Hz 把这段压到 0.05 s。消息是 Bool + 短 JSON，负载可忽略。
    rate = rospy.Rate(20.0)
    while not rospy.is_shutdown():
        now = rospy.Time.now().to_sec()
        # ★与回调共用 freshness()★：两处各写一遍是这次 KeyError 的根源，
        # 且必须用**源戳**判定，而不是本地时钟——停流时源戳不动，age 才会涨。
        fresh, age = freshness(state['selected'], state['accepted_stamp'], now, max_age,
                               time.monotonic(), state['accepted_wall'])
        publish_ok(navigation_allowed and fresh)
        status_pub.publish(String(data=json.dumps({
            'ok': navigation_allowed and fresh, 'source': source,
            'navigation_allowed': navigation_allowed,
            'fresh': fresh, 'age_s': age, 'samples': state['count'],
            'rejected': {k: state.get(k, 0) for k in ('rej_bad', 'rej_future', 'rej_back')},
            'last_error': state['last_error']}, ensure_ascii=False)))
        rate.sleep()


if __name__ == '__main__':
    main()
