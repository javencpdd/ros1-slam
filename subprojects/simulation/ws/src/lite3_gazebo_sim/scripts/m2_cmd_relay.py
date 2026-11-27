#!/usr/bin/env python3
# m2_cmd_relay.py —— M2 自研话题转发器（小张，2026-10-06）
# 用途一：/cmd_vel_raw（mbf 输出，mbf_navigation.launch 已 remap）→ /cmd_vel（lite3_planar_drive 消费）
#         —— M6 safe_cmd_vel_mux 不在仿真闭环内，此 relay 仅 sim 域（ROS master 11313），不触生产 11312。
# 用途二：/livox/imu → /imu/data（hdl_localization 生产 remap 契约，M2-3 仿真域喂 IMU）。
# 用法：m2_cmd_relay.py <from_topic> <to_topic> <Twist|Imu>
# 红线：本节点不写任何真机话题；生产 master 下禁用（m2_*.launch 全部在 11313 隔离域内起）。
import math, sys, time, rospy

# ★IMU 坐标契约适配（2026-11-27）★
# 仿真里 IMU 挂在 rslidar 上，随雷达下倾 15°（urdf/lite3_planar.urdf.xacro 的 lidar_pitch_deg）。
# HDL 的 predict() 直接把 linear_acceleration／angular_velocity 的分量当作基座量积分
# （hdl_localization_nodelet.cpp:263-267，没有任何旋转），于是 15° 的倾斜被当成"车在斜着加速"，
# 收敛后很快发散——单变量实测：use_imu=false 全判据 PASS，use_imu=true 30s 窗最差 337 m。
# 这里在**接口适配层**把向量从 rslidar 系旋到 base_link 系（R_y(+pitch)），并把 frame_id 标成 base_link。
# 只做刚体向量旋转，不改滤波、不改门控；三个诊断状态共用同一份地图/外参/初值。
def _sleep_guarded(rate, warn=None):
    """转发器的心跳等待：**时钟回跳不得杀进程**。

    返回 True＝继续循环，False＝收到正常关机请求、应退出。
    异常顺序必须"子类在前"：ROSTimeMovedBackwardsException 是 ROSInterruptException 的子类
    （MRO 实测：[ROSTimeMovedBackwardsException, ROSInterruptException, ROSException, Exception]）。
    抽成独立函数是为了能做零节点负控（见 verify/test_m2_relay_clock_guard.py）。
    """
    warn = warn or (lambda m: None)
    try:
        rate.sleep()
        return True
    except rospy.ROSTimeMovedBackwardsException as e:
        warn('时钟回跳（继续转发）: %s' % e)
        return True
    except rospy.ROSInterruptException:
        return False
    except rospy.ROSException as e:
        warn('时钟异常（继续转发）: %s' % e)
        return True


def _rot_y(v, c, s):
    return (c * v[0] + s * v[2], v[1], -s * v[0] + c * v[2])


def _quat_to_matrix(q):
    """四元数 → 3x3 旋转矩阵（行主序的九元组）。"""
    x, y, z, w = q.x, q.y, q.z, q.w
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return (1 - 2 * (yy + zz), 2 * (xy - wz),     2 * (xz + wy),
            2 * (xy + wz),     1 - 2 * (xx + zz), 2 * (yz - wx),
            2 * (xz - wy),     2 * (yz + wx),     1 - 2 * (xx + yy))


def _mat_vec(m, v):
    return (m[0] * v[0] + m[1] * v[1] + m[2] * v[2],
            m[3] * v[0] + m[4] * v[1] + m[5] * v[2],
            m[6] * v[0] + m[7] * v[1] + m[8] * v[2])


def _tf_rotation(parent, child, timeout=30.0, lookup=None, clock=None, sleep=None, is_shutdown=None):
    """从**真实 TF** 取 parent←child 的旋转矩阵；取不到返回 (None, 原因)。

    ★为什么不是写死 15°★（2026-11-27 真人裁决）：URDF 里 lidar_pitch_deg=15 已经由
    robot_state_publisher 发成 base_link→rslidar 的 TF；若在 launch/脚本再写一份，
    就有两个维护点，改 URDF 忘了改这里会静默错。这里直接读 TF：单一来源。

    ★截止时间用 wall clock（time.monotonic）★，不能用 rospy.get_time()：
    仿真时钟会冻结/回跳，用 ROS 时间做超时会让循环永远等下去。
    lookup/sleep/is_shutdown 可注入，便于零节点做真实函数调用测试
    （见 verify/test_m2_relay_clock_guard.py；py_compile 抓不到 NameError 这类错误，
      2026-11-27 就是这么漏掉 `time` 未导入、relay 在 tf 模式当场 NameError 死掉的）。
    """
    clock = clock or time.monotonic
    is_shutdown = is_shutdown or rospy.is_shutdown
    if sleep is None:
        sleep = rospy.sleep
    if lookup is None:
        try:
            import tf2_ros
        except ImportError:
            return None, 'tf2_ros 不可用'
        buf = tf2_ros.Buffer()
        tf2_ros.TransformListener(buf)
        lookup = lambda: buf.lookup_transform(parent, child, rospy.Time(0), rospy.Duration(1.0))
    t0 = clock()
    last = '尚未尝试'
    while clock() - t0 < timeout:
        if is_shutdown():
            return None, '启动期收到 shutdown，未取到 TF'
        try:
            tr = lookup()
            return _quat_to_matrix(tr.transform.rotation), None
        except Exception as e:
            last = str(e)
            sleep(0.2)
    return None, 'TF %s←%s 在 %.0fs 内不可得（%s）' % (parent, child, timeout, last)

if __name__ == '__main__':
    rospy.init_node('m2_cmd_relay')
    src, dst, typ = sys.argv[1], sys.argv[2], sys.argv[3]
    # 第 4 参可选：`tf` 表示从真实 TF 取旋转（推荐、单一来源）；给数值则按该 pitch（度）绕 Y 旋转，
    # 缺省 0 = 与旧版逐字一致。真机/仿真都不该在脚本里再抄一份安装角。
    _arg4 = sys.argv[4] if len(sys.argv) > 4 else '0'
    _R = None
    pitch_deg = 0.0
    if _arg4 == 'tf':
        _R, _err = _tf_rotation('base_link', 'rslidar')
        if _R is None:
            # ★失败即阻断，不原样转发★（2026-11-27 真人裁决）：把未旋转的 rslidar 系
            # acc/gyro 冒充 base_link 喂给 HDL，比"没有 IMU"危险得多——它会静默把 15° 倾角
            # 当成加速度积分，正是此前 337 m 发散的成因。这里宁可让进程带错退出、在 launch
            # 日志里显式可见，也不产出错误数据。
            rospy.logerr('[m2_cmd_relay] 取 TF 失败，**阻断 IMU 输出**（不转发）：%s', _err)
            raise SystemExit(2)
        else:
            # 由旋转矩阵反推等效俯仰角，只为日志可读（真实使用的是矩阵本身）
            pitch_deg = math.degrees(math.atan2(-_R[6], math.hypot(_R[0], _R[3])))
            rospy.loginfo('[m2_cmd_relay] IMU 旋转源=TF(base_link←rslidar) 等效pitch=%.3fdeg R=%s',
                          pitch_deg, ['%.4f' % v for v in _R])
    else:
        pitch_deg = float(_arg4)
        _th = math.radians(pitch_deg)
        _c, _s = math.cos(_th), math.sin(_th)
    _shown = [False]
    mod = {'Twist': 'geometry_msgs.msg', 'Imu': 'sensor_msgs.msg'}[typ]
    msg_cls = getattr(__import__(mod, fromlist=['x']), typ)
    pub = rospy.Publisher(dst, msg_cls, queue_size=10)
    n = [0]
    def cb(m):
        _rot_on = (typ == 'Imu') and (_R is not None or pitch_deg != 0.0)
        if _rot_on:
            _in_frame = m.header.frame_id
            a, w = m.linear_acceleration, m.angular_velocity
            if _R is not None:
                ax, ay, az = _mat_vec(_R, (a.x, a.y, a.z))
                wx, wy, wz = _mat_vec(_R, (w.x, w.y, w.z))
            else:
                ax, ay, az = _rot_y((a.x, a.y, a.z), _c, _s)
                wx, wy, wz = _rot_y((w.x, w.y, w.z), _c, _s)
            m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = ax, ay, az
            m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = wx, wy, wz
            if m.header.frame_id != 'base_link':
                m.header.frame_id = 'base_link'
            if not _shown[0]:
                _shown[0] = True
                _src = 'TF' if _R is not None else ('pitch=%.1fdeg' % pitch_deg)
                rospy.loginfo('[m2_cmd_relay] IMU 旋转生效 源=%s 首帧 acc=(%.3f,%.3f,%.3f) gyro=(%.3f,%.3f,%.3f) in_frame=%s out_frame=base_link',
                              _src, ax, ay, az, wx, wy, wz, _in_frame)
        pub.publish(m); n[0] += 1
    rospy.Subscriber(src, msg_cls, cb, queue_size=10)
    rate = rospy.Rate(0.5, reset=True)
    while not rospy.is_shutdown():
        rospy.loginfo_throttle(60, '[m2_cmd_relay] %s -> %s (%s) fwd=%d pitch=%.1f' % (src, dst, typ, n[0], pitch_deg))
        # ★异常顺序是有讲究的★（2026-11-27 真人指出，已用 MRO 核实）：
        # rospy.ROSTimeMovedBackwardsException 的 MRO 是
        #   [ROSTimeMovedBackwardsException, ROSInterruptException, ROSException, Exception]
        # 也就是说它是 ROSInterruptException 的**子类**。上一版把 ROSInterruptException 放在前面，
        # 时钟回跳会被那个分支捕获并 break —— 节点照样退出，等于没修。
        # 因此必须"子类在前、父类在后"，并且 Rate 用 reset=True。
        if not _sleep_guarded(rate, lambda m: rospy.logwarn_throttle(30, '[m2_cmd_relay] %s', m)):
            break
