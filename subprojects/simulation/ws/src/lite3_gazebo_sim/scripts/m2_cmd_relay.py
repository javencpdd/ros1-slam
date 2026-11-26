#!/usr/bin/env python3
# m2_cmd_relay.py —— M2 自研话题转发器（小张，2026-10-06）
# 用途一：/cmd_vel_raw（mbf 输出，mbf_navigation.launch 已 remap）→ /cmd_vel（lite3_planar_drive 消费）
#         —— M6 safe_cmd_vel_mux 不在仿真闭环内，此 relay 仅 sim 域（ROS master 11313），不触生产 11312。
# 用途二：/livox/imu → /imu/data（hdl_localization 生产 remap 契约，M2-3 仿真域喂 IMU）。
# 用法：m2_cmd_relay.py <from_topic> <to_topic> <Twist|Imu>
# 红线：本节点不写任何真机话题；生产 master 下禁用（m2_*.launch 全部在 11313 隔离域内起）。
import math, sys, rospy

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

if __name__ == '__main__':
    rospy.init_node('m2_cmd_relay')
    src, dst, typ = sys.argv[1], sys.argv[2], sys.argv[3]
    # 第 4 参可选：pitch 角（度）。缺省 0 = 行为与旧版逐字一致。
    pitch_deg = float(sys.argv[4]) if len(sys.argv) > 4 else 0.0
    _th = math.radians(pitch_deg)
    _c, _s = math.cos(_th), math.sin(_th)
    _shown = [False]
    mod = {'Twist': 'geometry_msgs.msg', 'Imu': 'sensor_msgs.msg'}[typ]
    msg_cls = getattr(__import__(mod, fromlist=['x']), typ)
    pub = rospy.Publisher(dst, msg_cls, queue_size=10)
    n = [0]
    def cb(m):
        if typ == 'Imu' and pitch_deg != 0.0:
            a, w = m.linear_acceleration, m.angular_velocity
            ax, ay, az = _rot_y((a.x, a.y, a.z), _c, _s)
            wx, wy, wz = _rot_y((w.x, w.y, w.z), _c, _s)
            m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = ax, ay, az
            m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = wx, wy, wz
            if m.header.frame_id != 'base_link':
                m.header.frame_id = 'base_link'
            if not _shown[0]:
                _shown[0] = True
                rospy.loginfo('[m2_cmd_relay] IMU 旋转 rslidar→base_link pitch=%.3fdeg 首帧 acc=(%.3f,%.3f,%.3f) gyro=(%.3f,%.3f,%.3f)',
                              pitch_deg, ax, ay, az, wx, wy, wz)
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
