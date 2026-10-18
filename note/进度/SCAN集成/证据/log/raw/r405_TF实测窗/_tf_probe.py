#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""_tf_probe.py —— 第 0 步活体实测：雷达离地高与俯仰（r405 窗内只读探针，★非在册件、★不入判据）。

读三样：
  ①TF base_link→rslidar（静态、源自 robot_description）＝安装高与俯仰的真身；
  ②TF odom→base_link（插件所发）＝载体在世界系里的离地高度；若该边不在位则照实报缺、改用 model_states；
  ③/gazebo/model_states 里 lite3_planar 的位姿（多次采样，报 首／末／min／max）。
输出＝stdout 逐行（供留底）＋一行 JSON 汇总。★不改任何参数、★不发命令、★只监听。
"""
import json
import math
import sys
import time

import rospy
import tf2_ros
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import TransformStamped


def euler_xyzw(x, y, z, w):
    """返回 (roll, pitch, yaw) 弧度，ZYX 内旋（与 ROS 的 rpy 约定一致）。"""
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)
    sinp = 2.0 * (w * y - z * x)
    sinp = 1.0 if sinp > 1.0 else (-1.0 if sinp < -1.0 else sinp)
    pitch = math.asin(sinp)
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny, cosy)
    return roll, pitch, yaw


def main():
    rospy.init_node('r405_tf_probe', anonymous=True)
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf)

    samples = []

    def cb(m):
        try:
            i = m.name.index('lite3_planar')
        except (ValueError, IndexError):
            return
        p = m.pose[i].position
        q = m.pose[i].orientation
        r, pi_, y = euler_xyzw(q.x, q.y, q.z, q.w)
        samples.append((rospy.Time.now().to_sec(), p.x, p.y, p.z,
                        math.degrees(r), math.degrees(pi_), math.degrees(y)))

    sub = rospy.Subscriber('/gazebo/model_states', ModelStates, cb, queue_size=50)

    deadline = time.time() + float(sys.argv[1] if len(sys.argv) > 1 else 12.0)
    while time.time() < deadline:
        rospy.sleep(0.5)

    out = {}
    for a, b in [('base_link', 'rslidar'), ('odom', 'base_link'),
                 ('base_link', 'TORSO'), ('odom', 'rslidar')]:
        key = '%s->%s' % (a, b)
        try:
            st = buf.lookup_transform(a, b, rospy.Time(0), rospy.Duration(4.0))
            t = st.transform.translation
            r, p, y = euler_xyzw(st.transform.rotation.x, st.transform.rotation.y,
                                 st.transform.rotation.z, st.transform.rotation.w)
            out[key] = {'xyz': [round(t.x, 4), round(t.y, 4), round(t.z, 4)],
                        'rpy_deg': [round(math.degrees(r), 3), round(math.degrees(p), 3),
                                     round(math.degrees(y), 3)],
                        'age_s': round(st.header.stamp.to_sec() and (rospy.Time.now() - st.header.stamp).to_sec(), 2)}
            print('[TF ] %-18s xyz=%s rpy_deg=%s  age=%ss' % (
                key, out[key]['xyz'], out[key]['rpy_deg'], out[key]['age_s']))
        except Exception as e:
            out[key] = {'error': str(e)[:90]}
            print('[TF ] %-18s ★缺边：%s' % (key, str(e)[:90]))

    if samples:
        zs = [s[3] for s in samples]
        first, last = samples[0], samples[-1]
        print('[MDL] lite3_planar 采样=%d 首 z=%.4f 末 z=%.4f min=%.4f max=%.4f 末 rpy_deg=[%.2f, %.2f, %.2f]'
              % (len(samples), first[3], last[3], min(zs), max(zs), last[4], last[5], last[6]))
        out['model'] = {'n': len(samples), 'z_first': round(first[3], 4), 'z_last': round(last[3], 4),
                        'z_min': round(min(zs), 4), 'z_max': round(max(zs), 4),
                        'rpy_last_deg': [round(last[4], 3), round(last[5], 3), round(last[6], 3)]}
    else:
        print('[MDL] ★model_states 零采样（该话题无发布者或模型名不符）')
        out['model'] = {'n': 0}

    mz = out['model'].get('z_last')
    lz = out.get('base_link->rslidar', {}).get('xyz', [None, None, None])[2]
    direct = out.get('odom->rslidar', {}).get('xyz', [None, None, None])[2]
    comp = None
    if mz is not None and lz is not None:
        comp = round(mz + lz, 4)
    print('[合成] 雷达离地（model_z + TF 安装高）＝%s ‖ 直查 odom->rslidar z＝%s ‖ 在册推导值＝0.53（0.15 spawn ＋ 0.38 安装）'
          % (comp, direct))
    print('[俯仰] TF base_link->rslidar pitch＝%s deg（在册 xacro 字面＝0）'
          % out.get('base_link->rslidar', {}).get('rpy_deg', [None, None, None])[1])
    out['derived'] = {'lidar_clearance_composed': comp, 'lidar_clearance_direct': direct,
                      'in_register_claim': 0.53}
    print('JSON ' + json.dumps(out, ensure_ascii=False, sort_keys=True))


if __name__ == '__main__':
    main()
