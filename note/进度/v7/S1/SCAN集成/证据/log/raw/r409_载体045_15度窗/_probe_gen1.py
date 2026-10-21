#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""_probe_gen1.py —— 载切到 0.45／15° 后的活体量法＋★真人 2026-10-19 要的四项相容验算。

四项＝①雷达系地面 z 落 −0.4347±一板 ②甲档板数＝13 ③k0＝1 ④truncated＝False。
★几何一律取在册件 t3 自己的函数（ground_z_radar／k0_of／is_truncated）＋t1 的甲档常量，★不另写第二套。
★★任何一项被推翻 ⇒ 本件以 rc=5 退出（★真人明令：停手带数来、不得自选备选档）。
"""
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REG = os.path.join('note', '进度', 'SCAN集成', '设计', 'scripts')
ROOT = HERE
while ROOT and ROOT != os.sep:
    if os.path.isdir(os.path.join(ROOT, REG)):
        break
    ROOT = os.path.dirname(ROOT)
else:
    ROOT = ''
if not ROOT:
    sys.exit('★前置缺失：从 %s 往上找不到仓根（★禁硬写机器绝对路径＝坑66 同族）' % HERE)
sys.path.insert(0, os.path.join(ROOT, REG))

import rospy
import tf2_ros

import t1_multilevel_raster_offline as t1
import t3_decidable_input_offline as t3

ANCHOR = -0.4347          # ★在册 R310 锚（三值之 x_b=0）


def euler_xyzw(x, y, z, w):
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)
    sinp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sinp)
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return roll, pitch, math.atan2(siny, cosy)


def main():
    rospy.init_node('r409_probe', anonymous=True)
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf)
    rospy.sleep(4.0)                       # 等 TF 流稳定
    out = {}
    for a, b, key in [('base_link', 'rslidar', 'mount'), ('odom', 'base_link', 'base'),
                      ('odom', 'rslidar', 'lidar_world')]:
        try:
            st = buf.lookup_transform(a, b, rospy.Time(0), rospy.Duration(6.0))
            tr, ro = st.transform.translation, st.transform.rotation
            r, p, y = euler_xyzw(ro.x, ro.y, ro.z, ro.w)
            out[key] = {'xyz': [round(tr.x, 4), round(tr.y, 4), round(tr.z, 4)],
                        'rpy_deg': [round(math.degrees(r), 3), round(math.degrees(p), 3), round(math.degrees(y), 3)]}
            print('[TF ] %-18s xyz=%s rpy_deg=%s' % (a + '->' + b, out[key]['xyz'], out[key]['rpy_deg']))
        except Exception as e:
            out[key] = {'error': str(e)[:80]}
            print('[TF ] %-18s ★缺边：%s' % (a + '->' + b, str(e)[:80]))
            sys.exit(9)

    h = out['lidar_world']['xyz'][2]                    # 雷达离地（世界系，活体实量）
    th = out['mount']['rpy_deg'][1]                     # 俯仰（★正＝下倾，REP-103）
    print('\n★活体量得＝离地 h=%.4f m ‖ 俯仰 θ=%.3f°（安装高 TF=%s、基座稳态 z=%s）'
          % (h, th, out['mount']['xyz'], out['base']['xyz'][2]))

    zg = t3.ground_z_radar(0.0, h, th)
    nz = int(round((t1.Z_MAX - t1.Z_MIN) / t1.Z_SLAB))
    k0 = t3.k0_of(zg, t1.Z_MIN)
    trunc = t3.is_truncated(zg, t1.Z_MIN)
    roll = out['lidar_world']['rpy_deg'][0]
    yaw = out['lidar_world']['rpy_deg'][2]
    checks = [
        # ★槽位自检（★R409 自纠新增：首刀把 15° 放进 rpy 第一槽＝roll ⇒ 四项会按 θ＝0 全绿＝假绿）
        ('⓪槽位自检：非零角必须在 pitch（第二槽）、roll/yaw 须≈0',
         abs(th) > 1.0 and abs(roll) < 0.01 and abs(yaw) < 0.01,
         'rpy_deg=[roll=%.3f, pitch=%.3f, yaw=%.3f]（★若 roll 非零＝角度落错槽＝雷达被侧倾而非仲俯）' % (roll, th, yaw)),
        ('①地面 z 落 −0.4347±一板', abs(zg - ANCHOR) <= t1.Z_SLAB, 'z_ground=%.4f 锚=%.4f 差=%.4f 板厚=%.2f' % (zg, ANCHOR, zg - ANCHOR, t1.Z_SLAB)),
        ('②甲档板数＝13', nz == 13, 'nz=%d（窗 [%.2f, %.2f]、板厚 %.2f＝在册 t1 常量、★未改）' % (nz, t1.Z_MIN, t1.Z_MAX, t1.Z_SLAB)),
        ('③k0＝1', k0 == 1, 'k0=%d' % k0),
        ('④truncated＝False', trunc is False, 'truncated=%s' % trunc),
    ]
    print('\n==== ⓪槽位自检＋四项相容验算（★用活体实量 (h,θ) 喂在册式子）====')
    nfail = 0
    for name, ok, detail in checks:
        print('[%s] %s ｜ %s' % ('PASS' if ok else '★FAIL', name, detail))
        nfail += 0 if ok else 1
    out['derived'] = {'h_live': h, 'theta_live': round(th, 3), 'roll_deg': round(roll, 3), 'yaw_deg': round(yaw, 3), 'z_ground': round(zg, 4), 'nz': nz, 'k0': k0,
                      'truncated': bool(trunc), 'nfail': nfail}
    print('JSON ' + json.dumps(out, ensure_ascii=False, sort_keys=True))
    if nfail:
        print('★四项里有 %d 项被推翻 ⇒ 按真人明令★停手带数来、★不自选备选档' % nfail)
        sys.exit(5)
    print('★⓪＋四项全中 ⇒ 按真人判语＝"不是换档、是坐实"（甲档名实相符）')


if __name__ == '__main__':
    main()
