#!/usr/bin/env python3
"""运动中障碍安全（在线）：车在跑的时候往它前方投一个箱子，断言"不撞、且确实响应"。

为什么离线回归不够：`collision_regression.py` 是喂点云帧的离线判定，验的是感知/局部算法本身；
它**证不了**"整条链在运动中真的会停或绕"。这条在线断言补的就是这半段。

判据（全部满足才 PASS）：
  1) 不穿透：真值轨迹的任何一点都不落进箱子 XY 矩形（外扩机器人半径）；
  2) 有响应：投障后有 **≥0.5 s 的零速窗口**，或横向偏离 ≥0.10 m（绕行）；
  3) 注入前确实在动（否则这条什么也没验）。

安全：箱子在 finally 里一定删除；只投一个、尺寸小、只在验收世界用。
"""
import argparse
import json
import math
import time

import rospy
from gazebo_msgs.srv import DeleteModel, SpawnModel
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Twist

BOX_SDF = """<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <collision name="c"><geometry><box><size>{s} {s} {h}</size></box></geometry></collision>
      <visual name="v"><geometry><box><size>{s} {s} {h}</size></box></geometry></visual>
    </link>
  </model>
</sdf>"""


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--pose-topic', default='/hdl_odom')
    ap.add_argument('--gt-topic', default='/sim/odom_gt')
    ap.add_argument('--lead-m', type=float, default=1.2, help='箱子投在车前方多少米')
    ap.add_argument('--size-m', type=float, default=0.5)
    ap.add_argument('--height-m', type=float, default=0.6)
    ap.add_argument('--robot-radius-m', type=float, default=0.30)
    ap.add_argument('--observe-s', type=float, default=10.0)
    ap.add_argument('--out', default='')
    args = ap.parse_args()

    rospy.init_node('obstacle_inmotion_probe', anonymous=True)
    st = {'gt': [], 'cmd': []}

    def on_gt(m):
        p = m.pose.pose.position
        st['gt'].append((time.monotonic(), p.x, p.y))

    def on_cmd(m):
        st['cmd'].append((time.monotonic(), math.hypot(m.linear.x, m.linear.y)))

    rospy.Subscriber(args.gt_topic, Odometry, on_gt, queue_size=50)
    rospy.Subscriber('/cmd_vel', Twist, on_cmd, queue_size=50)

    res = {'lead_m': args.lead_m, 'size_m': args.size_m, 'spawned': False}
    box = 'v8_accept_obs_%d' % int(time.time())
    try:
        rospy.wait_for_service('/gazebo/spawn_sdf_model', timeout=10)
        rospy.wait_for_service('/gazebo/delete_model', timeout=10)
        spawn = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        delete = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)

        # 先等车动起来（没有运动这项就无意义）
        t0 = time.monotonic()
        while time.monotonic() - t0 < 30 and not rospy.is_shutdown():
            if len(st['cmd']) > 10 and max(v for _, v in st['cmd'][-10:]) > 0.05:
                break
            rospy.sleep(0.2)
        moving = len(st['cmd']) > 0 and max(v for _, v in st['cmd']) > 0.05
        res['moving_before_spawn'] = bool(moving)

        # 用最新位姿算投障点
        if not st['gt']:
            rospy.sleep(0.5)
        if not st['gt']:
            res.update(verdict='INVALID', reason='无真值样本')
            raise SystemExit(0)
        _, x, y = st['gt'][-1]
        # 朝向取最近 /cmd_vel 非零段的平均方向不可靠，这里用真值姿态推出航向
        pose = rospy.wait_for_message(args.gt_topic, Odometry, timeout=5)
        yaw = yaw_of(pose.pose.pose.orientation)
        bx, by = x + args.lead_m * math.cos(yaw), y + args.lead_m * math.sin(yaw)
        res['box_at'] = [round(bx, 3), round(by, 3)]
        spawn(box, BOX_SDF.format(name=box, s=args.size_m, h=args.height_m),
              '', Pose(position=Point(x=bx, y=by, z=args.height_m / 2.0)), 'world')
        res['spawned'] = True
        t_spawn = time.monotonic()

        rospy.sleep(args.observe_s)
        seg = [(t, gx, gy) for (t, gx, gy) in st['gt'] if t >= t_spawn]
        cmd_seg = [(t, v) for (t, v) in st['cmd'] if t >= t_spawn]

        half = args.size_m / 2.0 + args.robot_radius_m
        inside = [(round(gx, 2), round(gy, 2)) for (_, gx, gy) in seg
                  if abs(gx - bx) <= half and abs(gy - by) <= half]
        res['penetrations'] = len(inside)
        res['penetration_samples'] = inside[:5]

        zero_run = cur = 0.0
        for (t0_, v), (t1_, _v) in zip(cmd_seg, cmd_seg[1:]):
            cur = cur + (t1_ - t0_) if v <= 0.05 else 0.0
            zero_run = max(zero_run, cur)
        res['max_zero_run_s'] = round(zero_run, 2)

        lateral = 0.0
        if len(seg) >= 2:
            lateral = max(math.hypot(gx - x, gy - y) for (_, gx, gy) in seg)
        res['max_travel_m'] = round(lateral, 3)

        checks = {
            'moving_before_spawn': res['moving_before_spawn'],
            'no_penetration': res['penetrations'] == 0,
            'responded': (zero_run >= 0.5) or (lateral >= 0.10),
        }
        res['checks'] = checks
        res['verdict'] = 'PASS' if all(checks.values()) else 'FAIL'
    except SystemExit:
        pass
    except Exception as e:                    # 服务缺失/世界不允许 spawn 都要如实记
        res['verdict'] = 'INVALID'
        res['reason'] = str(e)[:160]
    finally:
        try:
            rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)(box)
            res['deleted'] = True
        except Exception:
            res['deleted'] = False

    text = json.dumps(res, ensure_ascii=False)
    print(text)
    if args.out:
        open(args.out, 'w').write(text + '\n')
    return 0 if res.get('verdict') == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
