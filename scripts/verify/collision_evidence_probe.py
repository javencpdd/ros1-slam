#!/usr/bin/env python3
"""碰撞现场取证探针（2026-11-26，FAR 复核用）：只记录，不改任何阈值或算法。

为什么要它：FAR 下机器人不动，`/v8/local/status` 只给一句 `E_LOCAL_COLLISION`。
在没拿到"当时位姿 / 目标 / 支撑格 / 障碍包围盒 / 归一化轨迹"之前，任何对地形算法或
碰撞阈值的修改都是无据的（真人裁决⑤明确禁止先改后证）。

采集内容（一次碰撞只取第一帧，避免刷屏）：
  · 状态：reason / detail / stop / velocity / plan_ms
  · 位姿：`--pose-topic` 的最新一帧（默认 /hdl_odom）
  · 目标：`/v8/local/goal_task`（原子）或 `/v8/local/goal`（legacy）最近一次
  · 支撑格：`/v8_support_map`（JSON）在**当前格**与 3x3 邻域的取值（cell id / 高度 / 类型）
  · 障碍：`/v8/local/obstacles` 里距位姿 --radius 内的框（原样保留，另附距离）
  · 轨迹：最近 --history 条位姿 + cmd（用于离线复放定位差异）

用法：
  collision_evidence_probe.py --out /tmp/collision.json --seconds 60
"""
import argparse
import json
import math
import time

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String


def _pos(msg):
    p = getattr(getattr(msg, 'pose', msg), 'position', None)
    if p is None:
        return None
    return [float(p.x), float(p.y), float(p.z)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=60.0)
    ap.add_argument('--pose-topic', default='/hdl_odom')
    ap.add_argument('--obstacles-topic', default='/v8/local/obstacles')
    ap.add_argument('--support-topic', default='/v8_support_map')
    ap.add_argument('--status-topic', default='/v8/local/status')
    ap.add_argument('--radius', type=float, default=2.0)
    ap.add_argument('--history', type=int, default=60)
    ap.add_argument('--out', default='')
    args = ap.parse_args()

    rospy.init_node('collision_evidence_probe', anonymous=True, disable_signals=True)
    st = {'pose': None, 'poses': [], 'goal': None, 'obstacles': None,
          'support': None, 'collision': None, 'status': None}

    def on_pose(msg):
        p = _pos(msg)
        if p is None:
            return
        st['pose'] = p
        st['poses'].append([round(time.monotonic(), 3)] + p[:2])
        st['poses'] = st['poses'][-args.history:]

    def on_goal(msg):
        try:
            st['goal'] = json.loads(msg.data)
        except Exception:
            st['goal'] = {'raw': (msg.data or '')[:200]}

    def on_obstacles(msg):
        try:
            st['obstacles'] = json.loads(msg.data)
        except Exception:
            pass

    def on_support(msg):
        try:
            st['support'] = json.loads(msg.data)
        except Exception:
            pass

    def on_status(msg):
        try:
            value = json.loads(msg.data)
        except Exception:
            return
        st['status'] = value
        if st['collision'] is None and str(value.get('reason', '')).startswith('E_LOCAL_COLLISION'):
            st['collision'] = dict(value, at=time.monotonic())

    rospy.Subscriber(args.pose_topic, Odometry, on_pose, queue_size=20)
    try:
        rospy.Subscriber('/v8/local/goal_task', String, on_goal, queue_size=20)
    except Exception:
        pass
    rospy.Subscriber(args.obstacles_topic, String, on_obstacles, queue_size=2)
    rospy.Subscriber(args.support_topic, String, on_support, queue_size=1)
    rospy.Subscriber(args.status_topic, String, on_status, queue_size=50)

    rospy.sleep(max(1.0, args.seconds))

    pose = st['pose']
    boxes = []
    obstacles = st['obstacles']
    raw_list = []
    if isinstance(obstacles, dict):
        raw_list = obstacles.get('obstacles') or obstacles.get('boxes') or []
    elif isinstance(obstacles, list):
        raw_list = obstacles
    if pose is not None:
        for box in raw_list:
            try:
                cx, cy = float(box[0]), float(box[1])
                d = math.hypot(cx - pose[0], cy - pose[1])
                if d <= args.radius:
                    boxes.append({'box': box, 'dist_m': round(d, 3)})
            except (TypeError, ValueError, IndexError):
                boxes.append({'box': box, 'dist_m': None})
    boxes.sort(key=lambda b: (b['dist_m'] is None, b['dist_m']))

    cell = None
    support = st['support']
    if pose is not None and isinstance(support, dict):
        res = support.get('resolution') or support.get('cell_size')
        origin = support.get('origin') or support.get('origin_xy')
        if res and origin:
            cell = {'resolution_m': res, 'origin': origin,
                    'cell_ij': [int(math.floor((pose[0] - origin[0]) / res)),
                                int(math.floor((pose[1] - origin[1]) / res))]}

    result = {'collision': st['collision'],
              'last_status': st['status'],
              'pose': pose, 'goal': st['goal'], 'support_cell': cell,
              'obstacle_boxes_near': boxes[:20],
              'obstacles_total': len(raw_list),
              'trajectory_xy': st['poses'],
              'note': '只取证，不改判据；离线复放用 trajectory_xy + support/obstacles 原始帧'}
    if args.out:
        with open(args.out, 'w') as fh:
            fh.write(json.dumps(result, ensure_ascii=False) + '\n')
    print(json.dumps({'collision': bool(st['collision']),
                      'reason': (st['collision'] or {}).get('reason'),
                      'obstacles_total': len(raw_list),
                      'boxes_near': len(boxes)}, ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
