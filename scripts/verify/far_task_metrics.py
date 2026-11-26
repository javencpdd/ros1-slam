#!/usr/bin/env python3
"""FAR/Support 任务链读数器：一次采集六项实测数字（真人裁决的"最终提交实际数据"）。

采什么（全部只读订阅，不参与控制）：
  · `loc_hz`          定位频率 —— `/v8/local/pose`
  · `plan_hz`         规划频率 —— `/global_path`（每次规划一帧）
  · `path_updates`    路径更新次数 —— `/v8/3d/global/path_task` 帧数
  · `path_versions`   路径版本次数 —— path_task 里 `path_revision` 的去重个数
  · `local_goals`     local 目标更新次数 —— `/v8/local/goal_task` 帧数
  · `exec_resets`     执行重置次数 —— path_goal 状态里 `generation` 的最大值（每接受一版 +1）
  · `travel_m / gt_to_goal_m`  实际位移与末端真值距离 —— `/sim/odom_gt`
  · `pose_jitter_m`   采集窗口内位姿增量的中位数（用来证明"位姿在动但任务没换"）

用法：
  far_task_metrics.py --seconds 60 --out /tmp/metrics.json [--pose-topic /v8/local/pose]
"""
import argparse
import json
import math
import statistics
import time

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=60.0)
    ap.add_argument('--pose-topic', default='/v8/local/pose')
    ap.add_argument('--gt-topic', default='/sim/odom_gt')
    ap.add_argument('--path-topic', default='/global_path')
    ap.add_argument('--path-task-topic', default='/v8/3d/global/path_task')
    ap.add_argument('--local-goal-topic', default='/v8/local/goal_task')
    ap.add_argument('--status-topic', default='/v8/3d/path_goal/status')
    ap.add_argument('--out', default='')
    args = ap.parse_args()

    rospy.init_node('far_task_metrics', anonymous=True, disable_signals=True)
    st = {'pose': [], 'poses': 0, 'paths': 0, 'path_tasks': 0, 'local_goals': 0,
          'revisions': set(), 'missions': set(), 'generation_max': 0,
          'gt': [], 'poses_jitter': [], 'last_pose': None, 'invalidations': 0}

    def on_pose(msg):
        p = msg.pose.position
        st['poses'] += 1
        st['pose'].append((p.x, p.y))
        if st['last_pose'] is not None:
            d = math.hypot(p.x - st['last_pose'][0], p.y - st['last_pose'][1])
            st['poses_jitter'].append(d)
        st['last_pose'] = (p.x, p.y)

    def on_path(_msg):
        st['paths'] += 1

    def on_path_task(msg):
        st['path_tasks'] += 1
        try:
            value = json.loads(msg.data)
        except Exception:
            return
        if 'path_revision' in value:
            st['revisions'].add((int(value.get('mission_id', 0)), int(value['path_revision'])))
        if 'mission_id' in value:
            st['missions'].add(int(value['mission_id']))
        if value.get('invalidated'):
            st['invalidations'] += 1

    def on_local_goal(_msg):
        st['local_goals'] += 1

    def on_status(msg):
        try:
            value = json.loads(msg.data)
        except Exception:
            return
        gen = value.get('generation')
        if isinstance(gen, int):
            st['generation_max'] = max(st['generation_max'], gen)

    def on_gt(msg):
        p = msg.pose.pose.position
        st['gt'].append((p.x, p.y))

    rospy.Subscriber(args.pose_topic, PoseStamped, on_pose, queue_size=50)
    rospy.Subscriber(args.path_topic, __import__('nav_msgs.msg', fromlist=['Path']).Path,
                     on_path, queue_size=20)
    rospy.Subscriber(args.path_task_topic, String, on_path_task, queue_size=50)
    rospy.Subscriber(args.local_goal_topic, String, on_local_goal, queue_size=50)
    rospy.Subscriber(args.status_topic, String, on_status, queue_size=50)
    rospy.Subscriber(args.gt_topic, Odometry, on_gt, queue_size=50)

    t0 = time.monotonic()
    rospy.sleep(max(0.1, args.seconds))
    wall = max(1e-6, time.monotonic() - t0)

    travel = 0.0
    gt_to_goal = None
    if len(st['gt']) > 1:
        travel = sum(math.hypot(b[0] - a[0], b[1] - a[1])
                     for a, b in zip(st['gt'], st['gt'][1:]))
    jitter = 0.0
    if st['poses_jitter']:
        jitter = statistics.median(st['poses_jitter'])

    result = {'window_s': round(wall, 2),
              'loc_hz': round(st['poses'] / wall, 2),
              'plan_hz': round(st['paths'] / wall, 2),
              'path_updates': st['path_tasks'],
              'path_versions': len(st['revisions']),
              'missions': sorted(st['missions']),
              'local_goals': st['local_goals'],
              'exec_resets': st['generation_max'],
              'invalidations': st['invalidations'],
              'travel_m': round(travel, 3),
              'pose_step_median_m': round(jitter, 4),
              'gt_samples': len(st['gt'])}
    if args.out:
        with open(args.out, 'w') as fh:
            fh.write(json.dumps(result, ensure_ascii=False) + '\n')
    print(json.dumps(result, ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
