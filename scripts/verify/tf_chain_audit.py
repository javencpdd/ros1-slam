#!/usr/bin/env python3
"""TF 生产链审计：谁是谁的父、有没有双父/环路/超龄、生产链是否偷偷经过 GT。

背景（2026-11-27）：`odom→base_link` 由 lite3_planar_drive 用 Gazebo WorldPose 发布，
而 FAR 的 on_cloud 走 `map→rslidar`，中间经过 base_link ⇒ **地形转换间接消费 GT**。
"本地能查到 map→base_link"并不能证明坐标正确，更不能证明没用 GT。

判定口径：
  · 双父：同一 child 有两个不同 parent（TF 语义上非法）
  · 环路：沿 parent 链走回自己
  · 超龄：动态边的最近时间戳落后于 now 超过 max_age
  · 生产链途经 GT：map→rslidar 的父链上出现 GT 源帧（默认名单 odom）

用法：tf_chain_audit.py [--max-age 1.0] [--out out.json]
只读。退出码：0 全部通过；1 有违例；2 无 TF 数据。
"""
import argparse
import json
import sys
import time

import rospy
from tf2_msgs.msg import TFMessage

GT_FRAMES = {'odom'}          # 当前 GT 源帧；改名后此处同步


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--max-age', type=float, default=1.0)
    ap.add_argument('--out', default='')
    a = ap.parse_args()

    rospy.init_node('tf_chain_audit', anonymous=True, disable_signals=True)
    edges = {}          # child -> [parent]
    stamps = {}         # (parent, child) -> (stamp, wall)

    def cb(msg):
        now = time.monotonic()
        for tr in msg.transforms:
            p, c = tr.header.frame_id, tr.child_frame_id
            edges.setdefault(c, [])
            if p not in edges[c]:
                edges[c].append(p)
            stamps[(p, c)] = (tr.header.stamp.to_sec(), now)

    rospy.Subscriber('/tf', TFMessage, cb, queue_size=100)
    rospy.Subscriber('/tf_static', TFMessage, cb, queue_size=100)
    time.sleep(6)

    if not edges:
        print('VERDICT NO_DATA')
        return 2

    problems = []
    for c, parents in edges.items():
        if len(parents) > 1:
            problems.append({'kind': 'double_parent', 'child': c, 'parents': parents})

    def chain(target):
        """返回 target 到根的父链（检测环路）。"""
        out, node, seen = [], target, set()
        while node in edges and edges[node]:
            if node in seen:
                problems.append({'kind': 'cycle', 'at': node, 'chain': out})
                break
            seen.add(node)
            node = edges[node][0]
            out.append(node)
        return out

    prod = chain('rslidar')
    gt_hit = [f for f in prod if f in GT_FRAMES]
    if gt_hit:
        problems.append({'kind': 'production_uses_gt', 'chain': prod, 'gt_frames': gt_hit})

    # 超龄：动态边（/tf 里的）用观测墙钟判，不看 ROS 时间
    now_wall = time.monotonic()
    stale = []
    for (p, c), (stamp, wall) in stamps.items():
        if now_wall - wall > a.max_age:
            stale.append({'edge': '%s->%s' % (p, c), 'age_s': round(now_wall - wall, 2)})

    print('CHAIN rslidar <- %s' % ' <- '.join(prod))
    print('EDGES %d  DYN_STALE %d' % (len(edges), len(stale)))
    for p in problems:
        print('PROBLEM %s' % json.dumps(p, ensure_ascii=False))
    verdict = 'FAIL' if problems else 'PASS'
    print('VERDICT %s' % verdict)
    if a.out:
        json.dump({'verdict': verdict, 'problems': problems, 'chain': prod,
                   'edges': {k: v for k, v in edges.items()}, 'stale': stale},
                  open(a.out, 'w', encoding='utf-8'), ensure_ascii=False, indent=1)
    return 0 if verdict == 'PASS' else 1


if __name__ == '__main__':
    sys.exit(main())
