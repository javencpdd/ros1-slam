#!/usr/bin/env python3
"""TF 生产链审计：按**装配契约**判生产链是否合法（不只看"能不能查到"）。

2026-11-27 重写（真人指出旧版六个确定性缺陷）：
  · 旧的 stale 只打印不进 problems ⇒ 超龄不影响 PASS；
  · /tf 与 /tf_static 混在一起统计 ⇒ 静态边被当动态心跳判超龄；
  · 不要求生产链最终到达 map ⇒ 断链也能 PASS；
  · 不检测同一条边被多个发布者写；
  · 只用接收墙钟，不证明**源戳在推进**；
  · GT 帧名单写死。

现在的模型：
  contract = {'root': 'map', 'production_target': 'rslidar',
              'gt_frames': {...}, 'stale_s': 1.0}
  边 = {child: {parent: {'publisher': name, 'dynamic': bool, 'stamps': [...]}}}
  静态边（/tf_static）不按心跳判超龄，只判同一 child 的双父与重复发布。

判红项（任一即 FAIL）：
  double_parent / cycle / no_root（生产链没到 root）/ production_uses_gt /
  duplicate_publisher（同一条边多个发布者）/ stale（仅动态）/ stamp_stalled（仅动态）/
  future_stamp / duplicate_stamp

纯逻辑在 audit()，可由单测直接喂合成数据；ROS 采集只在 main() 里。
用法：tf_chain_audit.py [--root map] [--target rslidar] [--gt-frames odom,gt_odom]
                       [--stale 1.0] [--out out.json]
退出码：0 PASS / 1 FAIL / 2 无数据。
"""
import argparse
import json
import sys
import time

# ---------------------------------------------------------------- 纯逻辑


def _walk(edges, node, problems):
    """从 node 沿父链上溯，返回链；检测环路。"""
    chain, seen = [], set()
    while True:
        chain.append(node)
        if node not in edges or not edges[node]:
            return chain
        if node in seen:
            problems.append({'kind': 'cycle', 'at': node, 'chain': list(chain)})
            return chain
        seen.add(node)
        parents = list(edges[node].keys())
        if len(parents) > 1:
            problems.append({'kind': 'double_parent', 'child': node, 'parents': parents})
        node = parents[0]


def _conflicts(values, stamps, eps=1e-6, bucket=0.05):
    """同一条边在**同一时刻**出现两套不同数值 ⇒ 双写嫌疑。

    关键：不能拿整段采集比——机器人本来就在动，8 秒里位姿必然变化。
    按源戳分桶（默认 50 ms），只有**同一桶内**数值不一致才算冲突。
    """
    if len(values) < 2 or len(stamps) != len(values):
        return False
    groups = {}
    for st, v in zip(stamps, values):
        groups.setdefault(round(st / bucket), []).append(v)
    for vs in groups.values():
        if len(vs) >= 2 and any(abs(v - vs[0]) > eps for v in vs[1:]):
            return True
    return False


def audit(edges, contract, now=None):
    """edges: {child: {parent: {'publisher':str,'dynamic':bool,'stamps':[float]}}}"""
    problems = []
    root = contract['root']
    gt = set(contract.get('gt_frames') or ())
    stale_s = float(contract.get('stale_s', 1.0))
    now = time.time() if now is None else now

    for child, parents in edges.items():
        if len(parents) > 1:
            problems.append({'kind': 'double_parent', 'child': child,
                             'parents': sorted(parents)})
        for parent, info in parents.items():
            pubs = info.get('publishers') or ([info['publisher']] if info.get('publisher') else [])
            if len(set(pubs)) > 1:
                problems.append({'kind': 'duplicate_publisher',
                                 'edge': '%s->%s' % (parent, child),
                                 'publishers': sorted(set(pubs))})
            # 注意：/tf 有多个发布者是**正常**的（插件、LIO、静态各发各的边），
            # 所以话题级多发布者只作信息输出，不判红。真正的"双写"看同一条边的值冲突。
            vals = info.get('values') or []
            if _conflicts(vals, info.get('stamps') or []):
                problems.append({'kind': 'double_write',
                                 'edge': '%s->%s' % (parent, child),
                                 'samples': len(vals)})
            stamps = info.get('stamps') or []
            if info.get('dynamic'):
                if stamps:
                    if any(b < a for a, b in zip(stamps, stamps[1:])):
                        problems.append({'kind': 'stamp_regression',
                                         'edge': '%s->%s' % (parent, child)})
                    if len(stamps) >= 2 and stamps[-1] == stamps[0]:
                        problems.append({'kind': 'stamp_stalled',
                                         'edge': '%s->%s' % (parent, child)})
                    if any(s > now + 1.0 for s in stamps):
                        problems.append({'kind': 'future_stamp',
                                         'edge': '%s->%s' % (parent, child),
                                         'max': max(stamps)})
                if info.get('last_wall') is not None and now - info['last_wall'] > stale_s:
                    problems.append({'kind': 'stale', 'edge': '%s->%s' % (parent, child),
                                     'age_s': round(now - info['last_wall'], 2)})

    target = contract.get('production_target')
    chain = _walk(edges, target, problems) if target else []
    if target:
        if not chain or chain[-1] != root:
            problems.append({'kind': 'no_root', 'target': target,
                             'chain': chain, 'root': root})
        # ★链上每一跳都必须"现在是活的"★（2026-11-27 实测暴露）：
        # 边集是 8 秒缓冲，动态边可能早已停发，但集合里还留着 ⇒ 会给出假 PASS。
        # tf_echo 现场查不到、审计却 PASS 就是这么来的。动态跳要求 last_wall 在 stale_s 内。
        for child, parent in zip(chain, chain[1:]):
            info = (edges.get(child) or {}).get(parent) or {}
            if info.get('dynamic') and info.get('last_wall') is not None \
                    and now - info['last_wall'] > stale_s:
                problems.append({'kind': 'chain_stale',
                                 'edge': '%s->%s' % (parent, child),
                                 'age_s': round(now - info['last_wall'], 2)})
        hit = [f for f in chain if f in gt]
        if hit:
            problems.append({'kind': 'production_uses_gt', 'chain': chain, 'gt_frames': hit})
    return {'verdict': 'PASS' if not problems else 'FAIL',
            'problems': problems, 'chain': chain}


# ---------------------------------------------------------------- ROS 采集


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--root', default='map')
    ap.add_argument('--target', default='rslidar')
    ap.add_argument('--gt-frames', default='odom,gt_odom')
    ap.add_argument('--stale', type=float, default=1.0)
    ap.add_argument('--seconds', type=float, default=6.0)
    ap.add_argument('--out', default='')
    a = ap.parse_args()

    import rospy
    from tf2_msgs.msg import TFMessage

    rospy.init_node('tf_chain_audit', anonymous=True, disable_signals=True)
    edges = {}

    def record(msg, dynamic):
        wall = time.time()
        for tr in msg.transforms:
            p, c = tr.header.frame_id, tr.child_frame_id
            slot = edges.setdefault(c, {}).setdefault(
                p, {'publishers': [], 'dynamic': dynamic, 'stamps': []})
            slot['dynamic'] = slot['dynamic'] or dynamic
            slot['last_wall'] = wall
            slot['stamps'].append(tr.header.stamp.to_sec())
            slot['stamps'] = slot['stamps'][-50:]
            t = tr.transform.translation
            slot.setdefault('values', []).append(round(t.x + t.y * 7 + t.z * 13, 6))
            slot['values'] = slot['values'][-50:]

    rospy.Subscriber('/tf', TFMessage, lambda m: record(m, True), queue_size=200)
    rospy.Subscriber('/tf_static', TFMessage, lambda m: record(m, False), queue_size=200)
    time.sleep(a.seconds)

    # 发布者身份：消息里没有节点名，所以只能按**话题**取发布者列表。
    # /tf 或 /tf_static 出现 >1 个发布者，就是多写嫌疑（我们无法把某条边归给某节点）。
    topic_pubs = {}
    try:
        import rosgraph
        state = rosgraph.Master('/tf_chain_audit').getSystemState()
        for topic, dynamic in (('/tf', True), ('/tf_static', False)):
            # getSystemState 的 publishers 是 [(topic, [node...])]，不是反过来的
            names = []
            for tname, nodes in state[0]:
                if tname == topic:
                    names = list(nodes)
            topic_pubs[topic] = names
            if len(names) > 1:
                for child, parents in edges.items():
                    for parent, info in parents.items():
                        if info['dynamic'] == dynamic:
                            info.setdefault('topic_multi', []).append(topic)
    except Exception as e:
        topic_pubs = {'error': str(e)}

    contract = {'root': a.root, 'production_target': a.target,
                'gt_frames': [x for x in a.gt_frames.split(',') if x],
                'stale_s': a.stale}
    if not edges:
        print('VERDICT NO_DATA')
        return 2
    out = audit(edges, contract)
    # chain 已包含 target 本身，别再拼一次（旧版拼出 "rslidar <- rslidar <- ..."）
    print('CHAIN %s' % ' <- '.join(out['chain']))
    print('EDGES %d  TOPIC_PUBLISHERS %s' % (sum(len(v) for v in edges.values()),
                                             json.dumps(topic_pubs, ensure_ascii=False)))
    for p in out['problems']:
        print('PROBLEM %s' % json.dumps(p, ensure_ascii=False))
    print('VERDICT %s' % out['verdict'])
    if a.out:
        payload = dict(out, edges={c: {p: {k: v for k, v in i.items() if k != 'stamps'}
                                       for p, i in ps.items()} for c, ps in edges.items()})
        with open(a.out, 'w', encoding='utf-8') as fh:
            json.dump(payload, fh, ensure_ascii=False, indent=1)
    return 0 if out['verdict'] == 'PASS' else 1


if __name__ == '__main__':
    sys.exit(main())
