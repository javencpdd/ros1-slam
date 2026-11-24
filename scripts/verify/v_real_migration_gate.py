#!/usr/bin/env python3
"""真机迁移门禁（组合件）：TF 唯一性/非恒等 + 标定记录 + 阈值已确值。

为什么要有这一件（而不是只看 TF 门禁）：真机导航能不能开，是三个条件的合取——
① base_link→rslidar 这条 TF 边只有一个发布者且不是恒等；② 有标定记录且与现值对得上；
③ quality_min 不再是 0.0 占位。任一条不成立，运动入口就不该开（启动侧的硬阻断由
`real_preflight_guard.py` + launch 的 `required="true"` 执行，本件负责把状态摆出来并给
门禁串一个可复跑的读数）。

rc 语义：0 三条全过／5 未通过（列出每条的原因）／9 前置缺失（仓根、图或 launch 读不到）。

★本件只报不判★：未标定是当前真机常态（等 G8），若把它做成拦性成员，门禁串会恒红。
真机侧的"拦"在启动链上，不在提交链上。
"""
import argparse
import json
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import v_tf_extrinsic_gate as tfg  # noqa: E402

GRAPH = 'ws/src/90_platform/v8_platform/configs/lite3/graph_3d_lite3_bt.yaml'
NAV_LAUNCH = 'ws/src/70_bringup/lite3_bringup/launch/mode_navigation.launch'
ATT_DEFAULT = 'data/calib/rslidar_extrinsic.json'


def graph_quality_min(path):
    """从真机图里取 carrier 的 quality_min（简单行解析，避免为一件校验引 yaml 依赖）。"""
    if not os.path.isfile(path):
        return None
    for line in open(path, encoding='utf-8'):
        m = re.match(r'^\s+quality_min:\s*([0-9.eE+-]+)\s*$', line)
        if m:
            return float(m.group(1))
    return None


def launch_quality_min(path):
    if not os.path.isfile(path):
        return None
    body = open(path, encoding='utf-8').read()
    m = re.search(r'<arg name="quality_min" default="([0-9.eE+-]+)"', body)
    return float(m.group(1)) if m else None


def aggregate(tf_ok, att_ok, qmin_graph, qmin_launch):
    """三条合取：TF 边合格、标定记录在位且字段全、两处 quality_min 都已确值。

    单独抽出来是为了能用四条反控逐条证明"任一条不成立就整体不通过"——
    直接测 main() 只能验证"现在这一种状态"，改不动其中一条。
    """
    qmin_ok = (qmin_graph is not None and qmin_graph > 0.0
               and qmin_launch is not None and qmin_launch > 0.0)
    reasons = []
    if not tf_ok:
        reasons.append('TF')
    if not att_ok:
        reasons.append('attestation')
    if not qmin_ok:
        reasons.append('quality_min')
    return {'ok': not reasons, 'tf': bool(tf_ok), 'attestation': bool(att_ok),
            'quality_min': {'graph': qmin_graph, 'launch': qmin_launch},
            'failed': reasons}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--repo', default=os.environ.get('REALGATE_ROOT')
                    or os.path.dirname(os.path.dirname(HERE)))
    ap.add_argument('--edge', default='base_link:rslidar')
    ap.add_argument('--attestation', default=ATT_DEFAULT)
    ap.add_argument('--json', action='store_true')
    a = ap.parse_args()
    repo = os.path.abspath(a.repo)

    graph_p, launch_p = os.path.join(repo, GRAPH), os.path.join(repo, NAV_LAUNCH)
    if not os.path.isfile(graph_p) or not os.path.isfile(launch_p):
        print('前置缺失：图或真机 launch 不在位（%s / %s）' % (graph_p, launch_p))
        return 9

    roots = tfg.package_roots(repo)
    if not roots:
        print('前置缺失：ws/src 下找不到 package.xml')
        return 9

    parent, child = a.edge.split(':')
    entries = [os.path.join(repo, 'ws/src/70_bringup/lite3_bringup/launch/mode_navigation.launch'),
               os.path.join(repo, 'ws/src/70_bringup/lite3_bringup/launch/mode_localize.launch')]
    tf_rows = [tfg.audit(e, roots, (parent, child)) for e in entries if os.path.isfile(e)]
    tf_ok = all(ok for ok, _ in tf_rows)

    att_path = a.attestation if os.path.isabs(a.attestation) else os.path.join(repo, a.attestation)
    att = None
    if os.path.isfile(att_path):
        try:
            att = json.load(open(att_path, encoding='utf-8'))
        except ValueError:
            att = None
    att_ok = bool(att) and all(att.get(k) for k in ('xyz', 'measured_at', 'measured_by'))

    qmin_graph = graph_quality_min(graph_p)
    qmin_launch = launch_quality_min(launch_p)
    payload = aggregate(tf_ok, att_ok, qmin_graph, qmin_launch)
    reasons = []
    if 'TF' in payload['failed']:
        reasons.append('TF：%s' % '；'.join(d.get('reason', '?') for _, d in tf_rows))
    if 'attestation' in payload['failed']:
        reasons.append('标定记录：缺失或字段不全（%s）' % a.attestation)
    if 'quality_min' in payload['failed']:
        reasons.append('quality_min 仍是占位（图=%s，launch=%s）' % (qmin_graph, qmin_launch))
    ok = payload['ok']
    payload['reasons'] = reasons
    if a.json:
        print(json.dumps(payload, ensure_ascii=False))
    else:
        print('  TF 边唯一直且非恒等：%s' % ('是' if tf_ok else '否'))
        print('  标定记录在位且字段全：%s' % ('是' if att_ok else '否'))
        print('  quality_min 已确值：%s（图=%s，launch=%s）'
              % ('是' if 'quality_min' not in payload['failed'] else '否',
                 qmin_graph, qmin_launch))
        for r in reasons:
            print('  · %s' % r)
        print('REALGATE VERDICT: %s（真机导航准入三条 %d/3）'
              % ('PASS' if ok else 'FAIL', 3 - len(reasons)))
    return 0 if ok else 5


if __name__ == '__main__':
    sys.exit(main())
