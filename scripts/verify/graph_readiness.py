#!/usr/bin/env python3
"""Graph 就绪判定：从"数节点个数"升级为"身份 + 存活 + 关键接口活数据"。

背景（2026-11-27）：只数 `rosnode list | grep ^/v8` 会把
  · 名字对但进程已死的残留注册 当成就绪；
  · 少一个实例但别的实例多起来 也算够数；
  · 全部节点在册、但关键话题一条数据没有（loc 一死，local 还在但永远没输入）。
实测就是这么被误导的：loc 因 KeyError 退出后 p4_nav_hdl 清掉整组，
local/BT "没有状态"被当成自己崩了，其实是连带清理。

判定三条，缺一即 FAIL：
  1) 身份：`plan.json` 里**每个非 include 实例**都要在 `rosnode list` 里找到对应节点；
  2) 存活：`rosnode ping` 有响应（不看注册残留）；
  3) 活数据：本表列出的关键接口在 N 秒窗内至少有 1 条消息。

复用既有产物（plan.json / rosnode / rostopic），不新建平行系统。

用法：graph_readiness.py <plan.json> [--window 6] [--json-out PATH]
rc：0 全过；1 有失败。stdout 逐条打印，便于落证据。
"""
import argparse
import json
import os
import subprocess
import sys
import time

# 关键接口表：instance id -> (node 后缀匹配, [必须活的话题])
KEY_TOPICS = {
    'loc': (['/v8/3d/loc'], ['/v8/local/pose', '/v8/local/localization_ok']),
    'local': (['/v8/3d/local'], ['/v8/local/status']),
    'bt': (['/v8/bt/bt'], ['/v8/bt/status']),
    'map': (['/v8/3d/map'], ['/v8_support_map']),
}


def run(cmd, timeout=8):
    try:
        p = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        return p.returncode, p.stdout
    except Exception as e:                      # 超时/缺失都当"拿不到"
        return 99, str(e)


def node_list():
    rc, out = run(['rosnode', 'list'])
    return set(x.strip() for x in out.splitlines() if x.strip().startswith('/'))


def node_alive(name, tries=3):
    """有限次 ping + 看返回码：`rosnode ping` 对死节点 rc≠0，不能只靠字符串。"""
    for i in range(tries):
        rc, out = run(['rosnode', 'ping', '-c', '1', name], timeout=5)
        if rc == 0 and 'reply' in out.lower():
            return True, 'ok'
        last = ('rc=%d' % rc)
    return False, 'ping 无响应（%s，%d 次）' % (last, tries)


def topic_publishers(topic):
    rc, out = run(['rostopic', 'info', topic], timeout=6)
    if rc != 0:
        return []
    block = out.split('Publishers:')[-1].split('Subscribers:')[0] if 'Publishers:' in out else ''
    return [ln.strip(' *') for ln in block.splitlines() if ln.strip().startswith('*')]


def topic_sustained(topic, window, need=3):
    """持续活数据：窗内至少 need 条，而不是"有一条就算活"。"""
    rc, out = run(['timeout', str(int(window)), 'rostopic', 'hz', topic], timeout=window + 2)
    last = [l for l in out.splitlines() if 'window:' in l]
    if not last:
        return False, '窗内无数据'
    try:
        n = int(last[-1].rsplit('window:', 1)[-1].strip())
    except Exception:
        return False, '无法解析窗口计数'
    return n >= need, 'window=%d（需≥%d）' % (n, need)


def topic_alive(topic, window):
    rc, out = run(['timeout', str(int(window)), 'rostopic', 'echo', '-n1', topic], timeout=window + 2)
    return rc == 0 and len(out.strip()) > 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('plan')
    ap.add_argument('--window', type=float, default=6.0)
    ap.add_argument('--json-out', default='')
    a = ap.parse_args()

    if not os.path.exists(a.plan):
        # 负控：没有 plan.json 就不能"默认就绪"，直接判 FAIL 且给不同 rc
        print('VERDICT FAIL 缺少 plan.json（无法做身份级判定）')
        return 2
    plan = json.load(open(a.plan, encoding='utf-8'))
    instances = plan.get('instances') or []
    nodes = node_list()

    results, ok_all = [], True
    for inst in instances:
        iid = inst.get('id')
        launch = inst.get('launch') or {}
        if launch.get('kind') == 'include':
            # include 型：不能直接跳过（内部节点无人核对）。至少要求**它的命名空间下
            # 有节点在册**，否则等于这一整块没起。
            ns = (inst.get('namespace') or '').rstrip('/')
            covered = [n for n in nodes if ns and n.startswith(ns + '/')]
            rec = {'instance': iid, 'expect': ns or '<include>', 'include': True,
                   'found': sorted(covered)[:3], 'ok': bool(covered),
                   'why': 'ok' if covered else 'include 命名空间下无任何节点在册'}
            results.append(rec)
            ok_all = ok_all and rec['ok']
            continue
        ns = inst.get('namespace') or ''
        want = ns.rstrip('/') + '/' + iid if ns else '/' + iid
        hit = [n for n in nodes if n == want or n.endswith('/' + iid)]
        rec = {'instance': iid, 'expect': want, 'found': sorted(hit)[:3]}
        if not hit:
            rec.update(ok=False, why='节点不在册')
        else:
            live, why = [], ''
            for n in hit:
                good, why = node_alive(n)
                if good:
                    live.append(n)
            if not live:
                rec.update(ok=False, why=why or '在册但 ping 无响应（残留注册）')
            else:
                rec.update(ok=True, why='ok', alive=sorted(live)[:3])
        results.append(rec)
        ok_all = ok_all and rec['ok']

    # 关键接口活数据
    topic_rec = []
    for iid, (_pat, topics) in KEY_TOPICS.items():
        if not any(r['instance'] == iid for r in results):
            continue
        for t in topics:
            sustained, why = topic_sustained(t, a.window)
            pubs = topic_publishers(t)
            # ★发布者归属★：有数据还不够，得确认数据来自**本实例**，而不是旁路/残留发布者。
            owns = any(any(p.endswith('/' + iid) or ('/' + iid + '/' + iid) in p for p in pubs))
            ok = sustained and bool(pubs) and owns
            topic_rec.append({'instance': iid, 'topic': t, 'ok': ok,
                              'sustained': sustained, 'why': why,
                              'publishers': pubs[:3], 'owns_publisher': owns})
            ok_all = ok_all and ok

    for r in results:
        print('NODE %-10s ok=%s %s' % (r['instance'], r['ok'], r['why']))
    for r in topic_rec:
        print('TOPIC %-10s %-32s ok=%s pubs=%s owns=%s %s'
              % (r['instance'], r['topic'], r['ok'], r.get('publishers'),
                 r.get('owns_publisher'), r.get('why', '')))
    print('VERDICT %s nodes=%d topics=%d' % ('PASS' if ok_all else 'FAIL', len(results), len(topic_rec)))

    if a.json_out:
        json.dump({'ok': ok_all, 'nodes': results, 'topics': topic_rec},
                  open(a.json_out, 'w', encoding='utf-8'), ensure_ascii=False, indent=1)
    return 0 if ok_all else 1


if __name__ == '__main__':
    sys.exit(main())
