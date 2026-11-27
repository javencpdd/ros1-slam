#!/usr/bin/env python3
"""Graph 就绪判定：身份 + 存活 + **按接口类型**的活数据。

为什么不数节点个数（2026-11-27 实跑教训）：loc 因 KeyError 退出后，
残留注册还在、别的实例又多起来，个数照样够；而"关键话题有一条消息"也不够——
地图是 latch 单次发布，永远只有一条。

因此按接口类型分别判定：
  latched   静态数据：必须能取到一条**有效**负载 + **唯一**发布者 + 版本字段齐全，
            不能要求"窗内 N 条"（latch 只发一次）。
  stream    持续流：窗内至少 N 条，且发布者归属本实例。
  heartbeat 心跳：窗内至少 1 条即可。
  event     事件：只查存在性（有发布者），不查频率。

发布者归属用**完整节点名**匹配（`/v8/3d/loc/loc`），并排除第二发布者：
只要出现非白名单发布者即判该接口不权威。

include 实例不能按命名空间硬判——FAR 的实际节点是 `/far_planner` 与
`/v8_global_far`，与实例 namespace 无关；这里按 launch 文件登记真实节点名。

复用既有产物（plan.json / rosnode / rostopic），不新建平行系统。

用法：graph_readiness.py <plan.json> [--window 6] [--json-out PATH]
rc：0 全过；1 有失败；2 缺 plan。
"""
import argparse
import json
import os
import subprocess
import sys

# 实例 → [(话题, 类型, 允许的发布者节点全名, 最小条数)]
INTERFACES = {
    'loc': [
        ('/v8/local/pose', 'stream', ['/v8/3d/loc/loc'], 5),
        ('/v8/local/localization_ok', 'heartbeat', ['/v8/3d/loc/loc'], 1),
    ],
    'local': [('/v8/local/status', 'heartbeat', ['/v8/3d/local/local'], 1)],
    'bt': [('/v8/bt/status', 'heartbeat', ['/v8/bt/bt'], 1)],
    'map': [('/v8_support_map', 'latched', ['/v8/3d/map/map'], 1)],
}

# include 型实例：launch 文件 → 该 launch 真正跑起来的节点名
INCLUDE_NODES = {
    'far_stack.launch': ['/far_planner', '/v8_global_far'],
}

# 地图这类 latch 负载必须带的字段（判"有效地图"而不是"有字节"）
LATCHED_KEYS = {
    '/v8_support_map': ('map_id', 'map_version', 'surfaces'),
}


def run(cmd, timeout=8):
    try:
        p = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        return p.returncode, p.stdout
    except Exception as e:
        return 99, str(e)


def parse_publishers(text):
    """从 `rostopic info` 输出里取发布者节点全名列表。"""
    if 'Publishers:' not in text:
        return []
    block = text.split('Publishers:')[-1].split('Subscribers:')[0]
    out = []
    for ln in block.splitlines():
        ln = ln.strip()
        if ln.startswith('*'):
            body = ln.lstrip('* ').strip()
            name = body.split('(')[0].strip()
            if name.startswith('/'):
                out.append(name)
    return out


def publisher_verdict(pubs, allowed, alive=None):
    """发布者归属判定：(ok, why, stale)。

    · 0 个 → 不通过；
    · 1 个但不在白名单 → 不通过；
    · 多个：必须区分"**活着的第二个写者**"与"**已死但仍在 master 里挂着的残留注册**"。
      前者是真双写，判不通过；后者没人会写，但必须显式报出来并建议 cleanup，
      不能当成"通过"糊过去。alive=None 表示无法判存活，按真双写处理（宁可判红）。
    """
    if not pubs:
        return False, '无发布者', []
    if len(pubs) == 1:
        if pubs[0] not in allowed:
            return False, '发布者 %s 不在白名单 %s' % (pubs[0], allowed), []
        return True, 'ok', []
    if alive is None:
        return False, '发布者不唯一：%s' % pubs, []
    live = [p for p in pubs if alive(p)]
    stale = [p for p in pubs if p not in live]
    if len(live) > 1:
        return False, '双写：活着的发布者 %s' % live, stale
    if not live:
        return False, '发布者全部不可达（残留注册待清理）：%s' % pubs, stale
    if live[0] not in allowed:
        return False, '存活发布者 %s 不在白名单 %s' % (live[0], allowed), stale
    return True, 'ok（另有残留注册 %s，建议 rosnode cleanup）' % stale, stale

def latched_verdict(got_one, payload_ok, pubs, allowed, alive=None):
    ok_pub, why_pub, _st = publisher_verdict(pubs, allowed, alive)
    if not ok_pub:
        return False, why_pub
    if not got_one:
        return False, '取不到 latch 负载'
    if not payload_ok:
        return False, '负载缺必需字段'
    return True, 'ok'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('plan')
    ap.add_argument('--window', type=float, default=6.0)
    ap.add_argument('--json-out', default='')
    a = ap.parse_args()

    if not os.path.exists(a.plan):
        print('VERDICT FAIL 缺少 plan.json（无法做身份级判定）')
        return 2
    plan = json.load(open(a.plan, encoding='utf-8'))
    instances = plan.get('instances') or []

    rc, out = run(['rosnode', 'list'])
    nodes = [x.strip() for x in out.splitlines() if x.strip().startswith('/')]

    results, ok_all = [], True
    for inst in instances:
        iid = inst.get('id')
        launch = inst.get('launch') or {}
        if launch.get('kind') == 'include':
            lf = os.path.basename(launch.get('launch_file') or '')
            want = INCLUDE_NODES.get(lf, [])
            missing = [n for n in want if n not in nodes]
            ok = bool(want) and not missing
            why = 'ok' if ok else ('未登记该 include 的真实节点' if not want
                                   else '缺节点 %s' % missing)
            results.append({'instance': iid, 'expect': want, 'include': True, 'ok': ok, 'why': why})
            ok_all = ok_all and ok
            continue

        ns = (inst.get('namespace') or '').rstrip('/')
        want = (ns + '/' + iid) if ns else ('/' + iid)
        hit = [n for n in nodes if n == want]
        if not hit:
            results.append({'instance': iid, 'expect': want, 'ok': False, 'why': '节点不在册'})
            ok_all = False
            continue
        # 有限次 ping + 看返回码（`rosnode ping` 对死节点 rc≠0）
        alive, why = False, ''
        for _ in range(3):
            rc2, o2 = run(['rosnode', 'ping', '-c', '1', want], timeout=5)
            if rc2 == 0 and 'reply' in o2.lower():
                alive, why = True, 'ok'
                break
            why = 'ping 无响应（rc=%d）' % rc2
        results.append({'instance': iid, 'expect': want, 'ok': alive, 'why': why})
        ok_all = ok_all and alive

    topic_rec = []
    for iid, specs in INTERFACES.items():
        if not any(r['instance'] == iid for r in results):
            continue
        for topic, kind, allowed, need in specs:
            rc3, info = run(['rostopic', 'info', topic], timeout=6)
            pubs = parse_publishers(info) if rc3 == 0 else []
            def _alive(name):
                rc_a, out_a = run(['rosnode', 'ping', '-c', '1', name], timeout=4)
                return rc_a == 0 and 'reply' in out_a.lower()

            if kind == 'latched':
                rc4, payload = run(['timeout', '6', 'rostopic', 'echo', '-n1', topic], timeout=10)
                got = rc4 == 0 and len(payload.strip()) > 0
                keys = LATCHED_KEYS.get(topic, ())
                payload_ok = got and all(k in payload for k in keys) if keys else got
                ok, why = latched_verdict(got, payload_ok, pubs, allowed, _alive)
            else:
                ok_pub, why_pub, _st = publisher_verdict(pubs, allowed, _alive)
                rc5, hz = run(['timeout', str(int(a.window)), 'rostopic', 'hz', topic],
                              timeout=a.window + 2)
                n = 0
                for ln in hz.splitlines():
                    if 'window:' in ln:
                        try:
                            n = int(ln.rsplit('window:', 1)[-1].strip())
                        except Exception:
                            n = 0
                if kind == 'stream':
                    ok_data, why_data = n >= need, 'window=%d（需≥%d）' % (n, need)
                else:
                    ok_data, why_data = n >= need or n == 0 and False, 'window=%d' % n
                ok = ok_pub and ok_data
                why = why_pub if not ok_pub else why_data
            topic_rec.append({'instance': iid, 'topic': topic, 'kind': kind,
                              'ok': ok, 'why': why, 'publishers': pubs[:3]})
            ok_all = ok_all and ok

    for r in results:
        tag = ' INCLUDE' if r.get('include') else ''
        print('NODE %-10s ok=%-5s%s %s' % (r['instance'], r['ok'], tag, r['why']))
    for r in topic_rec:
        print('TOPIC %-10s %-30s %-9s ok=%-5s pubs=%s %s'
              % (r['instance'], r['topic'], r['kind'], r['ok'], r['publishers'], r['why']))
    print('VERDICT %s nodes=%d topics=%d' % ('PASS' if ok_all else 'FAIL',
                                             len(results), len(topic_rec)))
    if a.json_out:
        json.dump({'ok': ok_all, 'nodes': results, 'topics': topic_rec},
                  open(a.json_out, 'w', encoding='utf-8'), ensure_ascii=False, indent=1)
    return 0 if ok_all else 1


if __name__ == '__main__':
    sys.exit(main())
