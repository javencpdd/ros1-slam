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


def node_alive(name):
    rc, out = run(['rosnode', 'ping', name], timeout=4)
    return 'reply' in out.lower()


def topic_alive(topic, window):
    rc, out = run(['timeout', str(int(window)), 'rostopic', 'echo', '-n1', topic], timeout=window + 2)
    return rc == 0 and len(out.strip()) > 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('plan')
    ap.add_argument('--window', type=float, default=6.0)
    ap.add_argument('--json-out', default='')
    a = ap.parse_args()

    plan = json.load(open(a.plan, encoding='utf-8'))
    instances = plan.get('instances') or []
    nodes = node_list()

    results, ok_all = [], True
    for inst in instances:
        iid = inst.get('id')
        launch = inst.get('launch') or {}
        if launch.get('kind') == 'include':
            continue                      # include 型由被嵌 launch 自己保证
        ns = inst.get('namespace') or ''
        want = ns.rstrip('/') + '/' + iid if ns else '/' + iid
        hit = [n for n in nodes if n == want or n.endswith('/' + iid)]
        rec = {'instance': iid, 'expect': want, 'found': sorted(hit)[:3]}
        if not hit:
            rec.update(ok=False, why='节点不在册')
        else:
            live = [n for n in hit if node_alive(n)]
            if not live:
                rec.update(ok=False, why='在册但 ping 无响应（残留注册）')
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
            alive = topic_alive(t, a.window)
            topic_rec.append({'instance': iid, 'topic': t, 'ok': alive})
            ok_all = ok_all and alive

    for r in results:
        print('NODE %-10s ok=%s %s' % (r['instance'], r['ok'], r['why']))
    for r in topic_rec:
        print('TOPIC %-10s %-32s ok=%s' % (r['instance'], r['topic'], r['ok']))
    print('VERDICT %s nodes=%d topics=%d' % ('PASS' if ok_all else 'FAIL', len(results), len(topic_rec)))

    if a.json_out:
        json.dump({'ok': ok_all, 'nodes': results, 'topics': topic_rec},
                  open(a.json_out, 'w', encoding='utf-8'), ensure_ascii=False, indent=1)
    return 0 if ok_all else 1


if __name__ == '__main__':
    sys.exit(main())
