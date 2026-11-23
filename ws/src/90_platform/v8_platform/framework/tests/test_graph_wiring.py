"""图装配的**运行接线**回归测试：目标请求、路径返回、状态返回、局部目标四条方向。

为什么需要它（2026-11-23 两次漏检的教训）：
1. `graph.check` / `cli.py plan` 只校「端口存在 + 消息类型」，**不校话题是否相等**，
   断线照样 `ok:true`；
2. `render.py` 生成的 `<remap>` 用的是 **manifest 默认端口**，且对 `kind: include` 的实例
   **直接跳过**。因此：
   - include 后端（FAR）不吃 remap ⇒ 生产/消费两侧的 graph 参数必须同名；
   - 普通节点的 remap `from` 是 manifest 默认名，**盖不住 graph 里改过的参数值** ⇒
     参数改过的节点，其有效订阅就是参数值本身，remap 不起作用。
3. 于是唯一可靠的判据是「**有效话题相等**」：有效话题 = graph 参数 `<port>_topic`（若有），
   否则取 manifest 该端口的默认 topic。

本测试按该判据检查四个方向 + 局部目标唯一写者，并用「内存里改错参数」做反例，
保证测试本身不会变成空控（只断言"有边"是最典型的空控）。
"""
import json
import unittest
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parents[6]
V8P = REPO / 'ws/src/90_platform/v8_platform'
GRAPHS = sorted((V8P / 'configs/lite3').glob('*.yaml')) + \
         sorted((V8P / 'framework/fixtures/graphs').glob('*.yaml'))
MANIFESTS = V8P / 'framework/fixtures/manifests'


def load_manifests():
    out = {}
    for p in MANIFESTS.glob('*.json'):
        d = json.loads(p.read_text())
        out[d['id']] = d
    return out


def port_spec(manifest, direction, name):
    for p in manifest.get('ports', {}).get(direction, []):
        if p.get('name') == name:
            return p
    return None


def effective_topic(graph, manifests, iid, port, direction):
    """有效话题：graph 参数 `<port>_topic` 优先，否则 manifest 该端口默认。"""
    inst = next(i for i in graph['instances'] if i['id'] == iid)
    params = inst.get('params') or {}
    key = port + '_topic'
    if key in params:
        return params[key]
    spec = port_spec(manifests[inst['component']], direction, port)
    return (spec or {}).get('topic')


def has_edge(graph, src, dst):
    return '%s -> %s' % (src, dst) in {'%s -> %s' % (e['from'], e['to'])
                                       for e in graph.get('edges', [])}


def violations(graph, manifests):
    """返回该图所有「声明了边但有效话题接不上」的问题。"""
    bad = []
    ids = [i['id'] for i in graph['instances']]

    def eff(iid, port, direction):
        return effective_topic(graph, manifests, iid, port, direction)

    def need(name, src, sp, dst, dp, sdirection):
        if not has_edge(graph, '%s.%s' % (src, sp), '%s.%s' % (dst, dp)):
            bad.append('%s: 缺少边 %s.%s → %s.%s' % (name, src, sp, dst, dp))
            return
        a, b = eff(src, sp, 'outputs'), eff(dst, dp, sdirection)
        if a == b:
            return
        # 话题不等时，只有「普通节点且该端口未改参数」才可能被 remap 桥接
        # （renderer 的 remap from = manifest 默认名；改了参数就盖不住）
        inst = next(i for i in graph['instances'] if i['id'] == dst)
        kind = manifests[inst['component']].get('launch', {}).get('kind')
        overridden = (dp + '_topic') in (inst.get('params') or {})
        if kind == 'include' or overridden:
            bad.append('%s: %s.%s 发 %s，而 %s.%s 读 %s（%s ⇒ remap 救不了）'
                       % (name, src, sp, a, dst, dp, b,
                          'include 实例不生成 remap' if kind == 'include' else '该节点用了参数覆盖'))

    if 'bt' in ids and 'global' in ids:
        need('目标请求', 'bt', 'planner_goal', 'global', 'goal', 'inputs')
        need('路径返回', 'global', 'path', 'bt', 'path', 'inputs')
        need('状态返回', 'global', 'status', 'bt', 'path_status', 'inputs')

    if 'global' in ids and 'path_goal' in ids:
        need('路径进 path_goal', 'global', 'path', 'path_goal', 'path', 'inputs')

    if 'path_goal' in ids and 'local' in ids:
        need('局部目标', 'path_goal', 'goal', 'local', 'goal', 'inputs')

    if 'local' in ids:
        target = eff('local', 'goal', 'inputs')
        writers = [who for who, port in (('bt', 'out_goal'), ('path_goal', 'goal'))
                   if who in ids and eff(who, port, 'outputs') == target]
        if len(writers) > 1:
            bad.append('局部目标双写者：%s 都写 %s' % (writers, target))
        if not writers:
            bad.append('局部目标无写者：%s 没有任何实例在写' % target)
    return bad


class GraphWiringTest(unittest.TestCase):

    def setUp(self):
        self.manifests = load_manifests()

    def test_all_graphs_are_wired_end_to_end(self):
        checked = 0
        for path in GRAPHS:
            graph = yaml.safe_load(path.read_text())
            ids = [i['id'] for i in graph.get('instances', [])]
            if 'bt' not in ids or 'global' not in ids:
                continue
            with self.subTest(graph=path.name):
                self.assertEqual([], violations(graph, self.manifests), path.name)
            checked += 1
        self.assertGreaterEqual(checked, 4, '覆盖的图太少：%d' % checked)

    def _mutated_far(self, setter):
        graph = yaml.safe_load((V8P / 'configs/lite3/graph_3d_lite3_far.yaml').read_text())
        setter(graph)
        return graph

    def test_detects_broken_goal(self):
        def s(g):
            glob = next(i for i in g['instances'] if i['id'] == 'global')
            glob['params']['goal_topic'] = '/v8/global/goal'
        self.assertTrue(violations(self._mutated_far(s), self.manifests), '改错 goal 未被发现')

    def test_detects_broken_path(self):
        def s(g):
            bt = next(i for i in g['instances'] if i['id'] == 'bt')
            bt['params']['path_topic'] = '/v8/3d/global/path'
        self.assertTrue(violations(self._mutated_far(s), self.manifests), '改错 path 未被发现')

    def test_detects_broken_status(self):
        def s(g):
            bt = next(i for i in g['instances'] if i['id'] == 'bt')
            bt['params']['path_status_topic'] = '/v8/3d/global/status'
        self.assertTrue(violations(self._mutated_far(s), self.manifests), '改错 status 未被发现')

    def test_detects_double_writer(self):
        def s(g):
            bt = next(i for i in g['instances'] if i['id'] == 'bt')
            bt['params']['out_goal_topic'] = '/v8/local/goal'   # FAR 版故意制造双写者
        self.assertTrue(violations(self._mutated_far(s), self.manifests), '双写者未被发现')

    def test_gate_exists_but_its_output_is_unused_by_the_3d_backends(self):
        """把 D2 的现状钉住：gate 在位（preflight 契约），但输出没人读。

        若将来真把 gate 输出接进执行链，本测试会失败——那时必须同步更新
        `bt_console.launch` 的边界说明与所有权声明，而不是让文档继续过期。
        """
        bt_launch = (REPO / 'ws/src/90_platform/lite3_bt/launch/bt_console.launch').read_text()
        self.assertIn('goal_gate.py', bt_launch, 'gate 节点被移除前须先改 real_preflight 契约')
        self.assertIn('output_topic" value="/v8/3d/global/goal"', bt_launch)

        pre = (REPO / 'ws/src/40_navigation/lite3_scan/verify/real_preflight.py').read_text()
        self.assertIn("'v8_goal_gate'", pre, 'gate 仍是真机预检的必需节点')

        trad = (REPO / 'ws/src/70_bringup/lite3_bringup/launch/mode_sim_bt.launch').read_text()
        self.assertIn('<arg name="goal_topic" value="/v8/global/goal"/>', trad,
                      '传统链后端读 /v8/global/goal，不是 gate 的输出')

        topic_node = (REPO / 'ws/src/80_algorithm/v8_algorithm/planner/topic_node.py').read_text()
        self.assertIn("get_param('~goal_topic', '/v8/global/goal')", topic_node)


if __name__ == '__main__':
    unittest.main()
