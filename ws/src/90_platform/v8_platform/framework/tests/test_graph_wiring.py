"""图装配的运行接线回归测试（基于**实际渲染出来的 launch**）。

判据不是"图里有边"，也不是"也许会生成 remap"，而是：
1. 用框架自己的 `graph.check` + `render.render_launch` 渲染出真实 launch；
2. 从渲染结果读出每个节点/包含的 **param/arg** 与 **remap**；
3. 有效话题 = `param.get(<port>_topic)`（无则 manifest 默认），再套一层 remap（from 命中则替换）；
4. 生产者有效话题必须等于消费者有效话题，否则视为断线。

这样能抓住两类真实事故（2026-11-23 都发生过）：
- include 实例不生成 remap，两侧参数不同名 ⇒ 目标无人消费；
- 节点改了参数，而 remap 的 from 取的是 manifest 默认名 ⇒ remap 空转、连线静默断。
"""
import json
import sys
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


REPO = Path(__file__).resolve().parents[6]
V8P = REPO / 'ws/src/90_platform/v8_platform'
FRAMEWORK = V8P / 'framework'
MANIFESTS = FRAMEWORK / 'fixtures/manifests'
GRAPHS = sorted((V8P / 'configs/lite3').glob('*.yaml')) + \
         sorted((FRAMEWORK / 'fixtures/graphs').glob('*.yaml'))

sys.path.insert(0, str(FRAMEWORK))
from config import load_components, load_graph            # noqa: E402
import graph as graph_mod                                  # noqa: E402
import render as render_mod                                # noqa: E402

_MANIFESTS = {}


def manifest_index():
    if not _MANIFESTS:
        for p in MANIFESTS.glob('*.json'):
            d = json.loads(p.read_text())
            _MANIFESTS[d['id']] = d
    return _MANIFESTS


def port_default(manifest, direction, name):
    for p in manifest.get('ports', {}).get(direction, []):
        if p.get('name') == name:
            return p.get('topic')
    return None


def render_graph(graph):
    """渲染 graph 字典并解析成 (nodes, includes)。"""
    catalog = load_components(str(MANIFESTS))
    report = graph_mod.check(graph, catalog)
    xml = render_mod.render_launch(graph, catalog, report)
    root = ET.fromstring(xml)
    nodes, includes = {}, {}
    for el in root:
        if el.tag == 'node':
            nodes[el.get('name')] = {
                'params': {p.get('name'): p.get('value') for p in el.findall('param')},
                'remaps': [(r.get('from'), r.get('to')) for r in el.findall('remap')],
            }
        elif el.tag == 'include':
            includes[el.get('file')] = {
                a.get('name'): a.get('value') for a in el.findall('arg')}
    return nodes, includes


def load(path):
    """用框架自己的载入器（instances 会被规范成 id→spec 的字典）。"""
    return load_graph(str(path))


def inst_of(graph, iid):
    return graph['instances'][iid]


def element_for(graph, nodes, includes, iid):
    comp = inst_of(graph, iid)['component']
    if iid in nodes:
        return nodes[iid]['params'], nodes[iid]['remaps']
    launch = manifest_index()[comp].get('launch', {})
    for path, args in includes.items():
        if launch.get('launch_file') and launch['launch_file'] in path:
            return args, []
    raise AssertionError('渲染结果里找不到实例 %s（%s）' % (iid, comp))


def effective(graph, nodes, includes, iid, port, direction):
    params, remaps = element_for(graph, nodes, includes, iid)
    default = port_default(manifest_index()[inst_of(graph, iid)['component']],
                           direction, port)
    topic = params.get(port + '_topic', default)
    for frm, to in remaps:
        if frm == topic:
            return to
    return topic


def problems(graph, nodes=None, includes=None):
    """返回该图的接线问题；传入被篡改的 nodes 可做反例。"""
    if nodes is None:
        nodes, includes = render_graph(graph)
    ids = list(graph['instances'])
    bad = []

    def need(name, sid, sp, sd, did, dp, dd):
        a = effective(graph, nodes, includes, sid, sp, sd)
        b = effective(graph, nodes, includes, did, dp, dd)
        if a != b:
            bad.append('%s: %s.%s 有效发到 %s，而 %s.%s 有效读 %s'
                       % (name, sid, sp, a, did, dp, b))

    if 'bt' in ids and 'global' in ids:
        need('目标请求', 'bt', 'planner_goal', 'outputs', 'global', 'goal', 'inputs')
        need('路径返回', 'global', 'path', 'outputs', 'bt', 'path', 'inputs')
        need('状态返回', 'global', 'status', 'outputs', 'bt', 'path_status', 'inputs')
    if 'global' in ids and 'path_goal' in ids:
        need('路径进 path_goal', 'global', 'path', 'outputs', 'path_goal', 'path', 'inputs')
    if 'path_goal' in ids and 'local' in ids:
        need('局部目标', 'path_goal', 'goal', 'outputs', 'local', 'goal', 'inputs')

    if 'local' in ids:
        target = effective(graph, nodes, includes, 'local', 'goal', 'inputs')
        writers = [w for w, port in (('bt', 'out_goal'), ('path_goal', 'goal'))
                   if w in ids and effective(graph, nodes, includes, w, port, 'outputs') == target]
        if len(writers) > 1:
            bad.append('局部目标双写者：%s 都发 %s' % (writers, target))
        if not writers:
            bad.append('局部目标无写者：%s' % target)
    return bad


class GraphWiringTest(unittest.TestCase):

    def test_all_graphs_render_connected(self):
        checked = 0
        for path in GRAPHS:
            if 'pipeline_graph/v1' not in path.read_text(errors='ignore'):
                continue           # 只测装配图，跳过其它 schema 的样例
            try:
                graph = load(path)
            except Exception:      # noqa: BLE001 — 非装配图样例（schema 不同/故意重复键）
                continue
            ids = list(graph.get('instances', {}))
            if 'bt' not in ids or 'global' not in ids:
                continue
            with self.subTest(graph=path.name):
                self.assertEqual([], problems(graph), path.name)
            checked += 1
        self.assertGreaterEqual(checked, 4, '覆盖的图太少：%d' % checked)

    def _far(self, mutate):
        graph = load(V8P / 'configs/lite3/graph_3d_lite3_far.yaml')
        mutate(graph)
        return graph

    def test_detects_broken_goal(self):
        def m(g):
            inst_of(g, 'global')['params']['goal_topic'] = '/v8/global/goal'
        self.assertTrue(problems(self._far(m)), '改错 goal 未被发现')

    def _mutate_then_drop(self, key, want_from):
        """先改一侧参数（此时应靠 remap 接上），再删掉那条 remap（此时必须报断）。"""
        graph = load(V8P / 'configs/lite3/graph_3d_lite3_far.yaml')
        inst_of(graph, 'bt')['params'][key] = want_from
        nodes, includes = render_graph(graph)
        self.assertEqual([], problems(graph, nodes, includes),
                         '%s 改动后 remap 未能接上（渲染器补偿失效）' % key)
        bt = nodes['bt']
        before = len(bt['remaps'])
        bt['remaps'] = [(f, t) for f, t in bt['remaps'] if f != want_from]
        self.assertLess(len(bt['remaps']), before, '反例前提不成立：没删到 %s 的 remap' % want_from)
        self.assertTrue(problems(graph, nodes, includes),
                        '删掉 %s 的 remap 后仍未报错' % want_from)

    def test_detects_broken_path(self):
        self._mutate_then_drop('path_topic', '/v8/3d/global/path')

    def test_detects_broken_status(self):
        self._mutate_then_drop('path_status_topic', '/v8/3d/global/status')

    def test_detects_double_writer(self):
        def m(g):
            inst_of(g, 'bt')['params']['out_goal_topic'] = '/v8/local/goal'
        self.assertTrue(problems(self._far(m)), '双写者未被发现')

    def test_detects_missing_required_remap(self):
        """删掉 path_goal 那条必要 remap，判据必须翻转（防止退化成"可能生成"）。"""
        graph = load(V8P / 'configs/lite3/graph_3d_lite3_far.yaml')
        nodes, includes = render_graph(graph)
        self.assertEqual([], problems(graph, nodes, includes))
        pg = nodes['path_goal']
        before = len(pg['remaps'])
        pg['remaps'] = [(f, t) for f, t in pg['remaps']
                        if not (f.endswith('/path') and t == '/global_path')]
        self.assertLess(len(pg['remaps']), before, '反例前提不成立：没删到那条 remap')
        self.assertTrue(problems(graph, nodes, includes),
                        '删掉必要 remap 后仍未报错 ⇒ 判据只看"可能有 remap"')


if __name__ == '__main__':
    unittest.main()
