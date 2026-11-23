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
SUPPORTED = [
    V8P / 'configs/lite3/graph_3d_lite3_bt.yaml',
    V8P / 'configs/lite3/graph_3d_lite3_far.yaml',
    FRAMEWORK / 'fixtures/graphs/graph_3d_gazebo_bt.yaml',
    FRAMEWORK / 'fixtures/graphs/graph_3d_gazebo_far.yaml',
]

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
    key = render_mod._name(iid)          # 渲染时实例 ID 的点号会转成下划线
    if key in nodes:
        return nodes[key]['params'], nodes[key]['remaps']
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
        writers = []
        for iid in ids:
            outs = {p.get('name') for p in
                    manifest_index()[inst_of(graph, iid)['component']]
                    .get('ports', {}).get('outputs', [])}
            for port in ('out_goal', 'goal'):
                if port in outs and effective(graph, nodes, includes, iid, port,
                                              'outputs') == target:
                    writers.append(iid)
                    break
        if len(writers) > 1:
            bad.append('局部目标双写者：%s 都发 %s' % (writers, target))
        if not writers:
            bad.append('局部目标无写者：%s' % target)
    return bad


class GraphWiringTest(unittest.TestCase):

    def test_supported_graphs_render_connected(self):
        """四张受支持图逐一加载→校验→渲染；任何一步失败即 FAIL，不跳过。"""
        for path in SUPPORTED:
            with self.subTest(graph=path.name):
                self.assertTrue(path.exists(), '受支持图缺失：%s' % path)
                graph = load(path)                      # 失败＝测试失败（无 try/except）
                ids = list(graph.get('instances', {}))
                self.assertIn('bt', ids, '%s 缺少 bt 实例' % path.name)
                self.assertIn('global', ids, '%s 缺少 global 实例' % path.name)
                nodes, includes = render_graph(graph)   # 校验或渲染失败同样 FAIL
                self.assertTrue(nodes, '%s 渲染后没有任何节点' % path.name)
                self.assertEqual([], problems(graph, nodes, includes), path.name)

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

    def test_dotted_instance_id_keeps_port_name(self):
        """INSTANCE_ID 允许点号（config.py:21）；切端口必须按最后一个点号。

        用 bt→bt.a 改一张真图：若渲染器用 split('.', 1)，端口会被切成 a.out_goal，
        取不到参数就回落到 manifest 默认名 ⇒ local 被 remap 到 /v8/bt/goal_request。
        """
        graph = load(V8P / 'configs/lite3/graph_3d_lite3_bt.yaml')
        inst = graph['instances'].pop('bt')
        graph['instances']['bt.a'] = inst
        for edge in graph.get('edges', []) + graph.get('external', []):
            for key in ('from', 'to'):
                if key in edge and edge[key].startswith('bt.'):
                    edge[key] = 'bt.a.' + edge[key][len('bt.'):]
        nodes, includes = render_graph(graph)          # check/render 必须接受带点实例 ID
        self.assertIn(render_mod._name('bt.a'), nodes, '带点实例 ID 未出现在渲染结果里')
        goal_remaps = [r for r in nodes['local']['remaps'] if r[0] == '/v8/local/goal']
        self.assertEqual([('/v8/local/goal', '/v8/local/goal')], goal_remaps,
                         '带点实例 ID 下局部目标被 remap 到别处 ⇒ 端口名切错了')
        self.assertEqual([], problems(graph, nodes, includes))

    def test_known_param_name_exceptions_do_not_drift(self):
        """渲染器只认 `<port>_topic`；已知例外若被改值，remap 会按默认名空转。"""
        local_man = json.loads((MANIFESTS / 'manifest_local_native_raw.json').read_text())
        defaults = {p['name']: p.get('topic')
                    for d in ('inputs', 'outputs') for p in local_man['ports'][d]}
        for path in SUPPORTED:
            graph = load(path)
            params = graph['instances']['local'].get('params') or {}
            with self.subTest(graph=path.name):
                if 'twist_topic' in params:
                    self.assertEqual(defaults['cmd_twist'], params['twist_topic'],
                                     '%s: local 的 cmd_twist 走了 twist_topic（非 <port>_topic），'
                                     '改值前须在 render.py 增加端口→参数名映射' % path.name)
                if 'command_topic' in params:
                    self.assertEqual(defaults['cmd'], params['command_topic'],
                                     '%s: local 的 cmd 走了 command_topic，同上' % path.name)

    def test_mux_output_not_confused_with_its_input(self):
        """cmd3d_mux 的端口 `cmd` 在输入侧读 cmd_topic、输出侧读 output_topic。

        2026-11-23 实测：混用会给载体生成 `/cmd_vel → /v8/local/cmd_vel_3d` 的错误 remap，
        载体订阅了 TwistStamped 却声明 Twist ⇒ 消息被静默丢弃、车不动。
        """
        graph = load(V8P / 'framework/fixtures/graphs/graph_3d_gazebo_bt.yaml')
        nodes, includes = render_graph(graph)
        carrier = nodes['carrier']
        self.assertIn(('/cmd_vel', '/cmd_vel'), carrier['remaps'],
                      '载体的 cmd 应 remap 到 mux 输出 /cmd_vel')
        self.assertNotIn('/v8/local/cmd_vel_3d',
                         [t for _, t in carrier['remaps']],
                         '载体被错误 remap 到 mux 的输入话题')
        self.assertEqual([], problems(graph, nodes, includes))


if __name__ == '__main__':
    unittest.main()
