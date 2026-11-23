"""图装配的「目标路由」回归测试：BT 请求必须真的到达全局后端。

背景（2026-11-23 实测）：`render.py` 对 `kind: include` 的实例**不生成 remap**
（`render_launch` 里 `continue` 跳过），所以当全局后端是 include（FAR）时，
`bt.planner_goal_topic` 与后端 `goal_topic` **必须同名**，否则目标请求没有任何
消费者——而 `cli.py plan` 只校消息类型，仍返回 ok:true，属静默断线。

本测试直接读 graph + manifest，不启动 ROS，专抓这一类"校验通过但接线断"的缺陷。
"""
import json
import unittest
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parents[6]  # …/tests→framework→v8_platform→90_platform→src→ws→repo
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


def port_default(manifest, direction, name):
    for p in manifest.get('ports', {}).get(direction, []):
        if p.get('name') == name:
            return p.get('topic')
    return None


class GoalRouteTest(unittest.TestCase):
    """凡有 bt→global 的目标边，请求话题必须与后端订阅话题一致。"""

    def test_every_graph_routes_goal_to_the_backend(self):
        manifests = load_manifests()
        checked = []
        for path in GRAPHS:
            graph = yaml.safe_load(path.read_text())
            instances = {i['id']: i for i in graph.get('instances', [])}
            if 'bt' not in instances or 'global' not in instances:
                continue
            bt, glob = instances['bt'], instances['global']
            bt_m = manifests[bt['component']]
            gl_m = manifests[glob['component']]
            bt_topic = (bt.get('params') or {}).get('planner_goal_topic') \
                or port_default(bt_m, 'outputs', 'planner_goal')
            gl_topic = (glob.get('params') or {}).get('goal_topic') \
                or port_default(gl_m, 'inputs', 'goal')
            kind = gl_m.get('launch', {}).get('kind')

            with self.subTest(graph=path.name):
                self.assertIsNotNone(bt_topic, 'bt.planner_goal_topic 未定义')
                self.assertIsNotNone(gl_topic, 'global.goal_topic 未定义')

                # 边必须存在：否则"请求→后端"这条链在图里根本没声明
                edges = ['%s -> %s' % (e['from'], e['to']) for e in graph.get('edges', [])]
                self.assertIn('bt.planner_goal -> global.goal', edges,
                              '%s 缺少 bt.planner_goal → global.goal 边' % path.name)

                if kind == 'include':
                    # include 实例不会拿到 remap ⇒ 只能靠话题同名接线
                    self.assertEqual(
                        bt_topic, gl_topic,
                        '%s: 后端是 include（不生成 remap），BT 发 %s 而后端读 %s ⇒ 目标请求无人消费'
                        % (path.name, bt_topic, gl_topic))
                checked.append(path.name)
        self.assertGreaterEqual(len(checked), 4, '目标路由检查覆盖的图太少：%r' % checked)


if __name__ == '__main__':
    unittest.main()
