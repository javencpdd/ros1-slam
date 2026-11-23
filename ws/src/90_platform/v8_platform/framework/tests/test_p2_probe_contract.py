"""P2 运行探针与图契约的一致性：起点是「外部输入」，必须由操作员给。

2026-11-23 的坑：support 图把 `global.start` 声明为 `external: source: operator`（不是 carrier 的边），
而 `topic_node.replan()` 在 `start is None` 时只发 `E_PLAN_NOT_READY`，不会自己找起点。
探针因此必须先发起点，否则整条链停在地图之后。
"""
import json
import sys
import unittest
from pathlib import Path

REPO = Path(__file__).resolve().parents[6]
V8P = REPO / 'ws/src/90_platform/v8_platform'
FRAMEWORK = V8P / 'framework'
PROBE = REPO / 'scripts/verify/p2_runtime_probe.py'
SUPPORT_GRAPHS = [
    V8P / 'configs/lite3/graph_3d_lite3_bt.yaml',
    FRAMEWORK / 'fixtures/graphs/graph_3d_gazebo_bt.yaml',
]

sys.path.insert(0, str(FRAMEWORK))
import config                                   # noqa: E402


class ProbeContractTest(unittest.TestCase):

    def test_support_graphs_declare_global_start_as_external(self):
        for path in SUPPORT_GRAPHS:
            graph = config.load_graph(str(path))
            externals = {'%s.%s' % (e['to'].rsplit('.', 1)[0], e['to'].rsplit('.', 1)[1])
                         for e in graph.get('external', [])}
            with self.subTest(graph=path.name):
                self.assertIn('global.start', externals,
                              '%s 未把 global.start 声明为外部输入' % path.name)

    def test_probe_publishes_start_on_the_declared_topic(self):
        text = PROBE.read_text()
        self.assertIn("'--start-topic'", text, '探针没有起点话题参数')
        self.assertIn("default='/v8/3d/global/start'", text,
                      '探针默认起点话题与图契约不一致（应为 /v8/3d/global/start）')
        self.assertIn('start_pub.publish', text, '探针没有实际发布起点')

    def test_planner_requires_start_before_planning(self):
        src = (REPO / 'ws/src/80_algorithm/v8_algorithm/planner/topic_node.py').read_text()
        self.assertIn("state['start'] is None", src)
        self.assertIn('E_PLAN_NOT_READY', src)


if __name__ == '__main__':
    unittest.main()
