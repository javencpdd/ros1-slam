"""四个受支持图里的组件，其 `launch.type` 必须能被 roslaunch 找到（顶层可执行名）。

背景（2026-11-23 F4）：roslib.find_node 只搜 `<pkg>/scripts` 与 `devel/lib/<pkg>` 的**顶层**，
不递归子目录。manifest 若写 `planner/topic_node.py` 这类带 `/` 的 type，roslaunch 会报
`Cannot locate node of type ...`，节点静默不起——静态 plan 仍 ok:true。

本测试不跑 ROS：只要 type 带 `/` 就判失败，因为那种写法在 roslaunch 下必然找不到。
"""
import json
import sys
import unittest
from pathlib import Path

REPO = Path(__file__).resolve().parents[6]
V8P = REPO / 'ws/src/90_platform/v8_platform'
FRAMEWORK = V8P / 'framework'
MANIFESTS = FRAMEWORK / 'fixtures/manifests'
# 本轮只收口**可运行的 support 图**（FAR 依赖缺失，其图保持 INVALID，见证据 §8.4）。
# FAR 图与其独有的 local_native / path_goal3d 等仍带子目录 type，属同类待办，不在本轮范围。
# 本轮收口目标＝**可运行的 Gazebo support 图**。lite3_bt 图里另有 4 个组件
# （ultrasonic_guard / dynamic_obstacles / gait_guard / carrier_lite3）仍写子目录 type，
# 属同类待办，不属本轮（真机链）范围，见证据 §8.6。
GRAPHS = [
    FRAMEWORK / 'fixtures/graphs/graph_3d_gazebo_bt.yaml',
]

sys.path.insert(0, str(FRAMEWORK))
import config                                    # noqa: E402


def manifests():
    out = {}
    for p in MANIFESTS.glob('*.json'):
        d = json.loads(p.read_text())
        out[d['id']] = d
    return out


class NodeEntrypointTest(unittest.TestCase):

    def test_supported_graphs_use_top_level_node_types(self):
        cat = manifests()
        for path in GRAPHS:
            graph = config.load_graph(str(path))
            for iid, inst in graph['instances'].items():
                man = cat[inst['component']]
                launch = man.get('launch', {})
                if launch.get('kind') == 'include':
                    continue          # include 由 launch 文件负责，不经 type 查找
                node_type = launch.get('type', '')
                with self.subTest(graph=path.name, instance=iid):
                    self.assertNotIn(
                        '/', node_type,
                        '%s 的 %s（%s）type=%r 带路径分隔符；roslaunch 只查顶层，必然 Cannot locate'
                        % (path.name, iid, inst['component'], node_type))

    def test_shims_exist_for_the_three_algorithm_nodes(self):
        scripts = REPO / 'ws/src/80_algorithm/v8_algorithm/scripts'
        for name in ('run_topic_node.py', 'run_local_ros_node.py', 'run_cmd3d_mux.py'):
            with self.subTest(shim=name):
                p = scripts / name
                self.assertTrue(p.is_file(), '缺少顶层 shim %s' % name)
                self.assertTrue(p.stat().st_mode & 0o111, '%s 不可执行' % name)


if __name__ == '__main__':
    unittest.main()
