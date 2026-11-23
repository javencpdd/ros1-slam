"""资产路径的两条回归：graph 的 tree_file（F2）与 app 的 simulation_declaration（F3）。

F2 背景：graph 里 `tree_file: ws/src/.../lite3_bt/trees/x.xml` 是**仓库相对**路径；节点以 devel 前缀
为 cwd 运行时解析不到，BT 起不来。修法是把「落在 catkin 包内的仓库相对路径」在渲染时改写成
`$(find pkg)/…`（roslaunch 解析），不改 graph、不写死绝对路径。

F3 背景：app 夹具引用的 `ws/src/95_simulation/...` 在分仓后已不存在，实际资产在
`subprojects/simulation/ws/src/...`；引用失效会让 app_runner 直接 `E_EXEC_BACKEND`。
"""
import sys
import unittest
from pathlib import Path

REPO = Path(__file__).resolve().parents[6]
V8P = REPO / 'ws/src/90_platform/v8_platform'
FRAMEWORK = V8P / 'framework'
APPS = FRAMEWORK / 'fixtures' / 'apps'

sys.path.insert(0, str(FRAMEWORK))
sys.path.insert(0, str(FRAMEWORK / 'nodes'))
import render as render_mod                     # noqa: E402
import app_config                              # noqa: E402
from app_runner_prepare import resolve_sim_launch   # noqa: E402
import config                                  # noqa: E402

TREE = '$(find lite3_bt)/trees/navigate_with_planner.xml'
GRAPHS = [
    V8P / 'configs/lite3/graph_3d_lite3_bt.yaml',
    V8P / 'configs/lite3/graph_3d_lite3_far.yaml',
    FRAMEWORK / 'fixtures/graphs/graph_3d_gazebo_bt.yaml',
    FRAMEWORK / 'fixtures/graphs/graph_3d_gazebo_far.yaml',
]


class TreeFilePathTest(unittest.TestCase):

    def test_repo_relative_asset_becomes_find_macro(self):
        got = render_mod._pkg_rel('ws/src/90_platform/lite3_bt/trees/navigate_with_planner.xml')
        self.assertEqual(TREE, got)

    def test_non_asset_values_are_left_alone(self):
        for value in ('/abs/already.xml', '$(find x)/y.xml', 'plain_value', '/v8/bt/goal'):
            self.assertEqual(value, render_mod._pkg_rel(value))

    def test_missing_repo_relative_path_is_not_guessed(self):
        # 缺失路径必须原样返回（由消费端报错），不能猜成某个 $(find)
        self.assertEqual('ws/src/nowhere/missing.xml',
                         render_mod._pkg_rel('ws/src/nowhere/missing.xml'))

    def test_four_supported_graphs_render_find_macro(self):
        catalog = config.load_components(str(FRAMEWORK / 'fixtures/manifests'))
        import graph as graph_mod
        for path in GRAPHS:
            graph = config.load_graph(str(path))
            report = graph_mod.check(graph, catalog)
            launch = render_mod.render_launch(graph, catalog, report)
            self.assertIn('name="tree_file" value="%s"' % TREE.replace('"', '&quot;'),
                          launch, '%s 的 tree_file 未改写成 $(find …)' % path.name)
            self.assertNotIn('value="ws/src/90_platform/lite3_bt/trees/', launch,
                             '%s 仍保留仓库相对路径' % path.name)


class MapJsonTest(unittest.TestCase):
    """F5：graph 里的仓库相对地图路径必须能被解析；缺失必须明确失败。"""

    def setUp(self):
        import sys as _s
        _s.path.insert(0, str(FRAMEWORK / 'nodes'))
        import map_file_node
        self.mod = map_file_node

    def test_repo_relative_map_resolves_from_any_cwd(self):
        got = self.mod.resolve_map_path('data/maps/generated/nist_sensor_v2.json')
        self.assertTrue(Path(got).is_file())
        self.assertTrue(Path(got).is_absolute())

    def test_absolute_map_path_still_works(self):
        abs_path = str(REPO / 'data/maps/generated/nist_sensor_v2.json')
        self.assertEqual(abs_path, self.mod.resolve_map_path(abs_path))

    def test_missing_map_raises_instead_of_falling_back(self):
        with self.assertRaises(FileNotFoundError) as ctx:
            self.mod.resolve_map_path('data/maps/generated/does_not_exist.json')
        self.assertIn('map_json not found', str(ctx.exception))

    def test_graph_declares_the_same_asset(self):
        graph = config.load_yaml(V8P / 'framework/fixtures/graphs/graph_3d_gazebo_bt.yaml')[0]
        declared = graph['instances'][0]['params']['map_json'] if isinstance(
            graph['instances'], list) else graph['instances']['map']['params']['map_json']
        self.assertTrue((REPO / declared).is_file(),
                        'graph 声明的 %s 必须真实存在' % declared)


class SimulationDeclarationTest(unittest.TestCase):

    def test_active_apps_resolve_to_existing_assets(self):
        for app in sorted(APPS.glob('*.yaml')):
            raw, _ = config.load_yaml(app)
            with self.subTest(app=app.name):
                resolve_sim_launch(raw, REPO)   # 不抛异常即证明声明文件与后端都可解析

    def test_missing_asset_is_invalid_not_pass(self):
        raw = {'carrier': {'backend': 'gazebo',
                           'simulation_declaration': 'subprojects/simulation/nope.yaml'}}
        with self.assertRaises(config.FrameworkError) as ctx:
            resolve_sim_launch(raw, REPO)
        self.assertIn('E_EXEC_BACKEND', str(ctx.exception))

    def test_no_app_points_at_removed_simulation_tree(self):
        for app in sorted(APPS.glob('*.yaml')):
            text = app.read_text()
            with self.subTest(app=app.name):
                self.assertNotIn('ws/src/95_simulation', text,
                                 '%s 仍引用已拆分的 95_simulation 路径' % app.name)


if __name__ == '__main__':
    unittest.main()
