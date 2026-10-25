"""V8-07 适配器契约面测试：消息级匹配、外部输入白名单、后端交换（A6）证据。

七件套真实组合（六件套＋apt GlobalPlanner——R647 新入册）应全接线闭合；
缺全局规划器＝dwa 的 nav_msgs/Path 无源（负例）；hdl⇄lite3 同接口换后端（A6 面）。
"""
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / 'composer'))
import composer as comp  # noqa: E402
import io_match  # noqa: E402

MDIR = HERE.parent / 'registry' / 'data' / 'manifests'
EIGHT = ['v8.odometry.faster_lio', 'v8.global_localization.lite3', 'v8.map_server.apt',
         'v8.representation.costmap_layers', 'v8.local_planner.scan_planner',
         'v8.global_planner.apt', 'v8.controller.dwa_adapter', 'v8.safety_arbiter.lite3']


class IoMatchTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.store = comp.load(MDIR)

    def test_msg_type_parse(self):
        self.assertEqual(io_match.msg_type('nav_msgs/Path（全局路径）'),
                         ('nav_msgs/Path', '全局路径'))
        self.assertEqual(io_match.msg_type('sensor_msgs/PointCloud2'),
                         ('sensor_msgs/PointCloud2', ''))

    def test_seven_stack_wiring_closed(self):
        wires = io_match.wiring(self.store, EIGHT)
        by_input = {(w['component'], w['input']): w for w in wires}
        # dwa 的全局路径 ← apt GlobalPlanner（R647 缺口闭合的实证）
        dwa_path = by_input[('v8.controller.dwa_adapter', 'nav_msgs/Path')]
        self.assertEqual(dwa_path['providers'], ['v8.global_planner.apt'])
        # scan 的 body_pose Odometry ← faster_lio
        scan_odom = by_input[('v8.local_planner.scan_planner', 'nav_msgs/Odometry')]
        self.assertIn('v8.odometry.faster_lio', scan_odom['providers'])
        # 点云输入有真实生产者（faster_lio 配准云）——报告为可行接线
        scan_cloud = by_input[('v8.local_planner.scan_planner', 'sensor_msgs/PointCloud2')]
        self.assertIn('v8.odometry.faster_lio', scan_cloud['providers'])
        # mode3 参考路径可由全局规划器供给（比外部白名单更实的接线）
        scan_ref = by_input[('v8.local_planner.scan_planner', 'nav_msgs/Path')]
        self.assertIn('v8.global_planner.apt', scan_ref['providers'])

    def test_missing_global_planner_negative(self):
        """七件套（无全局规划器）＝dwa 的 nav_msgs/Path 无源——消息级匹配的真负例。"""
        six = [cid for cid in EIGHT if cid != 'v8.global_planner.apt']
        with self.assertRaises(io_match.AdapterError) as caught:
            io_match.wiring(self.store, six)
        self.assertEqual(caught.exception.code, 'E_ADAPTER_NO_SOURCE')
        self.assertIn('nav_msgs/Path', caught.exception.detail)

    def test_localization_backend_swap_a6(self):
        """A6 面：hdl⇄lite3 同任务接口换后端——接线结构同形、锁摘要仅因该组件而异。"""
        with_hdl = [cid if cid != 'v8.global_localization.lite3'
                    else 'v8.global_localization.hdl' for cid in EIGHT]
        wires_lite = io_match.wiring(self.store, EIGHT)
        wires_hdl = io_match.wiring(self.store, with_hdl)
        norm = lambda s: (s.replace('v8.global_localization.hdl', 'LOC')
                           .replace('v8.global_localization.lite3', 'LOC'))
        strip = lambda ws: sorted((norm(w['component']), w['input'],
                                   tuple(sorted(norm(x) for x in w.get('providers', []))))
                                  for w in ws)
        self.assertEqual(strip(wires_lite), strip(wires_hdl))  # 接线同形
        self.assertNotEqual(comp.lock(self.store, EIGHT)['digest'],
                            comp.lock(self.store, with_hdl)['digest'])  # 锁面可区分

    def test_unknown_id(self):
        with self.assertRaises(io_match.AdapterError) as caught:
            io_match.wiring(self.store, EIGHT + ['v8.no.such'])
        self.assertEqual(caught.exception.code, 'E_ADAPTER_UNKNOWN_ID')

    def test_cli(self):
        import subprocess
        result = subprocess.run([sys.executable, '-B', str(HERE / 'io_match.py'),
                                 'wiring', str(MDIR), '--select=' + ','.join(EIGHT)],
                                capture_output=True, text=True)
        self.assertEqual(result.returncode, 0)
        import json
        verdict = json.loads(result.stdout)
        self.assertTrue(verdict['ok'])
        self.assertGreaterEqual(len(verdict['wires']), 8)
        result = subprocess.run([sys.executable, '-B', str(HERE / 'io_match.py')],
                                capture_output=True, text=True)
        self.assertEqual(result.returncode, 9)


if __name__ == '__main__':
    unittest.main(verbosity=2)
