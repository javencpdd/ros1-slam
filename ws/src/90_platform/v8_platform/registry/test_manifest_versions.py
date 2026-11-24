"""两套清单一并验证：framework 侧 v3（命名端口）与 registry 侧 v1。

2026-11-17 更名：原文件名 test_v2_manifest.py、变量 V2 均指"ports-only 那一代"，
但该代（manifest v2）已按真人裁定退役并删除，磁盘上实际只剩 v3（framework 49 件）
与 v1（registry 14 件）。文件名与变量名按现状改正，避免再次误导。
"""
import json
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str((Path(__file__).resolve().parent / '../../..' / '90_platform/v8_platform/composer').resolve()))
import composer  # noqa: E402
import registry as reg  # noqa: E402

V3 = HERE.parent / 'framework' / 'fixtures' / 'manifests'
V1 = HERE / 'data' / 'manifests'
CHAIN = ['v8.fw.sensors', 'v8.fw.faster_lio', 'v8.fw.map2d', 'v8.fw.global2d',
         'v8.fw.local2d', 'v8.fw.mux', 'v8.fw.robot_lite3']


class ManifestVersionTests(unittest.TestCase):
    def test_registry_derives_io_from_ports(self):
        store = reg.Registry().load_dir(V3)
        # 2026-11-17 修：此处原断言 48 → 49（8135a370 新增 bt_console 未同步）。
        # 2026-11-25 再修：不再写死件数（9a6e48c5 新增 global_far 后变 50，同类事故第三次）。
        # 改成与契约层**共用的独立清单**互通——注册表加载到的 id 集合必须等于清单里
        # runtime 类别的 id 集合，缺件/多件都红。清单本身由 contracts 层的双向守卫把关。
        expected = set()
        for line in (HERE.parent / 'contracts' / 'expected_components.txt').read_text(
                encoding='utf-8').splitlines():
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            kind, _, cid = line.partition('\t')
            if kind == 'runtime':
                expected.add(cid)
        self.assertTrue(expected, '独立清单为空 —— 空集不得判绿')
        self.assertEqual(set(store.entries), expected,
                         '注册表与独立清单不一致：缺 %s／多 %s' % (
                             sorted(expected - set(store.entries)),
                             sorted(set(store.entries) - expected)))
        self.assertEqual(store.get('v8.fw.map2d')['io']['map_schema'], 'occupancy_grid/v1')
        self.assertEqual(store.get('v8.fw.map2d')['io']['dimensions'], 2)
        self.assertEqual(store.get('v8.fw.map3d')['io']['map_schema'], 'support_map/v1')
        self.assertEqual(store.get('v8.fw.map3d')['io']['dimensions'], 3)
        self.assertEqual(store.get('v8.fw.map_merge')['io']['map_schema'], 'not_applicable')

    def test_manifest_file_has_no_io_but_composer_view_does(self):
        raw = json.loads((V3 / 'manifest_map2d.json').read_text())
        self.assertNotIn('io', raw)
        self.assertIn('ports', raw)
        store = composer.load(V3)
        self.assertIn('io', store._composer_raws['v8.fw.map2d'])
        self.assertEqual(store._composer_raws['v8.fw.map2d']['io']['map_schema'],
                         'occupancy_grid/v1')

    def test_v3_chain_composes_and_locks(self):
        store = composer.load(V3)
        result = composer.compose(store, CHAIN)
        self.assertTrue(result['ok'])
        checks = [row['check'] for row in result['compatibility']]
        self.assertIn('map_supply', checks)
        self.assertIn('kinematics_pair', checks)
        self.assertEqual(result['command_topics'],
                         {'/cmd_vel': 'v8.fw.mux', '/cmd_vel_advice/local': 'v8.fw.local2d'})
        lock = composer.lock(store, CHAIN)
        self.assertEqual(len(lock['digest']), 64)
        self.assertEqual(lock['digest'], composer.lock(store, CHAIN)['digest'])

    def test_v1_registry_still_loads(self):
        store = reg.Registry().load_dir(V1)
        # 2026-11-22：costmap_layers 清单随包退役删除 ⇒ 14→13。
        self.assertEqual(len(store.entries), 13)
        entry = store.get('v8.controller.dwa_adapter')
        self.assertIn('io', entry)
        self.assertEqual(entry['io']['map_schema'], 'occupancy_grid/v1')


if __name__ == '__main__':
    unittest.main()
