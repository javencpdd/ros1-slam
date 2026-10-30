"""v2 manifests (ports only) through registry and composer normalization."""
import json
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / 'composer'))
import composer  # noqa: E402
import registry as reg  # noqa: E402

V2 = HERE.parent / 'framework' / 'fixtures' / 'manifests'
V1 = HERE / 'data' / 'manifests'
CHAIN = ['v8.fw.sensors', 'v8.fw.faster_lio', 'v8.fw.map2d', 'v8.fw.global2d',
         'v8.fw.local2d', 'v8.fw.mux', 'v8.fw.robot_lite3']


class V2ManifestTests(unittest.TestCase):
    def test_registry_derives_io_from_ports(self):
        store = reg.Registry().load_dir(V2)
        self.assertEqual(len(store.entries), 14)
        self.assertEqual(store.get('v8.fw.map2d')['io']['map_schema'], 'occupancy_grid/v1')
        self.assertEqual(store.get('v8.fw.map2d')['io']['dimensions'], 2)
        self.assertEqual(store.get('v8.fw.map3d')['io']['map_schema'], 'support_map/v1')
        self.assertEqual(store.get('v8.fw.map3d')['io']['dimensions'], 3)
        self.assertEqual(store.get('v8.fw.map_merge')['io']['map_schema'], 'not_applicable')

    def test_manifest_file_has_no_io_but_composer_view_does(self):
        raw = json.loads((V2 / 'manifest_map2d.json').read_text())
        self.assertNotIn('io', raw)
        self.assertIn('ports', raw)
        store = composer.load(V2)
        self.assertIn('io', store._composer_raws['v8.fw.map2d'])
        self.assertEqual(store._composer_raws['v8.fw.map2d']['io']['map_schema'],
                         'occupancy_grid/v1')

    def test_v2_chain_composes_and_locks(self):
        store = composer.load(V2)
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
        self.assertEqual(len(store.entries), 13)
        entry = store.get('v8.controller.dwa_adapter')
        self.assertIn('io', entry)
        self.assertEqual(entry['io']['map_schema'], 'occupancy_grid/v1')


if __name__ == '__main__':
    unittest.main()
