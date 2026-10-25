"""V8-04 注册表测试：加载/校验/检索/证据分离，负例 fail-closed 且理由码可归因。

负例不只看「被拒」，还要求理由码一致；加载顺序确定性（按文件名字典序）也被锁定，
组合层不得静默跳件。
"""
import json
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import registry as reg  # noqa: E402

FIX = HERE / 'fixtures'
POS_IDS = ['v8.controller.teb_adapter', 'v8.map_server.occupancy', 'v8.support_graph.astar']


def reason(fn):
    try:
        fn()
    except reg.RegistryError as error:
        return error.code, error.detail
    return 'OK', ''


class RegistryTests(unittest.TestCase):
    def setUp(self):
        self.r = reg.Registry().load_dir(FIX)

    def test_load_positive(self):
        self.assertEqual([e['id'] for e in self.r.all()], POS_IDS)
        for entry in self.r.all():
            self.assertEqual(len(entry['sha256']), 64)
            self.assertTrue(entry['roles'] and entry['version'])

    def test_get_and_missing(self):
        self.assertEqual(self.r.get('v8.map_server.occupancy')['roles'], ['map_server'])
        self.assertIsNone(self.r.get('v8.no.such'))

    def test_by_role(self):
        self.assertEqual([e['id'] for e in self.r.by_role('map_server')], ['v8.map_server.occupancy'])
        self.assertEqual([e['id'] for e in self.r.by_role('global_planner')], ['v8.support_graph.astar'])
        self.assertEqual(self.r.by_role('evaluator'), [])

    def test_by_tag(self):
        self.assertEqual([e['id'] for e in self.r.by_tag('indoor')],
                         ['v8.controller.teb_adapter', 'v8.map_server.occupancy'])
        self.assertEqual([e['id'] for e in self.r.by_tag('multilevel')], ['v8.support_graph.astar'])
        self.assertEqual(self.r.by_tag('gpu_required'), [])

    def test_evidence_separation(self):
        sep = self.r.evidence_separation()
        self.assertEqual(sep['declared'], ['v8.controller.teb_adapter', 'v8.map_server.occupancy'])
        self.assertEqual(sep['tested'], ['v8.support_graph.astar'])
        tested = self.r.get('v8.support_graph.astar')['evidence']
        self.assertEqual(tested['level'], 'contract_tested')
        self.assertEqual(tested['reports'], ['contracts/test_contracts.py'])
        self.assertTrue(self.r.get('v8.map_server.occupancy')['evidence']['claimed_only'])

    def test_invalid_manifest_surfaces_contract_reason(self):
        code, detail = reason(lambda: reg.Registry().load_file(FIX / 'invalid' / 'manifest_stop.json'))
        self.assertEqual(code, 'E_REG_INVALID')
        self.assertIn('E_MANIFEST_STOP', detail)

    def test_duplicate_id(self):
        r = reg.Registry()
        r.load_file(FIX / 'invalid' / 'manifest_dup_a.json')
        code, detail = reason(lambda: r.load_file(FIX / 'invalid' / 'manifest_dup_b.json'))
        self.assertEqual(code, 'E_REG_DUP_ID')
        self.assertIn('manifest_dup_a.json', detail)

    def test_not_json(self):
        code, _ = reason(lambda: reg.Registry().load_file(FIX / 'invalid' / 'not_json.json'))
        self.assertEqual(code, 'E_INPUT_JSON')

    def test_array_json(self):
        code, detail = reason(lambda: reg.Registry().load_file(FIX / 'invalid' / 'array.json'))
        self.assertEqual(code, 'E_REG_INVALID')
        self.assertIn('E_SCHEMA_TYPE', detail)

    def test_fail_closed_directory(self):
        with tempfile.TemporaryDirectory() as tmp:
            shutil.copy(FIX / 'manifest_map_server_2d.json', tmp)
            shutil.copy(FIX / 'invalid' / 'manifest_stop.json', tmp)
            code, detail = reason(lambda: reg.Registry().load_dir(tmp))
            self.assertEqual(code, 'E_REG_INVALID')
            self.assertIn('manifest_stop.json', detail)

    def test_empty_and_missing_directory(self):
        with tempfile.TemporaryDirectory() as tmp:
            code, _ = reason(lambda: reg.Registry().load_dir(tmp))
            self.assertEqual(code, 'E_REG_NO_MANIFESTS')
        code, _ = reason(lambda: reg.Registry().load_dir('/no/such/dir'))
        self.assertEqual(code, 'E_INPUT_MISSING')

    def test_provenance_sidecar(self):
        r = reg.Registry().load_dir(FIX)
        r.load_provenance(FIX / 'provenance.json')
        self.assertEqual(sorted(r.provenance), POS_IDS)
        self.assertEqual(r.unprovenanced(), [])
        entry = r.provenance['v8.support_graph.astar']
        self.assertEqual(sorted(entry), ['build_entry', 'license', 'source'])

    def test_provenance_unknown_id(self):
        r = reg.Registry().load_dir(FIX)
        code, detail = reason(lambda: r.load_provenance(FIX / 'invalid' / 'provenance_unknown.json'))
        self.assertEqual(code, 'E_PROV_UNKNOWN_ID')
        self.assertIn('v8.no.such', detail)

    def test_provenance_blank_field(self):
        r = reg.Registry().load_dir(FIX)
        code, detail = reason(lambda: r.load_provenance(FIX / 'invalid' / 'provenance_blank.json'))
        self.assertEqual(code, 'E_PROV_INVALID_FIELD')
        self.assertIn('source', detail)

    def test_provenance_bad_json_and_missing(self):
        r = reg.Registry().load_dir(FIX)
        code, _ = reason(lambda: r.load_provenance(FIX / 'invalid' / 'provenance_not_json.json'))
        self.assertEqual(code, 'E_INPUT_JSON')
        code, _ = reason(lambda: r.load_provenance(FIX / 'no' / 'such.json'))
        self.assertEqual(code, 'E_INPUT_MISSING')

    def test_core_data_shape(self):
        """真实溯源数据件（R616 核心格）内部一致性：id 唯一、三字段非空白。

        该件待 manifest 化后才能经 load_provenance 挂载（引用未注册 id 会 fail-closed），
        本测试只做数据面校验。
        """
        data = json.loads((HERE / 'data' / 'provenance_core.json').read_text())
        self.assertEqual(len(data), len(set(data)))
        self.assertGreaterEqual(len(data), 12)
        for component_id, record in data.items():
            self.assertTrue(component_id.startswith('v8.'), component_id)
            for field in reg.Registry.PROVENANCE_FIELDS:
                self.assertIsInstance(record[field], str, '%s.%s' % (component_id, field))
                self.assertTrue(record[field].strip(), '%s.%s 空白' % (component_id, field))

    def test_cli_provenance(self):
        def run(*args):
            result = subprocess.run([sys.executable, '-B', str(HERE / 'registry.py'), *args],
                                    capture_output=True, text=True)
            return result.returncode, json.loads(result.stdout) if result.stdout.strip() else None

        rc, out = run(str(FIX), '--provenance', str(FIX / 'provenance.json'))
        self.assertEqual(rc, 0)
        self.assertEqual(out['provenance_count'], 3)
        self.assertEqual(out['unprovenanced'], [])

        rc, out = run(str(FIX), '--provenance', str(FIX / 'invalid' / 'provenance_unknown.json'))
        self.assertEqual(rc, 5)
        self.assertEqual(out['reason'], 'E_PROV_UNKNOWN_ID')

        rc, out = run(str(FIX), '--provenance', str(HERE / 'data' / 'provenance_core.json'))
        self.assertEqual(rc, 5)
        self.assertEqual(out['reason'], 'E_PROV_UNKNOWN_ID')  # 核心 manifest 未落地前 fail-closed

    def test_core_manifests_with_provenance(self):
        """核心格 13 份真实 manifest 全部过契约校验，provenance 全量挂载零缺口。"""
        r = reg.Registry().load_dir(HERE / 'data' / 'manifests')
        r.load_provenance(HERE / 'data' / 'provenance_core.json')
        self.assertEqual(len(r.entries), 13)
        self.assertEqual(len(r.provenance), 13)
        self.assertEqual(r.unprovenanced(), [])
        sep = r.evidence_separation()
        self.assertEqual(sep['tested'], ['v8.local_planner.scan_planner', 'v8.representation.costmap_layers'])
        self.assertEqual(len(sep['declared']), 11)
        self.assertEqual(sorted(e['id'] for e in r.by_role('controller')),
                         ['v8.controller.dwa_adapter', 'v8.controller.teb_adapter'])
        scan = r.get('v8.local_planner.scan_planner')
        self.assertEqual(scan['io']['map_schema'], 'not_applicable')
        self.assertTrue(scan['evidence']['reports'])

    def test_cli(self):
        def run(*args):
            result = subprocess.run([sys.executable, '-B', str(HERE / 'registry.py'), *args],
                                    capture_output=True, text=True)
            return result.returncode, json.loads(result.stdout) if result.stdout.strip() else None

        rc, out = run(str(FIX))
        self.assertEqual(rc, 0)
        self.assertEqual(out['count'], 3)
        self.assertEqual(out['evidence']['tested'], ['v8.support_graph.astar'])

        rc, out = run(str(FIX), '--tag', 'indoor')
        self.assertEqual(rc, 0)
        self.assertEqual(out['matches'], ['v8.controller.teb_adapter', 'v8.map_server.occupancy'])

        rc, out = run(str(FIX), '--id', 'v8.support_graph.astar')
        self.assertEqual(rc, 0)
        self.assertEqual(out['matches'], ['v8.support_graph.astar'])

        # invalid/ 目录内按文件名字典序：manifest_dup_a < manifest_dup_b < manifest_stop，
        # 故先触发重复 id（加载顺序确定性被本断言一并锁定）。
        rc, out = run(str(FIX / 'invalid'))
        self.assertEqual(rc, 5)
        self.assertEqual(out['reason'], 'E_REG_DUP_ID')

        rc, _ = run()
        self.assertEqual(rc, 9)


if __name__ == '__main__':
    unittest.main(verbosity=2)
