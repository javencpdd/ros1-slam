"""Explicit execution declarations without changing the frozen v1 contract."""
import copy
import json
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import validate


def ports_manifest():
    """命名端口清单底稿：只用于派生 v3 样本，本身不对应任何合法版本。

    2026-11-17：本函数原在 test_manifest_v2.py 里叫 `v2()`（带 schema_version=2）。
    manifest v2 按真人裁定退役后，那份文件已删，底稿内联到这里且不再自称某个版本，
    由 v3() 补上 schema_version=3。
    """
    return {
        'id': 'test.comp', 'version': '0.0.0',
        'roles': ['map_server'], 'dimensions': 2,
        'capability': {'kinematics': 'not_applicable', 'cancellation': 'none',
                       'supports_dynamic_obstacles': False},
        'ownership': {'tf_edges': [], 'command_topics': [], 'namespace': '/', 'stop': 'none'},
        'launch': {'package': 'map_server', 'type': 'map_server'},
        'ports': {'inputs': [],
                  'outputs': [{'name': 'map', 'type': 'nav_msgs/OccupancyGrid',
                               'schema': 'occupancy_grid/v1'}]},
        'evidence': {'level': 'declared', 'reports': []},
    }


def v3():
    value = ports_manifest()
    value.update(schema_version=3, execution={'modes': ['simulation'], 'robot_id': None})
    return value


class ManifestV3Tests(unittest.TestCase):
    def rejected(self, value, code):
        with self.assertRaises(validate.ContractError) as caught:
            validate.validate('manifest', value)
        self.assertEqual(caught.exception.code, code)

    def test_v3_is_normalized_without_changing_named_port_entry(self):
        value = v3()
        self.assertTrue(validate.validate('manifest', value))
        self.assertEqual(validate.normalize_manifest(value)['io']['map_schema'], 'occupancy_grid/v1')
        self.assertNotIn('io', value)

    def test_retired_v2_is_rejected_with_explicit_reason(self):
        """v2 已退役：必须显式 E_VERSION，不能掉进 v1 schema 报成"未知字段"。

        2026-11-17 裁剪前的旧断言是 `v2()+execution` ⇒ E_SCHEMA_UNKNOWN_FIELD
        （即"v2 不认 execution"）。v2 整体退役后这条语义已不存在，改为证明裁剪
        生效：任何 schema_version=2 的 manifest 都被 validate 拦下并给出可归因理由。
        """
        old = ports_manifest()
        old.update(schema_version=2, execution={'modes': ['simulation'], 'robot_id': None})
        self.rejected(old, 'E_VERSION')

    def test_v3_requires_strict_explicit_execution_and_binding(self):
        for field in ('execution',):
            value = v3(); value.pop(field)
            self.rejected(value, 'E_SCHEMA_REQUIRED')
        value = v3(); value['execution'].pop('robot_id')
        self.rejected(value, 'E_SCHEMA_REQUIRED')
        value = v3(); value['execution']['unknown'] = True
        self.rejected(value, 'E_SCHEMA_UNKNOWN_FIELD')
        for modes in ([], ['simulation', 'simulation'], ['training']):
            value = v3(); value['execution']['modes'] = modes
            self.rejected(value, 'E_SCHEMA_MINITEMS' if not modes else
                          'E_SCHEMA_UNIQUE' if len(modes) == 2 else 'E_SCHEMA_ENUM')

    def test_v3_does_not_skip_original_port_dimension_and_stop_invariants(self):
        value = v3(); value['ports']['outputs'].append(copy.deepcopy(value['ports']['outputs'][0]))
        self.rejected(value, 'E_MANIFEST_PORT_DUP')
        value = v3(); value['dimensions'] = 3
        self.rejected(value, 'E_MANIFEST_DIM')
        value = v3(); value['roles'] = ['controller']
        self.rejected(value, 'E_MANIFEST_STOP')
        value = v3(); value['ports']['outputs'][0]['frame'] = 'map'
        self.rejected(value, 'E_SCHEMA_UNKNOWN_FIELD')

    def test_hardware_binding_and_test_double_are_checked(self):
        value = v3(); value['execution']['modes'] = ['real_robot']; value['roles'] = ['robot_adapter']
        self.rejected(value, 'E_MANIFEST_EXECUTION')
        value['execution']['robot_id'] = 'lite3'
        self.assertTrue(validate.validate('manifest', value))
        value['launch']['test_double'] = True
        self.rejected(value, 'E_MANIFEST_EXECUTION')
        value = v3(); value['execution']['robot_id'] = '*'
        self.rejected(value, 'E_MANIFEST_EXECUTION')

    def test_selector_defaults_and_fields_are_strict(self):
        value = v3(); value['execution']['selectors'] = [
            {'name': 'source', 'default': 'sim_gt', 'values': ['sim_gt', 'faster_lio']}]
        self.assertTrue(validate.validate('manifest', value))
        value['execution']['selectors'][0]['default'] = 'unknown'
        self.rejected(value, 'E_MANIFEST_EXECUTION')
        value['execution']['selectors'][0]['default'] = 'sim_gt'
        value['execution']['selectors'][0]['anything'] = 'else'
        self.rejected(value, 'E_SCHEMA_UNKNOWN_FIELD')

    def test_all_runtime_manifests_declare_their_domains_without_evidence_upgrade(self):
        """双向完整性守卫：独立清单 ↔ 磁盘枚举，缺件/多件/重复件都红。

        ★不用"文件数等于文件数"★（2026-11-25 修）：原先是 `assertEqual(len(paths), 53)`，
        新增 `manifest_global_far.json` 后变 54 就红；而如果改成"从同一个 glob 派生总数再和自己比"，
        删掉一个 manifest 也会绿——那等于取消了完整性校验。这里用**独立工件**
        `contracts/expected_components.txt`（人维护、与磁盘互为校验）：
        清单有磁盘无 → 缺件；磁盘有清单无 → 未登记新件；同 id 两次 → 重复。
        逐件的 schema / 执行域 / 能力 / 证据等级校验全部保留在下面，一条没减。
        """
        expected = {}
        for line in (HERE / 'expected_components.txt').read_text(encoding='utf-8').splitlines():
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            kind, _, cid = line.partition('\t')
            self.assertIn(kind, ('runtime', 'verification'), '清单行格式错：%r' % line)
            self.assertNotIn(cid, expected, '清单内组件 ID 重复：%s' % cid)
            expected[cid] = kind
        self.assertTrue(expected, '清单为空 —— 空集不得判绿')

        seen = {}
        for kind, sub in (('runtime', 'framework/fixtures/manifests'),
                          ('verification', 'framework/fixtures/verification_manifests')):
            for path in sorted((HERE.parent / sub).glob('manifest_*.json')):
                value = json.loads(path.read_text())
                cid = value['id']
                with self.subTest(file=path.name, component=cid):
                    self.assertIn(cid, expected, '磁盘上存在但清单未登记：%s' % cid)
                    self.assertEqual(expected[cid], kind,
                                     '%s 的类别应为 %s（按所在目录判定）' % (cid, kind))
                    self.assertNotIn(cid, seen, '重复的组件 ID：%s（另见 %s）' % (cid, seen.get(cid)))
                    seen[cid] = path.name
                    self.assertEqual(value['schema_version'], 3)
                    self.assertTrue(validate.validate('manifest', value))

        missing = sorted(set(expected) - set(seen))
        self.assertEqual(missing, [], '清单登记但磁盘缺件：%s' % missing)

    def test_capability_feedback_modes_and_timeout_are_validated(self):
        value = v3()
        value['capability']['feedback_modes'] = ['pose', 'ack']
        value['capability']['timeout_s'] = 5.0
        validate.validate('manifest', value)

        value = v3()
        value['capability']['feedback_modes'] = ['pose', 'pose']
        with self.assertRaises(validate.ContractError):
            validate.validate('manifest', value)
        value = v3()
        value['capability']['feedback_modes'] = ['unknown']
        with self.assertRaises(validate.ContractError):
            validate.validate('manifest', value)
        value = v3()
        value['capability']['timeout_s'] = 0
        with self.assertRaises(validate.ContractError):
            validate.validate('manifest', value)
        value = v3()
        value['capability']['cancellation'] = 'sometimes'
        with self.assertRaises(validate.ContractError):
            validate.validate('manifest', value)

    def test_shipped_bt_and_local_declare_cooperative_cancel(self):
        root = Path(__file__).resolve().parents[1] / 'framework' / 'fixtures' / 'manifests'
        for name in ('manifest_bt_console.json', 'manifest_local_native.json',
                     'manifest_local_native_raw.json'):
            value = json.loads((root / name).read_text(encoding='utf-8'))
            validate.validate('manifest', value)
            self.assertEqual(value['capability']['cancellation'], 'cooperative', name)


if __name__ == '__main__':
    unittest.main()
