"""Explicit execution declarations without changing the frozen v1/v2 contracts."""
import copy
import json
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import validate
from test_manifest_v2 import v2


def v3():
    value = v2()
    value.update(schema_version=3, execution={'modes': ['simulation'], 'robot_id': None})
    return value


class ManifestV3Tests(unittest.TestCase):
    def rejected(self, value, code):
        with self.assertRaises(validate.ContractError) as caught:
            validate.validate('manifest', value)
        self.assertEqual(caught.exception.code, code)

    def test_v3_is_normalized_without_changing_named_port_or_v2_input(self):
        value = v3()
        self.assertTrue(validate.validate('manifest', value))
        self.assertEqual(validate.normalize_manifest(value)['io']['map_schema'], 'occupancy_grid/v1')
        self.assertNotIn('io', value)
        old = v2(); old['execution'] = value['execution']
        self.rejected(old, 'E_SCHEMA_UNKNOWN_FIELD')

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
        paths = list((HERE.parent / 'framework/fixtures/manifests').glob('manifest_*.json'))
        paths += list((HERE.parent / 'framework/fixtures/verification_manifests').glob('manifest_*.json'))
        self.assertEqual(len(paths), 53)
        for path in paths:
            with self.subTest(file=path.name):
                value = json.loads(path.read_text())
                self.assertEqual(value['schema_version'], 3)
                self.assertTrue(validate.validate('manifest', value))

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
