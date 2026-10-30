"""manifest v2: ports-only input, derived io, v1 untouched."""
import copy
import json
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import validate  # noqa: E402


def v2():
    return {
        'schema_version': 2, 'id': 'test.comp', 'version': '0.0.0',
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


class ManifestV2Tests(unittest.TestCase):
    def test_ports_only_is_valid_and_io_is_derived(self):
        value = v2()
        self.assertTrue(validate.validate('manifest', value))
        derived = validate.normalize_manifest(value)
        self.assertEqual(derived['io']['map_schema'], 'occupancy_grid/v1')
        self.assertEqual(derived['io']['dimensions'], 2)
        self.assertEqual(derived['io']['outputs'], ['nav_msgs/OccupancyGrid（map）'])

    def test_support_schema_collapses_to_v1_enum(self):
        value = v2()
        value['dimensions'] = 3
        value['ports']['outputs'] = [{'name': 'map', 'type': 'v8_msgs/SupportMap',
                                      'schema': 'support_map/v2'}]
        self.assertTrue(validate.validate('manifest', value))
        derived = validate.normalize_manifest(value)
        self.assertEqual(derived['io']['map_schema'], 'support_map/v1')
        self.assertEqual(derived['io']['dimensions'], 3)

    def test_io_field_is_rejected(self):
        value = v2()
        value['io'] = {'map_schema': 'occupancy_grid/v1', 'dimensions': 2,
                       'inputs': [], 'outputs': ['x']}
        with self.assertRaises(validate.ContractError) as caught:
            validate.validate('manifest', value)
        self.assertEqual(caught.exception.code, 'E_SCHEMA_UNKNOWN_FIELD')

    def test_missing_dimensions_is_rejected(self):
        value = v2()
        del value['dimensions']
        with self.assertRaises(validate.ContractError) as caught:
            validate.validate('manifest', value)
        self.assertEqual(caught.exception.code, 'E_SCHEMA_REQUIRED')

    def test_mixed_map_ports_are_rejected_by_normalization(self):
        value = v2()
        value['ports']['outputs'].append({'name': 'support', 'type': 'v8_msgs/SupportMap',
                                          'schema': 'support_map/v1'})
        with self.assertRaises(validate.ContractError) as caught:
            validate.normalize_manifest(value)
        self.assertEqual(caught.exception.code, 'E_MANIFEST_MAP_MIXED')

    def test_v1_manifest_is_returned_unchanged(self):
        path = HERE.parent / 'registry' / 'data' / 'manifests' / 'manifest_controller_dwa_adapter.json'
        value = json.loads(path.read_text())
        self.assertIs(validate.normalize_manifest(value), value)
        self.assertTrue(validate.validate('manifest', value))


if __name__ == '__main__':
    unittest.main()
