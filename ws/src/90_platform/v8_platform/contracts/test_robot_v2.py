import copy
import json
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import validate as contract  # noqa: E402


def load(name):
    return json.loads((HERE / 'fixtures' / name).read_text())


def reason(value):
    try:
        contract.validate('robot', value)
    except contract.ContractError as error:
        return error.code
    return 'OK'


class RobotV2Tests(unittest.TestCase):
    def test_v1_remains_accepted(self):
        self.assertEqual(reason(load('robot_lite3_design.json')), 'OK')

    def test_v2_declared_example_accepted(self):
        self.assertEqual(reason(load('robot_lite3_v2_declared.json')), 'OK')

    def test_v2_tested_fixture_accepted(self):
        self.assertEqual(reason(load('robot_v2_tested_fixture.json')), 'OK')

    def test_tested_requires_evidence_block(self):
        value = load('robot_v2_tested_fixture.json')
        value.pop('evidence')
        self.assertEqual(reason(value), 'E_ROBOT_EVIDENCE')

    def test_tested_requires_nonzero_sample(self):
        value = load('robot_v2_tested_fixture.json')
        value['evidence']['sample_size'] = 0
        self.assertEqual(reason(value), 'E_ROBOT_EVIDENCE')

    def test_declared_must_stay_prohibited(self):
        value = load('robot_lite3_v2_declared.json')
        value['capability']['max_slope_deg']['composition_use'] = 'allowed'
        self.assertEqual(reason(value), 'E_ROBOT_UNSOURCED')

    def test_tested_item_requires_report(self):
        value = load('robot_v2_tested_fixture.json')
        value['capability']['max_speed_mps'].pop('report')
        self.assertEqual(reason(value), 'E_ROBOT_EVIDENCE')

    def test_success_rate_is_bounded(self):
        value = load('robot_v2_tested_fixture.json')
        value['capability']['success_rate']['value'] = 1.01
        self.assertEqual(reason(value), 'E_SCHEMA_MAXIMUM')

    def test_capability_units_are_checked(self):
        value = load('robot_v2_tested_fixture.json')
        value['capability']['max_speed_mps']['unit'] = 'km/h'
        self.assertEqual(reason(value), 'E_ROBOT_UNIT')

    def test_model_is_required_in_v2(self):
        value = load('robot_v2_tested_fixture.json')
        value.pop('model')
        self.assertEqual(reason(value), 'E_SCHEMA_REQUIRED')

    def test_unknown_field_is_rejected(self):
        value = load('robot_v2_tested_fixture.json')
        value['capability']['flight_ceiling_m'] = copy.deepcopy(value['capability']['max_speed_mps'])
        self.assertEqual(reason(value), 'E_SCHEMA_UNKNOWN_FIELD')

    def test_version_must_be_one_or_two(self):
        value = load('robot_v2_tested_fixture.json')
        value['schema_version'] = 3
        self.assertEqual(reason(value), 'E_VERSION')

    def test_v1_cannot_be_relabelled_without_model(self):
        value = load('robot_lite3_design.json')
        value['schema_version'] = 2
        self.assertEqual(reason(value), 'E_SCHEMA_REQUIRED')


if __name__ == '__main__':
    unittest.main(verbosity=2)