"""独立评价器对坏证据的反例；不以被测状态机作为判分参考。"""
import copy
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import runtime_evaluator as evaluator


def wire(t, vx=0):
    return {'t': t, 'vx': vx, 'vy': 0, 'vz': 0, 'wx': 0, 'wy': 0, 'wz': 0}


def record(seed=0):
    profile = evaluator.input_profile(seed)
    value = {'profile': profile, 'profile_hash': evaluator.digest(profile), 'repeat': 1,
             'input_mode': 'declared_fixture', 'source_topic': '', 'cases': [], 'outputs': [],
             'inputs': [{'t': 0, 'kind': 'odom', 'xyz': profile['bias_xyz'], 'stamp': 1.0},
                        {'t': 0, 'kind': 'candidate', 'vx': profile['candidate_speed']}]}
    for index, name in enumerate(('stale', 'jump', 'relocalization', 'stamp_regression')):
        base = 2 * index
        value['cases'].append({'case': name, 'fault_at': base + 0.30,
            'armed_window': [base, base + 0.2], 'stop_window': [base + 0.4, base + 0.6],
            'hold_window': [base + 0.6, base + 0.9], 'resume_at': base + 1,
            'resume_window': [base + 1, base + 1.2]})
        value['outputs'].extend([wire(base + 0.1, 0.2), wire(base + 0.5),
                                  wire(base + 0.7), wire(base + 1.1, 0.2)])
        if name == 'stale':
            value['inputs'].extend([{'t': base + 0.30, 'kind': 'odom', 'stamp': 1.1, 'xyz': [0, 0, 0]},
                                    {'t': base + 0.60, 'kind': 'odom', 'stamp': 1.2, 'xyz': [0, 0, 0]}])
        elif name == 'jump':
            value['inputs'].extend([{'t': base + 0.29, 'kind': 'odom', 'stamp': 2.0, 'xyz': [0, 0, 0]},
                                    {'t': base + 0.30, 'kind': 'odom', 'stamp': 2.1, 'xyz': [1, 0, 0]}])
        elif name == 'stamp_regression':
            value['inputs'].extend([{'t': base + 0.29, 'kind': 'odom', 'stamp': 3.0, 'xyz': [1, 0, 0]},
                                    {'t': base + 0.30, 'kind': 'odom', 'stamp': 2.0, 'xyz': [1, 0, 0]}])
        else:
            value['inputs'].append({'t': base + 0.30, 'kind': 'relocalized', 'value': True})
    return value


class A6EvidenceTests(unittest.TestCase):
    def test_wire_evidence_passes_without_reading_status_green_flags(self):
        value = record()
        value['statuses'] = [{'state': 'FAILED'}]
        result = evaluator.evaluate(value)
        self.assertTrue(result['ok'], result)
        self.assertFalse(result['accuracy_backend_switch_tested'])
        self.assertAlmostEqual(result['cases'][0]['stop_latency_s'], 0.2)

    def test_old_lease_revival_is_rejected(self):
        value = record()
        value['outputs'][2]['vx'] = 0.2
        result = evaluator.evaluate(value)
        self.assertFalse(result['ok'])
        self.assertIn('old_lease_revived', result['cases'][0]['failures'])

    def test_missing_stop_and_missing_injection_are_rejected(self):
        value = record()
        value['outputs'][1]['vx'] = 0.2
        value['inputs'] = [row for row in value['inputs'] if row['kind'] != 'relocalized']
        result = evaluator.evaluate(value)
        self.assertFalse(result['ok'])
        self.assertIn('no_zero_after_fault', result['cases'][0]['failures'])
        self.assertIn('fault_injection_unobserved', result['cases'][2]['failures'])

    def test_nonfinite_output_cannot_be_counted_as_zero(self):
        value = record()
        value['outputs'][1]['vx'] = float('nan')
        self.assertFalse(evaluator.evaluate(value)['ok'])

    def test_real_seed_diversity_and_duplicate_cell_detection(self):
        values = [record(seed) for seed in (0, 1, 2)]
        result = evaluator.matrix(values)
        self.assertTrue(result['ok'], result)
        self.assertFalse(evaluator.matrix(values + [copy.deepcopy(values[0])])['ok'])
        self.assertFalse(evaluator.matrix([])['ok'])

    def test_renaming_seed_without_changing_packets_is_rejected(self):
        first = record(0)
        second = copy.deepcopy(first)
        second['profile']['seed'] = 1
        second['profile_hash'] = evaluator.digest(second['profile'])
        result = evaluator.matrix([first, second])
        self.assertFalse(result['ok'])
        self.assertIn('nominal_seed_inputs_identical', result['errors'])


if __name__ == '__main__':
    unittest.main(verbosity=2)
