"""A3 查询边界与运行驱动判据：宿主测试不冒充 ROS 运行证据。"""
import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import a3_runtime as a3
import map_node
import support_planner


def snapshots(first, second, ev1, ev2, queries):
    with tempfile.TemporaryDirectory() as folder:
        map_path = Path(folder) / 'map.json'
        evidence_path = Path(folder) / 'evidence.json'
        observed = []
        for value, evidence in ((first, ev1), (second, ev2)):
            map_path.write_text(json.dumps(value))
            evidence_path.write_text(json.dumps(evidence))
            observed.append(map_node.snapshot(map_path, evidence_path, queries))
        return observed


def planner(value, start, goal, budget, name):
    try:
        result = support_planner.plan(value, start, goal, budget=budget)
        return [row[:3] for row in result['path']], None, result
    except support_planner.PlannerError as error:
        return None, error.code, {'detail': error.detail}


class A3RuntimeTests(unittest.TestCase):
    def test_full_query_semantics_and_legal_oneway_map(self):
        cells = a3.run(snapshots, planner)
        self.assertEqual(len(cells), 15)
        self.assertEqual([cell for cell in cells if cell['outcome'] != 'pass'], [])

    def test_query_assertions_detect_collapsed_layer_and_unknown_as_free(self):
        def corrupted(*args):
            one, two = snapshots(*args)
            if args[0]['multiple_surfaces_per_xy']:
                one['queries'][0]['layers'] = one['queries'][0]['layers'][1:]
                one['queries'][1]['state'] = 'free'
            return one, two

        cells = a3.run(corrupted, planner)
        failures = [cell['case'] for cell in cells if cell['outcome'] != 'pass']
        self.assertEqual(failures, ['same_xy_layers', 'upper_unknown'])

    def test_bad_evidence_never_becomes_free(self):
        with tempfile.TemporaryDirectory() as folder:
            map_path = Path(folder) / 'map.json'
            evidence_path = Path(folder) / 'evidence.json'
            map_path.write_text(json.dumps(a3.fixture()))
            evidence_path.write_text(json.dumps([{'xyz': [0.25, 0.25, 0], 'state': 'empty'}]))
            with self.assertRaises(ValueError):
                map_node.snapshot(map_path, evidence_path, [[0.25, 0.25, 0]])


if __name__ == '__main__':
    unittest.main(verbosity=2)
