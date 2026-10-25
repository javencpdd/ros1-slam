"""V8-02 契约正反例测试：每个反例必须命中指定理由码，修复后必须通过。

反例不只看「被拒」，还要求理由码一致，避免用一个笼统错误吞掉另一处缺陷；
修复式可复跑，证明红灯来自注入的那一处而不是夹具整体不可用。
"""
import copy
import json
import math
import subprocess
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import validate as contract  # noqa: E402

EXPECT = json.loads((HERE / 'fixtures/expectations.json').read_text())


def load(name):
    return json.loads((HERE / 'fixtures' / name).read_text())


def apply(data, ops):
    for op in ops:
        node = data
        parts = op['path'].split('.')
        for part in parts[:-1]:
            node = node[int(part)] if isinstance(node, list) else node[part]
        last = parts[-1]
        if op['op'] == 'remove':
            node.pop(int(last)) if isinstance(node, list) else node.pop(last)
        elif op['op'] == 'set':
            if isinstance(node, list):
                node[int(last)] = op['value']
            else:
                node[last] = op['value']
        else:
            raise AssertionError(op)
    return data


def reason(kind, value):
    try:
        contract.validate(kind, value)
    except contract.ContractError as error:
        return error.code
    return 'OK'


class ContractTests(unittest.TestCase):
    def test_cases(self):
        seen = set()
        for case in EXPECT:
            with self.subTest(case=case['file']):
                self.assertNotIn(case['file'], seen)
                seen.add(case['file'])
                data = load(case['file'])
                if case['ok']:
                    self.assertEqual(reason(case['kind'], data), 'OK')
                else:
                    self.assertEqual(reason(case['kind'], data), case['reason'])
                    repaired = apply(copy.deepcopy(data), case['repair'])
                    self.assertEqual(reason(case['kind'], repaired), 'OK',
                                     'repair did not clear ' + case['reason'])
        self.assertEqual(len(seen), len(EXPECT))
        self.assertEqual((len(EXPECT), len([c for c in EXPECT if c['ok']])), (61, 9))

    def test_two_kinds_expressible(self):
        flat = load('map_2d_flat.json')
        multi = load('map_ml_two_surfaces.json')
        self.assertEqual(reason('map', flat), 'OK')
        self.assertEqual(reason('map', multi), 'OK')
        self.assertIn('layers', flat)
        self.assertNotIn('surfaces', flat)
        self.assertIn('surfaces', multi)
        self.assertNotIn('layers', multi)
        self.assertEqual(len({s['surface_id'] for s in multi['surfaces']}), 2)
        self.assertTrue(multi['multiple_surfaces_per_xy'])

    def test_one_way_edge_is_legal(self):
        # 单向 A->B、B 为汇点：B 可由 A 到达，必须放行（复核指出旧式会误拒）。
        data = load('map_ml_two_surfaces.json')
        data['surfaces'][0]['edges'] = [{'to': 'mezz', 'kind': 'slope', 'max_slope_deg': 12.0, 'bidirectional': False}]
        data['surfaces'][1]['edges'] = []
        self.assertEqual(reason('map', data), 'OK')

    def test_two_separate_rings_rejected(self):
        # 两个各自闭环、彼此无边的子图：每个面都有出边，但整体不连通，必须拒。
        data = load('map_ml_two_surfaces.json')
        third = copy.deepcopy(data['surfaces'][0])
        fourth = copy.deepcopy(data['surfaces'][1])
        third['surface_id'] = 'annex_a'
        third['footprint'] = {'min_x': 10, 'min_y': 10, 'max_x': 12, 'max_y': 12}
        third['edges'] = [{'to': 'annex_b', 'kind': 'slope', 'max_slope_deg': 12.0, 'bidirectional': True}]
        fourth['surface_id'] = 'annex_b'
        fourth['footprint'] = {'min_x': 10.5, 'min_y': 10.5, 'max_x': 12.5, 'max_y': 12.5}
        fourth['edges'] = [{'to': 'annex_a', 'kind': 'slope', 'max_slope_deg': 12.0, 'bidirectional': True}]
        data['surfaces'] += [third, fourth]
        self.assertEqual(reason('map', data), 'E_MAP_DISCONNECTED')

    def test_pair_task_and_map(self):
        task = load('task_ml_goal.json')
        mapping = load('map_ml_two_surfaces.json')
        self.assertTrue(contract.check_pair(task, mapping))
        wrong = copy.deepcopy(task)
        wrong['goal']['surface_hint'] = 'attic'
        with self.assertRaises(contract.ContractError) as caught:
            contract.check_pair(wrong, mapping)
        self.assertEqual(caught.exception.code, 'E_PAIR_SURFACE')
        old = copy.deepcopy(task)
        old['map']['map_version'] = 1
        with self.assertRaises(contract.ContractError) as caught:
            contract.check_pair(old, mapping)
        self.assertEqual(caught.exception.code, 'E_PAIR_MAP_VERSION')
        flat = load('map_2d_flat.json')
        with self.assertRaises(contract.ContractError) as caught:
            contract.check_pair(copy.deepcopy(task), flat)
        self.assertEqual(caught.exception.code, 'E_PAIR_MAP_KIND')
        other = copy.deepcopy(task)
        other['frame'] = 'odom'
        with self.assertRaises(contract.ContractError) as caught:
            contract.check_pair(other, mapping)
        self.assertEqual(caught.exception.code, 'E_PAIR_FRAME')
        # 缺字段的地图必须以理由码失败，不得裸抛 KeyError（三次复核缺口 B6）。
        scrub = copy.deepcopy(mapping)
        scrub.pop('frame')
        with self.assertRaises(contract.ContractError) as caught:
            contract.check_pair(task, scrub)
        self.assertEqual(caught.exception.code, 'E_PAIR_FRAME')
        no_map_id = copy.deepcopy(mapping)
        no_map_id.pop('map_id')
        with self.assertRaises(contract.ContractError) as caught:
            contract.check_pair(task, no_map_id)
        self.assertEqual(caught.exception.code, 'E_PAIR_MAP_ID')

    def test_validator_rejects_unsupported_keyword(self):
        schema = {'type': 'object', 'properties': {}, 'format': 'date'}
        with self.assertRaises(contract.ContractError) as caught:
            contract._walk(schema, {}, schema, '')
        self.assertEqual(caught.exception.code, 'E_SCHEMA_UNSUPPORTED')

    def test_validator_rejects_nested_additional_properties(self):
        schema = {'type': 'object', 'properties': {}, 'additionalProperties': {'type': 'string'}}
        with self.assertRaises(contract.ContractError) as caught:
            contract._walk(schema, {}, schema, '')
        self.assertEqual(caught.exception.code, 'E_SCHEMA_UNSUPPORTED')

    def test_validator_rejects_array_without_items(self):
        schema = {'type': 'array'}
        with self.assertRaises(contract.ContractError) as caught:
            contract._walk(schema, [1], schema, '')
        self.assertEqual(caught.exception.code, 'E_SCHEMA_UNSUPPORTED')

    def test_positive_control_removing_required_fails(self):
        data = load('task_2d_goal.json')
        data['goal'].pop('tolerance')
        self.assertEqual(reason('task', data), 'E_SCHEMA_REQUIRED')

    def test_cli_rc_and_reason(self):
        ok = subprocess.run([sys.executable, '-B', str(HERE / 'validate.py'), 'map',
                             str(HERE / 'fixtures/map_ml_two_surfaces.json')], capture_output=True, text=True)
        self.assertEqual(ok.returncode, 0, ok.stdout + ok.stderr)
        self.assertTrue(json.loads(ok.stdout)['ok'])
        bad = subprocess.run([sys.executable, '-B', str(HERE / 'validate.py'), 'map',
                              str(HERE / 'fixtures/neg_map_collapse.json')], capture_output=True, text=True)
        self.assertEqual(bad.returncode, 5)
        self.assertEqual(json.loads(bad.stdout)['reason'], 'E_MAP_ML_COLLAPSE')
        usage = subprocess.run([sys.executable, '-B', str(HERE / 'validate.py')], capture_output=True, text=True)
        self.assertEqual(usage.returncode, 9)


if __name__ == '__main__':
    unittest.main(verbosity=2)
