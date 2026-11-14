#!/usr/bin/env python3
"""B2: swapping the global planner must not change the BT instance/tree.

Positive: an alternative planner manifest with the same named ports checks OK
and the BT instance is byte-identical to the original graph.
Negative: a planner whose `path` output type differs is refused by graph.check.
"""
import json
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent
FRAMEWORK = HERE.parent
REPO = FRAMEWORK.parents[4]
MANIFESTS = FRAMEWORK / 'fixtures' / 'manifests'
GRAPH = FRAMEWORK / 'fixtures' / 'graphs' / 'graph_3d_gazebo_bt.yaml'
INDEX = FRAMEWORK / 'fixtures' / 'package_index.json'


def check(manifests, graph):
    proc = subprocess.run(
        [sys.executable, '-B', str(FRAMEWORK / 'cli.py'), 'check', str(manifests),
         str(graph), '--package-index', str(INDEX)],
        cwd=str(REPO), text=True, capture_output=True)
    try:
        report = json.loads(proc.stdout)
    except ValueError:
        report = None
    return proc.returncode, report, (proc.stdout or '') + (proc.stderr or '')


def bt_instance(graph):
    value = yaml.safe_load(Path(graph).read_text(encoding='utf-8'))
    return next(item for item in value['instances'] if item['id'] == 'bt')


class PlannerSwapContractTests(unittest.TestCase):
    def test_same_port_swap_keeps_bt_and_wrong_type_fails(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp = Path(tmp)
            fixtures = tmp / 'fixtures'
            shutil.copytree(FRAMEWORK / 'fixtures', fixtures)
            manifests = fixtures / 'manifests'
            base = json.loads((MANIFESTS / 'manifest_global3d_topic.json').read_text(encoding='utf-8'))
            alt = json.loads(json.dumps(base))
            alt['id'] = 'v8.fw.global3d_alt'
            (manifests / 'manifest_global3d_alt.json').write_text(
                json.dumps(alt, ensure_ascii=False, indent=1) + '\n', encoding='utf-8')
            bad = json.loads(json.dumps(base))
            bad['id'] = 'v8.fw.global3d_bad'
            for port in bad['ports']['outputs']:
                if port['name'] == 'path':
                    port['type'] = 'nav_msgs/NotAPath'
            (manifests / 'manifest_global3d_bad.json').write_text(
                json.dumps(bad, ensure_ascii=False, indent=1) + '\n', encoding='utf-8')
            text = (fixtures / 'graphs' / 'graph_3d_gazebo_bt.yaml').read_text(encoding='utf-8')
            alt_graph = fixtures / 'graphs' / 'graph_bt_alt.yaml'
            alt_graph.write_text(text.replace('component: v8.fw.global3d_topic',
                                              'component: v8.fw.global3d_alt'), encoding='utf-8')
            bad_graph = fixtures / 'graphs' / 'graph_bt_bad.yaml'
            bad_graph.write_text(text.replace('component: v8.fw.global3d_topic',
                                              'component: v8.fw.global3d_bad'), encoding='utf-8')

            code, report, raw = check(manifests, alt_graph)
            self.assertEqual(code, 0, raw[-800:])
            self.assertTrue(report and report.get('ok'), raw[-800:])
            self.assertEqual(bt_instance(GRAPH), bt_instance(alt_graph))

            code, report, raw = check(manifests, bad_graph)
            self.assertNotEqual(code, 0, raw[-800:])
            self.assertIn('E_GRAPH', raw)


if __name__ == '__main__':
    unittest.main()
