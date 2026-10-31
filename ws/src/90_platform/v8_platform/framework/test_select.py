"""V8-X3 selector: variant precheck, selected graph, HTML page."""
import io
import json
import sys
import tempfile
import unittest
from contextlib import redirect_stdout
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / 'contracts'))
import config  # noqa: E402
import graph as graph_mod  # noqa: E402
import selector  # noqa: E402

MANIFESTS = HERE / 'fixtures' / 'manifests'
G2D = HERE / 'fixtures' / 'graphs' / 'graph_2d.yaml'


def setup():
    return config.load_components(MANIFESTS), config.load_graph(G2D)


class SelectorTests(unittest.TestCase):
    def test_variant_matrix_marks_single_swaps(self):
        catalog, graph = setup()
        data = selector.variant_matrix(graph, catalog)
        by_id = {item['id']: item for item in data['instances']}
        robot = {v['component']: v for v in by_id['robot']['variants']}
        self.assertTrue(robot['v8.fw.robot_lite3']['ok'])
        self.assertTrue(robot['v8.fw.robot_test_b']['ok'])
        local = {v['component']: v for v in by_id['local']['variants']}
        self.assertTrue(local['v8.fw.local2d']['ok'])
        self.assertFalse(local['v8.fw.local3d']['ok'])
        self.assertEqual(local['v8.fw.local3d']['reason'], 'E_GRAPH_DIMENSION')

    def test_selected_graph_passes_check_with_new_digest(self):
        catalog, graph = setup()
        selected = selector.selected_graph(graph, {'robot': 'v8.fw.robot_test_b'})
        report = graph_mod.check(selected, catalog)
        self.assertTrue(report['ok'])
        self.assertEqual(report['instances']['robot']['component'], 'v8.fw.robot_test_b')
        self.assertNotEqual(report['digest'], graph_mod.check(graph, catalog)['digest'])

    def test_combination_validates_whole_choice_set(self):
        catalog, graph = setup()
        ok = selector.combination(graph, catalog, {'robot': 'v8.fw.robot_test_b'})
        self.assertTrue(ok['ok'])
        expected = graph_mod.check(
            selector.selected_graph(graph, {'robot': 'v8.fw.robot_test_b'}), catalog)
        self.assertEqual(ok['digest'], expected['digest'])
        self.assertEqual(ok['command_writers'], expected['command_writers'])
        bad = selector.combination(graph, catalog, {'local': 'v8.fw.local3d'})
        self.assertFalse(bad['ok'])
        self.assertEqual(bad['reason'], 'E_GRAPH_DIMENSION')
        self.assertEqual(selector.combination(graph, catalog, {'nope': 'v8.fw.robot_test_b'})['reason'],
                         'E_SELECT_INSTANCE')
        self.assertEqual(selector.combination(graph, catalog, {'robot': 'v8.fw.nope'})['reason'],
                         'E_SELECT_COMPONENT')
        self.assertEqual(selector.combination(graph, catalog, {'robot': 'v8.fw.map2d'})['reason'],
                         'E_SELECT_ROLE')

    def test_cli_rejects_bad_combination_and_exports_good_one(self):
        with tempfile.TemporaryDirectory() as directory:
            html = Path(directory) / 'selector.html'
            out = Path(directory) / 'selector.json'
            selected = Path(directory) / 'graph_selected.json'
            with redirect_stdout(io.StringIO()):
                rc = selector.main([str(MANIFESTS), str(G2D), '--out-html', str(html),
                                    '--choose', 'local=v8.fw.local3d'])
            self.assertEqual(rc, 5)
            self.assertFalse(html.exists())
            buffer = io.StringIO()
            with redirect_stdout(buffer):
                rc = selector.main([str(MANIFESTS), str(G2D), '--out-html', str(html),
                                    '--out-json', str(out), '--out-selected', str(selected),
                                    '--choose', 'robot=v8.fw.robot_test_b'])
            self.assertEqual(rc, 0)
            payload = json.loads(out.read_text())
            self.assertTrue(payload['combination']['ok'])
            self.assertEqual(payload['combination']['choices'],
                             {'robot': 'v8.fw.robot_test_b'})
            saved = json.loads(selected.read_text())
            self.assertEqual(saved['instances']['robot']['component'], 'v8.fw.robot_test_b')
            self.assertTrue(json.loads(buffer.getvalue())['combination'])

    def test_html_is_self_contained_and_exportable(self):
        catalog, graph = setup()
        data = selector.variant_matrix(graph, catalog)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'selector.html'
            path.write_text(selector._html(data, str(MANIFESTS), str(G2D)), encoding='utf-8')
            text = path.read_text(encoding='utf-8')
            self.assertIn('v8.fw.robot_test_b', text)
            self.assertIn("createElement('select')", text)
            self.assertIn('graph_selected.json', text)
            self.assertIn('sensors', text)


if __name__ == '__main__':
    unittest.main()
