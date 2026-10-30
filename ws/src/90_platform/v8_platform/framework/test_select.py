"""V8-X3 selector: variant precheck, selected graph, HTML page."""
import sys
import tempfile
import unittest
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
