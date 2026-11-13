#!/usr/bin/env python3
"""Port conformance tests: an external truth table breaks declaration consensus.

Two manifests can agree with each other and both be wrong; graph.check accepts
that.  These checks prove the conformance layer catches what the pairwise check
cannot, using a live topic/type snapshot as the truth source.
"""
import copy
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
PLATFORM = HERE.parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(PLATFORM / 'framework'))
sys.path.insert(0, str(PLATFORM / 'contracts'))
import config  # noqa: E402
import graph  # noqa: E402
import port_conformance  # noqa: E402

TRUTH = HERE / 'port_truth.json'
GRAPH = PLATFORM / 'framework' / 'fixtures' / 'graphs' / 'graph_3d_gazebo_bt.yaml'
CATALOG = PLATFORM / 'framework' / 'fixtures' / 'manifests'


def command_port(component, direction, name):
    for port in component[direction]:
        if port['name'] == name:
            return port
    raise KeyError(name)


class PortConformanceTests(unittest.TestCase):
    def setUp(self):
        self.catalog = config.load_components(CATALOG)
        self.value = config.load_graph(GRAPH)
        self.report = graph.check(self.value, self.catalog)
        self.truth = port_conformance.load_truth(TRUTH)

    def test_bt_graph_conforms_to_live_snapshot(self):
        result = port_conformance.check(self.report, self.catalog, self.truth)
        self.assertTrue(result['ok'], result['violations'])
        self.assertGreaterEqual(result['checked'], 30)

    def test_declared_type_mismatch_is_caught(self):
        catalog = copy.deepcopy(self.catalog)
        command_port(catalog['components']['v8.fw.bt_console'], 'inputs', 'path')['type'] = 'std_msgs/String'
        result = port_conformance.check(self.report, catalog, self.truth)
        self.assertFalse(result['ok'])
        self.assertTrue(any(v['code'] == 'E_PORT_TYPE' and v['at'] == 'bt.path'
                            for v in result['violations']))

    def test_unobserved_topic_fails_closed(self):
        truth = copy.deepcopy(self.truth)
        truth['topics'].pop('/v8/3d/global/path')
        result = port_conformance.check(self.report, self.catalog, truth)
        self.assertFalse(result['ok'])
        self.assertTrue(any(v['code'] == 'E_PORT_UNOBSERVED' for v in result['violations']))

    def test_port_without_topic_is_rejected(self):
        catalog = copy.deepcopy(self.catalog)
        component = catalog['components']['v8.fw.bt_console']
        command_port(component, 'outputs', 'lease').pop('topic', None)
        result = port_conformance.check(self.report, catalog, self.truth)
        self.assertFalse(result['ok'])
        self.assertTrue(any(v['code'] == 'E_PORT_NO_TOPIC' for v in result['violations']))

    def test_truth_version_is_gated(self):
        import json
        import tempfile
        with tempfile.NamedTemporaryFile('w', suffix='.json', delete=False) as handle:
            json.dump({'schema': 'v8_port_truth/v0', 'topics': {}}, handle)
        try:
            with self.assertRaises(port_conformance.PortError) as caught:
                port_conformance.load_truth(handle.name)
            self.assertEqual(caught.exception.code, 'E_PORT_TRUTH_VERSION')
        finally:
            Path(handle.name).unlink()

    def test_two_wrong_declarations_do_not_cross_certify(self):
        # Both sides of the edge agree on Twist, so the pairwise graph check
        # passes; the live topic is TwistStamped, so conformance must fail.
        catalog = copy.deepcopy(self.catalog)
        command_port(catalog['components']['v8.fw.local_native'], 'outputs', 'cmd')['type'] = 'geometry_msgs/Twist'
        command_port(catalog['components']['v8.fw.cmd3d_mux'], 'inputs', 'cmd')['type'] = 'geometry_msgs/Twist'
        report = graph.check(self.value, catalog)
        result = port_conformance.check(report, catalog, self.truth)
        self.assertFalse(result['ok'])
        self.assertTrue(any(v['code'] == 'E_PORT_TYPE' and v['at'].endswith('.cmd')
                            for v in result['violations']))


if __name__ == '__main__':
    unittest.main()
