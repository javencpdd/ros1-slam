"""F6: verification backend selection, truth isolation, injected runner."""
import copy
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import config  # noqa: E402
import graph as graph_mod  # noqa: E402
import launch as launch_mod  # noqa: E402
import render  # noqa: E402
import verification  # noqa: E402

MANIFESTS = HERE / 'fixtures' / 'manifests'
BACKENDS = HERE / 'fixtures' / 'verification_manifests'
INDEX = HERE / 'fixtures' / 'package_index.json'
G2D = HERE / 'fixtures' / 'graphs' / 'graph_2d.yaml'
VERIFY_A = HERE / 'fixtures' / 'verification_fake_a.yaml'
VERIFY_B = HERE / 'fixtures' / 'verification_fake_b.yaml'
VERIFY_G = HERE / 'fixtures' / 'verification_gazebo.yaml'
VERIFY_M = HERE / 'fixtures' / 'verification_mujoco.yaml'


def setup():
    catalog = config.load_components(MANIFESTS)
    graph = config.load_graph(G2D)
    plan = render.render_plan(graph, catalog, graph_mod.check(graph, catalog))
    backends = verification.load_backends(BACKENDS)
    packages = launch_mod.load_index(INDEX)
    return catalog, plan, backends, packages


class VerificationTests(unittest.TestCase):
    def test_two_backends_share_algorithm_digest(self):
        catalog, plan, backends, packages = setup()
        a = verification.check(verification.load_config(VERIFY_A), plan, catalog,
                               backends, packages)
        b = verification.check(verification.load_config(VERIFY_B), plan, catalog,
                               backends, packages)
        self.assertEqual(a['algorithm_digest'], b['algorithm_digest'])
        self.assertNotEqual(a['backend'], b['backend'])
        self.assertNotEqual(a['verification_digest'], b['verification_digest'])

    def test_backend_must_be_a_simulator(self):
        catalog, plan, backends, packages = setup()
        spec = verification.load_config(VERIFY_A)
        spec['backend'] = 'v8.fw.map2d'
        with self.assertRaises(config.FrameworkError) as caught:
            verification.check(spec, plan, catalog, backends, packages)
        self.assertEqual(caught.exception.code, 'E_VERIFY_BACKEND')

    def test_backend_inside_algorithm_graph_is_rejected(self):
        catalog, plan, backends, packages = setup()
        polluted = copy.deepcopy(plan)
        polluted['instances'].append({'id': 'sim', 'component': 'v8.fw.sim_fake_a'})
        with self.assertRaises(config.FrameworkError) as caught:
            verification.check(verification.load_config(VERIFY_A), polluted, catalog,
                               backends, packages)
        self.assertEqual(caught.exception.code, 'E_VERIFY_BACKEND_IN_PLAN')

    def test_truth_channel_must_not_be_an_algorithm_topic(self):
        catalog, plan, backends, packages = setup()
        spec = verification.load_config(VERIFY_A)
        spec['truth_channel'] = '/v8_support_map'
        with self.assertRaises(config.FrameworkError) as caught:
            verification.check(spec, plan, catalog, backends, packages)
        self.assertEqual(caught.exception.code, 'E_VERIFY_TRUTH_LEAK')

    def test_gazebo_backend_is_selected_outside_algorithm_graph(self):
        catalog, plan, backends, packages = setup()
        checked = verification.check(verification.load_config(VERIFY_G), plan,
                                     catalog, backends, packages)
        self.assertEqual(checked['backend'], 'v8.verify.gazebo_truth')
        self.assertEqual(checked['truth_channel'], '/v8/verify/truth')
        self.assertEqual(checked['algorithm_digest'], plan['digest'])
        self.assertNotIn(checked['backend'],
                         {item['component'] for item in plan['instances']})

    def test_mujoco_backend_is_selected_outside_algorithm_graph(self):
        catalog, plan, backends, packages = setup()
        checked = verification.check(verification.load_config(VERIFY_M), plan,
                                     catalog, backends, packages)
        self.assertEqual(checked['backend'], 'v8.verify.mujoco_truth')
        self.assertEqual(checked['truth_channel'], '/v8/verify/truth_mj')
        self.assertEqual(checked['algorithm_digest'], plan['digest'])

    def test_runner_success_and_failure_keep_algorithm_digest(self):
        catalog, plan, backends, packages = setup()
        spec = verification.load_config(VERIFY_A)
        checked = verification.check(spec, plan, catalog, backends, packages)
        ok = verification.run(spec, checked, lambda _: {'metric': 1.0})
        self.assertEqual(ok['status'], 'ok')
        self.assertEqual(ok['algorithm_digest'], plan['digest'])
        bad = verification.run(spec, checked, lambda _: (_ for _ in ()).throw(RuntimeError('boom')))
        self.assertEqual(bad['status'], 'error')
        self.assertEqual(bad['algorithm_digest'], plan['digest'])
        self.assertIn('boom', bad['error'])


if __name__ == '__main__':
    unittest.main()
