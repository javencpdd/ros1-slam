"""V8-X3 hot swap: compatibility gate and stop-before-start apply."""
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import config  # noqa: E402
import graph as graph_mod  # noqa: E402
import hotswap  # noqa: E402
import render  # noqa: E402
import runtime  # noqa: E402

MANIFESTS = HERE / 'fixtures' / 'manifests'
G2D = HERE / 'fixtures' / 'graphs' / 'graph_2d.yaml'
PY = sys.executable
SLEEPER = "import time; print('READY', flush=True); time.sleep(3600)"


def plan_for():
    catalog = config.load_components(MANIFESTS)
    graph = config.load_graph(G2D)
    report = graph_mod.check(graph, catalog)
    return render.render_plan(graph, catalog, report), catalog


def fake(_instance, _component):
    return [PY, '-c', SLEEPER], 'READY'


class HotSwapTests(unittest.TestCase):
    def test_prepare_compatible_robot_swap(self):
        plan, catalog = plan_for()
        record, new_plan = hotswap.prepare(plan, catalog, 'robot', 'v8.fw.robot_test_b',
                                           {'task': 'nav', 'progress': 3}, epoch=2)
        self.assertEqual(record['old_component'], 'v8.fw.robot_lite3')
        self.assertEqual(record['new_component'], 'v8.fw.robot_test_b')
        self.assertEqual(record['state'], {'task': 'nav', 'progress': 3})
        self.assertTrue(record['preconditions']['no_command_overlap'])
        self.assertNotEqual(new_plan['digest'], plan['digest'])
        robot = next(item for item in new_plan['instances'] if item['id'] == 'robot')
        self.assertEqual(robot['component'], 'v8.fw.robot_test_b')
        self.assertEqual(robot['launch']['type'], 'robot_test_b')

    def test_incompatible_ports_are_rejected(self):
        plan, catalog = plan_for()
        with self.assertRaises(config.FrameworkError) as caught:
            hotswap.prepare(plan, catalog, 'local', 'v8.fw.local3d')
        self.assertEqual(caught.exception.code, 'E_SWAP_PORTS')

    def test_unknown_instance_and_component(self):
        plan, catalog = plan_for()
        with self.assertRaises(config.FrameworkError) as caught:
            hotswap.prepare(plan, catalog, 'missing', 'v8.fw.robot_test_b')
        self.assertEqual(caught.exception.code, 'E_SWAP_INSTANCE')
        with self.assertRaises(config.FrameworkError) as caught:
            hotswap.prepare(plan, catalog, 'robot', 'v8.fw.missing')
        self.assertEqual(caught.exception.code, 'E_SWAP_COMPONENT')

    def test_record_validation(self):
        plan, catalog = plan_for()
        record, _ = hotswap.prepare(plan, catalog, 'robot', 'v8.fw.robot_test_b')
        broken = dict(record)
        broken['epoch'] = -1
        with self.assertRaises(config.FrameworkError):
            hotswap.validate_record(broken)
        broken = dict(record)
        broken['state'] = {'bad': object()}
        with self.assertRaises(config.FrameworkError):
            hotswap.validate_record(broken)

    def test_apply_stops_old_before_new_starts(self):
        plan, catalog = plan_for()
        record, new_plan = hotswap.prepare(plan, catalog, 'robot', 'v8.fw.robot_test_b',
                                           {'task': 'nav'})
        old = runtime.Assembly(plan, catalog, command_factory=fake, ready_timeout=10)
        old.start()
        old_runners = list(old.runners.values())
        self.assertTrue(old.ready())
        record, new = hotswap.apply(old, catalog, record, new_plan, ready_timeout=10)
        self.addCleanup(new.stop)
        self.assertTrue(record['preconditions']['assembly_stopped'])
        self.assertTrue(record['postconditions']['new_ready'])
        self.assertEqual(record['epoch'], 1)
        self.assertTrue(all(not runner.is_alive() for runner in old_runners))
        self.assertTrue(new.ready())
        self.assertEqual(new.runners['robot'].spec.roles, ['robot_adapter'])
        self.assertFalse(old.started)


if __name__ == '__main__':
    unittest.main()
