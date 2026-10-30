"""F5: robot profile gating and command/telemetry port contract."""
import copy
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import config  # noqa: E402
import graph as graph_mod  # noqa: E402
import render  # noqa: E402
import robot  # noqa: E402

MANIFESTS = HERE / 'fixtures' / 'manifests'
G2D = HERE / 'fixtures' / 'graphs' / 'graph_2d.yaml'
LITE3 = HERE.parent / 'contracts' / 'fixtures' / 'robot_lite3_design.json'
ROBOT_B = HERE / 'fixtures' / 'robot_test_b.json'


class RobotTests(unittest.TestCase):
    def test_declared_profile_cannot_drive_planning(self):
        profile = robot.load_profile(LITE3)
        with self.assertRaises(config.FrameworkError) as caught:
            robot.limits(profile)
        self.assertEqual(caught.exception.code, 'E_ROBOT_UNVERIFIED')

    def test_tested_profile_limits(self):
        profile = robot.load_profile(ROBOT_B)
        limits = robot.limits(profile)
        self.assertEqual(limits['max_step_height_m'], 0.1)
        self.assertEqual(limits['max_slope_deg'], 20.0)
        self.assertEqual(limits['max_decel_mps2'], 1.2)

    def test_adapter_ports(self):
        catalog = config.load_components(MANIFESTS)
        for component_id in ('v8.fw.robot_lite3', 'v8.fw.robot_test_b'):
            ports = robot.adapter_ports(catalog['components'][component_id])
            self.assertEqual(ports['command']['type'], 'geometry_msgs/TwistStamped')
            self.assertEqual(sorted(ports['telemetry']), ['imu', 'odom'])
        broken = copy.deepcopy(catalog['components']['v8.fw.robot_test_b'])
        broken['outputs'] = []
        with self.assertRaises(config.FrameworkError) as caught:
            robot.adapter_ports(broken)
        self.assertEqual(caught.exception.code, 'E_ROBOT_PORTS')

    def test_robot_switch_changes_launch_not_ports(self):
        catalog = config.load_components(MANIFESTS)
        value = config.load_graph(G2D)
        switched = copy.deepcopy(value)
        switched['instances']['robot']['component'] = 'v8.fw.robot_test_b'
        before = graph_mod.check(value, catalog)
        after = graph_mod.check(switched, catalog)
        self.assertEqual(before['command_writers'], after['command_writers'])
        plan = render.render_plan(switched, catalog, after)
        instance = next(item for item in plan['instances'] if item['id'] == 'robot')
        self.assertEqual(instance['component'], 'v8.fw.robot_test_b')
        self.assertEqual(instance['launch']['type'], 'robot_test_b')
        self.assertTrue(instance['launch']['test_double'])


if __name__ == '__main__':
    unittest.main()
