#!/usr/bin/env python3
"""Simulation carrier decoupling: the platform asks v8_simulation, not a path.

The navigation platform must not hard-code a simulator package or world.  These
checks pin the declaration boundary and the legacy default it preserves.
"""
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / 'contracts'))
import app_config  # noqa: E402
import app_runner  # noqa: E402
import config  # noqa: E402

ROOT = app_config.ROOT


def app(world_launch=None, backend='gazebo'):
    carrier = {'backend': backend}
    if world_launch is not None:
        carrier['world_launch'] = world_launch
    return {'carrier': carrier}


class SimBackendTests(unittest.TestCase):
    def test_declared_world_launch_resolves(self):
        value = app_runner.resolve_sim_launch(
            app('ws/src/95_simulation/lite3_gazebo_sim/launch/app_world.launch'), root=ROOT)
        self.assertTrue(value['declared'])
        self.assertTrue(value['argv'][0].endswith('app_world.launch'))

    def test_undeclared_world_launch_rejected(self):
        with self.assertRaises(config.FrameworkError) as caught:
            app_runner.resolve_sim_launch(
                app('ws/src/95_simulation/lite3_gazebo_sim/launch/other.launch'), root=ROOT)
        self.assertEqual(caught.exception.code, 'E_EXEC_BACKEND')

    def test_legacy_default_preserved(self):
        value = app_runner.resolve_sim_launch(app(), root=ROOT)
        self.assertFalse(value['declared'])
        self.assertEqual(value['argv'], ['lite3_gazebo_sim', 'nist_stairs_120_s020x170.launch'])

    def test_missing_declaration_fails_closed(self):
        with self.assertRaises(config.FrameworkError) as caught:
            app_runner.resolve_sim_launch(app(), root=ROOT,
                                          platform_root=HERE / 'fixtures' / 'no_such_root')
        self.assertEqual(caught.exception.code, 'E_EXEC_BACKEND')

    def test_undeclared_backend_rejected(self):
        with self.assertRaises(config.FrameworkError) as caught:
            app_runner.resolve_sim_launch(app(backend='kinematic'), root=ROOT)
        self.assertEqual(caught.exception.code, 'E_EXEC_BACKEND')


if __name__ == '__main__':
    unittest.main()
