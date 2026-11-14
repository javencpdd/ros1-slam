#!/usr/bin/env python3
"""Simulation carrier decoupling: the app declares the sim backend.

The navigation platform must not know the simulation package or its paths.
These checks pin the app-declared contract and the legacy default it preserves.
"""
import os
import sys as _sys
from pathlib import Path as _Path

_BASE = _Path(__file__).resolve().parent.parent
for _entry in ('', 'nodes', 'tools', 'smokes'):
    _sys.path.insert(0, str(_BASE / _entry) if _entry else str(_BASE))
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
FRAMEWORK = HERE.parent
sys.path.insert(0, str(FRAMEWORK))
sys.path.insert(0, str(FRAMEWORK.parent / 'contracts'))
import app_config  # noqa: E402
import app_runner  # noqa: E402
import config  # noqa: E402

ROOT = app_config.ROOT
SIM_ROOT = Path(os.environ.get('V8_SIM_ROOT') or (ROOT.parent / 'ros1-slam-simulation'))
os.environ.setdefault('V8_SIM_ROOT', str(SIM_ROOT))
DECLARATION = 'sim_ws/src/v8_simulation/config/sim_backends.yaml'


def app(world_launch=None, backend='gazebo', declaration=DECLARATION):
    carrier = {'backend': backend}
    if world_launch is not None:
        carrier['world_launch'] = world_launch
    if declaration is not None:
        carrier['simulation_declaration'] = declaration
    return {'carrier': carrier}


@unittest.skipUnless(SIM_ROOT.is_dir(), 'P2 simulation repo not beside P1')
class SimBackendTests(unittest.TestCase):
    def test_declared_world_launch_resolves(self):
        value = app_runner.resolve_sim_launch(
            app('sim_ws/src/lite3_gazebo_sim/launch/app_world.launch'), root=SIM_ROOT)
        self.assertTrue(value['declared'])
        self.assertTrue(value['argv'][0].endswith('app_world.launch'))

    def test_undeclared_world_launch_rejected(self):
        with self.assertRaises(config.FrameworkError) as caught:
            app_runner.resolve_sim_launch(
                app('sim_ws/src/lite3_gazebo_sim/launch/other.launch'), root=SIM_ROOT)
        self.assertEqual(caught.exception.code, 'E_EXEC_BACKEND')

    def test_legacy_default_preserved(self):
        value = app_runner.resolve_sim_launch(app(), root=SIM_ROOT)
        self.assertFalse(value['declared'])
        self.assertEqual(value['argv'], ['lite3_gazebo_sim', 'nist_stairs_120_s020x170.launch'])

    def test_missing_declaration_fails_closed(self):
        with self.assertRaises(config.FrameworkError) as caught:
            app_runner.resolve_sim_launch(app(declaration=None), root=SIM_ROOT)
        self.assertEqual(caught.exception.code, 'E_EXEC_BACKEND')

    def test_undeclared_backend_rejected(self):
        with self.assertRaises(config.FrameworkError) as caught:
            app_runner.resolve_sim_launch(app(backend='kinematic'), root=SIM_ROOT)
        self.assertEqual(caught.exception.code, 'E_EXEC_BACKEND')


if __name__ == '__main__':
    unittest.main()
