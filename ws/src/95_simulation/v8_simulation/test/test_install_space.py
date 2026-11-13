"""Install-space guard: P2 geometry modules import without platform source."""
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

PACKAGE = Path(__file__).resolve().parents[1]
SCRIPTS = PACKAGE / 'scripts'
MODULES = ('carrier_kinematics.py', 'support_cells.py')


class InstallSpaceTests(unittest.TestCase):
    def test_geometry_modules_import_without_platform_source(self):
        with tempfile.TemporaryDirectory() as directory:
            for name in MODULES:
                shutil.copy(SCRIPTS / name, Path(directory) / name)
            code = (
                'import sys\n'
                'sys.path[:] = [%r] + [p for p in sys.path '
                'if "90_platform" not in p and "/ws/src" not in p]\n'
                'import carrier_kinematics\n'
                'import support_cells\n'
                'surface = {"footprint": {"min_x": 0, "max_x": 1, '
                '"min_y": 0, "max_y": 1}}\n'
                'print(support_cells.contains_xy(surface, 1.0, 0.5, 0.5))\n'
            ) % directory
            result = subprocess.run([sys.executable, '-B', '-c', code],
                                    capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertEqual(result.stdout.strip(), 'True')

    def test_cmake_installs_every_runtime_entry(self):
        cmake = (PACKAGE / 'CMakeLists.txt').read_text(encoding='utf-8')
        for name in ('carrier_kinematics.py', 'support_cells.py',
                     'gazebo_carrier3d_node.py', 'gazebo_truth.py', 'mujoco_truth.py'):
            self.assertIn(name, cmake)
        self.assertIn('install(DIRECTORY launch', cmake)
        self.assertIn('install(DIRECTORY fixtures', cmake)


if __name__ == '__main__':
    unittest.main()
