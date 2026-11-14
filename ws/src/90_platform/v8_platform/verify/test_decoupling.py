#!/usr/bin/env python3
"""Three-project boundary guards after the physical repo split.

P1 navigation platform, P2 simulation, P3 mapping/navigation visualization.
Each lives in its own repository and catkin/host workspace; these checks fail
on couplings that would silently reappear. Roots resolve from V8_SIM_ROOT and
V8_CONSOLE_ROOT, defaulting to the sibling repositories.
"""
import os
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[4]
P1 = REPO / 'ws/src/90_platform/v8_platform'
SIM_ROOT = Path(os.environ.get('V8_SIM_ROOT') or (REPO.parent / 'ros1-slam-simulation'))
CONSOLE_ROOT = Path(os.environ.get('V8_CONSOLE_ROOT') or (REPO.parent / 'ros1-slam-console'))
P2 = SIM_ROOT / 'ws/src/v8_simulation'
CONSOLE = CONSOLE_ROOT / 'console'
RUNTIME_DIRS = ('framework', 'registry', 'composer', 'contracts', 'orchestrator',
                'representation', 'planner', 'local_planner')


def python_files(root):
    root = Path(root)
    if not root.is_dir():
        return
    for path in root.rglob('*.py'):
        if '__pycache__' in path.parts:
            continue
        if path.name.startswith('test_') or 'fixtures' in path.parts:
            continue
        yield path


class DecouplingTests(unittest.TestCase):
    def test_projects_are_physically_separate(self):
        self.assertFalse((REPO / 'ws/src/95_simulation').exists(),
                         'P2 must live in its own repository, not P1')
        self.assertFalse((REPO / 'scripts/console').exists(),
                         'P3 must live in its own repository, not P1')
        self.assertTrue((P2 / 'package.xml').is_file(), P2)
        self.assertTrue((CONSOLE / 'agent.py').is_file(), CONSOLE)

    def test_p2_has_own_workspace_and_release(self):
        self.assertTrue((SIM_ROOT / 'ws/src').is_dir(), SIM_ROOT)
        self.assertTrue((SIM_ROOT / 'Makefile').is_file(), SIM_ROOT)
        self.assertTrue((SIM_ROOT / 'scripts/release_manifest.py').is_file(), SIM_ROOT)
        self.assertTrue((SIM_ROOT / 'README.md').is_file(), SIM_ROOT)

    def test_p3_has_own_workspace_and_release(self):
        self.assertTrue((CONSOLE_ROOT / 'Makefile').is_file(), CONSOLE_ROOT)
        self.assertTrue((CONSOLE / 'paths.py').is_file(), CONSOLE)
        self.assertTrue((CONSOLE_ROOT / 'scripts/release_manifest.py').is_file(), CONSOLE_ROOT)
        self.assertTrue((CONSOLE_ROOT / 'README.md').is_file(), CONSOLE_ROOT)

    def test_sim_runtime_has_no_platform_source_dependency(self):
        for path in python_files(P2 / 'scripts'):
            text = path.read_text(encoding='utf-8')
            for token in ('import v8_platform', 'from v8_platform', 'rospkg', '_source_entry'):
                self.assertNotIn(token, text, path)

    def test_platform_runtime_has_no_hardcoded_sim_path(self):
        for name in RUNTIME_DIRS:
            directory = P1 / name
            if not directory.is_dir():
                continue
            for path in python_files(directory):
                text = path.read_text(encoding='utf-8')
                for token in ('ws/src/95_simulation', 'ros1-slam-simulation'):
                    self.assertNotIn(token, text, path)

    def test_platform_runtime_has_no_sim_import(self):
        for name in RUNTIME_DIRS:
            directory = P1 / name
            if not directory.is_dir():
                continue
            for path in python_files(directory):
                text = path.read_text(encoding='utf-8')
                for token in ('import v8_simulation', 'from v8_simulation'):
                    self.assertNotIn(token, text, path)

    def test_sim_launch_has_no_platform_package_reference(self):
        for path in Path(SIM_ROOT).rglob('*'):
            if path.suffix not in ('.launch', '.xml') or not path.is_file():
                continue
            self.assertNotIn('$(find v8_platform)', path.read_text(encoding='utf-8'), path)

    def test_console_sources_have_no_platform_or_sim_import(self):
        for path in Path(CONSOLE).rglob('*.py'):
            if '__pycache__' in path.parts:
                continue
            text = path.read_text(encoding='utf-8')
            for token in ('import v8_platform', 'from v8_platform',
                          'import v8_simulation', 'from v8_simulation'):
                self.assertNotIn(token, text, '%s: %s' % (path, token))

    def test_console_agent_is_read_only(self):
        for path in Path(CONSOLE).rglob('*.py'):
            if '__pycache__' in path.parts:
                continue
            text = path.read_text(encoding='utf-8')
            for token in ('rospy.Publisher', 'advertise(', 'rostopic pub'):
                self.assertNotIn(token, text, '%s: %s' % (path, token))


if __name__ == '__main__':
    unittest.main()
