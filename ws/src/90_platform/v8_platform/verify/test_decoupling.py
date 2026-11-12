#!/usr/bin/env python3
"""Three-project boundary guards.

P1 navigation platform, P2 simulation, P3 mapping/navigation visualization.
Each project must run without the others' source; these checks fail on the
couplings that would silently reappear.
"""
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[4]
P1 = REPO / 'ws/src/90_platform/v8_platform'
P2 = REPO / 'ws/src/95_simulation/v8_simulation'
CONSOLE = REPO / 'scripts/console'
RUNTIME_DIRS = ('framework', 'registry', 'composer', 'contracts', 'orchestrator',
                'representation', 'planner', 'local_planner')


def python_files(root):
    for path in Path(root).rglob('*.py'):
        if '__pycache__' in path.parts:
            continue
        if path.name.startswith('test_') or 'fixtures' in path.parts:
            continue
        yield path


class DecouplingTests(unittest.TestCase):
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
                self.assertNotIn('ws/src/95_simulation', text, path)

    def test_console_agent_is_read_only(self):
        for path in Path(CONSOLE).rglob('*.py'):
            if '__pycache__' in path.parts:
                continue
            text = path.read_text(encoding='utf-8')
            for token in ('rospy.Publisher', 'advertise(', 'rostopic pub'):
                self.assertNotIn(token, text, '%s: %s' % (path, token))


if __name__ == '__main__':
    unittest.main()
