"""Parity guard: the sim's frozen support-cell copy matches the platform contract."""
import importlib.util
import os
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / 'scripts'))
import support_cells  # noqa: E402

ROOT = Path(os.environ.get('V8_REPO_ROOT') or (HERE.parents[3].parent / 'ros1-slam')).resolve()
PLATFORM = ROOT / 'ws' / 'src' / '90_platform' / 'v8_platform' / 'representation'


def platform_support_map():
    spec = importlib.util.spec_from_file_location(
        'platform_support_map', PLATFORM / 'support_map.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class SupportCellParityTests(unittest.TestCase):
    def test_copy_matches_platform_contract(self):
        try:
            platform = platform_support_map()
        except Exception as error:  # platform source may be absent in install-only P2
            self.skipTest('platform support_map unavailable: %s' % error)
        surfaces = [
            {'footprint': {'min_x': 0, 'max_x': 1, 'min_y': 0, 'max_y': 1}},
            {'footprint': {'min_x': 0, 'max_x': 2, 'min_y': 0, 'max_y': 2},
             'cells': {'runs': [[0, 0, 2], [1, 0, 2]]}},
        ]
        for surface in surfaces:
            self.assertEqual(support_cells.surface_cells(surface),
                             platform.surface_cells(surface))
            for x, y in ((0.5, 0.5), (1.5, 0.5), (-0.1, 0.5), (2.0, 2.0), (1.9, 1.9)):
                for margin in (0.0, -0.1):
                    self.assertEqual(
                        support_cells.contains_xy(surface, 1.0, x, y, margin=margin),
                        platform.contains_xy(surface, 1.0, x, y, margin=margin),
                        (surface, x, y, margin))


if __name__ == '__main__':
    unittest.main()
