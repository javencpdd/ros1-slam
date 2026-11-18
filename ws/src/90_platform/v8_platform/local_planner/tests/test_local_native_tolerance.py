"""Support tolerance must be consistent between resolve and collision checks."""
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'representation'))
import local_native
from test_local_native import layers


class SupportToleranceTests(unittest.TestCase):
    def test_slightly_below_support_is_accepted_as_same_surface(self):
        result = local_native.plan(
            layers(), (0.19999976, 0.0, 0.14999994), (0.6, 0.0, 0.15), now=100.0)
        self.assertEqual(result['reason'], 'arrived')
        self.assertGreaterEqual(len(result['trajectory']), 2)

    def test_outside_support_tolerance_is_still_rejected(self):
        with self.assertRaises(local_native.LocalError) as caught:
            local_native.plan(
                layers(), (0.2, 0.0, 0.149999), (0.6, 0.0, 0.15), now=100.0)
        self.assertEqual(caught.exception.code, 'E_LOCAL_SUPPORT')


if __name__ == '__main__':
    unittest.main()
