import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from localization_state import LocalizationState


def healthy(state, start=1.0, stamp=1.0, position=(0, 0, 0)):
    for i in range(3):
        state.observe(stamp + i * 0.05, position, start + i * 0.05)


class LocalizationStateTests(unittest.TestCase):
    def test_startup_requires_stable_samples_and_explicit_lease(self):
        state = LocalizationState()
        self.assertFalse(state.activate(1, 1.0))
        healthy(state)
        self.assertEqual(state.state, 'HEALTHY')
        self.assertFalse(state.tick(1.11))
        self.assertTrue(state.activate(1, 1.11))
        self.assertTrue(state.tick(1.12))

    def test_stale_revokes_once_and_old_lease_cannot_revive(self):
        state = LocalizationState()
        healthy(state)
        state.activate(7, 1.11)
        self.assertFalse(state.tick(1.40))
        self.assertEqual(state.reason, 'E_LOC_STALE')
        self.assertEqual(state.revocations, 1)
        state.tick(1.50)
        self.assertEqual(state.faults, 1)
        healthy(state, start=2.0, stamp=2.0)
        self.assertFalse(state.tick(2.11))
        self.assertFalse(state.activate(7, 2.11))
        self.assertTrue(state.activate(8, 2.11))

    def test_jump_and_relocalization_invalidate_even_with_fresh_messages(self):
        state = LocalizationState()
        healthy(state)
        state.activate(1, 1.11)
        self.assertFalse(state.observe(1.2, (2, 0, 0), 1.2))
        self.assertEqual(state.reason, 'E_LOC_JUMP')
        healthy(state, start=1.3, stamp=1.3, position=(2, 0, 0))
        self.assertTrue(state.activate(2, 1.41))
        state.relocalize()
        self.assertEqual(state.state, 'RELOCALIZING')
        self.assertFalse(state.tick(1.42))
        healthy(state, start=1.5, stamp=1.5, position=(-2, 0, 0))
        self.assertTrue(state.activate(3, 1.61))
        self.assertEqual(state.revocations, 2)

    def test_stale_or_invalid_sample_never_refreshes_receipt(self):
        state = LocalizationState()
        healthy(state)
        state.activate(1, 1.11)
        state.observe(1.0, (0, 0, 0), 1.2)
        self.assertEqual(state.reason, 'E_LOC_STAMP_NONMONOTONIC')
        self.assertEqual(state.last_received, 1.1)
        state.observe(float('nan'), (0, 0, 0), 1.3)
        self.assertEqual(state.reason, 'E_LOC_INVALID')
        self.assertFalse(state.tick(1.31))

    def test_invalid_covariance_flag_revokes(self):
        state = LocalizationState()
        healthy(state)
        state.activate(1, 1.11)
        self.assertFalse(state.observe(1.2, (0, 0, 0), 1.2, valid=False))
        self.assertEqual(state.revocations, 1)

    def test_backend_rejection_holds_even_if_old_pose_is_republished(self):
        state = LocalizationState()
        healthy(state)
        state.activate(1, 1.11)
        state.backend_health(False)
        healthy(state, start=1.2, stamp=1.2)
        self.assertEqual(state.reason, 'E_LOC_BACKEND_REJECTED')
        self.assertFalse(state.activate(2, 1.31))
        state.backend_health(True)
        self.assertNotEqual(state.state, 'HEALTHY')
        healthy(state, start=1.4, stamp=1.4)
        self.assertTrue(state.activate(2, 1.51))


if __name__ == '__main__':
    unittest.main(verbosity=2)
