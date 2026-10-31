import sys
from pathlib import Path
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from localization_state import LocalizationState
from recovery_supervisor import RecoverySupervisor, TRACKING, RELOCALIZING, RECOVERING, FAILED


def healthy(state, start=1.0, stamp=1.0, position=(0, 0, 0), count=3):
    for i in range(count):
        state.observe(stamp + i * 0.05, position, start + i * 0.05)


class RecoverySupervisorTests(unittest.TestCase):
    def make(self, deadline_s=5.0):
        state = LocalizationState()
        healthy(state)
        self.assertTrue(state.activate(1, 1.11))
        calls = []
        supervisor = RecoverySupervisor(
            state, lambda: calls.append('relocalize'),
            lambda lease, pose: calls.append(('switch', lease, pose)),
            score_min=0.5, deadline_s=deadline_s, bounds=((-10, 10), (-10, 10)))
        return state, supervisor, calls

    def test_stale_triggers_one_relocalize_and_blocks_commands(self):
        state, supervisor, calls = self.make()
        self.assertTrue(supervisor.tick(1.2))
        self.assertFalse(supervisor.tick(2.0))
        self.assertEqual(supervisor.phase, RELOCALIZING)
        self.assertEqual(calls, ['relocalize'])
        self.assertFalse(supervisor.command_allowed(2.0))

    def test_low_score_or_out_of_bounds_candidate_is_rejected(self):
        state, supervisor, calls = self.make()
        self.assertFalse(supervisor.tick(2.0))
        self.assertFalse(supervisor.offer_candidate((0, 0, 0), 0.1, 2.1))
        self.assertFalse(supervisor.offer_candidate((50, 0, 0), 1.0, 2.1))
        self.assertEqual(supervisor.rejected, 2)
        self.assertEqual(calls, ['relocalize'])

    def test_valid_candidate_switches_and_new_lease_revives(self):
        state, supervisor, calls = self.make()
        self.assertFalse(supervisor.tick(2.0))
        self.assertTrue(supervisor.offer_candidate((1, 2, 0), 0.9, 2.1))
        self.assertEqual(supervisor.phase, RECOVERING)
        self.assertEqual(calls, ['relocalize', ('switch', 2, (1.0, 2.0, 0.0))])
        healthy(state, start=2.2, stamp=2.2, position=(1, 2, 0))
        self.assertTrue(supervisor.tick(2.4))
        self.assertEqual(supervisor.phase, TRACKING)
        self.assertEqual(state.active_lease, 2)
        self.assertEqual(supervisor.recoveries, 1)

    def test_deadline_fails_closed(self):
        state, supervisor, calls = self.make(deadline_s=0.5)
        self.assertFalse(supervisor.tick(2.0))
        self.assertFalse(supervisor.tick(2.6))
        self.assertEqual(supervisor.phase, FAILED)
        self.assertFalse(supervisor.command_allowed(2.7))

    def test_candidate_after_failure_is_ignored(self):
        state, supervisor, calls = self.make(deadline_s=0.5)
        supervisor.tick(2.0)
        supervisor.tick(2.6)
        self.assertFalse(supervisor.offer_candidate((0, 0, 0), 1.0, 2.7))
        self.assertEqual(len(calls), 1)


if __name__ == '__main__':
    unittest.main()
