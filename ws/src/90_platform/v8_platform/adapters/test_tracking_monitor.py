"""SC11 tracking monitor positive/negative controls."""
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import tracking_monitor

REFERENCE = [(0., 0., 0.), (1., 0., 0.)]
CLEAN = (0., .05, 0.)
DEVIATED = (.5, .05, 0.)


class TrackingMonitorTests(unittest.TestCase):
    def test_clean_sample_is_accepted(self):
        monitor = tracking_monitor.TrackingMonitor(REFERENCE)
        result = monitor.observe(CLEAN)
        self.assertTrue(result['ok'])
        self.assertFalse(result['stop'])
        self.assertEqual(result['tracker'], 'accept')

    def test_two_consecutive_deviations_stop(self):
        monitor = tracking_monitor.TrackingMonitor(REFERENCE)
        first = monitor.observe(DEVIATED)
        self.assertEqual(first['tracker'], 'replan')
        self.assertFalse(first['stop'])
        second = monitor.observe(DEVIATED)
        self.assertEqual(second['reason'], tracking_monitor.REASON)
        self.assertEqual(second['tracker'], 'stop')
        self.assertTrue(second['stop'])
        self.assertTrue(monitor.stopped)
        latched = monitor.observe(CLEAN)
        self.assertTrue(latched['stop'])
        self.assertEqual(latched['reason'], 'E_TRACK_STOP_LATCHED')

    def test_clean_sample_resets_consecutive_count(self):
        monitor = tracking_monitor.TrackingMonitor(REFERENCE)
        self.assertEqual(monitor.observe(DEVIATED)['tracker'], 'replan')
        self.assertEqual(monitor.observe(CLEAN)['tracker'], 'accept')
        after_reset = monitor.observe(DEVIATED)
        self.assertEqual(after_reset['tracker'], 'replan')
        self.assertFalse(after_reset['stop'])

    def test_empty_reference_is_rejected(self):
        with self.assertRaises(ValueError):
            tracking_monitor.TrackingMonitor([])


if __name__ == '__main__':
    unittest.main()
