import contextlib
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import stair_pilot


class StairPilotTests(unittest.TestCase):
    def test_green_phase_prefix_does_not_certify_all_feet_landed(self):
        with tempfile.TemporaryDirectory() as directory:
            xml, out = Path(directory) / 'world.xml', Path(directory) / 'result.json'
            xml.write_text('<mujoco/>')
            result = {'pass': True, 'completed_steps': 10, 'fell': False,
                      'world_feet_end': {'FL': [1.1, .2, .082], 'FR': [1.1, -.2, .082],
                                         'HL': [.8, .2, .022], 'HR': [.8, -.2, .022]}}
            with patch.object(sys, 'argv', ['stair_pilot', '--xml', str(xml), '--out', str(out)]), \
                    patch.object(stair_pilot.crawl_gait, 'run', return_value=result), \
                    contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(stair_pilot.main(), 6)
            observed = json.loads(out.read_text())
            self.assertFalse(observed['pass'])
            self.assertFalse(observed['full_stair_completed'])
            self.assertEqual(observed['verdict'], 'FAIL')

    def test_front_refresh_and_paired_schedule_reach_the_runner(self):
        with tempfile.TemporaryDirectory() as directory:
            xml, out = Path(directory) / 'world.xml', Path(directory) / 'result.json'
            xml.write_text('<mujoco/>')
            result = {'pass': True, 'completed_steps': 18, 'fell': False,
                      'world_feet_end': {leg: [1.1, .2, .082] for leg in ('FL', 'FR', 'HL', 'HR')}}
            with patch.object(sys, 'argv', ['stair_pilot', '--xml', str(xml), '--out', str(out),
                                           '--paired', '--front-refresh']), \
                    patch.object(stair_pilot.crawl_gait, 'run', return_value=result) as run, \
                    contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(stair_pilot.main(), 0)
            self.assertTrue(run.call_args.kwargs['rear_advance_pair'])
            self.assertTrue(run.call_args.kwargs['front_refresh'])


if __name__ == '__main__':
    unittest.main()
