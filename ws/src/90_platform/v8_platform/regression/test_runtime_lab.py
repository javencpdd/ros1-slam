"""Runtime manifest must name the backend that actually ran."""
import copy
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import runtime_lab


BASE = {
    'combo': 'lite3_dwa_mux',
    'config_lock': {
        'planner': 'move_base global_planner/GlobalPlanner + '
                   'dwa_local_planner/DWAPlannerROS',
        'stack': 'flat2d_stack_sc09_f64nli25.launch（DWA 参数在 DWAPlannerROS ns；DWA 默认块）',
    },
}


class ManifestBackendTests(unittest.TestCase):
    def test_teb_names_teb_and_keeps_template(self):
        original = copy.deepcopy(BASE)
        result = runtime_lab.manifest_for_backend(BASE, 'teb')
        self.assertEqual(result['backend'], 'teb')
        self.assertEqual(result['combo'], 'lite3_global_planner_teb_mux')
        self.assertIn('TebLocalPlannerROS', result['local_planner'])
        self.assertIn('teb_local_planner/TebLocalPlannerROS',
                      result['config_lock']['planner'])
        self.assertIn('flat2d_stack_sc09_f64nli25_teb.launch',
                      result['config_lock']['stack'])
        self.assertIn('TEB 参数在 TebLocalPlannerROS ns',
                      result['config_lock']['stack'])
        self.assertIn('TEB 参数块', result['config_lock']['stack'])
        self.assertNotIn('DWA', result['config_lock']['stack'])
        self.assertEqual(BASE, original)

    def test_navfn_global_backend_is_named(self):
        result = runtime_lab.manifest_for_backend(BASE, 'dwa', 'navfn')
        self.assertEqual(result['global_backend'], 'navfn')
        self.assertEqual(result['global_planner'], 'navfn/NavfnROS')
        self.assertIn('navfn/NavfnROS', result['config_lock']['planner'])
        self.assertEqual(result['combo'], 'lite3_navfn_dwa_mux')

    def test_dwa_names_dwa(self):
        result = runtime_lab.manifest_for_backend(BASE, 'dwa')
        self.assertEqual(result['backend'], 'dwa')
        self.assertEqual(result['combo'], 'lite3_global_planner_dwa_mux')
        self.assertIn('DWAPlannerROS', result['local_planner'])
        self.assertIn('dwa_local_planner/DWAPlannerROS',
                      result['config_lock']['planner'])


if __name__ == '__main__':
    unittest.main()
