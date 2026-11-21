#!/usr/bin/env python3
import unittest
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[5]
BT = ROOT / 'ws/src/90_platform/lite3_bt'
NAV = ROOT / 'ws/src/40_navigation/lite3_navigation'
BRINGUP = ROOT / 'ws/src/70_bringup/lite3_bringup/launch'


class NavigationContractTest(unittest.TestCase):
    def test_active_entries_no_longer_include_mbf(self):
        for name in ('mode_navigation', 'mode_nav_stub', 'mode_nav_sim', 'mode_sim_teb'):
            with self.subTest(name=name):
                includes = [node.get('file') for node in ET.parse(BRINGUP / (name + '.launch')).iter('include')]
                self.assertFalse(any('mbf_navigation' in value for value in includes))
                self.assertTrue(any('navigation_executor.launch' in value for value in includes))
                self.assertTrue(any('bt_navigation.launch' in value for value in includes))

    def test_backend_preserves_costmap_files(self):
        old = ET.parse(NAV / 'launch/mbf_navigation.launch').find('node')
        new = ET.parse(NAV / 'launch/navigation_executor.launch').find('node')
        self.assertEqual((new.get('pkg'), new.get('type')), ('move_base', 'move_base'))
        shared = lambda node: {(item.get('file').rsplit('/', 1)[-1], item.get('ns', ''))
                               for item in node.findall('rosparam') if 'costmap' in item.get('file', '')}
        self.assertEqual(shared(old), shared(new))
        remaps = {item.get('from'): item.get('to') for item in new.findall('remap')}
        self.assertEqual(remaps['cmd_vel'], '/v8/navigation/cmd_vel_raw_candidate')
        self.assertNotEqual(remaps['move_base_simple/goal'], '/move_base_simple/goal')

    def test_action_tree_terminates_before_dwell_and_recovery(self):
        tree = ET.parse(BT / 'trees/navigate_with_action.xml')
        for name, after in (('mission', 'Dwell'), ('recover_after_termination', 'Recovery')):
            children = list(tree.find('.//Sequence[@name="' + name + '"]'))
            tags = [item.tag for item in children]
            self.assertLess(tags.index('StopExecutor'), tags.index(after))
        self.assertIsNone(tree.find('.//RequestPath'))

    def test_sim_uses_mux_not_relay(self):
        tree = ET.parse(BRINGUP / 'mode_sim_teb.launch')
        includes = tree.findall('include')
        base = next(node for node in includes if 'm2_teb.launch' in node.get('file'))
        self.assertEqual(base.find('arg[@name="cmd_relay"]').get('value'), 'false')
        self.assertTrue(any('60_safety.launch' in node.get('file') for node in includes))

    def test_robot_flags_and_profile_stay_fail_closed(self):
        tree = ET.parse(BRINGUP / 'mode_navigation.launch')
        self.assertEqual(tree.find('arg[@name="enable_motion"]').get('default'), 'false')
        self.assertEqual(tree.find('arg[@name="enable_bt"]').get('default'), 'false')
        self.assertEqual(tree.find('arg[@name="bt_profile"]').get('default'), 'real')
        self.assertEqual(tree.find('node[@type="ros2qnx"]').get('if'), '$(arg enable_motion)')
        self.assertFalse((BT / 'config/bt_console_real.yaml').exists())

    def test_rviz_goal_enters_bt_not_backend(self):
        tree = ET.parse(BRINGUP / 'interactive_2d.launch')
        self.assertIsNone(tree.find('node[@name="mbf_goal_bridge"]'))
        self.assertEqual(tree.find('include/arg[@name="goal_topic"]').get('value'), '/move_base_simple/goal')


if __name__ == '__main__':
    unittest.main()
