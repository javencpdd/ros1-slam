"""Hand-checkable statics and experiment isolation; no MuJoCo runtime claimed."""
import hashlib
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import force_crawl as force


class ForceStaticsTests(unittest.TestCase):
    def test_reaction_torque_sign_from_static_virtual_work(self):
        jacobian = np.array([[0., .1], [.2, 0.], [.3, -.4]])
        reaction = np.array([0., 0., 100.])
        bias = np.array([1., 2.])
        torque = force.reaction_feedforward(jacobian, reaction, bias)
        np.testing.assert_allclose(torque, [-29., 42.])
        dq = np.array([.2, -.3])
        self.assertAlmostEqual(float((torque - bias) @ dq + reaction @ (jacobian @ dq)), 0.)

    def test_lever_arm_cross_force_has_correct_moment_sign(self):
        matrix = force.wrench_matrix([[1., 0., 0.]], np.zeros(3))
        np.testing.assert_allclose(matrix @ [0., 0., 10.], [0., 0., 10., 0., -10., 0.])

    def test_four_contact_standing_has_no_pull_and_balances_weight(self):
        points = np.array([[.3, .2, 0.], [.3, -.2, 0.], [-.3, .2, 0.], [-.3, -.2, 0.]])
        forces, info = force.allocate_contact_forces(points, [0., 0., .35], [0, 0, 120, 0, 0, 0])
        np.testing.assert_allclose(forces[:, 2], [30, 30, 30, 30], atol=1e-5)
        self.assertLess(info['force_residual_N'], 1e-5)
        self.assertLess(info['moment_residual_Nm'], 1e-5)

    def test_three_supports_at_different_heights_can_balance_static_wrench(self):
        points = np.array([[.3, .2, .10], [.3, -.2, .10], [-.3, 0., 0.]])
        forces, info = force.allocate_contact_forces(points, [0., 0., .35], [0, 0, 120, 0, 0, 0])
        self.assertTrue(np.all(forces[:, 2] > 0))
        self.assertLess(info['force_residual_N'], 1e-5)
        self.assertLess(info['moment_residual_Nm'], 1e-5)
        self.assertLessEqual(info['max_friction_ratio'], .8 + 1e-8)

    def test_infeasible_outside_support_wrench_is_reported_not_hidden(self):
        points = np.array([[.3, .2, .10], [.3, -.2, .10], [-.3, 0., 0.]])
        forces, info = force.allocate_contact_forces(points, [1.5, 0., .35], [0, 0, 120, 0, 0, 0])
        self.assertTrue(np.all(forces[:, 2] >= 0))
        self.assertGreater(info['moment_residual_Nm'], 1.)
        self.assertLessEqual(info['max_friction_ratio'], .8 + 1e-8)

    def test_friction_projection_is_not_componentwise_clipping(self):
        projected = force.project_friction([[6., 8., -1.], [0., 0., -5.]], .5)
        np.testing.assert_allclose(projected[0], [.96, 1.28, 3.2], atol=1e-8)
        np.testing.assert_allclose(projected[1], [0., 0., 0.])

    def test_five_cm_margin_gate_is_not_lowered_by_orientation(self):
        triangle = np.array([[-1., -1., 0], [1., -1., 0], [0., 1., 0]])
        self.assertGreater(force.triangle_margin([0, 0], triangle), .05)
        self.assertLess(force.triangle_margin([0, -.96], triangle), .05)
        self.assertAlmostEqual(force.triangle_margin([0, -1.1], triangle[::-1]), -.1)

    def test_xml_step_is_only_in_memory_and_original_hash_does_not_change(self):
        captured = []
        class Model:
            @staticmethod
            def from_xml_string(xml):
                captured.append(xml)
                return types.SimpleNamespace(ngeom=0)
        stub = types.SimpleNamespace(MjModel=Model)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'model.xml'
            path.write_text('<mujoco><compiler meshdir="meshes"/><asset><mesh file="foot.STL"/></asset><worldbody/></mujoco>')
            before = hashlib.sha256(path.read_bytes()).hexdigest()
            with patch.dict(sys.modules, mujoco=stub):
                force.load_model(path, step_h=.06, step_x=1.)
            self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(), before)
            tree = ET.fromstring(captured[0])
            step = tree.find('worldbody/geom')
            self.assertEqual(step.get('size'), '2 2 0.03')
            self.assertTrue(Path(tree.find('compiler').get('meshdir')).is_absolute())
            self.assertEqual(tree.find('asset/mesh').get('file'), str(Path(directory) / 'meshes' / 'foot.STL'))

    def test_partial_stair_never_passes_and_model_loader_is_restored(self):
        import crawl_gait
        class Model:
            from_xml_string = staticmethod(lambda xml: None)
        stub = types.SimpleNamespace(MjModel=Model)
        partial = {'pass': True, 'world_feet_end': {'FL': [1.1, .1, .082], 'FR': [1.1, -.1, .082],
                                                   'HL': [.7, .1, .022], 'HR': [.7, -.1, .022]}}
        with patch.dict(sys.modules, mujoco=stub), patch.object(crawl_gait, 'run', return_value=partial):
            result = force.run_stair('unused.xml', backend='pd')
            self.assertFalse(result['pass'])
            self.assertFalse(result['full_stair_completed'])
            self.assertIs(stub.MjModel, Model)
        with patch.dict(sys.modules, mujoco=stub), patch.object(crawl_gait, 'run', side_effect=RuntimeError('probe')):
            with self.assertRaises(RuntimeError):
                force.run_stair('unused.xml', backend='pd')
            self.assertIs(stub.MjModel, Model)


if __name__ == '__main__':
    unittest.main(verbosity=2)
