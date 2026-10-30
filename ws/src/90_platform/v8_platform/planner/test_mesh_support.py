"""Mesh top-face to support_map adapter tests (no MuJoCo/ROS)."""
import os
import sys
import tempfile
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / 'representation'))
sys.path.insert(0, str(HERE.parent / 'contracts'))
import mesh_support

DAE = '''<?xml version="1.0"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
  <library_geometries><geometry><mesh>
    <source id="p"><float_array id="positions-array" count="12">
      0 0 0.1  1 0 0.1  1 1 0.1  0 1 0.1
    </float_array></source>
    <source id="n"><float_array id="normals-array" count="12">
      0 0 1  0 0 1  0 0 1  0 0 1
    </float_array></source>
    <vertices id="v"><input semantic="POSITION" source="#p"/></vertices>
    <polylist count="1">
      <input semantic="VERTEX" source="#v" offset="0"/>
      <input semantic="NORMAL" source="#n" offset="1"/>
      <vcount>4</vcount><p>0 0 1 1 2 2 3 3</p>
    </polylist>
  </mesh></geometry></library_geometries>
</COLLADA>'''


class MeshSupportTests(unittest.TestCase):
    def test_top_face_and_map(self):
        with tempfile.TemporaryDirectory() as directory:
            dae = Path(directory) / 'one.dae'
            dae.write_text(DAE)
            faces = mesh_support.top_faces(dae)
            self.assertEqual(len(faces), 1)
            self.assertAlmostEqual(faces[0]['z'], .1, delta=1e-6)
            value, _ = mesh_support.build(dae, 'one_step', resolution=.1, margin=.5)
            self.assertEqual(len(value['surfaces']), 2)
            self.assertEqual(value['surfaces'][0]['surface_id'], 'ground')
            self.assertEqual(value['surfaces'][1]['surface_id'], 'step1')
            self.assertAlmostEqual(
                value['surfaces'][1]['edges'][0]['max_step_height_m'], .1, delta=1e-6)
            scaled, _ = mesh_support.build(dae, 'one_step_scaled', resolution=.1,
                                           margin=.5, z_scale=.8)
            self.assertAlmostEqual(scaled['surfaces'][1]['z_reference'], .08, delta=1e-6)
            front = mesh_support.faces_to_surfaces(faces, margin=.5, ground_y_max=0.)
            self.assertAlmostEqual(front[0]['footprint']['max_y'], .05, delta=1e-9)
            wide, _ = mesh_support.build(dae, 'one_step_wide', resolution=.2,
                                         margin=.5, xy_scale=2.)
            self.assertAlmostEqual(wide['surfaces'][1]['footprint']['max_x'], 2., delta=1e-6)

    def test_no_top_faces_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            dae = Path(directory) / 'down.dae'
            dae.write_text(DAE.replace('0 0 1  0 0 1  0 0 1  0 0 1',
                                       '0 0 -1  0 0 -1  0 0 -1  0 0 -1'))
            with self.assertRaises(ValueError):
                mesh_support.top_faces(dae)


if __name__ == '__main__':
    unittest.main()
