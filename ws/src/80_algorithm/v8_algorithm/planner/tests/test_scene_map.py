import json, os, sys, tempfile, unittest
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import scene_map

WORLD = '''<sdf version="1.6"><world name="w">
<model name="support_ground"><static>true</static><pose>0 0 -0.05 0 0 0</pose>
<link name="l"><collision name="c"><geometry><box><size>4 4 0.1</size></box></geometry></collision></link></model>
<model name="support_deck"><static>true</static><pose>1 0 0.3 0 0 0</pose>
<link name="l"><collision name="c"><geometry><box><size>1 1 0.1</size></box></geometry></collision></link></model>
</world></sdf>'''

SCENE = {"map_id": "deck_scene", "map_version": 1, "frame": "map", "resolution": 0.1,
         "origin": [-2.0, -2.0, 0.0], "extent": {"width": 40, "height": 40},
         "surfaces": [
             {"world_model": "support_ground", "surface_id": "ground", "z_reference": 0.0,
              "thickness_m": 0.1, "clearance_min_m": 0.4,
              "footprint": {"min_x": -2.0, "min_y": -2.0, "max_x": 2.0, "max_y": 2.0},
              "edges": [{"to": "deck", "kind": "step", "max_step_height_m": 0.4, "bidirectional": True}]},
             {"world_model": "support_deck", "surface_id": "deck", "z_reference": 0.35,
              "thickness_m": 0.1, "clearance_min_m": 0.4,
              "footprint": {"min_x": 0.5, "min_y": -0.5, "max_x": 1.5, "max_y": 0.5},
              "edges": [{"to": "ground", "kind": "step", "max_step_height_m": 0.4, "bidirectional": True}]}]}


class SceneMapTests(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.mkdtemp()
        self.world = os.path.join(self.dir, 'w.world')
        self.scene = os.path.join(self.dir, 's.json')
        with open(self.world, 'w') as stream:
            stream.write(WORLD)
        with open(self.scene, 'w') as stream:
            json.dump(SCENE, stream)

    def test_build_matches_world(self):
        value = scene_map.build(self.world, self.scene)
        self.assertEqual(len(value['surfaces']), 2)
        self.assertEqual(value['surfaces'][1]['z_reference'], 0.35)
        self.assertEqual(len(value['source']['sha256']), 64)

    def test_wrong_footprint_rejected(self):
        bad = json.loads(json.dumps(SCENE))
        bad['surfaces'][0]['footprint']['max_x'] = 9.0
        with open(self.scene, 'w') as stream:
            json.dump(bad, stream)
        with self.assertRaises(ValueError):
            scene_map.build(self.world, self.scene)

    def test_wrong_z_rejected(self):
        bad = json.loads(json.dumps(SCENE))
        bad['surfaces'][1]['z_reference'] = 1.0
        with open(self.scene, 'w') as stream:
            json.dump(bad, stream)
        with self.assertRaises(ValueError):
            scene_map.build(self.world, self.scene)


if __name__ == '__main__':
    unittest.main(verbosity=2)
