import copy
import unittest
import verify_evidence as evidence


class EvidenceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.baseline = evidence.load()

    def rejects(self, mutate):
        data = copy.deepcopy(self.baseline)
        mutate(data)
        with self.assertRaises((AssertionError, KeyError)):
            evidence.validate(data)
        self.assertEqual(evidence.validate(self.baseline), 65)

    def test_positive(self):
        self.assertEqual(evidence.validate(self.baseline), 65)

    def test_missing_check(self):
        self.rejects(lambda d: d['runtime']['checks'].pop('basic_stop'))

    def test_false_check(self):
        self.rejects(lambda d: d['runtime']['checks'].update(ray_geometry=False))

    def test_result_hash(self):
        self.rejects(lambda d: d.update(result_sha256='0' * 64))

    def test_script_drift(self):
        self.rejects(lambda d: d['snapshot_hashes'].update({'runtime.py': '0' * 64}))

    def test_empty_snapshot(self):
        self.rejects(lambda d: d.update(snapshot_hashes={}))

    def test_source_drift(self):
        self.rejects(lambda d: d['source_hashes'].update({'CMakeLists.txt': '0' * 64}))

    def test_loaded_library(self):
        self.rejects(lambda d: d['runtime']['loaded_files']['costmap'].update({evidence.LIB: '0' * 64}))

    def test_old_workspace(self):
        self.rejects(lambda d: d['runtime']['loaded_files']['costmap'].update({'/opt/ros1-slam/ws/devel/lib/old.so': '0' * 64}))

    def test_empty_sensor(self):
        self.rejects(lambda d: d['runtime']['scan'].update(stamps=[]))

    def test_duplicate_sensor_time(self):
        self.rejects(lambda d: d['runtime']['imu']['stamps'].__setitem__(1, d['runtime']['imu']['stamps'][0]))

    def test_fake_publisher(self):
        self.rejects(lambda d: d['runtime']['scan'].update(publisher='/fake'))

    def test_duplicate_tf(self):
        self.rejects(lambda d: d['runtime']['tf_writers']['map->odom'].append('/other'))

    def test_outside_map(self):
        self.rejects(lambda d: d['runtime']['scan']['ranges'].__setitem__(0, 9.0))

    def test_empty_map(self):
        self.rejects(lambda d: d['runtime']['map'].update(occupied=0))

    def test_forced_stop(self):
        self.rejects(lambda d: d['runtime']['stopped'][0].update(forced=True))

    def test_wrong_run(self):
        self.rejects(lambda d: d['runtime'].update(run='/opt/ros1-slam/r625/runtime-other'))

    def test_shared_resource_change(self):
        self.rejects(lambda d: d['stage'].update(after={'changed': True}))

    def test_host_network(self):
        self.rejects(lambda d: d['stage']['isolation']['HostConfig'].update(NetworkMode='host'))


if __name__ == '__main__':
    unittest.main(verbosity=2)
