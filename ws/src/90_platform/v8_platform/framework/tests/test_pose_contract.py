import math
import sys
import unittest
from pathlib import Path

FRAMEWORK = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(FRAMEWORK))
import pose_contract as pc  # noqa: E402

MOUNT_XYZ = (0.0, 0.0, 0.34)
MOUNT_RPY = (0.0, math.radians(15.0), 0.0)
BASE0_XYZ = (5.2, 2.45, 0.11)
BASE0_RPY = (0.0, 0.0, 0.0)
CONTACT_DZ = -0.11


def assert_pose_close(test, got, want, tol=1e-6):
    for index, (left, right) in enumerate(zip(got[:3], want[:3])):
        test.assertAlmostEqual(left, right, delta=tol, msg='translation axis %d' % index)
    for index, (left, right) in enumerate(zip(got[3:], want[3:])):
        test.assertAlmostEqual(left, right, delta=tol, msg='quaternion component %d' % index)


class PoseContractTests(unittest.TestCase):
    def test_extrinsic_inverse_translation_and_rotation(self):
        xyz, rpy = pc.invert_extrinsic(MOUNT_XYZ, MOUNT_RPY)
        self.assertAlmostEqual(xyz[0], 0.34 * math.sin(math.radians(15.0)), delta=1e-9)
        self.assertAlmostEqual(xyz[1], 0.0, delta=1e-9)
        self.assertAlmostEqual(xyz[2], -0.34 * math.cos(math.radians(15.0)), delta=1e-9)
        self.assertAlmostEqual(rpy[0], 0.0, delta=1e-9)
        self.assertAlmostEqual(rpy[1], math.radians(-15.0), delta=1e-9)
        self.assertAlmostEqual(rpy[2], 0.0, delta=1e-9)

    def test_inverse_round_trip_for_nonzero_pose(self):
        pose = pc.rigid((1.2, -0.7, 0.8), (0.2, -0.3, 1.1))
        assert_pose_close(self, pc.compose(pose, pc.inverse(pose)), pc.rigid((0, 0, 0), (0, 0, 0)))

    def test_full_t1_t2_recovers_nonzero_base_attitude(self):
        initial_map_base = pc.rigid(BASE0_XYZ, BASE0_RPY)
        mount = pc.rigid(MOUNT_XYZ, MOUNT_RPY)
        map_camera = pc.compose(initial_map_base, mount)
        t2_xyz, t2_rpy = pc.invert_extrinsic(MOUNT_XYZ, MOUNT_RPY)

        for map_base in (
            pc.rigid((5.8, 2.1, 0.4), (0.0, 0.0, math.radians(35.0))),
            pc.rigid((4.3, 3.2, 0.6), (0.12, -0.18, -0.7)),
            pc.rigid((6.0, 1.0, 0.2), (0.05, 0.22, 2.4)),
        ):
            map_lidar = pc.compose(map_base, mount)
            lio_pose = pc.compose(pc.inverse(map_camera), map_lidar)
            recovered = pc.lio_pose_to_base(
                lio_pose, map_camera[:3], pc.rpy_from_quat(map_camera[3:]), t2_xyz, t2_rpy)
            assert_pose_close(self, recovered, map_base, tol=2e-6)

    def test_pure_translation_t2_is_not_vacuous(self):
        initial_map_base = pc.rigid(BASE0_XYZ, BASE0_RPY)
        mount = pc.rigid(MOUNT_XYZ, MOUNT_RPY)
        map_camera = pc.compose(initial_map_base, mount)
        map_base = pc.rigid((5.8, 2.1, 0.4), (0.12, -0.18, 0.7))
        map_lidar = pc.compose(map_base, mount)
        lio_pose = pc.compose(pc.inverse(map_camera), map_lidar)
        wrong = pc.lio_pose_to_base(
            lio_pose, (0.0, 0.0, 0.45), (0.0, 0.0, 0.0), (0.0, 0.0, -0.34), (0.0, 0.0, 0.0))
        error = math.sqrt(sum((wrong[i] - map_base[i]) ** 2 for i in range(3)))
        self.assertGreater(error, 0.10)

    def test_contact_offset_is_world_z_from_base_reference(self):
        base = pc.rigid((5.2, 2.45, 0.25), (0.1, -0.2, 0.5))
        contact = pc.contact_pose(base, CONTACT_DZ)
        self.assertAlmostEqual(contact[2], 0.14, delta=1e-9)
        self.assertEqual(contact[3:], pc.normalize_quat(base[3:]))

    def test_rpy_round_trip(self):
        rpy = (0.2, -0.3, 1.1)
        self.assertTrue(all(abs(a - b) < 1e-9 for a, b in zip(pc.rpy_from_quat(pc.quat_from_rpy(*rpy)), rpy)))


if __name__ == '__main__':
    unittest.main()