#!/usr/bin/env python3
import copy
import importlib.util
import unittest
from pathlib import Path

MODULE_PATH = Path(__file__).with_name("a1_sensor_chain_probe.py")
SPEC = importlib.util.spec_from_file_location("a1_sensor_chain_probe", MODULE_PATH)
PROBE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(PROBE)


def passing_report():
    return {
        "clock": {"count": 100, "nonadvancing": 0},
        "raw_lidar": {
            "count": 80,
            "source_rate_hz": 10.0,
            "unique_stamp": 80,
            "nonadvancing": 0,
            "callerids": ["/gazebo"],
            "frames": {"point_num": 20000, "nonzero_points": 5000},
        },
        "adapter_cloud": {
            "count": 80,
            "source_rate_hz": 10.0,
            "callerids": ["/sim_adapter"],
            "frames": {"frame_id": "rslidar", "width": 5000},
        },
        "imu": {
            "count": 1600,
            "source_rate_hz": 200.0,
            "nonadvancing": 0,
            "frames": {"frame_id": "rslidar"},
        },
        "odometry": {
            "count": 80,
            "nonadvancing": 0,
            "frames": {"frame_id": "camera_init", "child_frame_id": "body"},
        },
        "loaded_plugins": {"/opt/ros1-slam/sim_ws/devel/lib/liblivox_laser_simulation.so": {}},
    }


class VerdictTests(unittest.TestCase):
    def test_pass_requires_full_chain(self):
        self.assertEqual(PROBE.verdict_for(passing_report())["verdict"], "PASS")

    def test_missing_raw_lidar_fails(self):
        report = passing_report()
        report["raw_lidar"]["count"] = 0
        report["raw_lidar"]["frames"] = {}
        result = PROBE.verdict_for(report)
        self.assertEqual(result["verdict"], "FAIL")
        self.assertIn("raw_10hz", result["failed"])
        self.assertIn("raw_points_nonzero", result["failed"])

    def test_all_padding_points_fails(self):
        report = passing_report()
        report["raw_lidar"]["frames"]["nonzero_points"] = 0
        self.assertIn("raw_points_nonzero", PROBE.verdict_for(report)["failed"])

    def test_adapter_wrong_publisher_fails(self):
        report = passing_report()
        report["adapter_cloud"]["callerids"] = ["/gazebo"]
        self.assertIn("adapter_publisher_sim_adapter", PROBE.verdict_for(report)["failed"])

    def test_raw_stamp_stall_fails(self):
        report = copy.deepcopy(passing_report())
        report["raw_lidar"]["nonadvancing"] = 1
        report["raw_lidar"]["unique_stamp"] = 79
        self.assertIn("raw_stamp_advances", PROBE.verdict_for(report)["failed"])

    def test_plugin_absent_fails(self):
        report = passing_report()
        report["loaded_plugins"] = {}
        self.assertIn("plugin_loaded", PROBE.verdict_for(report)["failed"])


if __name__ == "__main__":
    unittest.main()