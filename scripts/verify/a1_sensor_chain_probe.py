#!/usr/bin/env python3
"""A1 sensor-chain probe: read-only evidence for raw lidar, adapter, IMU, clock and LIO odometry.

The Gazebo Mid360 plugin only publishes when /livox/lidar has a subscriber, so this
probe's subscription is the sensor activation step. It never rewrites stamps or injects data.
"""
import argparse
import glob
import hashlib
import json
import os
import statistics
import time

import rospy
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, PointCloud2


TOPICS = {
    "clock": "/clock",
    "raw_lidar": "/livox/lidar",
    "adapter_cloud": "/rslidar_points",
    "imu": "/livox/imu",
    "odometry": "/Odometry",
}


def sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def loaded_gazebo_plugins():
    found = {}
    for comm in glob.glob("/proc/[0-9]*/comm"):
        try:
            with open(comm, "r", encoding="utf-8") as stream:
                if stream.read().strip() != "gzserver":
                    continue
        except OSError:
            continue
        pid = comm.split("/")[2]
        maps_path = "/proc/%s/maps" % pid
        try:
            with open(maps_path, "r", encoding="utf-8", errors="replace") as stream:
                for line in stream:
                    for part in line.split():
                        if part.startswith("/") and part.endswith("liblivox_laser_simulation.so"):
                            found.setdefault(part, os.path.realpath(part))
        except OSError:
            pass
    return {
        path: {"realpath": real, "sha256": sha256(real)}
        for path, real in sorted(found.items())
    }


def summarize(name, samples, duration):
    stamps = [item["stamp"] for item in samples if item["stamp"] is not None]
    deltas = [b - a for a, b in zip(stamps, stamps[1:])]
    stamp_span = stamps[-1] - stamps[0] if len(stamps) >= 2 else 0.0
    return {
        "topic": TOPICS[name],
        "count": len(samples),
        "wall_rate_hz": round(len(samples) / duration, 3) if duration > 0 else 0.0,
        "source_rate_hz": round((len(stamps) - 1) / stamp_span, 3) if stamp_span > 0 else 0.0,
        "first_stamp": stamps[0] if stamps else None,
        "last_stamp": stamps[-1] if stamps else None,
        "unique_stamp": len(set(stamps)),
        "median_delta_s": round(statistics.median(deltas), 6) if deltas else None,
        "nonadvancing": sum(1 for delta in deltas if delta <= 1e-9),
        "frames": {key: value for key, value in samples[0].items() if key != "stamp"} if samples else {},
        "callerids": sorted({item["callerid"] for item in samples if item["callerid"]}),
    }


def verdict_for(report):
    raw = report["raw_lidar"]
    adapter = report["adapter_cloud"]
    imu = report["imu"]
    odom = report["odometry"]
    clock = report["clock"]
    checks = {
        "clock_alive": clock["count"] >= 5 and clock["nonadvancing"] == 0,
        "raw_10hz": raw["count"] >= 5 and 5.0 <= raw["source_rate_hz"] <= 15.0,
        "raw_stamp_advances": raw["count"] >= 2 and raw["unique_stamp"] >= max(2, raw["count"] * 0.9) and raw["nonadvancing"] == 0,
        "raw_points_nonzero": raw["frames"].get("point_num", 0) > 0 and raw["frames"].get("nonzero_points", 0) > 0,
        "raw_publisher_known": bool(raw["callerids"]),
        "adapter_10hz": adapter["count"] >= 5 and 5.0 <= adapter["source_rate_hz"] <= 15.0,
        "adapter_frame_rslidar": adapter["frames"].get("frame_id") == "rslidar",
        "adapter_points_nonzero": adapter["frames"].get("width", 0) > 0,
        "adapter_publisher_sim_adapter": any("sim_adapter" in cid for cid in adapter["callerids"]),
        "imu_200hz": imu["count"] >= 5 and 150.0 <= imu["source_rate_hz"] <= 250.0 and imu["nonadvancing"] == 0,
        "imu_frame_rslidar": imu["frames"].get("frame_id") == "rslidar",
        "odometry_present": odom["count"] >= 2,
        "odometry_lio_frames": odom["frames"].get("frame_id") == "camera_init" and odom["frames"].get("child_frame_id") == "body",
        "odometry_stamp_advances": odom["count"] >= 2 and odom["nonadvancing"] == 0,
        "plugin_loaded": bool(report["loaded_plugins"]),
    }
    failed = [key for key, value in checks.items() if not value]
    return {"verdict": "PASS" if not failed else "FAIL", "checks": checks, "failed": failed}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--seconds", type=float, default=10.0)
    parser.add_argument("--wait-first", type=float, default=20.0)
    parser.add_argument("--out", default="")
    args = parser.parse_args()

    rospy.init_node("a1_sensor_chain_probe", anonymous=True, disable_signals=True)
    samples = {name: [] for name in TOPICS}

    def bind(name):
        def callback(message):
            header = getattr(message, "header", None)
            stamp = header.stamp.to_sec() if header is not None else message.clock.to_sec()
            callerid = ""
            if hasattr(message, "_connection_header"):
                callerid = message._connection_header.get("callerid", "")
            frame = {}
            if header is not None:
                frame["frame_id"] = header.frame_id
                frame["seq"] = getattr(header, "seq", None)
            if name == "raw_lidar":
                frame["point_num"] = int(getattr(message, "point_num", 0))
                points = getattr(message, "points", [])
                if points:
                    frame["nonzero_points"] = sum(
                        1 for point in points if abs(point.x) > 1e-9 or abs(point.y) > 1e-9 or abs(point.z) > 1e-9
                    )
            elif name == "adapter_cloud":
                frame["width"] = int(getattr(message, "width", 0))
                frame["height"] = int(getattr(message, "height", 1))
            elif name == "odometry":
                frame["child_frame_id"] = message.child_frame_id
                pose = message.pose.pose.position
                frame["position"] = [round(pose.x, 6), round(pose.y, 6), round(pose.z, 6)]
            elif name == "imu":
                angular = message.angular_velocity
                frame["angular_velocity"] = [round(angular.x, 6), round(angular.y, 6), round(angular.z, 6)]
            samples[name].append({"stamp": stamp if stamp > 1e-9 else None, "callerid": callerid, **frame})
        return callback

    try:
        from livox_laser_simulation.msg import CustomMsg
        raw_type = CustomMsg
    except Exception as error:
        raw_type = None
        print("[probe] WARN raw lidar type unavailable: %s" % str(error)[:160])

    if raw_type is not None:
        rospy.Subscriber(TOPICS["raw_lidar"], raw_type, bind("raw_lidar"), queue_size=100)
    rospy.Subscriber(TOPICS["adapter_cloud"], PointCloud2, bind("adapter_cloud"), queue_size=100)
    rospy.Subscriber(TOPICS["imu"], Imu, bind("imu"), queue_size=200)
    rospy.Subscriber(TOPICS["odometry"], Odometry, bind("odometry"), queue_size=50)
    rospy.Subscriber(TOPICS["clock"], Clock, bind("clock"), queue_size=200)

    deadline = time.monotonic() + args.wait_first
    while time.monotonic() < deadline and not all(samples[name] for name in ("raw_lidar", "adapter_cloud", "imu", "odometry")):
        rospy.sleep(0.2)

    start = time.monotonic()
    rospy.sleep(max(1.0, args.seconds))
    duration = time.monotonic() - start

    report = {
        "node": rospy.get_name(),
        "wall_duration_s": round(duration, 3),
        "ros_now": rospy.get_rostime().to_sec(),
        "use_sim_time": bool(rospy.get_param("/use_sim_time", False)),
        "loaded_plugins": loaded_gazebo_plugins(),
    }
    for name in TOPICS:
        report[name] = summarize(name, samples[name], duration)
    report.update(verdict_for(report))
    rendered = json.dumps(report, indent=2, sort_keys=True)
    print(rendered)
    if args.out:
        with open(args.out, "w", encoding="utf-8") as stream:
            stream.write(rendered + "\n")
    raise SystemExit(0 if report["verdict"] == "PASS" else 1)


if __name__ == "__main__":
    main()
