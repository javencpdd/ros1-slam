#!/usr/bin/env python3
"""Offline wrapper/Compose controls; fake setup files never run ROS or compilers."""
import ast
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


def invoke(script, *args, env=None):
    return subprocess.run(["/bin/bash", str(script), *args], cwd=ROOT,
                          env={**os.environ, **(env or {})}, capture_output=True, text=True, timeout=30)


class BuildEntrypointTests(unittest.TestCase):
    def test_both_host_plan_entrypoints_select_x86_without_writes(self):
        before = (ROOT / "ws/.profiles").exists()
        for name in ("build.sh", "catkin_build.sh"):
            with self.subTest(name=name):
                result = invoke(ROOT / "docker" / name, "--plan", env={"PROFILE": "sim", "SLIM": "0"})
                self.assertEqual(result.returncode, 0, result.stderr)
                plan = json.loads(result.stdout)
                self.assertEqual(plan["target"], "x86_sim")
                self.assertIn("v8_simulation", [p["name"] for p in plan["packages"]])
        self.assertEqual((ROOT / "ws/.profiles").exists(), before)

    def test_unknown_arguments_and_plan_clean_fail_before_ros(self):
        for arguments in [("--slim",), ("--target", "lite3"), ("--plan", "--clean")]:
            with self.subTest(arguments=arguments):
                result = invoke(ROOT / "docker/build.sh", *arguments)
                self.assertEqual(result.returncode, 2)
                self.assertNotIn("Noetic is required", result.stderr)

    def test_profile_slim_and_jobs_rejected_before_ros(self):
        for env, fragment in [({"PROFILE": "core"}, "core is unsupported"),
                              ({"SLIM": "1"}, "SLIM is retired"),
                              ({"JOBS": "0"}, "positive integer"),
                              ({"JOBS": "1; touch INJECTED"}, "positive integer")]:
            with self.subTest(env=env):
                result = invoke(ROOT / "docker/build.sh", "--plan", env={"PROFILE": "sim", "SLIM": "0", **env})
                self.assertEqual(result.returncode, 2)
                self.assertIn(fragment, result.stderr)

    def test_no_ros_build_is_clear_and_does_not_create_outputs(self):
        if Path("/opt/ros/noetic/setup.bash").exists():
            self.skipTest("No-ROS negative control requires a host without Noetic; never attempt a real build")
        before = (ROOT / "ws/.profiles").exists()
        result = invoke(ROOT / "docker/build.sh", "--clean", env={"PROFILE": "sim", "SLIM": "0"})
        self.assertEqual(result.returncode, 2)
        self.assertIn("Noetic is required", result.stderr)
        self.assertEqual((ROOT / "ws/.profiles").exists(), before)

    def test_owned_fixture_passes_argv_and_discards_all_ambient_build_inputs(self):
        with tempfile.TemporaryDirectory(prefix="docker-wrapper-test-") as temporary:
            fixture = Path(temporary) / "fixture root ; literal"
            for relative in ("docker", "scripts/build", "tooling/build", "ws/src"):
                (fixture / relative).mkdir(parents=True)
            capture = fixture / "capture.json"
            setup = Path(temporary) / "setup.bash"
            setup.write_text('export ROS_DISTRO=noetic ROS_VERSION=1\n'
                             'export CMAKE_PREFIX_PATH=/opt/ros/noetic\n'
                             'export ROS_PACKAGE_PATH=/opt/ros/noetic/share\n')
            wrapper = (ROOT / "docker/build.sh").read_text()
            # Only an owned test copy uses this setup stub; production stays pinned.
            self.assertEqual(wrapper.count("/opt/ros/noetic/setup.bash"), 2)
            (fixture / "docker/build.sh").write_text(wrapper.replace("/opt/ros/noetic/setup.bash", str(setup)))
            shutil.copyfile(ROOT / "docker/catkin_build.sh", fixture / "docker/catkin_build.sh")
            (fixture / "tooling/build/profiles.json").write_text("{}")
            (fixture / "scripts/build/target_workspace.py").write_text(
                "import json, os, sys\nfrom pathlib import Path\n"
                f"Path({str(capture)!r}).write_text(json.dumps({{'argv': sys.argv, 'environment': dict(os.environ)}}))\n")
            workspace = fixture / "ws ; touch INJECTED `printf bad` $(printf bad)"
            contaminated = {"WS_ROOT": str(fixture), "WS_DIR": str(workspace), "JOBS": "3", "PROFILE": "sim", "SLIM": "0",
                **{key: "/old/sim/devel" for key in ("CMAKE_PREFIX_PATH", "ROS_PACKAGE_PATH", "PYTHONPATH", "PYTHONHOME",
                    "LD_LIBRARY_PATH", "PKG_CONFIG_PATH", "CPATH", "CPLUS_INCLUDE_PATH", "C_INCLUDE_PATH", "LIBRARY_PATH",
                    "CC", "CXX", "CFLAGS", "LDFLAGS", "MAKEFLAGS", "AMENT_PREFIX_PATH", "COLCON_PREFIX_PATH")},
                "BASH_ENV": "/nonexistent-shell-hook", "ENV": "/nonexistent-shell-hook"}
            for script in (fixture / "docker/build.sh", fixture / "docker/catkin_build.sh"):
                with self.subTest(script=script.name):
                    result = invoke(script, "--clean", env=contaminated)
                    self.assertEqual(result.returncode, 0, result.stderr)
                    captured = json.loads(capture.read_text())
                    arguments, environment = captured["argv"], captured["environment"]
                    self.assertEqual(arguments[1:], ["build", "--target", "x86_sim", "--workspace", str(workspace),
                        "--profiles", str(fixture / "tooling/build/profiles.json"), "--jobs", "3", "--clean"])
                    self.assertEqual(environment["CMAKE_PREFIX_PATH"], "/opt/ros/noetic")
                    self.assertEqual(environment["ROS_PACKAGE_PATH"], "/opt/ros/noetic/share")
                    self.assertEqual(environment["CPATH"], "/opt/ros/noetic/include")
                    self.assertEqual(environment["PATH"], "/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin")
                    for key in contaminated.keys() - {"CMAKE_PREFIX_PATH", "ROS_PACKAGE_PATH", "CPATH"}:
                        self.assertNotIn(key, environment, key)
                    self.assertFalse((ROOT / "INJECTED").exists())

    def test_dockerfile_guard_is_first_and_rejects_core_slim_unknown(self):
        lines = (ROOT / "docker/Dockerfile").read_text().splitlines()
        instructions, buffer = [], ""
        for line in lines:
            if line.rstrip().endswith("\\"):
                buffer += line.rstrip()[:-1] + " "
                continue
            instructions.append(buffer + line)
            buffer = ""
        guard = next(line[4:] for line in instructions if line.startswith("RUN "))
        self.assertNotIn("apt-get", guard)
        for profile, slim, expected in [("sim", "0", 0), ("dev", "0", 0), ("core", "0", 2), ("sim", "1", 2), ("unknown", "0", 2)]:
            with self.subTest(profile=profile, slim=slim):
                result = subprocess.run(["/bin/bash", "-c", guard], env={**os.environ, "PROFILE": profile, "SLIM": slim}, capture_output=True, text=True)
                self.assertEqual(result.returncode, expected, result.stderr)

    def test_compose_uses_four_fresh_target_volumes_and_retains_no_legacy_mounts(self):
        if shutil.which("docker") is None:
            self.skipTest("Compose CLI unavailable")
        result = subprocess.run(["docker", "compose", "--env-file", str(ROOT / "docker/.env.example"),
            "-f", str(ROOT / "docker/docker-compose.yml"), "--profile", "*", "config", "--format", "json"],
            env={**os.environ, "PROFILE": "sim", "SLIM": "0"}, capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stderr)
        config = json.loads(result.stdout)
        expected = {"ros1_x86_sim_build": "/opt/ros1-slam/ws/build", "ros1_x86_sim_devel": "/opt/ros1-slam/ws/devel",
                    "ros1_x86_sim_install": "/opt/ros1-slam/ws/install", "ros1_x86_sim_profiles": "/opt/ros1-slam/ws/.profiles/x86_sim"}
        for name in ("slam", "slam-host", "sim", "sim-viz"):
            volumes = {item["source"]: item["target"] for item in config["services"][name]["volumes"] if item["type"] == "volume"}
            self.assertTrue(expected.items() <= volumes.items(), name)
            self.assertTrue({"ros1_ws_build", "ros1_ws_devel"}.isdisjoint(volumes), name)

    def test_python_source_has_valid_ast(self):
        for path in [ROOT / "docker/test_build_entrypoints.py", *sorted((ROOT / "scripts/build").glob("*.py"))]:
            ast.parse(path.read_text(), filename=str(path))


if __name__ == "__main__":
    unittest.main(verbosity=2)
