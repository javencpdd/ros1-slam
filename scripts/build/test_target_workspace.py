#!/usr/bin/env python3
"""Offline positive/negative controls; never invoke ROS, Docker or a compiler."""
import contextlib
import importlib.util
import io
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest import mock

SPEC = importlib.util.spec_from_file_location("target_workspace", Path(__file__).with_name("target_workspace.py"))
tw = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(tw)


class TargetWorkspaceTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="target-workspace-test-")
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.ws = self.root / "ws"
        (self.ws / "src").mkdir(parents=True)
        self.config = {"schema_version": 1, "local_packages": ["app", "core"],
                       "targets": {name: {"supported": name != "m20", "roots": ["app"]}
                                   for name in tw.TARGETS}}
        self.profiles = self.root / "profiles.json"
        self.pkg("core")
        self.pkg("app", "<depend>core</depend><exec_depend>rospy</exec_depend>")

    def pkg(self, name, dependencies="", location=None, filename="package.xml"):
        directory = self.ws / "src" / (location or name)
        directory.mkdir(parents=True, exist_ok=True)
        (directory / filename).write_text(f'<package format="3"><name>{name}</name><version>0.1.0</version>{dependencies}</package>')
        return directory

    def plan(self, target="lite3"):
        self.profiles.write_text(json.dumps(self.config))
        return tw.plan(self.ws, target, self.profiles)

    def build(self, result, runner=None):
        runner = runner or mock.Mock(return_value=subprocess.CompletedProcess([], 0, stdout=""))
        with mock.patch.object(tw, "require_noetic"):
            tw.build(result, 2, runner=runner)
        return runner

    def test_plan_exact_transitive_closure_is_read_only_and_external_unchecked(self):
        self.pkg("unused")
        result = self.plan()
        self.assertEqual([p["name"] for p in result["packages"]], ["app", "core"])
        self.assertEqual(result["external_dependencies"], ["rospy"])
        self.assertEqual(result["external_dependency_status"], "NOT_CHECKED")
        self.assertFalse((self.ws / ".profiles").exists())

    def test_transitive_local_simulation_forbidden_on_robot_allowed_on_x86(self):
        self.pkg("core", "<depend>simulator</depend>")
        self.pkg("simulator", location="95_simulation/simulator")
        with self.assertRaisesRegex(tw.PlanError, "app -> core -> simulator"):
            self.plan()
        self.assertIn("simulator", [p["name"] for p in self.plan("x86_sim")["packages"]])

    def test_transitive_external_gazebo_training_forbidden(self):
        for dependency in ["gazebo_ros", "gazebo11", "libgazebo11-dev", "ros-noetic-gazebo-ros",
                           "mujoco", "mujoco-py", "libmujoco-dev", "python3-mujoco", "torch",
                           "python3-torch", "torchvision", "gymnasium", "stable_baselines3", "python3-stable-baselines3"]:
            with self.subTest(dependency=dependency):
                self.pkg("core", f"<exec_depend>{dependency}</exec_depend>")
                with self.assertRaisesRegex(tw.PlanError, "robot target forbids dependency"):
                    self.plan()

    def test_test_and_documentation_dependencies_excluded(self):
        self.pkg("core", '<test_depend condition="unsupported()">gazebo_ros</test_depend><doc_depend>mujoco</doc_depend>')
        self.assertEqual(self.plan()["external_dependencies"], ["rospy"])

    def test_inference_runtime_is_not_misclassified_as_training(self):
        self.pkg("core", "<exec_depend>onnxruntime</exec_depend><build_depend>libogre-1.9-dev</build_depend>")
        self.assertEqual(self.plan()["external_dependencies"], ["libogre-1.9-dev", "onnxruntime", "rospy"])

    def test_catkin_ignore_and_nested_package_boundaries(self):
        self.pkg("core", location="core/nested")  # identical nested name is not discovered
        ignored = self.pkg("hiddenlib")
        (ignored / "CATKIN_IGNORE").touch()
        self.assertEqual(self.plan()["ignored_packages"], ["hiddenlib"])
        self.pkg("app", "<depend>hiddenlib</depend>")
        with self.assertRaisesRegex(tw.PlanError, "missing/ignored local package"):
            self.plan()

    def test_duplicate_packages_fail_instead_of_picking_one(self):
        self.pkg("core", location="other/core")
        with self.assertRaisesRegex(tw.PlanError, "duplicate package core"):
            self.plan()

    def test_missing_root_and_missing_local_dependency_fail(self):
        self.config["targets"]["lite3"]["roots"] = ["absent"]
        with self.assertRaisesRegex(tw.PlanError, "target root missing"):
            self.plan()
        self.config["targets"]["lite3"]["roots"] = ["app"]
        (self.ws / "src/core/package.xml").unlink()
        with self.assertRaisesRegex(tw.PlanError, "missing/ignored local package"):
            self.plan()

    def test_m20_cannot_borrow_lite3_and_target_mandatory(self):
        with self.assertRaisesRegex(tw.PlanError, "target m20 unsupported"):
            self.plan("m20")
        self.config["targets"]["m20"]["supported"] = True
        with self.assertRaisesRegex(tw.PlanError, "target m20 unsupported"):
            self.plan("m20")
        with self.assertRaisesRegex(tw.PlanError, "unknown target"):
            self.plan("arm64")
        for arguments in [["plan"], ["plan", "--target", "arm64"]]:
            with self.subTest(arguments=arguments), contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as exc:
                tw.main(arguments)
            self.assertEqual(exc.exception.code, 2)

    def test_host_only_image_context_is_rejected(self):
        context = self.root / "robot-image-context"
        context.mkdir()
        for relative in ("web", "subprojects/console", "data/console"):
            target = context / relative
            target.mkdir(parents=True, exist_ok=True)
            with self.subTest(relative=relative), self.assertRaisesRegex(tw.PlanError, "host-only"):
                tw.check_host_only_context(context)
            if target.is_dir() and not any(target.iterdir()):
                target.rmdir()
                parent = target.parent
                while parent != context and not any(parent.iterdir()):
                    parent.rmdir()
                    parent = parent.parent
        self.assertEqual(tw.check_host_only_context(context),
                         ["web", "subprojects/console", "data/console"])

    def test_robot_build_rejects_host_only_source_view(self):
        self.config["targets"]["lite3"]["roots"] = ["web"]
        self.pkg("web")
        result = self.plan()
        with self.assertRaisesRegex(tw.PlanError, "host-only"):
            self.build(result)

    def test_robot_plan_declares_host_only_policy(self):
        result = self.plan()
        self.assertTrue(result["host_only"]["robot_forbidden"])
        self.assertEqual(result["host_only"]["paths"],
                         ["web", "subprojects/console", "data/console"])
        self.assertFalse(self.plan("x86_sim")["host_only"]["robot_forbidden"])

    def test_known_ros_version_conditions_only(self):
        self.pkg("core", '<buildtool_depend condition="$ROS_VERSION == 1">catkin</buildtool_depend>'
                 '<buildtool_depend condition="$ROS_VERSION == 2">ament_cmake_auto</buildtool_depend>')
        self.assertEqual(self.plan()["external_dependencies"], ["catkin", "rospy"])
        self.pkg("core", '<depend condition="$ROS_DISTRO == noetic">rospy</depend>')
        with self.assertRaisesRegex(tw.PlanError, "unsupported dependency condition"):
            self.plan()

    def test_source_and_output_symlinks_cannot_escape(self):
        outside = self.root / "outside"
        outside.mkdir()
        (self.ws / "src/escape").symlink_to(outside, target_is_directory=True)
        with self.assertRaisesRegex(tw.PlanError, "source symlink escapes"):
            self.plan()
        (self.ws / "src/escape").unlink()
        (self.ws / ".profiles").symlink_to(outside, target_is_directory=True)
        with self.assertRaisesRegex(tw.PlanError, "symlink output"):
            self.plan()
        self.assertEqual(list(outside.iterdir()), [])

    def test_build_captures_only_selected_view_and_exact_nonempty_whitelist(self):
        self.pkg("unrelated")
        result = self.plan()
        runner = self.build(result)
        command = runner.call_args.args[0]
        source_view = Path(result["paths"]["source_view"])
        self.assertEqual(command[:2], ["catkin_make", "-C"])
        self.assertEqual(command[command.index("--source") + 1], str(source_view))
        self.assertIn("-DCATKIN_WHITELIST_PACKAGES=app;core", command)
        self.assertIn("-DCATKIN_ENABLE_TESTING=OFF", command)
        self.assertEqual({p.name for p in source_view.iterdir()}, {"app", "core"})
        self.assertFalse(os.readlink(source_view / "core").startswith("/"))
        self.assertFalse((self.ws / "src/core/CATKIN_IGNORE").exists())
        self.assertFalse((self.ws / "build").exists())

    def test_cross_target_marker_and_manifest_change_reject_cache(self):
        result = self.plan()
        self.build(result)
        marker = Path(result["paths"]["profile"]) / "identity.json"
        changed = json.loads(marker.read_text())
        changed["target"] = "x86_sim"
        marker.write_text(json.dumps(changed))
        with self.assertRaisesRegex(tw.PlanError, "identity mismatch"):
            tw.check_cache(result)
        marker.write_text(json.dumps(tw.identity(result)))
        self.pkg("core", "<depend>geometry_msgs</depend>")
        with self.assertRaisesRegex(tw.PlanError, "identity mismatch"):
            tw.check_cache(self.plan())

    def test_foreign_cmake_cache_and_unowned_output_rejected(self):
        result = self.plan()
        self.build(result)
        cache = Path(result["paths"]["build"]) / "CMakeCache.txt"
        cache.write_text("CMAKE_HOME_DIRECTORY:INTERNAL=/some/old/simulation/src\n")
        with self.assertRaisesRegex(tw.PlanError, "CMake cache mismatch"):
            tw.check_cache(result)
        x86 = self.plan("x86_sim")
        (self.ws / "devel").mkdir()
        (self.ws / "devel/old").touch()
        with self.assertRaisesRegex(tw.PlanError, "unowned existing target output"):
            tw.check_cache(x86)
        self.assertNotEqual(result["paths"]["devel"], x86["paths"]["devel"])

    def test_source_view_tamper_rejected_before_commands(self):
        result = self.plan()
        self.build(result)
        source_view = Path(result["paths"]["source_view"])
        (source_view / "extra").mkdir()
        runner = mock.Mock()
        with self.assertRaisesRegex(tw.PlanError, "unexpected source-view entries"):
            self.build(result, runner)
        runner.assert_not_called()

    def test_livox_ros1_manifest_in_generated_wrapper_without_source_mutation(self):
        driver = self.pkg("livox_ros_driver2", "<depend>roscpp</depend>", "drivers/livox", "package_ROS1.xml")
        self.pkg("ros2_driver", location="drivers/livox")
        self.config["manifest_overrides"] = {"drivers/livox": "package_ROS1.xml"}
        self.config["targets"]["lite3"]["roots"] = ["livox_ros_driver2"]
        with self.assertRaisesRegex(tw.PlanError, "differs from explicit ROS1 manifest"):
            self.plan()
        (driver / "package.xml").unlink()
        result = self.plan()
        self.build(result)
        wrapper = Path(result["paths"]["source_view"]) / "livox_ros_driver2"
        self.assertFalse(wrapper.is_symlink())
        self.assertEqual(tw.manifest(wrapper / "package.xml")[0], "livox_ros_driver2")
        self.assertFalse((driver / "package.xml").exists())
        self.build(result)  # valid existing wrapper can be reused

    def test_plan_reports_cache_conflicts_without_mutation(self):
        self.plan()
        (self.ws / "devel").mkdir()
        legacy = self.ws / "devel/old"
        legacy.write_text("old")
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            status = tw.main(["plan", "--target", "x86_sim", "--workspace", str(self.ws), "--profiles", str(self.profiles)])
        self.assertEqual(status, 0)
        self.assertIn("unowned existing", json.loads(output.getvalue())["cache_conflicts"][0])
        self.assertEqual(legacy.read_text(), "old")
        self.assertFalse((self.ws / ".profiles").exists())

    def test_dangling_livox_wrapper_symlink_rejected_before_commands(self):
        self.pkg("livox_ros_driver2", location="drivers/livox", filename="package_ROS1.xml")
        self.config["manifest_overrides"] = {"drivers/livox": "package_ROS1.xml"}
        self.config["targets"]["lite3"]["roots"] = ["livox_ros_driver2"]
        result = self.plan()
        self.build(result)
        wrapper = Path(result["paths"]["source_view"]) / "livox_ros_driver2"
        tw.shutil.rmtree(wrapper)
        wrapper.symlink_to("missing", target_is_directory=True)
        self.assertFalse(wrapper.exists())
        runner = mock.Mock()
        with self.assertRaisesRegex(tw.PlanError, "manifest wrapper mismatch"):
            self.build(result, runner)
        runner.assert_not_called()
        self.assertTrue(wrapper.is_symlink())

    def test_clean_only_owned_outputs_allows_metadata_update(self):
        result = self.plan()
        self.build(result)
        self.pkg("core", "<depend>geometry_msgs</depend>")
        updated = self.plan()
        with mock.patch.object(tw, "require_noetic"):
            tw.build(updated, 1, runner=mock.Mock(), clean=True)
        tw.check_cache(updated)
        self.assertTrue((self.ws / "src/core/package.xml").exists())
        marker = Path(updated["paths"]["profile"]) / "identity.json"
        wrong = json.loads(marker.read_text())
        wrong["target"] = "x86_sim"
        marker.write_text(json.dumps(wrong))
        with self.assertRaisesRegex(tw.PlanError, "--clean ownership mismatch"):
            tw.clean_owned(updated)
        self.assertTrue(marker.exists())

    def test_clean_refuses_legacy_and_symlink_outputs(self):
        result = self.plan("x86_sim")
        (self.ws / "devel").mkdir()
        (self.ws / "devel/old").touch()
        with self.assertRaisesRegex(tw.PlanError, "--clean requires an owned"):
            tw.clean_owned(result)
        self.assertTrue((self.ws / "devel/old").exists())
        result = self.plan()
        self.build(result)
        build_dir = Path(result["paths"]["build"])
        build_dir.rmdir()
        build_dir.symlink_to(self.ws / "src", target_is_directory=True)
        with self.assertRaisesRegex(tw.PlanError, "symlink output"):
            tw.clean_owned(result)
        self.assertTrue((self.ws / "src/core/package.xml").exists())

    def test_foreign_top_level_cmake_is_rejected_before_commands(self):
        result = self.plan()
        self.build(result)
        (Path(result["paths"]["source_view"]) / "CMakeLists.txt").write_text("arbitrary build code")
        runner = mock.Mock()
        with self.assertRaisesRegex(tw.PlanError, "foreign source-view CMakeLists"):
            self.build(result, runner)
        runner.assert_not_called()

    def test_no_ros_build_refuses_before_creating_output(self):
        result = self.plan()
        with mock.patch.object(tw.shutil, "which", return_value=None), self.assertRaisesRegex(tw.PlanError, "Noetic/catkin_make unavailable"):
            tw.build(result, 1)
        self.assertFalse((self.ws / ".profiles").exists())

    def test_foreign_overlay_and_wrong_ros_environment_rejected(self):
        good = {"ROS_DISTRO": "noetic", "ROS_VERSION": "1", "CMAKE_PREFIX_PATH": "/opt/ros/noetic",
                "ROS_PACKAGE_PATH": "/opt/ros/noetic/share"}
        with mock.patch.object(Path, "is_file", return_value=True), mock.patch.object(tw.shutil, "which", return_value="/opt/ros/noetic/bin/catkin_make"):
            tw.require_noetic(good)
            for key, value in [("ROS_DISTRO", "humble"), ("CMAKE_PREFIX_PATH", "/old/sim/devel:/opt/ros/noetic"),
                               ("ROS_PACKAGE_PATH", "/old/sim/src:/opt/ros/noetic/share")]:
                with self.subTest(key=key), self.assertRaises(tw.PlanError):
                    tw.require_noetic({**good, key: value})

    def test_simulator_message_prebuild_uses_same_selected_source_and_fails_closed(self):
        self.pkg("livox_laser_simulation", location="95_simulation/livox_laser_simulation")
        self.config["targets"]["x86_sim"]["roots"].append("livox_laser_simulation")
        result = self.plan("x86_sim")
        runner = mock.Mock(return_value=subprocess.CompletedProcess([], 0, stdout="... selected_generate_messages_cpp\n"))
        self.build(result, runner)
        commands = [call.args[0] for call in runner.call_args_list]
        self.assertEqual(commands[1][0:2], ["cmake", result["paths"]["source_view"]])
        self.assertEqual(commands[3], ["make", "-j1", "selected_generate_messages_cpp"])
        self.assertEqual(commands[-1][0], "catkin_make")
        runner.side_effect = subprocess.CalledProcessError(1, ["cmake"])
        with self.assertRaises(subprocess.CalledProcessError):
            self.build(result, runner)


if __name__ == "__main__":
    unittest.main(verbosity=2)
