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

    def test_compose_rosbridge_passes_master_uri_at_runtime(self):
        """回归：rosbridge 的 master URI 必须由容器运行期展开，不能是 compose 构建期替换。

        缺陷原样（2026-11-16 发现）：`command: bash -lc "... export ROS_MASTER_URI=${ROS1_MASTER_URI:-...}"`
        里的 ${...} 由 compose 在**宿主侧**替换，而宿主的 ROS1_MASTER_URI 通常为空 ⇒
        容器内 ROS_MASTER_URI 空 ⇒ 回落 127.0.0.1:11311 ⇒ 桥进程照起、日志照印
        "server started"，实际连不上 master。判据只能落在展开后的 argv 与环境上。
        """
        if shutil.which("docker") is None:
            self.skipTest("Compose CLI unavailable")
        result = subprocess.run(["docker", "compose", "--env-file", str(ROOT / "docker/.env.example"),
            "-f", str(ROOT / "docker/docker-compose.yml"), "--profile", "extras", "config", "--format", "json"],
            env={**os.environ, "PROFILE": "sim", "SLIM": "0", "ROS1_MASTER_URI": ""},
            capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stderr)
        bridge = json.loads(result.stdout)["services"]["rosbridge"]
        command = " ".join(bridge["command"])
        # 展开后必须还留着 bash 变量引用（$$ 逃逸成 $），而不是被替换成空串。
        self.assertIn("$${ROS1_MASTER_URI}", command.replace("$", "$$").replace("$$$$", "$$"), command)
        self.assertIn("ROS_MASTER_URI=", command)
        # 两个名字都要落进容器 env，因为 setup.bash 只认 ROS_MASTER_URI。
        self.assertEqual(bridge["environment"]["ROS_MASTER_URI"], "http://roscore:11312")
        self.assertEqual(bridge["environment"]["ROS1_MASTER_URI"], "http://roscore:11312")
        # 只绑宿主环回：桥是可下发的控制面，不该对整个局域网敞开。
        ports = bridge["ports"]
        self.assertEqual(len(ports), 1)
        self.assertEqual(ports[0]["host_ip"], "127.0.0.1")
        self.assertEqual(str(ports[0]["published"]), "9090")
        # 健康检查必须能证明「连上了 master」，不能只看进程在。
        self.assertIn("rosnode list", " ".join(bridge["healthcheck"]["test"]))

    def test_compose_hardening_applies_to_every_service(self):
        """回归：日志轮转 / no-new-privileges / core=0 必须落到**每一个**服务上。

        缺陷原样（2026-11-16）：这几项原先一个都没有。加防护时最典型的翻车方式是
        「只加在 anchor 上」—— sim/sim-viz/rosbridge/web 都没写 <<: *ros-base
        （它们自写 environment，YAML 合并是整体覆盖不是深合并），anchor 根本传不进去。
        实测第一次加完只有 roscore/slam 吃到，其余五个全空。判据必须逐服务查。
        （redis 服务于 2026-11-18 删除：外部件模板，本体无消费者且从未起过容器。）
        """
        if shutil.which("docker") is None:
            self.skipTest("Compose CLI unavailable")
        result = subprocess.run(["docker", "compose", "--env-file", str(ROOT / "docker/.env.example"),
            "-f", str(ROOT / "docker/docker-compose.yml"), "--profile", "*", "config", "--format", "json"],
            env={**os.environ, "PROFILE": "sim", "SLIM": "0"}, capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stderr)
        services = json.loads(result.stdout)["services"]
        self.assertGreaterEqual(len(services), 7, sorted(services))
        for name, service in sorted(services.items()):
            with self.subTest(service=name):
                # ① 日志轮转
                options = service.get("logging", {}).get("options", {})
                self.assertEqual(options.get("max-size"), "10m", name)
                self.assertEqual(str(options.get("max-file")), "3", name)
                # ② 禁提权
                self.assertIn("no-new-privileges:true", service.get("security_opt", []), name)
                # ③ core 不落盘（历史上 154 枚 core 堆了 104.3GB）
                self.assertIn("core", service.get("ulimits", {}), name)
                # ④ 内存上限（值可不同，但不能缺席）
                self.assertTrue(service.get("mem_limit"), name)

    def test_gazebo_plugin_path_prefers_split_sim_workspace(self):
        """回归：GAZEBO_PLUGIN_PATH 必须让 sim_ws 排在 ws 前面。

        缺陷原样（2026-11-16）：11-14 物理分仓后，ws/devel/lib 里留着 11-02 编的
        liblivox_laser_simulation.so。两份 .so 符号集完全相同、源码同一份，**唯一差别是
        编译期烧进 .so 的 __FILE__**：旧件指向 ws/src/95_simulation（分仓后已不存在）
        ⇒ 插件 Load() 读 CSV 失败就 return，/livox/lidar 照 advertise 但零数据。
        实测＝旧件优先 no new messages，sim_ws 优先 10.05Hz。
        三处必须一致：Dockerfile ENV、sim.sh、viz.sh。
        """
        delimiter = ":" if os.name != "nt" else ";"
        for path in (ROOT / "docker/Dockerfile", ROOT / "docker/sim.sh", ROOT / "docker/viz.sh"):
            with self.subTest(file=path.name):
                text = path.read_text()
                self.assertIn("GAZEBO_PLUGIN_PATH", text, path.name)
                # 取出 GAZEBO_PLUGIN_PATH 所在行，抽出其中的目录序列
                line = next(l for l in text.splitlines() if "GAZEBO_PLUGIN_PATH" in l and "=" in l
                            and not l.lstrip().startswith("#"))
                entries = [e.strip() for e in line.split("=", 1)[1].split(delimiter)]
                # ★ 注意：不能用 endswith("ws/devel/lib") 选 ws —— 'sim_ws/devel/lib' 也满足，
                #   两者会命中同一个下标（本用例第一版就栽在这里，报 0 not less than 0）。
                sim_idx = next((i for i, e in enumerate(entries) if "sim_ws/devel/lib" in e), None)
                ws_idx = next((i for i, e in enumerate(entries)
                               if "sim_ws/devel/lib" not in e and "ws/devel/lib" in e), None)
                self.assertIsNotNone(sim_idx, f"{path.name}: 缺 sim_ws/devel/lib ⇒ {entries}")
                self.assertIsNotNone(ws_idx, f"{path.name}: 缺 ws/devel/lib ⇒ {entries}")
                self.assertLess(sim_idx, ws_idx, f"{path.name}: ws 排在了 sim_ws 前面 ⇒ {entries}")

    def test_python_source_has_valid_ast(self):
        for path in [ROOT / "docker/test_build_entrypoints.py", *sorted((ROOT / "scripts/build").glob("*.py"))]:
            ast.parse(path.read_text(), filename=str(path))


if __name__ == "__main__":
    unittest.main(verbosity=2)
