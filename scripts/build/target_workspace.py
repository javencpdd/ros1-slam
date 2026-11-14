#!/usr/bin/env python3
"""Plan a target's ROS1 dependency closure; build only its isolated source view."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET

TARGETS = ("lite3", "m20", "x86_sim")
DEPEND_TAGS = {"depend", "build_depend", "build_export_depend", "buildtool_depend",
               "buildtool_export_depend", "exec_depend", "run_depend"}
NOETIC = Path("/opt/ros/noetic")
HOST_ONLY_PATHS = ('web', 'scripts/console', 'data/console')
ROBOT_FORBIDDEN = re.compile(
    r"(?:^|[-_])(?:lib)?(?:gazebo|mujoco)\d*(?:[-_]|$)"
    r"|(?:^|[-_])(?:torch|torchvision|torchaudio|pytorch|tensorflow|tensorboard|gym|gymnasium|rllib|ray|training)(?:[-_]|$)"
    r"|(?:^|[-_])stable[-_]baselines\d*(?:[-_]|$)", re.IGNORECASE)


class PlanError(ValueError):
    pass


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def active(element):
    """Deliberately narrow REP-149 evaluator: no eval or ambient substitutions."""
    condition = element.get("condition")
    if condition is None:
        return True
    match = re.fullmatch(r"\s*\$ROS_VERSION\s*(==|!=)\s*(?:([12])|'([12])'|\"([12])\")\s*", condition)
    if not match:
        raise PlanError(f"unsupported dependency condition: {condition!r}")
    return (next(value for value in match.groups()[1:] if value is not None) == "1") == (match[1] == "==")


def manifest(path):
    try:
        root = ET.parse(path).getroot()
    except (ET.ParseError, OSError) as exc:
        raise PlanError(f"invalid manifest {path}: {exc}") from exc
    name = (root.findtext("name") or "").strip()
    if root.tag != "package" or not re.fullmatch(r"[A-Za-z][A-Za-z0-9_-]*", name):
        raise PlanError(f"invalid package name/root in {path}")
    if root.get("format", "1") not in {"1", "2", "3"}:
        raise PlanError(f"unsupported package format in {path}")
    return name, root


def within(path, boundary):
    try:
        path.relative_to(boundary)
        return True
    except ValueError:
        return False


def check_host_only_context(context):
    """Reject a robot image/source context containing host-only demo trees."""
    context = Path(context)
    found = [rel for rel in HOST_ONLY_PATHS
             if (context / rel).exists() or (context / rel).is_symlink()]
    if found:
        raise PlanError("robot image context contains host-only paths: %s" % ", ".join(found))
    return list(HOST_ONLY_PATHS)


def discover(source, overrides):
    """CATKIN_IGNORE excludes a subtree; every manifest is a package boundary."""
    packages, ignored, visited = {}, set(), set()

    def walk(directory, excluded=False):
        resolved = directory.resolve()
        if not within(resolved, source):
            raise PlanError(f"source symlink escapes workspace: {directory}")
        if resolved in visited:
            raise PlanError(f"source directory alias/cycle: {directory}")
        visited.add(resolved)
        excluded |= (directory / "CATKIN_IGNORE").exists() or (directory / ".catkin").exists()
        relative = directory.relative_to(source).as_posix()
        xml_path = directory / overrides.get(relative, "package.xml")
        if relative in overrides and not excluded:
            if not xml_path.is_file():
                raise PlanError(f"required ROS1 override manifest missing: {xml_path}")
            default_xml = directory / "package.xml"
            if default_xml.is_symlink() and not default_xml.is_file():
                raise PlanError(f"dangling manifest symlink: {default_xml}")
            if default_xml.exists() and default_xml.read_bytes() != xml_path.read_bytes():
                raise PlanError(f"package.xml differs from explicit ROS1 manifest: {directory}; reconcile the source selection before building")
        if xml_path.is_file():
            if not within(xml_path.resolve(), source):
                raise PlanError(f"manifest escapes workspace: {xml_path}")
            try:
                name, root = manifest(xml_path)
            except PlanError:
                if excluded:
                    return
                raise
            if excluded:
                ignored.add(name)
            else:
                if name in packages:
                    raise PlanError(f"duplicate package {name}: {packages[name]['path']}, {directory}")
                packages[name] = {"path": directory, "manifest": xml_path, "xml": root}
            return
        for child in sorted(directory.iterdir()):
            if child.name.startswith(".") or not child.is_dir():
                continue
            walk(child, excluded)

    walk(source)
    return packages, ignored


def output_paths(workspace, target):
    profile = workspace / ".profiles" / target
    base = workspace if target == "x86_sim" else profile
    return {"profile": profile, "source_view": profile / "src",
            **{name: base / name for name in ("build", "devel", "install")}}


def check_outputs(paths, workspace):
    source = workspace / "src"
    for label, path in paths.items():
        if not within(path, workspace) or within(path, source) or path == workspace:
            raise PlanError(f"unsafe {label} path: {path}")
        for part in [path, *path.parents]:
            if part == workspace:
                break
            if part.is_symlink():
                raise PlanError(f"symlink output/ancestor forbidden: {part}")
        if path.exists() and not path.is_dir():
            raise PlanError(f"output is not a directory: {path}")


def plan(workspace, target, profiles_path, extra_sources=()):
    if target not in TARGETS:
        raise PlanError(f"unknown target: {target}")
    workspace = Path(os.path.abspath(workspace))
    if workspace.resolve() != workspace or not (workspace / "src").is_dir():
        raise PlanError(f"workspace must be a real directory with src/: {workspace}")
    source = workspace / "src"
    if source.is_symlink():
        raise PlanError(f"source root must not be a symlink: {source}")
    try:
        config = json.loads(Path(profiles_path).read_text())
        profile = config["targets"][target]
    except (OSError, ValueError, KeyError) as exc:
        raise PlanError(f"invalid profiles file: {exc}") from exc
    if config.get("schema_version") != 1:
        raise PlanError("unsupported profile schema_version")
    if target == "m20" or not profile.get("supported", False):
        raise PlanError(f"target {target} unsupported: {profile.get('reason', 'no adapter')}")
    roots = profile.get("roots")
    if not isinstance(roots, list) or not roots or not all(isinstance(n, str) for n in roots):
        raise PlanError(f"target {target} requires explicit nonempty package roots")
    packages, ignored = discover(source, config.get("manifest_overrides", {}))
    for extra in extra_sources:
        extra = Path(os.path.abspath(extra))
        if not extra.is_dir():
            raise PlanError(f"extra source is not a directory: {extra}")
        extra_packages, extra_ignored = discover(extra, {})
        for name, package in extra_packages.items():
            if name in packages:
                raise PlanError(f"duplicate package {name}: {packages[name]['path']}, {package['path']}")
            packages[name] = package
        ignored |= extra_ignored
    known_local = set(config.get("local_packages", [])) | set(roots) | ignored
    selected, external = {}, set()

    def select(name, chain):
        if target != "x86_sim" and ROBOT_FORBIDDEN.search(name):
            raise PlanError(f"robot target forbids dependency: {' -> '.join(chain + [name])}")
        if name not in packages:
            if name in known_local or name.startswith(("lite3_", "v8_")):
                raise PlanError(f"missing/ignored local package: {' -> '.join(chain + [name])}")
            external.add(name)
            return
        package = packages[name]
        try:
            relative = package["path"].relative_to(source).as_posix()
        except ValueError:
            relative = package["path"].as_posix()
        if target != "x86_sim" and any(part in {"95_simulation", "training"} for part in relative.split("/")):
            raise PlanError(f"robot target forbids simulation/training package: {' -> '.join(chain + [name])} ({relative})")
        if name in selected:
            return
        selected[name] = {"name": name, "path": str(package["path"]),
                          "manifest": str(package["manifest"]), "manifest_sha256": digest(package["manifest"])}
        root = package["xml"]
        for build_type in root.findall("export/build_type"):
            if active(build_type) and (build_type.text or "").strip() != "catkin":
                raise PlanError(f"non-catkin package unsupported by catkin_make: {name}")
        for element in root:
            if element.tag in {"group_depend", "member_of_group"}:
                raise PlanError(f"group dependencies unsupported: {name}")
            if element.tag in DEPEND_TAGS and active(element):
                dependency = (element.text or "").strip()
                if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.+-]*", dependency):
                    raise PlanError(f"invalid dependency name in {name}: {dependency!r}")
                select(dependency, chain + [name])

    for name in roots:
        if name not in packages:
            raise PlanError(f"target root missing/ignored: {name}")
        select(name, [])
    paths = output_paths(workspace, target)
    check_outputs(paths, workspace)
    result = {"schema_version": 1, "target": target, "ros_distro": "noetic", "ros_version": 1,
              "extra_sources": [str(Path(item).resolve()) for item in extra_sources],
              "workspace": str(workspace), "roots": roots, "packages": sorted(selected.values(), key=lambda p: p["name"]),
              "external_dependencies": sorted(external), "external_dependency_status": "NOT_CHECKED",
              "ignored_packages": sorted(ignored), "paths": {k: str(v) for k, v in paths.items()},
              "profiles_sha256": digest(Path(profiles_path)), "tests_enabled": False}
    result["rosdep_check_command"] = ["rosdep", "check", "--from-paths",
        *[str(paths["source_view"] / p["name"]) for p in result["packages"]],
        "--ignore-src", "--rosdistro", "noetic",
        *[arg for kind in ("build", "buildtool", "build_export", "buildtool_export", "exec")
          for arg in ("--dependency-types", kind)]]
    result["rosdep_check_note"] = "Advisory only; requires the generated source view. No check or installation was performed."
    host_only_status = "not-applicable"
    if target != "x86_sim":
        root = workspace.parent
        for package in result["packages"]:
            package_path = Path(package["path"]).resolve()
            for relative in HOST_ONLY_PATHS:
                boundary = (root / relative).resolve()
                if within(package_path, boundary):
                    raise PlanError("robot target selected host-only package: %s" % package_path)
        if paths["source_view"].exists():
            check_host_only_context(paths["source_view"])
            host_only_status = "checked"
        else:
            host_only_status = "pending-source-view"
    result["host_only"] = {
        "paths": list(HOST_ONLY_PATHS),
        "robot_forbidden": target != "x86_sim",
        "status": host_only_status,
    }
    return result


def identity(result):
    return {key: result[key] for key in ("schema_version", "target", "ros_distro", "ros_version",
            "workspace", "extra_sources", "roots", "packages", "paths", "profiles_sha256", "tests_enabled")}


def check_cache(result):
    paths = {key: Path(value) for key, value in result["paths"].items()}
    check_outputs(paths, Path(result["workspace"]))
    marker = paths["profile"] / "identity.json"
    if marker.is_symlink():
        raise PlanError(f"symlink profile identity forbidden: {marker}")
    if marker.exists():
        try:
            previous = json.loads(marker.read_text())
        except (OSError, ValueError) as exc:
            raise PlanError(f"invalid profile identity: {marker}") from exc
        if previous != identity(result):
            raise PlanError("profile/cache identity mismatch; use a separately reviewed clean target space")
    elif any(p.exists() and any(p.iterdir()) for p in paths.values()):
        raise PlanError("unowned existing target output; refusing legacy/full-workspace cache reuse")
    cache = paths["build"] / "CMakeCache.txt"
    if cache.is_symlink():
        raise PlanError("symlink CMakeCache.txt forbidden")
    if cache.exists():
        values = dict(re.findall(r"^([^#/:][^:=]*):[^=]*=(.*)$", cache.read_text(), re.M))
        expected = {"CMAKE_HOME_DIRECTORY": str(paths["source_view"]),
                    "CATKIN_DEVEL_PREFIX": str(paths["devel"]), "CMAKE_INSTALL_PREFIX": str(paths["install"]),
                    "CATKIN_WHITELIST_PACKAGES": ";".join(p["name"] for p in result["packages"])}
        for key, value in expected.items():
            if values.get(key) != value:
                raise PlanError(f"CMake cache mismatch: {key}")
    return paths


def require_noetic(env=None):
    env = os.environ if env is None else env
    if not (NOETIC / "setup.bash").is_file() or not shutil.which("catkin_make"):
        raise PlanError("ROS1 Noetic/catkin_make unavailable; plan works without ROS, build requires native Noetic")
    if env.get("ROS_DISTRO") != "noetic" or env.get("ROS_VERSION") != "1":
        raise PlanError("build requires a sourced ROS1 Noetic underlay (ROS_DISTRO=noetic, ROS_VERSION=1)")
    prefixes = [Path(p).resolve() for p in env.get("CMAKE_PREFIX_PATH", "").split(os.pathsep) if p]
    if prefixes != [NOETIC]:
        raise PlanError("foreign/missing CMAKE_PREFIX_PATH overlay; source only /opt/ros/noetic/setup.bash in a clean shell")
    for value in env.get("ROS_PACKAGE_PATH", "").split(os.pathsep):
        if value and Path(value).resolve() not in {NOETIC / "share", NOETIC / "stacks"}:
            raise PlanError("foreign ROS_PACKAGE_PATH overlay; use a clean Noetic shell")


def clean_owned(result):
    """Delete only fixed outputs whose target/workspace ownership is established."""
    paths = {key: Path(value) for key, value in result["paths"].items()}
    check_outputs(paths, Path(result["workspace"]))
    marker = paths["profile"] / "identity.json"
    if marker.is_symlink() or not marker.is_file():
        raise PlanError("--clean requires an owned profile identity; migrate legacy outputs explicitly")
    try:
        previous = json.loads(marker.read_text())
    except (OSError, ValueError) as exc:
        raise PlanError("--clean refuses invalid profile identity") from exc
    for key in ("schema_version", "target", "workspace", "paths", "ros_distro", "ros_version"):
        if previous.get(key) != identity(result)[key]:
            raise PlanError(f"--clean ownership mismatch: {key}")
    allowed = {"src", "identity.json"}
    if result["target"] != "x86_sim":
        allowed |= {"build", "devel", "install"}
    if {p.name for p in paths["profile"].iterdir()} - allowed:
        raise PlanError("--clean refuses unexpected files in the profile directory")
    for key in ("source_view", "build", "devel", "install"):
        if paths[key].exists():
            shutil.rmtree(paths[key])
    marker.unlink()


def prepare_view(result, paths):
    source_view = paths["source_view"]
    expected = {p["name"] for p in result["packages"]}
    if source_view.exists():
        unexpected = {p.name for p in source_view.iterdir()} - expected - {"CMakeLists.txt"}
        if unexpected:
            raise PlanError(f"unexpected source-view entries: {sorted(unexpected)}")
    top = source_view / "CMakeLists.txt"
    noetic_top = NOETIC / "share/catkin/cmake/toplevel.cmake"
    if top.exists() or top.is_symlink():
        if top.is_symlink():
            valid_top = top.resolve() == noetic_top
        else:
            valid_top = top.is_file() and noetic_top.is_file() and top.read_bytes() == noetic_top.read_bytes()
        if not valid_top:
            raise PlanError(f"foreign source-view CMakeLists.txt: {top}")
    for package in result["packages"]:
        original, xml_path = Path(package["path"]), Path(package["manifest"])
        destination = source_view / package["name"]
        if xml_path.name == "package.xml":
            if destination.is_symlink():
                if os.path.isabs(os.readlink(destination)) or destination.resolve() != original.resolve():
                    raise PlanError(f"source-view link mismatch: {destination}")
            elif destination.exists():
                raise PlanError(f"source-view package must be a managed symlink: {destination}")
        elif destination.exists() or destination.is_symlink():
            if destination.is_symlink() or not destination.is_dir():
                raise PlanError(f"manifest wrapper mismatch: {destination}")
            expected_children = {child.name for child in original.iterdir()} | {"package.xml"}
            if {p.name for p in destination.iterdir()} != expected_children:
                raise PlanError(f"manifest wrapper entries changed: {destination}")
            for child in destination.iterdir():
                if child.name == "package.xml":
                    if child.is_symlink() or child.read_bytes() != xml_path.read_bytes():
                        raise PlanError(f"manifest wrapper XML mismatch: {child}")
                elif not child.is_symlink() or child.resolve() != (original / child.name).resolve():
                    raise PlanError(f"manifest wrapper link mismatch: {child}")
    # Validate everything above before the first mutation.
    for path in paths.values():
        path.mkdir(parents=True, exist_ok=True)
    for package in result["packages"]:
        original, xml_path = Path(package["path"]), Path(package["manifest"])
        destination = source_view / package["name"]
        if destination.exists() or destination.is_symlink():
            continue
        if xml_path.name == "package.xml":
            destination.symlink_to(os.path.relpath(original, source_view), target_is_directory=True)
        else:
            destination.mkdir()
            for child in original.iterdir():
                if child.name != "package.xml":
                    (destination / child.name).symlink_to(os.path.relpath(child, destination), target_is_directory=child.is_dir())
            (destination / "package.xml").write_bytes(xml_path.read_bytes())
    if result["target"] != "x86_sim":
        check_host_only_context(paths["source_view"])
    (paths["profile"] / "identity.json").write_text(json.dumps(identity(result), indent=2) + "\n")


def build(result, jobs, runner=subprocess.run, clean=False):
    require_noetic()
    if clean:
        clean_owned(result)
    paths = check_cache(result)
    prepare_view(result, paths)
    names = ";".join(p["name"] for p in result["packages"])
    cmake_args = ["-DCMAKE_BUILD_TYPE=Release", "-DROS_EDITION=ROS1", "-DPYTHON_EXECUTABLE=/usr/bin/python3",
                  "-DCATKIN_ENABLE_TESTING=OFF", f"-DCATKIN_WHITELIST_PACKAGES={names}",
                  f"-DCATKIN_DEVEL_PREFIX={paths['devel']}", f"-DCMAKE_INSTALL_PREFIX={paths['install']}"]
    # The upstream simulator lacks a message-header target dependency. Configure
    # and generate selected-view messages before compilation; never edit upstream.
    if any(p["name"] == "livox_laser_simulation" for p in result["packages"]):
        top = paths["source_view"] / "CMakeLists.txt"
        if not top.exists():
            runner(["catkin_init_workspace", str(paths["source_view"])], check=True)
        runner(["cmake", str(paths["source_view"]), *cmake_args, "-G", "Unix Makefiles"], cwd=paths["build"], check=True)
        help_result = runner(["make", "help"], cwd=paths["build"], check=True, capture_output=True, text=True)
        message_targets = sorted(set(re.findall(r"^\.\.\. ([A-Za-z0-9_]+_generate_messages_cpp)$", help_result.stdout, re.M)))
        if message_targets:
            runner(["make", "-j1", *message_targets], cwd=paths["build"], check=True)
        else:
            jobs = 1
    base = Path(result["workspace"]) if result["target"] == "x86_sim" else paths["profile"]
    command = ["catkin_make", "-C", str(base), "--source", str(paths["source_view"]),
               "--build", str(paths["build"]), *cmake_args, f"-j{jobs}"]
    runner(command, check=True)
    return command


def main(argv=None):
    repo = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("plan", "build"))
    parser.add_argument("--target", required=True, choices=TARGETS)
    parser.add_argument("--workspace", type=Path, default=repo / "ws")
    parser.add_argument("--profiles", type=Path, default=repo / "tooling/build/profiles.json")
    parser.add_argument("--jobs", type=int, default=1)
    parser.add_argument("--clean", action="store_true", help="build only: clean fixed outputs owned by this target")
    parser.add_argument("--extra-source", action="append", default=[],
                        help="additional package roots merged into the target view (repeatable)")
    args = parser.parse_args(argv)
    try:
        if args.jobs < 1:
            raise PlanError("jobs must be positive")
        if args.clean and args.action != "build":
            raise PlanError("--clean is only valid with the explicit build action")
        extra = list(args.extra_source)
        if args.target == "x86_sim" and not extra:
            default_extra = repo.parent / "ros1-slam-simulation" / "ws" / "src"
            if default_extra.is_dir():
                extra.append(str(default_extra))
        result = plan(args.workspace, args.target, args.profiles, extra)
        if args.action == "build":
            build(result, args.jobs, clean=args.clean)
        else:
            try:
                check_cache(result)
                result["cache_conflicts"] = []
            except PlanError as exc:
                result["cache_conflicts"] = [str(exc)]
            print(json.dumps(result, indent=2))
        return 0
    except (PlanError, OSError, subprocess.CalledProcessError) as exc:
        print(f"[target-build] {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
