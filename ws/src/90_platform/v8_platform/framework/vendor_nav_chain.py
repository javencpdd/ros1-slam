"""Official Lite3 navigation startup chain -- read-only interface contract.

This module adapts the vendor's 8.2.1 single-goal localization/navigation
startup chain (observed read-only on 192.168.1.103, 2026-11-09) into an
inspection layer for the ROS1 side of this repository.

It is deliberately NOT a runtime driver:

* Pure stdlib, no ROS import, no topic writes, no process starts.
* Default mode is ``off``. ``shadow`` only reports; ``assist`` is the only
  mode that may veto, and it is never the default.
* Vendor numeric values are recorded as READ-ONLY reference facts.  They are
  never injected into project configs: the real-robot Runbook
  (note/10_真机上机Runbook.md:4, :230, :236) forbids changing extrinsics/TF,
  speed limits, judgement thresholds and map versions on site, and
  note/09_跨工具协作契约.md:213 lists real-robot parameters as an L2 stop.
* The vendor chain is ROS2 Foxy/Nav2; this repository is ROS1 Noetic with
  mbf_costmap_nav + safe_cmd_vel_mux.  Interface names are compared, never
  copied.

Use ``python3 vendor_nav_chain.py --report`` for the human-readable comparison,
or ``python3 vendor_nav_chain.py --selftest`` for the three-way self-test.
"""

from __future__ import annotations

import argparse
import copy
import json

OBSERVED_AT = "2026-11-09"
OBSERVED_HOST = "192.168.1.103"
READ_ONLY = True

# ---------------------------------------------------------------------------
# Vendor chain: three steps of the official 8.2.1 procedure.
# All paths/values below are verbatim read-only observations, not proposals.
# ---------------------------------------------------------------------------

VENDOR_CHAIN = (
    {
        "step": 1,
        "name": "lidar_driver",
        "manual": "cd ~/lite_cog_ros2/system/scripts/lidar; ./start_<vendor>.sh",
        "scripts": {
            "lsLidar": "start_lslidar.sh",
            "livox": "start_livox.sh",
            "rslidar": "start_rslidar.sh",
        },
        "scripts_present_on_host": ("start_lslidar.sh", "start_livox.sh"),
        "note": (
            "The manual lists three driver scripts but only two exist on the "
            "observed host; start_rslidar.sh is absent.  start_livox.sh sources "
            "driver/mid360_ws/install/setup.bash and launches "
            "livox_ros_driver2 msg_MID360s_launch.py; start_lslidar.sh sources "
            "driver/leishen_ws/install/setup.bash and launches "
            "lslidar_driver lslidar_c16_launch.py."
        ),
    },
    {
        "step": 2,
        "name": "localization_and_nav2",
        "manual": "cd /home/ysc/lite_cog_ros2/system/scripts/nav; ./start_nav.sh",
        "script": "start_nav.sh",
        "script_lines": (
            "source /home/ysc/lite_cog_ros2/navigation2-foxy/install/setup.bash",
            "source /home/ysc/lite_cog_ros2/nav/install/setup.bash",
            "ros2 launch hdl_localization lite_localization.launch.py",
        ),
        "launch": "nav/src/hdl_localization/launch/lite_localization.launch.py",
        "included_launch": "nav/src/dr_nav2/launch/dr_nav2.launch.py",
        "nav2_params": "nav/src/dr_nav2/config/lite_nav2.yaml",
        "note": (
            "debug=False, so map_server/rviz in this launch are NOT started; "
            "dr_nav2.launch.py includes nav2_bringup bringup_launch.py with "
            "autostart=True and starts rviz2.  The localization node is "
            "hdl_localization_composition with NDT_OMP."
        ),
    },
    {
        "step": 3,
        "name": "localization_initialization",
        "manual": "After navigation starts, initialize the robot localization.",
        "mechanisms": (
            "hdl_localization launch: specify_init_pose=True with a hard-coded origin pose",
            "nav2 BT plugin: nav2_initial_pose_received_condition_bt_node",
            "rviz2 tool: 2D Pose Estimate publishes /initialpose",
            "hdl_localization README: estimated pose can be reset with 2D Pose Estimate",
        ),
        "note": (
            "This is the step the ROS1 chain has no equivalent gate for.  It is "
            "the only functional gap addressed by the shadow readiness gate "
            "in this module."
        ),
    },
)

# Verbatim vendor interface facts.  Marked do_not_copy for every value whose
# counterpart in this repository is a real-robot parameter or a frozen gate.
VENDOR_INTERFACES = {
    "points_topic": {
        "value": "/rslidar_points",
        "source": "lite_localization.launch.py remapping /velodyne_points",
        "do_not_copy": False,
        "project": "/rslidar_points",
    },
    "imu_topic": {
        "value": "/imu/data",
        "source": "lite_localization.launch.py remapping /gpsimu_driver/imu_data",
        "do_not_copy": True,
        "project": "/imu/data (qnx2ros, UDP 43897); faster_lio uses /livox/imu",
        "reason": "Q12 port conflict; IMU source is a real-robot wiring decision",
    },
    "global_frame": {
        "value": "map",
        "source": "lite_nav2.yaml:4, :37, :149",
        "do_not_copy": False,
        "project": "map",
    },
    "robot_base_frame": {
        "value": "base_link",
        "source": "lite_nav2.yaml:5, :46, :158",
        "do_not_copy": False,
        "project": "base_link",
    },
    "odom_topic": {
        "value": "/leg_odom2",
        "source": "lite_nav2.yaml:6 bt_navigator; :427 controller_server",
        "do_not_copy": True,
        "project": "hdl remaps /odom to /hdl_odom; no leg odometry prediction",
        "reason": "enable_robot_odometry_prediction=False is a frozen project setting",
    },
    "cmd_vel_topic": {
        "value": "/cmd_vel_raw",
        "source": "lite_nav2.yaml:277, :428 controller_server",
        "do_not_copy": True,
        "project": "mbf -> /cmd_vel_raw -> safe_cmd_vel_mux -> /cmd_vel -> ros2qnx",
        "reason": (
            "Project has an extra safety arbitration layer; "
            "Runbook:236 forbids writing /cmd_vel directly and bypassing the mux"
        ),
    },
    "specify_init_pose": {
        "value": True,
        "source": "lite_localization.launch.py hdl_localization_composition params",
        "do_not_copy": False,
        "project": "hdl_localization.launch specify_init_pose=true (same semantics)",
    },
    "autostart": {
        "value": True,
        "source": "dr_nav2.launch.py bringup_launch.py launch argument",
        "do_not_copy": True,
        "project": "enable_motion defaults to false in mode_navigation.launch",
        "reason": "Turning motion on is L2; the project default is deliberately safer",
    },
    "use_sim_time": {
        "value": False,
        "source": "dr_nav2.launch.py argument; lite_nav2.yaml:3, :33, :145, :257",
        "do_not_copy": True,
        "project": "project simulation uses /clock gating; no sim-time flag here",
        "reason": "Vendor file internally disagrees: lite_nav2.yaml:419 says True",
    },
    "costmap_resolution": {
        "value": 0.10,
        "source": "lite_nav2.yaml:45, :157",
        "do_not_copy": False,
        "project": "costmap_common.yaml resolution 0.1 (numerically equal)",
    },
    "inflation_radius": {
        "value": 0.40,
        "source": "lite_nav2.yaml:137, :250",
        "do_not_copy": True,
        "project": "costmap_common.yaml inflation_radius 0.2",
        "reason": "Costmap gate value; changing it needs a signed judgement",
    },
    "ndt_resolution": {
        "value": 1.5,
        "source": "lite_localization.launch.py hdl_localization_composition params",
        "do_not_copy": True,
        "project": "hdl_localization.launch ndt_resolution 2.0",
        "reason": "Localization judgement threshold; Runbook:4 forbids on-site changes",
    },
    "downsample_resolution": {
        "value": 0.5,
        "source": "lite_localization.launch.py hdl_localization_composition params",
        "do_not_copy": True,
        "project": "hdl_localization.launch 1.0; globalmap_server_nodelet 0.1",
        "reason": "Localization judgement threshold; Runbook:4 forbids on-site changes",
    },
    "max_vel_x": {
        "value": 1.0,
        "source": "lite_nav2.yaml:471 DWB local planner",
        "do_not_copy": True,
        "project": "cmd_vel_mux.yaml max_linear_mps 0.5",
        "reason": "Speed limit; Runbook:4/:129 forbids on-site changes",
    },
    "base_link_to_rslidar_tf": {
        "value": (0.16, 0.0, 0.47, 1.57, 0.0, 0.0),
        "source": (
            "lite_localization.launch.py launch_base2lidar_tf_broadcaster() and "
            "dr_nav2.launch.py launch_static_tf_broadcaster()"
        ),
        "do_not_copy": True,
        "project": "hdl_localization.launch tf_x/tf_y/tf_z/tf_pitch are 0 placeholders",
        "reason": (
            "Both vendor broadcasters are dead code: the first is never added to "
            "the LaunchDescription, the second is commented out.  Project G8-2 "
            "explicitly fails an identity extrinsics matrix, and the vendor "
            "values are for a rslidar frame on a differently configured robot."
        ),
    },
    "globalmap_pcd": {
        "value": "/home/ysc/lite_cog_ros2/system/map/lite3.pcd",
        "source": "lite_localization.launch.py globalmap_pcd",
        "do_not_copy": True,
        "project": "/home/ros1-slam/data/maps/$(arg map_name).pcd",
        "reason": "Map version has no simulation-to-real comparison sheet (Runbook:230)",
    },
    "map_yaml": {
        "value": (
            "lite3.pgm, resolution 0.05, origin [-3.55, -7.05, 0], "
            "occupied_thresh 0.65, free_thresh 0.196, mode trinary"
        ),
        "source": "system/map/lite3.yaml",
        "do_not_copy": True,
        "project": "2D /map is supplied by M4 costmap; sample yaml origin z=nan was rejected",
        "reason": "Map version/origin is an L2 real-robot parameter",
    },
}

# Values that must NEVER appear as active settings in project runtime configs.
# Used by the external assertion script as a no-backfill guard.
NO_BACKFILL_KEYS = tuple(sorted(
    k for k, v in VENDOR_INTERFACES.items() if v["do_not_copy"]
))

# ---------------------------------------------------------------------------
# Project side: three-step counterpart chain, read from repository sources.
# ---------------------------------------------------------------------------

PROJECT_CHAIN = (
    {
        "step": 1,
        "name": "M0_drivers",
        "entry": "ws/src/00_drivers/lite3_drivers/launch/01_sensors.launch",
        "content": (
            "livox_ros_driver2 msg_MID360s.launch (remap /livox/lidar to "
            "/rslidar_points, frame_id rslidar) + qnx2ros (UDP 43897)",
        ),
        "lidar_selection": "fixed to Livox Mid-360; lslidar_c16.launch kept out of chain",
    },
    {
        "step": 2,
        "name": "M2_localization_M4_planner_M6_safety",
        "entry": "ws/src/70_bringup/lite3_bringup/launch/mode_navigation.launch",
        "content": (
            "SUP + M0 + M2 hdl_localization + M6 safe_cmd_vel_mux + "
            "M4 mbf_costmap_nav + ultrasonic guard + ros2qnx (enable_motion only)",
        ),
        "safety": "safe_cmd_vel_mux is the sole /cmd_vel writer; hold forces zero speed",
    },
    {
        "step": 3,
        "name": "localization_initialization",
        "entry": "none",
        "content": (
            "hdl_localization.launch sets specify_init_pose=true; /initialpose is "
            "only injected by historical m2_smoke.sh verification scripts",
        ),
        "safety": (
            "No startup gate requires an explicit initial pose or localization "
            "convergence before a goal/velocity can be accepted"
        ),
    },
)

# ---------------------------------------------------------------------------
# Fail-closed localization initialization readiness gate (shadow).
# ---------------------------------------------------------------------------

REQUIRED_SIGNALS = (
    "initial_pose_received",
    "localization_converged",
    "map_ready",
    "odom_chain_ready",
)

MODES = ("off", "shadow", "assist")


def resolve_mode(mode):
    """Return a validated mode. Unknown/empty values are refused, not defaulted."""
    if mode is None or mode == "":
        raise ValueError("mode must be one of off/shadow/assist; empty is refused")
    if mode not in MODES:
        raise ValueError("mode must be one of off/shadow/assist")
    return mode


def initial_pose_gate(signals, mode="off"):
    """Fail-closed readiness decision for the vendor step 3 gap.

    ``signals`` maps the four REQUIRED_SIGNALS to bool or None.  A missing key
    or a None value means "unknown", which is never treated as ready.

    Returns a dict with state/decision/missing/reasons.  In ``off`` and
    ``shadow`` the decision never blocks anything; only ``assist`` may veto,
    and ``assist`` is never the default.  An entirely empty or unknown signal
    set is refused as insufficient_data rather than printed green.
    """
    mode = resolve_mode(mode)
    if signals is None:
        signals = {}
    unknown = [k for k in REQUIRED_SIGNALS
               if k not in signals or signals.get(k) is None]
    known = [k for k in REQUIRED_SIGNALS if k in signals and signals[k] is not None]

    if not known or len(unknown) == len(REQUIRED_SIGNALS):
        return {
            "state": "insufficient_data",
            "decision": "refuse",
            "ready": False,
            "mode": mode,
            "missing": list(unknown),
            "reasons": ["no usable signal; refusing to judge"],
        }

    false_signals = [k for k in known if not bool(signals[k])]
    ready = not false_signals and not unknown
    state = "ready" if ready else "not_ready"
    reasons = []
    if false_signals:
        reasons.append("false signals: " + ",".join(false_signals))
    if unknown:
        reasons.append("unknown signals: " + ",".join(unknown))

    if mode == "off":
        decision = "allow"
    elif mode == "shadow":
        decision = "observe_only"
    else:
        decision = "allow" if ready else "veto"

    return {
        "state": state,
        "decision": decision,
        "ready": ready,
        "mode": mode,
        "missing": list(unknown) + list(false_signals),
        "reasons": reasons,
    }


def chain_diff():
    """Read-only side-by-side of the three steps. Never modifies anything."""
    rows = []
    for vendor, project in zip(VENDOR_CHAIN, PROJECT_CHAIN):
        rows.append({
            "step": vendor["step"],
            "vendor": vendor["name"],
            "project": project["name"],
            "vendor_entry": vendor.get("manual", vendor.get("script", "")),
            "project_entry": project["entry"],
            "gap": (
                "no project-side startup gate for localization initialization"
                if vendor["step"] == 3 else
                "vendor offers 3 lidar scripts, project fixes one driver"
                if vendor["step"] == 1 else
                "project adds SUP/M6 safety arbitration absent in vendor chain"
            ),
            "vendor_only_facts": (
                "start_rslidar.sh documented but absent on host"
                if vendor["step"] == 1 else
                "autostart=True; no safety mux; no estop guard"
                if vendor["step"] == 2 else
                "specify_init_pose hard-codes origin; BT has initial-pose condition"
            ),
        })
    return rows


def no_backfill_report(project_settings):
    """Check that no vendor value marked do_not_copy leaked into project settings.

    ``project_settings`` is a dict of {key: value} taken from project runtime
    configs.  Returns report with a list of leaked keys.  Empty input is
    refused (rc-style) instead of being reported as clean.
    """
    if not project_settings:
        return {"state": "insufficient_data", "leaks": [],
                "reasons": ["empty project settings; refusing to judge"]}
    leaks = [k for k in NO_BACKFILL_KEYS
             if k in project_settings
             and project_settings[k] == VENDOR_INTERFACES[k]["value"]]
    return {
        "state": "clean" if not leaks else "backfill_detected",
        "leaks": leaks,
        "checked_keys": len(NO_BACKFILL_KEYS),
        "reasons": (["vendor read-only value present as active project setting: "
                     + ",".join(leaks)] if leaks else []),
    }


def report_text():
    """Human-readable, read-only comparison for preflight --report."""
    out = [
        "== Vendor 8.2.1 chain vs project chain (READ-ONLY, no writes) ==",
        "observed: %s @ %s" % (OBSERVED_AT, OBSERVED_HOST),
        "vendor = ROS2 Foxy + Nav2 + hdl_localization; project = ROS1 Noetic + mbf + safe mux",
        "",
        "-- three steps --",
    ]
    for row in chain_diff():
        out.append("  step %d  %-32s | %s" % (row["step"], row["vendor"], row["project"]))
        out.append("          gap: %s" % row["gap"])
    out += ["", "-- vendor values marked DO NOT COPY (L2 / Runbook) --"]
    for key in NO_BACKFILL_KEYS:
        item = VENDOR_INTERFACES[key]
        out.append("  %-28s %s" % (key, item.get("reason", "read-only")))
    out += [
        "",
        "-- gate --",
        "  default mode = off; shadow = observe_only; assist = may veto (never default)",
        "  required signals = %s" % ", ".join(REQUIRED_SIGNALS),
        "  This layer starts no node, writes no topic, changes no parameter.",
    ]
    return "\n".join(out)


def _selftest():
    """Three-way self-test: main / injection effectiveness / reverse control."""
    ok = True

    # 1. main: all signals true -> ready; shadow does not block.
    full = {k: True for k in REQUIRED_SIGNALS}
    r = initial_pose_gate(full, "shadow")
    ok &= (r["state"] == "ready" and r["decision"] == "observe_only")

    # 1b. default mode is off and never blocks.
    ok &= initial_pose_gate(full)["decision"] == "allow"

    # 1c. empty set is refused, not green.
    empty = initial_pose_gate({}, "shadow")
    ok &= (empty["state"] == "insufficient_data" and empty["decision"] == "refuse")

    # 2. injection effectiveness: one false signal must flip ready->not_ready,
    #    and in assist mode must flip allow->veto.
    injected = copy.deepcopy(full)
    injected["localization_converged"] = False
    shadow_r = initial_pose_gate(injected, "shadow")
    assist_r = initial_pose_gate(injected, "assist")
    ok &= (shadow_r["state"] == "not_ready" and assist_r["decision"] == "veto")

    # 2b. no-backfill check must actually catch a leaked vendor value.
    clean_settings = {k: "project-own-value" for k in REQUIRED_SIGNALS}
    clean_settings["inflation_radius"] = 0.2
    ok &= no_backfill_report(clean_settings)["state"] == "clean"
    leaked = dict(clean_settings)
    leaked["inflation_radius"] = VENDOR_INTERFACES["inflation_radius"]["value"]
    ok &= no_backfill_report(leaked)["state"] == "backfill_detected"
    ok &= no_backfill_report({})["state"] == "insufficient_data"

    # 3. reverse control: restore the injected signal -> verdict flips back.
    restored = copy.deepcopy(injected)
    restored["localization_converged"] = True
    ok &= (initial_pose_gate(restored, "assist")["decision"] == "allow")

    # 3b. mode validation refuses unknown values rather than silently defaulting.
    try:
        resolve_mode("bogus")
        ok = False
    except ValueError:
        pass
    try:
        resolve_mode("")
        ok = False
    except ValueError:
        pass

    # 3c. structural: vendor facts are read-only and the chain has 3 steps.
    ok &= READ_ONLY is True and len(VENDOR_CHAIN) == 3 and len(PROJECT_CHAIN) == 3
    ok &= len(NO_BACKFILL_KEYS) >= 10

    return ok


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--report", action="store_true", help="print read-only comparison")
    ap.add_argument("--selftest", action="store_true", help="run three-way self-test")
    ap.add_argument("--json", action="store_true", help="machine-readable chain diff")
    args = ap.parse_args()
    if args.selftest:
        print("SELFTEST %s" % ("OK" if _selftest() else "FAIL"))
        return 0 if _selftest() else 1
    if args.json:
        print(json.dumps({
            "observed_at": OBSERVED_AT,
            "observed_host": OBSERVED_HOST,
            "read_only": READ_ONLY,
            "vendor_chain": list(VENDOR_CHAIN),
            "project_chain": list(PROJECT_CHAIN),
            "no_backfill_keys": list(NO_BACKFILL_KEYS),
            "chain_diff": chain_diff(),
        }, ensure_ascii=False, indent=2, default=list))
        return 0
    print(report_text())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
