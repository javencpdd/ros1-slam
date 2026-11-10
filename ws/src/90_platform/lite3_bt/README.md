# lite3_bt · BehaviorTree.CPP console demo

ROS1 Noetic C++ behavior-tree layer for the console demo. It is an orchestration
layer; it does not replace the MBF or V8 3D planner, and it is not a formal or
real-robot path.

## Dependency

BehaviorTree.CPP 3.8.8 is built for GCC 9 (BT.CPP 4.x needs a newer libstdc++
for floating-point `from_chars`). The library is installed outside the system
prefix, in the mounted path `/opt/ros1-slam/opt/btcpp_v3`.

Use the reproducible bootstrap (host, needs the sim-viz container running):

```bash
bash scripts/ops/bootstrap_bt_demo.sh --check
bash scripts/ops/bootstrap_bt_demo.sh --build   # BT.CPP + lite3_bt
bash scripts/ops/bootstrap_bt_demo.sh --groot2  # optional host Groot2 AppImage
```

The script pins the BT.CPP source SHA256 and Groot2 1.9.0 AppImage SHA256 and
fails closed on a mismatch. Manual build (inside `ros1-slam-sim-viz`):

```bash
cd /tmp
curl -L --fail -o btcpp_v3.tar.gz \
  https://gh-proxy.com/https://github.com/BehaviorTree/BehaviorTree.CPP/archive/refs/tags/3.8.8.tar.gz
tar xzf btcpp_v3.tar.gz
cd BehaviorTree.CPP-3.8.8
cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=/opt/ros1-slam/opt/btcpp_v3 \
  -DBUILD_EXAMPLES=OFF -DBUILD_UNIT_TESTS=OFF -DBUILD_TOOLS=OFF \
  -DBUILD_MANUAL_SELECTOR=OFF
cmake --build build -j4
cmake --install build
```

Build this package:

```bash
source /opt/ros/noetic/setup.bash
cd /opt/ros1-slam/ws
catkin_make --pkg lite3_bt -DCMAKE_BUILD_TYPE=Release \
  -Dbehaviortree_cpp_v3_DIR=/opt/ros1-slam/opt/btcpp_v3/lib/cmake/behaviortree_cpp_v3
```

## Run

The default tree is `trees/navigate_with_planner.xml` (Nav2-style orchestration);
the legacy single-goal tree can be selected with `tree_file:=...`:

```bash
roslaunch lite3_bt bt_console.launch
# legacy: roslaunch lite3_bt bt_console.launch \
#   tree_file:=/opt/ros1-slam/ws/src/90_platform/lite3_bt/trees/navigate_single_goal.xml
```

Topics:

- input goal: `/v8/bt/goal` (`geometry_msgs/PoseStamped`, FIFO queue)
- planner input: `/v8/global/goal` (BT `RequestPath` publishes; `planner/topic_node.py` subscribes)
- planner status: `/global_path/status` (`std_msgs/String`, JSON `ok`/`points`/`reason`)
- BT request: `/v8/bt/goal_request` (BT `FollowPath` publishes, via the goal gate)
- final goal: `/v8/3d/global/goal` (only `goal_gate` writes it; stale/missing lease drops requests)
- lease heartbeat: `/v8/bt/lease` (`std_msgs/Bool`, 2 Hz while BT is alive)
- arrival feedback: `/v8/local/pose` (pose mode) or an optional executor ack
- stop: zero Twist on `/lite3/teleop/app_cmd_vel` (through the safety mux)
- status: `/v8/bt/status` (JSON: nodes, event, queue_depth, current_seq, path_state)

## Goal publisher allow-list (demo only)

`goal_gate.py` forwards a `/v8/bt/goal_request` only when BOTH hold:

1. the publisher's ROS1 `callerid` is in `~allowed_callers` (default
   `/bt_console`, declared in `bt_console.launch`), and
2. the BT lease is fresh (`~require_lease`, 1 s timeout).

`~require_acl` defaults to true.  A missing/empty callerid is refused, and an
explicitly empty `~allowed_callers` refuses everything: it is fail closed,
not a wildcard.  The decision lives in `goal_gate_policy.py` (pure stdlib, no
ROS import) so it can be tested on a host without a sourced environment;
`test_goal_gate_policy.py` also asserts the wiring, i.e. that `on_request`
calls the policy before publishing.  `scripts/console/check_console.sh` runs
those tests as a standing check.

BOUNDARY — read before treating this as security:

- ROS1 `callerid` comes from the connection header and is trivially spoofable.
  This is demo-plane writer ownership (which node may ask for a goal), NOT
  authentication.  It does not contradict the decision that the console agent
  keeps trusting the LAN: that is the network side, this is the ROS topic side.
- The lease topic `/v8/bt/lease` is NOT publisher-checked in this slice.
- Formal / EVT / real-robot paths are unchanged; they stay on the work order.

## Nav2-style orchestration

The tree decides WHEN to request a global path, WHEN to dispatch a goal to the
downstream executor, WHEN to dwell and WHEN to recover.  It never computes a
path and never writes a command velocity other than the zero stop.  This is the
BT's orchestration role, not a replacement for MBF / the V8 3D planner.

| tree node | Nav2 analogue | what it actually does here |
|---|---|---|
| `WaitForGoal` | goal intake | dequeues the next FIFO goal |
| `RequestPath` | `ComputePathToPose` | publishes the goal on `/v8/global/goal`, invalidates the previous path status |
| `WaitForPath` | path result | waits on `/global_path/status`; transient reasons are not failures, terminal refusal / timeout fails the goal |
| `FollowPath` | `FollowPath` | dispatches the goal through the goal gate; local trajectory and velocity stay downstream |
| `WaitArrival` | goal feedback | pose mode, or an optional executor ack |
| `Dwell` | `Wait` | deliberate pause between goals (default 1 s) |
| `RecoveryStop` | recovery branch | zero command, invalidate the stale path, advance to the next goal |

Goals arriving on `/v8/bt/goal` are queued FIFO instead of overwriting a single
slot: a fast multi-goal submission used to collapse into the last goal only.
`~queue_max` defaults to 16; a full queue **refuses** the new goal and drops
nothing.  After a Sequence completes, the next tick returns to `WaitForGoal`.
The status JSON reports `queue_depth`, `goal_seq`, `current_seq`, `path_state`,
`path_points` and `planner_mode`.

Path status protocol: `planner/topic_node.py` publishes JSON built with
`json.dumps`, e.g. `{"ok": true, "points": 12}` or
`{"ok": false, "reason": "E_PLAN_RUNNING"}`.  Only `ok=true` with `points>0`
releases `FollowPath`.  `E_PLAN_RUNNING` / `E_PLAN_NOT_READY` /
`E_PLAN_CANCELLED_REQUESTED` are transient; other failures end the goal.
Malformed status is ignored rather than guessed.

There are two arrival modes:

| mode | when | who decides the goal is done |
|---|---|---|
| `pose` (default) | `~planner_result_topic` empty | BT compares `/v8/local/pose` with the goal (0.25 m, 60 s timeout) |
| `ack` | `~planner_result_topic` set | downstream executor publishes the result |

Ack wire format is `std_msgs/String` with `<goal_seq>:ok` or `<goal_seq>:fail`.
Only an `ok` ack matching `current_seq` advances the queue; a `fail` ack ends
that goal immediately, and stale / future / malformed acks are ignored.  The
timeout still applies so a silent executor cannot hang the tree forever.

To hand arrival ownership to an executor:

```bash
roslaunch lite3_bt bt_console.launch \
  planner_result_topic:=/v8/planner/goal_result
```

BOUNDARY — integration of the **hand-off contract**, not of the planner:

- Global path computation stays in `planner/topic_node.py`; local trajectory,
  velocity and replanning cadence stay downstream.  The BT only sequences the
  calls.  Formal / EVT / real-robot paths are unchanged and stay on the work order.
- No downstream executor in this repo publishes the ack topic yet.  With the
  parameter empty (default) pose-based arrival keeps the demo from deadlocking.
- The pure queue / ack / path-status helpers live in
  `include/lite3_bt/goal_queue.hpp` and are compiled and exercised by
  `test/test_goal_queue.cpp` on a plain host.
- `navigate_single_goal.xml` still loads: `SendGoal` is registered as an alias
  of `FollowPath`.  It skips the path-request / dwell / recovery stages.

Tree:

```
Fallback mission_or_recovery
  Sequence mission
    WaitForGoal
    RequestPath
    WaitForPath       timeout=30
    FollowPath
    WaitArrival       distance=0.25 timeout=60
    Dwell             seconds=1.0
    Stop
  RecoveryStop
```

## Groot2

The node starts the BehaviorTree.CPP ZMQ publisher on ports **5555/5556**.
Groot2 1.9.0 runs on the host X display and connects to the monitor port; it is
a local-only GUI. The console `本机桌面` card has the launch command and locks
it for remote clients.

Groot2 1.9.0 AppImage is cached at `~/.cache/groot2/` on this host; it is not
committed to the repository.
