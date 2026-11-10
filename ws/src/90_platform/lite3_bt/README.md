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

The console stack starts both the BT and the ROS-level goal gate:

```bash
roslaunch lite3_bt bt_console.launch \
  tree_file:=/opt/ros1-slam/ws/src/90_platform/lite3_bt/trees/navigate_single_goal.xml
```

Topics:

- input goal: `/v8/bt/goal` (`geometry_msgs/PoseStamped`)
- BT request: `/v8/bt/goal_request` (BT is the owner)
- lease heartbeat: `/v8/bt/lease` (`std_msgs/Bool`, 2 Hz while BT is alive)
- final goal: `/v8/3d/global/goal` (only `goal_gate` writes it; stale/missing lease drops requests)
- arrival feedback: `/v8/local/pose` (distance threshold from the tree)
- stop: zero Twist on `/lite3/teleop/app_cmd_vel` (through the safety mux)
- status: `/v8/bt/status` (JSON string, includes root/node status and last event)

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

Tree:

```
Fallback
  Sequence
    WaitForGoal
    SendGoal
    WaitArrival
    Stop
  Stop
```

## Groot2

The node starts the BehaviorTree.CPP ZMQ publisher on ports **5555/5556**.
Groot2 1.9.0 runs on the host X display and connects to the monitor port; it is
a local-only GUI. The console `本机桌面` card has the launch command and locks
it for remote clients.

Groot2 1.9.0 AppImage is cached at `~/.cache/groot2/` on this host; it is not
committed to the repository.
