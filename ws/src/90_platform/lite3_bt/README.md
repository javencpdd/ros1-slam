# lite3_bt · BehaviorTree.CPP console demo

ROS1 Noetic C++ behavior-tree layer for the console demo. It is an orchestration
layer; it does not replace the MBF or V8 3D planner, and it is not a formal or
real-robot path.

## Dependency

BehaviorTree.CPP 3.8.8 is built for GCC 9 (BT.CPP 4.x needs a newer libstdc++
for floating-point `from_chars`). The library is installed outside the system
prefix, in the mounted path:

```
/opt/ros1-slam/opt/btcpp_v3
```

Build (inside `ros1-slam-sim-viz`):

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

```bash
rosrun lite3_bt bt_console \
  _tree_file:=/opt/ros1-slam/ws/src/90_platform/lite3_bt/trees/navigate_single_goal.xml
```

Topics:

- input goal: `/v8/bt/goal` (`geometry_msgs/PoseStamped`)
- output goal: `/v8/3d/global/goal`
- arrival feedback: `/v8/local/pose` (distance threshold from the tree)
- stop: zero Twist on `/lite3/teleop/app_cmd_vel` (through the safety mux)
- status: `/v8/bt/status` (JSON string, includes root/node status and last event)

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

The node starts the BehaviorTree.CPP ZMQ publisher on ports **1666/1667**.
Groot2 1.9.0 runs on the host X display and connects to the monitor port; it is
a local-only GUI. The console `本机桌面` card has the launch command and locks
it for remote clients.

Groot2 1.9.0 AppImage is cached at `~/.cache/groot2/` on this host; it is not
committed to the repository.
