cmake_minimum_required(VERSION 3.0.2)
project(lite3_bt)

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs nav_msgs std_msgs)
# Prefer the in-image prefix (docker/Dockerfile "btcpp" stage).  The mounted
# /opt/ros1-slam/opt/btcpp_v3 path stays as the host/dev fallback, so the same
# source builds both on a laptop with the sim-viz mount and on the robot image.
find_package(behaviortree_cpp_v3 REQUIRED
  PATHS "/opt/btcpp_v3/lib/cmake/behaviortree_cpp_v3"
        "/opt/ros1-slam/opt/btcpp_v3/lib/cmake/behaviortree_cpp_v3"
  NO_DEFAULT_PATH)
find_package(Threads REQUIRED)

catkin_package(CATKIN_DEPENDS roscpp geometry_msgs nav_msgs std_msgs)

include_directories(${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
  /opt/ros/noetic/include
  /opt/ros1-slam/ws/devel/include
  /opt/btcpp_v3/include
  /opt/ros1-slam/opt/btcpp_v3/include)

add_executable(bt_console src/bt_console.cpp)
target_include_directories(bt_console PRIVATE
  /opt/ros/noetic/include
  /opt/ros1-slam/ws/devel/include
  /opt/btcpp_v3/include
  /opt/ros1-slam/opt/btcpp_v3/include)
target_compile_options(bt_console PRIVATE -I/opt/ros/noetic/include)
target_link_libraries(bt_console ${catkin_LIBRARIES} BT::behaviortree_cpp_v3 Threads::Threads)
set_target_properties(bt_console PROPERTIES
  BUILD_RPATH "/opt/btcpp_v3/lib:/opt/ros1-slam/opt/btcpp_v3/lib"
  INSTALL_RPATH "/opt/btcpp_v3/lib:/opt/ros1-slam/opt/btcpp_v3/lib")

# goal_gate.py is an entry point -> catkin_install_python generates a relay
# wrapper (executes the source, fine for a script).
# goal_gate_policy.py is IMPORTED by it, so it must land as a real module file:
# a relay wrapper exports nothing (the source runs inside a private context
# dict), which made `import goal_gate_policy` yield an empty module and the
# node die with AttributeError: no attribute 'parse_allowed'.
catkin_install_python(PROGRAMS scripts/goal_gate.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(FILES scripts/goal_gate_policy.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# The devel space executes the relay wrapper from devel/lib/lite3_bt and puts
# that directory on sys.path, but plain install(FILES) only lands at
# `make install` -- so a fresh catkin_make left the wrapper without its module
# and goal_gate died with ModuleNotFoundError (hit on 2026-11-11 and again on
# 2026-11-12).  Keep the devel copy in sync on every build.
add_custom_target(lite3_bt_devel_policy ALL
  COMMAND ${CMAKE_COMMAND} -E make_directory
          ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
          ${CMAKE_CURRENT_SOURCE_DIR}/scripts/goal_gate_policy.py
          ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/goal_gate_policy.py
  COMMENT "Syncing goal_gate_policy.py into the devel space")
