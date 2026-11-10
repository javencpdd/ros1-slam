cmake_minimum_required(VERSION 3.0.2)
project(lite3_bt)

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs std_msgs)
find_package(behaviortree_cpp_v3 REQUIRED
  PATHS "/opt/ros1-slam/opt/btcpp_v3/lib/cmake/behaviortree_cpp_v3"
  NO_DEFAULT_PATH)
find_package(Threads REQUIRED)

catkin_package(CATKIN_DEPENDS roscpp geometry_msgs std_msgs)

include_directories(${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
  /opt/ros/noetic/include
  /opt/ros1-slam/ws/devel/include
  /opt/ros1-slam/opt/btcpp_v3/include)

add_executable(bt_console src/bt_console.cpp)
target_include_directories(bt_console PRIVATE
  /opt/ros/noetic/include
  /opt/ros1-slam/ws/devel/include
  /opt/ros1-slam/opt/btcpp_v3/include)
target_compile_options(bt_console PRIVATE -I/opt/ros/noetic/include)
target_link_libraries(bt_console ${catkin_LIBRARIES} BT::behaviortree_cpp_v3 Threads::Threads)
set_target_properties(bt_console PROPERTIES
  BUILD_RPATH "/opt/ros1-slam/opt/btcpp_v3/lib"
  INSTALL_RPATH "/opt/ros1-slam/opt/btcpp_v3/lib")

# goal_gate.py imports goal_gate_policy.py from its own directory; both must
# land in the same devel/install lib dir or the ROS node fails at import time.
catkin_install_python(PROGRAMS scripts/goal_gate.py scripts/goal_gate_policy.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
