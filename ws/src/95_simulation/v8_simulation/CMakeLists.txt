cmake_minimum_required(VERSION 3.0.2)
project(v8_simulation)

find_package(catkin REQUIRED)
catkin_package()

catkin_install_python(PROGRAMS
  scripts/gazebo_carrier3d_node.py
  scripts/gazebo_truth.py
  scripts/mujoco_truth.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(FILES scripts/carrier_kinematics.py scripts/support_cells.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY fixtures
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
