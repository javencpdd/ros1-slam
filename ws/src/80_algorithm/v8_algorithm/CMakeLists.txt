cmake_minimum_required(VERSION 3.0.2)
project(v8_algorithm)

find_package(catkin REQUIRED)

# 纯 Python 算法层：节点入口安装到 devel/lib/<pkg>；框架/矩阵的 rosrun 用子路径，
# roslaunch 的 <node type> 用包根别名（原 v8_platform 的同名安装随目录迁移）。
catkin_package()

catkin_install_python(PROGRAMS
  planner/topic_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/planner)

catkin_install_python(PROGRAMS
  local_planner/path_goal_node.py
  local_planner/ros_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/local_planner)

install(PROGRAMS planner/topic_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} RENAME v8_global_planner.py)
install(PROGRAMS local_planner/path_goal_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} RENAME v8_path_goal.py)
install(PROGRAMS local_planner/ros_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} RENAME v8_local_planner.py)
