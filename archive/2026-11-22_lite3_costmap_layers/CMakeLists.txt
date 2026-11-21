cmake_minimum_required(VERSION 3.0.2)
project(lite3_costmap_layers)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  costmap_2d
  pluginlib
  grid_map_core
  grid_map_msgs
  grid_map_ros
)

# 无消息／无服务生成 ⇒ 不涉及 livox_laser_simulation 那类"消息头先生成"的目标排序坑。
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp costmap_2d pluginlib grid_map_core grid_map_msgs grid_map_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/elevation_merged_layer.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

# pluginlib 描述件：源码空间放在包根（devel 空间 ros::package::getPath 指到源码目录，
# pluginlib 就在那里找），安装空间再显式装进 share。
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(FILES plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
