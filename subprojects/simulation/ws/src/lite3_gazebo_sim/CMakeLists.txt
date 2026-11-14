cmake_minimum_required(VERSION 3.0.2)
project(lite3_gazebo_sim)

find_package(catkin REQUIRED COMPONENTS roscpp tf nav_msgs geometry_msgs)
catkin_package(CATKIN_DEPENDS roscpp tf nav_msgs geometry_msgs)
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
add_library(lite3_planar_drive src/lite3_planar_drive.cpp)
target_link_libraries(lite3_planar_drive ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})
set_target_properties(lite3_planar_drive PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib)

install(TARGETS lite3_planar_drive
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch worlds stairstudy scenarios config urdf dynamics
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
file(GLOB LITE3_GAZEBO_SCRIPTS scripts/*.py scripts/*.sh)
install(PROGRAMS ${LITE3_GAZEBO_SCRIPTS} DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
