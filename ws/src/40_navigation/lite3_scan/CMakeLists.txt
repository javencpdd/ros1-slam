cmake_minimum_required(VERSION 3.0.2)
project(lite3_scan)
find_package(catkin REQUIRED)
catkin_package()
install(DIRECTORY launch scripts
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
if(CATKIN_ENABLE_TESTING)
  install(DIRECTORY verify
          DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
endif()
