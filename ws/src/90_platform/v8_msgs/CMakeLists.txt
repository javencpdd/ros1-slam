cmake_minimum_required(VERSION 3.0.2)
project(v8_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
)

add_message_files(
  FILES
  SurfaceEdge.msg
  SupportSurface.msg
  SupportMap.msg
  MapOp.msg
  MapDelta.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs
)
