cmake_minimum_required(VERSION 2.8.3)
project(node_function_monitor)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

add_message_files(
  FILES
  BoundingBoxes.msg
  BoundingBox.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)

install (DIRECTORY launch msg src DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


