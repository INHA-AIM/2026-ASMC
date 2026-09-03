cmake_minimum_required(VERSION 3.0.2)
project(Msg)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  message_generation
)

add_message_files(
  DIRECTORY msg
  FILES
  Yolo_bbox.msg
  Yolo_bbox_arr.msg
  TrackedObject.msg
  TrackedObjectArray.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  std_msgs
  geometry_msgs
  message_runtime
)
