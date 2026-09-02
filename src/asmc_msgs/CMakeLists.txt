cmake_minimum_required(VERSION 3.0.2)
project(asmc_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  std_msgs
)

add_message_files(
  FILES
  VehicleState.msg
  DetectedObject.msg
  DetectedObjectArray.msg
  BehaviorContext.msg
  TrajectoryPoint.msg
  Trajectory.msg
  PlanFeedback.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  geometry_msgs
  message_runtime
  std_msgs
)
