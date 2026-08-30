cmake_minimum_required(VERSION 3.0.2)
project(data_collection)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  morai_msgs
  message_filters
)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS
  scripts/sync_collector_node.py
  scripts/capture_mode_collector_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
