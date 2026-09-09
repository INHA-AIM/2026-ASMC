cmake_minimum_required(VERSION 3.0.2)
project(morai_udp_bridge)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  diagnostic_msgs
  morai_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs geometry_msgs diagnostic_msgs morai_msgs
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_udp_bridge.py)
endif()

catkin_install_python(PROGRAMS
  scripts/morai_udp_bridge.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
