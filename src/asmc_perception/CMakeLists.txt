cmake_minimum_required(VERSION 3.0.2)
project(asmc_perception)

find_package(catkin REQUIRED COMPONENTS
  asmc_msgs
  rospy
  sensor_msgs
  std_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    asmc_msgs
    rospy
    sensor_msgs
    std_msgs
)

catkin_install_python(PROGRAMS
  scripts/traffic_light_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_facade_contract.py)
  catkin_add_nosetests(test/test_traffic_light_contract.py)
endif()

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
