cmake_minimum_required(VERSION 3.0.2)
project(asmc_perception)

find_package(catkin REQUIRED COMPONENTS
  asmc_msgs
  rospy
)

catkin_package(
  CATKIN_DEPENDS
    asmc_msgs
    rospy
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_facade_contract.py)
endif()

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
