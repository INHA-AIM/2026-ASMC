cmake_minimum_required(VERSION 3.0.2)
project(traffic_light)

find_package(catkin REQUIRED COMPONENTS rospy sensor_msgs std_msgs)

catkin_package(CATKIN_DEPENDS rospy sensor_msgs std_msgs)

catkin_install_python(PROGRAMS live_infer.py traffic_light.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(FILES model.py best.pt
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
