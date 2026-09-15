cmake_minimum_required(VERSION 3.0.2)
project(hd_map_builder)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  morai_msgs
  rospy
  std_msgs
  visualization_msgs
)

catkin_python_setup()
catkin_package(CATKIN_DEPENDS geometry_msgs morai_msgs rospy std_msgs visualization_msgs)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_frames.py)
  catkin_add_nosetests(test/test_correspondence.py)
  catkin_add_nosetests(test/test_schema.py)
  catkin_add_nosetests(test/test_map_overlay.py)
  catkin_add_nosetests(test/test_corridor.py)
  catkin_add_nosetests(test/test_nominal.py)
  catkin_add_nosetests(test/test_current_builder.py)
endif()

catkin_install_python(PROGRAMS
  scripts/audit_legacy_alignment.py
  scripts/build_current_corridor.py
  scripts/legacy_origin_overlay_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch rviz schemas
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
