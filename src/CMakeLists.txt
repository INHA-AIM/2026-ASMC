cmake_minimum_required(VERSION 3.10)
project(asmc_workspace)
find_package(catkin REQUIRED)
catkin_workspace()

include("${CMAKE_CURRENT_SOURCE_DIR}/../cmake/AsmcBuildProfiles.cmake")
