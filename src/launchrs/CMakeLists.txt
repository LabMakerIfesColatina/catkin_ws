cmake_minimum_required(VERSION 3.0.2)
project(launchrs)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)




catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES launchrs
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)






install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.yaml"
)