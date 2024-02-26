cmake_minimum_required(VERSION 3.0.2)
project(map_controller)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  cv_bridge
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES map_controller
#  CATKIN_DEPENDS rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/map.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
