cmake_minimum_required(VERSION 3.0.2)
project(vid_transmission_ip)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES vid_transmission_ip
#  CATKIN_DEPENDS rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/vid_transmission_ip.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
