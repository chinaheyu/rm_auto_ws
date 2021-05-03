cmake_minimum_required(VERSION 3.0.2)
project(flow_control)

find_package(catkin REQUIRED COMPONENTS
  common
  rospy
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES flow_control
#  CATKIN_DEPENDS common rospy
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/flow_control_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
