cmake_minimum_required(VERSION 3.0.2)
project(bringup)

find_package(catkin REQUIRED COMPONENTS
  common
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES bringup
#  CATKIN_DEPENDS common
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
