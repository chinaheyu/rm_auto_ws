cmake_minimum_required(VERSION 3.0.2)
project(observer)

find_package(catkin REQUIRED COMPONENTS
  common
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
)

catkin_package(

)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
