cmake_minimum_required(VERSION 3.0.2)
project(detector)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  common
)

find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES detector
#  CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIR}
  ${realsense2_INCLUDE_DIR}
)

add_executable(${PROJECT_NAME}_node src/detector_node.cpp)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${realsense2_LIBRARY}
)
