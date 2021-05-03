cmake_minimum_required(VERSION 3.0.2)
project(monocular_camera)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  common
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIR}
)

add_executable(${PROJECT_NAME}_node src/monocular_camera_node.cpp)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
