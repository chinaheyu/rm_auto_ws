cmake_minimum_required(VERSION 3.0.2)
project(turtle_bridge)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  common
  roscpp
  turtlesim
)

catkin_package(

)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/turtle_bridge_node.cpp)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
