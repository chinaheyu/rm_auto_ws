cmake_minimum_required(VERSION 3.0.2)
project(referee_system)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  common
  roscpp
)

catkin_package(
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

aux_source_directory(src DIR_SRCS)
add_executable(${PROJECT_NAME}_node ${DIR_SRCS})

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
