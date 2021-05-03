cmake_minimum_required(VERSION 3.0.2)
project(common)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  actionlib_msgs
)

catkin_python_setup()

add_message_files(
  FILES
  serial_received_msg.msg
  serial_send_msg.msg
  target_center.msg
  cmd_belt.msg
  cmd_action.msg
  cmd_vel.msg
  cmd_align.msg
  bounding_box.msg
  game_state.msg
)

add_service_files(
  FILES
)

add_action_files(
  FILES
)

generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES common
#  CATKIN_DEPENDS message_generation roscpp rospy
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
