cmake_minimum_required(VERSION 2.8.3)
project(robot_movement_interface)

find_package(catkin REQUIRED COMPONENTS std_msgs message_generation)

add_message_files(
  FILES
  Result.msg
  Command.msg
  CommandList.msg
  EulerFrame.msg
)

generate_messages(
  DEPENDENCIES std_msgs
)

catkin_package(
  CATKIN_DEPENDS std_msgs
)
