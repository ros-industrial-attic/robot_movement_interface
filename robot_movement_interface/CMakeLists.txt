cmake_minimum_required(VERSION 2.8.3)
project(robot_movement_interface)

find_package(catkin REQUIRED COMPONENTS message_generation actionlib_msgs actionlib std_msgs)

add_message_files(DIRECTORY msg
  FILES
  Result.msg
  Command.msg
  CommandList.msg
  EulerFrame.msg
)

add_service_files(
  FILES
  GetIO.srv
  SetIO.srv
  SetFloat.srv
)

add_action_files(DIRECTORY action
  FILES
  Commands.action
)

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime actionlib_msgs std_msgs
)
