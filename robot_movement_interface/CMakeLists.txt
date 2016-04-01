cmake_minimum_required(VERSION 2.8.3)
project(robot_movement_interface)

find_package(catkin REQUIRED COMPONENTS genmsg actionlib_msgs actionlib std_msgs message_generation)

add_message_files(
  FILES
  Result.msg
  Command.msg
  CommandList.msg
  EulerFrame.msg
)

add_action_files(
  DIRECTORY action FILES Commands.action
)

generate_messages(  
  DEPENDENCIES actionlib_msgs std_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs std_msgs
)
