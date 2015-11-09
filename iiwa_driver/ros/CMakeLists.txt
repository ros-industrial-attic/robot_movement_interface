cmake_minimum_required(VERSION 2.8.3)
project(iiwa_driver)

find_package(catkin REQUIRED COMPONENTS geometry_msgs roscpp rospy sensor_msgs control_msgs std_msgs message_generation actionlib_msgs robot_movement_interface)
find_package(Boost REQUIRED COMPONENTS system thread)

add_service_files(
   FILES StringCommand.srv
)

generate_messages(
   DEPENDENCIES geometry_msgs sensor_msgs std_msgs actionlib_msgs robot_movement_interface
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs std_msgs message_runtime actionlib_msgs robot_movement_interface
  DEPENDS Boost
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

