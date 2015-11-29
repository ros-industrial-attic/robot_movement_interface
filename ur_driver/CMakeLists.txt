cmake_minimum_required(VERSION 2.8.3)
project(ur_driver)

find_package(catkin REQUIRED COMPONENTS
	actionlib
	actionlib_msgs
	geometry_msgs
	trajectory_msgs
	roscpp
	rospy
	sensor_msgs
	tf
	message_generation
	control_msgs
	robot_movement_interface
)

find_package(Boost REQUIRED COMPONENTS
	system 
	thread)

## Set to debug compiler mode
SET(CMAKE_BUILD_TYPE debug)

add_action_files(
   FILES
   DigIOArray.action
   DigIO.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
)

catkin_package(
    CATKIN_DEPENDS
        message_runtime
        actionlib
        actionlib_msgs
        geometry_msgs
        trajectory_msgs
        roscpp
        rospy
        sensor_msgs
        tf
		robot_movement_interface
    DEPENDS Boost
)


include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)


file(GLOB_RECURSE ur_driver_src RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

add_executable(ur_driver
  ${ur_driver_src}
)

add_dependencies(ur_driver 
  sensor_msgs_gencpp
  ${PROJECT_NAME}_gencfg
  ${PROJECT_NAME}_gencpp
)

target_link_libraries(ur_driver
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)
