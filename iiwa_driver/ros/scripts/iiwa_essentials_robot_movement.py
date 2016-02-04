#!/usr/bin/env python
# -----------------------------------------------------------------------------
# Copyright 2015 Fraunhofer IPA
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# -----------------------------------------------------------------------------
# This node provides the Robot Movement Interface as well as state publishing
# - command list topic is used to control the robot
# - command result topic provides feedback after command completion
# - state publishing (tool, flang, joints, force, torque)
# The driver consists on two threads and a callback:
# 	Thread 1: state publishing: it peridically publishes robot state (100 Hz)
#	Thread 2: robot controlling: polling (100 Hz) to wait for new commands in the pipeline
#             if there are new commands, each will be sequentially executed but only after the termination of the current one
#	Command callback: adds the received commands to the pipeline. It is also possible to erase not executed commands from the pipeline
# -----------------------------------------------------------------------------
import math
import rospy
import thread
import copy
from geometry_msgs.msg			   import WrenchStamped
from sensor_msgs.msg               import JointState
from trajectory_msgs.msg           import *
from iiwa_driver.srv 	           import *
from robot_movement_interface.msg  import *
# ------------------------------------------------------------------------------------
# Notice that joint direct moves can be max. 0.087 radians away per axis (Kuka)  
# ------------------------------------------------------------------------------------

# ------------------------------------------------------------------------------------
# Main configuration - This default configuration will be overwritten by the config.yaml file
# ------------------------------------------------------------------------------------
conf_controlling_rate 			= 50 				# Hz for controlling 
conf_publishing_rate			= 100				# Hz for publishing robot state
conf_com_node     				= 'iiwa_telnet'		# Communication topic to sent the robot commands
conf_sub_topic_command_list		= 'command_list'	# Topic in which commands are received
conf_pub_topic_command_result 	= 'command_result' 	# Topic which gives command feedback
# ------------------------------------------------------------------------------------
# Published topics - This default configuration will be overwritten by the config.yaml file
# ------------------------------------------------------------------------------------
conf_pub_topic_joints		= 'joint_states'	# Joint configuration publishing
conf_pub_topic_wrench 		= 'tcp_wrench'		# Force and torque publishing
conf_pub_topic_iiwa_frame	= 'tool_frame' 		# Frame publishing as iiwa frame
conf_pub_topic_iiwa_flange	= 'flange_frame'	# Flange frame publishing as iiwa frame
# ------------------------------------------------------------------------------------
# Additional configuration - This default configuration will be overwritten by the config.yaml file
# ------------------------------------------------------------------------------------
conf_robot_base_name = 'base_link'
conf_joint_names  = ['joint_0','joint_1','joint_2','joint_3','joint_4','joint_5','joint_6']
conf_max_speed    = [1.71042267, 1.71042267, 1.74532925, 2.26892803, 2.44346095, 3.14159265, 3.14159265] # rad / s joint_0..joint_1

conf_cartesian_delta = 0.001 # Next position will be send to the robot x m in advance
conf_joints_delta = 0.001 # 0.01 radians

# ---------------------------------------------------------------------------------------
# Shared variables - Thread safe
# ---------------------------------------------------------------------------------------
lock 				= thread.allocate_lock() 	# Lock for shared variables
lockCom 			= thread.allocate_lock() 	# Lock for communaication node topic sharing
current_joints    	= []
current_effort    	= []
current_velocity  	= []
current_tcp_frame   = []
current_tcp_force   = []
current_tcp_torque  = []
command_list 		= [] 						# Contains the command list to be executed
command_active   	= None 						# Current active command
command_last 		= None 						# Last finished command

# ---------------------------------------------------------------------------------------
# Periodically asks the robot for the current state (joints, force, frame...) to update the shared variables
# Publishes sensor_msgs/JointState.msg into joint_states topic
# Publishes geometry_msgs/WrenchStamped.msg into tcp_wrench
# Publishes iiwa_frame and iiwa_flange
# ---------------------------------------------------------------------------------------
def publishers(rate):

	# Variables which are modified
	global current_joints
	global current_effort
	global current_velocity
	global current_tcp_frame
	global current_flange_frame
	global current_tcp_force
	global current_tcp_torque

	com = rospy.ServiceProxy(conf_com_node, StringCommand, persistent=True)	# Communication with the commander service

	publisher_joints = rospy.Publisher(conf_pub_topic_joints, JointState, queue_size = conf_publishing_rate) # Joint State publisher
	publisher_wrench = rospy.Publisher(conf_pub_topic_wrench, WrenchStamped, queue_size = conf_publishing_rate) # Force and Torque publisher
	publisher_frame  = rospy.Publisher(conf_pub_topic_iiwa_frame, EulerFrame, queue_size = conf_publishing_rate) # Publish frame
	publisher_flange = rospy.Publisher(conf_pub_topic_iiwa_flange, EulerFrame, queue_size = conf_publishing_rate)

	sleeper = rospy.Rate(rate)

	while not rospy.is_shutdown():

		now = rospy.Time.now()

		# Euler frame: intrinsic ZYX (alpha beta gamma)
		frame_msg = EulerFrame()
		flange_msg = EulerFrame()

		joint_state_msg = JointState()
		joint_state_msg.header.stamp = now
		joint_state_msg.header.frame_id = conf_robot_base_name

		wrench_stamp_msg = WrenchStamped()
		wrench_stamp_msg.header.stamp = now
		wrench_stamp_msg.header.frame_id = conf_robot_base_name

		with lockCom:
			strJoints   = com('get joint position','').response
			strEffort   = com('get joint torque','').response
			strForceTor = com('get cartesian force','').response
			strToolFrame= com('get tool frame','').response
			strFlangeFrame = com('get flange frame','').response

		joint_state_msg.name = conf_joint_names

		# -----------------------------------------------------------
		# Critical Section - Always for accesing shared variables (both write an read)
		# -----------------------------------------------------------
		with lock:
			# Update variables
			current_joints = [float(i) for i in strJoints.split()]
			current_effort = [float(i) for i in strEffort.split()]
			current_tcp_frame = [float(i) for i in strToolFrame.split()]
			current_tcp_frame[0] = current_tcp_frame[0] / 1000.0 # Kuka needs in mm but for anything else we use m
			current_tcp_frame[1] = current_tcp_frame[1] / 1000.0
			current_tcp_frame[2] = current_tcp_frame[2] / 1000.0
			current_tcp_force  = [float(i) for i in strForceTor.split()][:3]
			current_tcp_torque = [float(i) for i in strForceTor.split()][3:]
			# Fill iiwa frame
			frame_msg.x = current_tcp_frame[0]  		# m -> (Kuka needs in mm)
			frame_msg.y = current_tcp_frame[1]  		# m
			frame_msg.z = current_tcp_frame[2] 			# m
			frame_msg.alpha = current_tcp_frame[3]		# rz euler intrinsic zyx (rad)
			frame_msg.beta  = current_tcp_frame[4]		# ry euler intrinsic zyx
			frame_msg.gamma = current_tcp_frame[5]		# rx euler intrinsic zyx
			# Fill flange frame
			current_flange_frame = [float(i) for i in strFlangeFrame.split()]
			current_flange_frame[0] = current_flange_frame[0] / 1000.0
			current_flange_frame[1] = current_flange_frame[1] / 1000.0
			current_flange_frame[2] = current_flange_frame[2] / 1000.0
			flange_msg.x = current_flange_frame[0]
			flange_msg.y = current_flange_frame[1]
			flange_msg.z = current_flange_frame[2]
			flange_msg.alpha = current_flange_frame[3]
			flange_msg.beta  = current_flange_frame[4]
			flange_msg.gamma = current_flange_frame[5]
			# Fill joint states
			joint_state_msg.position = current_joints
			joint_state_msg.effort   = current_effort
			# Fill wrench
			wrench_stamp_msg.wrench.force.x = current_tcp_force[0]
			wrench_stamp_msg.wrench.force.y = current_tcp_force[1]
			wrench_stamp_msg.wrench.force.z = current_tcp_force[2]
			wrench_stamp_msg.wrench.torque.x = current_tcp_torque[0]
			wrench_stamp_msg.wrench.torque.y = current_tcp_torque[1]
			wrench_stamp_msg.wrench.torque.z = current_tcp_torque[2]
		# -----------------------------------------------------------

		publisher_frame.publish(frame_msg)
		publisher_flange.publish(flange_msg)
		publisher_joints.publish(joint_state_msg)
		publisher_wrench.publish(wrench_stamp_msg)

		sleeper.sleep()

# ---------------------------------------------------------------------------------------
# Move manager. This node does a polling waiting for new commands and executing then when available
# ---------------------------------------------------------------------------------------
def move_manager(rate):

	# Path variables
	global command_list		# Command pipeline
	global command_active	# Current active command
	global command_last		# Last executed command

	# If last command was a Direct Mode command, then we need to keep alive by resending the command (Kuka feature)

	com = rospy.ServiceProxy(conf_com_node, StringCommand, persistent=True) # IMPORTANT: Service calling must be locked, otherwise message order is not preserved

	# Command result feedback after command execution
	publisher_command_result = rospy.Publisher(conf_pub_topic_command_result, Result, queue_size = conf_publishing_rate)

	sleeper = rospy.Rate(rate)

	while not rospy.is_shutdown():

		# -----------------------------------------------------------
		# Critical Section
		# -----------------------------------------------------------
		with lock:
			code = 0
			# There is an active command, check if it is finished
			if command_active:
				if checkFinished(com, command_active, code):

					# Blending can be default or overwritten by the command
					blending = 0.0 # 1 mm
					if len(command_active.blending) > 0:
						blending = command_active.blending[0]
					if len(command_active.blending) > 1:
						delta = command_active.blending[1]

					# default result code is success
					# result code: 0 - success
					# result code: 100 - error (not implemented)
					# result code: 200 - collision
					result_code = 0

					# check if the target was reached when using force threshold commands or if there was a collision
					if command_active.command_type == "PTPFORCE" or command_active.command_type == "LINFORCE":
						# when target was not reached with the configuration
						if not multiDimensionalDistance(current_tcp_frame[:3], command_active.pose[:3], blending + conf_cartesian_delta):							
							result_code = 200

                    # Keep alive direct commands by coping the command again in the pipeline (DIRECT and SMART mode only)
					if len(command_list) == 0:
						if command_active.command_type == 'DIRECT' or command_active.command_type == 'SMART':
							command_list.insert(0, command_active)
					# Advance 1 in the pipeline
					command_last = command_active
					command_active = None
					# Publish message feedback
					command_result_msg = Result()
					command_result_msg.command_id = command_last.command_id
					command_result_msg.result_code = result_code
					publisher_command_result.publish(command_result_msg)
			# There is currently no active command, so execute next		
			if not command_active:
				if command_list:
					command_active = command_list.pop(0)
					executeCommand(com, command_active)
		# -----------------------------------------------------------
		sleeper.sleep()
		# -----------------------------------------------------------

# ---------------------------------------------------------------------------------------
# Returns true if active command is finished
# Returns code as parameter
# ---------------------------------------------------------------------------------------
def checkFinished(com, command, code):

	# Direct and Smart mode finishes immediately
	if command.command_type == 'DIRECT' or command.command_type == 'SMART': # Direct commands are processed at a define rate
		return True

	# Normal commands finish when the robot is in the near of the target position (Euclidean distance)
	# Blending and delta can be default or overwritten by the command
	blending = 0.0 # 1 mm
	delta = conf_cartesian_delta
	if command.pose_type == 'JOINTS':
		delta = conf_joints_delta

	if len(command.blending) > 0:
		blending = command.blending[0]
	if len(command.blending) > 1:
		delta = command.blending[1]

	if command.pose_type == 'EULER_INTRINSIC_ZYX':
		status = 'unknown'	# This is used to determine if the robot is stopped
		with lockCom:
			status = com('get status','').response
		if command.command_type == 'DIRECT' or command.command_type == 'SMART':
			# Status in direct or smart mode is stopped? -> TODO: Check it in sunrise
			if multiDimensionalDistance(current_flange_frame[:3], command.pose[:3], blending + delta):
				return True
		else:
			if status == 'stopped\r\n' or multiDimensionalDistance(current_tcp_frame[:3], command.pose[:3], blending + delta):
				return True
		return False
	elif command.pose_type == 'JOINTS':
		status = 'unknown'
		with lockCom:
			status = com('get status','').response
		if status == 'stopped\r\n' or multiDimensionalDistance(current_joints, command.pose, blending + delta):
			return True
		else:
			return False
	else:
		return True

# ---------------------------------------------------------------------------------------
# Executes a new command
# ---------------------------------------------------------------------------------------
def executeCommand(com, command):

	if command.command_type == 'LIN' and command.pose_type == 'EULER_INTRINSIC_ZYX':
		assert len(command.pose) == 6 and len(command.velocity) == 1 and len(command.blending) > 0 
		with lockCom:
			temp_pose = [command.pose[0] * 1000.0, command.pose[1] * 1000.0, command.pose[2] * 1000.0, command.pose[3], command.pose[4], command.pose[5]]
			com('lin move', ' '.join(str(i) for i in temp_pose) + ' ' + str(command.velocity[0]) + ' ' + str(command.blending[0] * 1000.0))
	elif command.command_type == 'PTP' and command.pose_type == 'EULER_INTRINSIC_ZYX':
		assert len(command.pose) == 6 and len(command.velocity) == 1 and len(command.blending) > 0 
		with lockCom:
			temp_pose = [command.pose[0] * 1000.0, command.pose[1] * 1000.0, command.pose[2] * 1000.0, command.pose[3], command.pose[4], command.pose[5]]
			com('ptp move', ' '.join(str(i) for i in temp_pose) + ' ' + str(command.velocity[0]) + ' ' + str(command.blending[0] * 1000.0))
	elif command.command_type == 'PTP' and command.pose_type == 'JOINTS':
		assert len(command.pose) == 7 and len(command.velocity) == 7
		with lockCom:
			com('joint move', ' '.join(str(i) for i in command.pose) + ' ' +  ' '.join(str(i) for i in command.velocity) )
	elif command.command_type == 'LINFORCE' and command.pose_type == 'EULER_INTRINSIC_ZYX':
		assert len(command.pose) == 6 and len(command.velocity) == 1 and len(command.blending) > 0 and len(command.force_threshold) == 3
		with lockCom:
			temp_pose = [command.pose[0] * 1000.0, command.pose[1] * 1000.0, command.pose[2] * 1000.0, command.pose[3], command.pose[4], command.pose[5]]
			temp_force_threshold = [command.force_threshold[0], command.force_threshold[1], command.force_threshold[2]]
			com('linforce move', ' '.join(str(i) for i in temp_pose) + ' ' + str(command.velocity[0]) + ' ' + str(command.blending[0] * 1000.0) + ' ' + ' '.join(str(i) for i in temp_force_threshold))
	elif command.command_type == 'PTPFORCE' and command.pose_type == 'EULER_INTRINSIC_ZYX':
		assert len(command.pose) == 6 and len(command.velocity) == 1 and len(command.blending) > 0 and len(command.force_threshold) == 3
		with lockCom:
			temp_pose = [command.pose[0] * 1000.0, command.pose[1] * 1000.0, command.pose[2] * 1000.0, command.pose[3], command.pose[4], command.pose[5]]
			temp_force_threshold = [command.force_threshold[0], command.force_threshold[1], command.force_threshold[2]]
			com('ptpforce move', ' '.join(str(i) for i in temp_pose) + ' ' + str(command.velocity[0]) + ' ' + str(command.blending[0] * 1000.0) + ' ' + ' '.join(str(i) for i in temp_force_threshold))
	elif command.command_type == 'DIRECT' and command.pose_type == 'JOINTS':
		assert len(command.pose) == 7 and len(command.velocity) == 7
		with lockCom:
			com('direct joint move', ' '.join(str(i) for i in command.pose) + ' ' + ' '.join(str(i) for i in command.velocity))
	elif command.command_type == 'SMART' and command.pose_type == 'EULER_INTRINSIC_ZYX':
		assert len(command.pose) == 6 and len(command.velocity) == 1
		with lockCom:
			temp_pose = [command.pose[0] * 1000.0, command.pose[1] * 1000.0, command.pose[2] * 1000.0, command.pose[3], command.pose[4], command.pose[5]]
			com('smart cartesian move', ' '.join(str(i) for i in temp_pose) + ' ' + str(command.velocity[0]))
	elif command.command_type == 'SMART' and command.pose_type == 'JOINTS':
		assert len(command.pose) == 7 and len(command.velocity) == 7
		with lockCom:
			com('smart joint move', ' '.join(str(i) for i in command.pose) + ' ' + ' '.join(str(i) for i in command.velocity))
					
# ---------------------------------------------------------------------------------------
# Returns true if the multidimensional euclidean distance between a and b is less than delta	
# ---------------------------------------------------------------------------------------
def multiDimensionalDistance(a, b, delta):
	assert len(a) == len(b)
	assert delta > 0
	return sum([ (x-y)*(x-y) for x, y in zip(a, b)]) < delta*delta
	
def maxDistance(a,b):
	assert len(a) == len(b)
	return max([ math.fabs(x-y) for x,y in zip(a,b) ])

# ---------------------------------------------------------------------------------------
# Actually not used, this relies on the client, not on the driver
# This function returns a command in the middle two if the distance is too big from current position to the command 
# This function must be inside a locked block  
def splitPath(command):
	if command.pose_type == 'EULER_INTRINSIC_ZYX':
		if maxDistance(command.pose[:3], current_flange_frame[:3]) > conf_direct_cartesian_delta:
			middle = copy.deepcopy(command)
			middle.command_id = 0
			middle.pose = [ (x + y) / 2.0 for x,y in zip(command.pose,current_flange_frame)]
			middle.pose[3] = command.pose[3]
			middle.pose[4] = command.pose[4]
			middle.pose[5] = command.pose[5]
			return middle
	elif command.pose_type == 'JOINTS':
		if maxDistance(command.pose, current_joints) > conf_direct_joints_delta:
			middle = copy.deepcopy(command)
			middle.command_id = 0
			middle.pose = [ (x + y) / 2.0 for x,y in zip(command.pose, current_joints)]
			return middle
	return None
# ---------------------------------------------------------------------------------------

# ---------------------------------------------------------------------------------------
# Subscription to joint_path_command (trajectory_msgs/JointTrajectory)
# Updates the current trajectory pipeline
# ---------------------------------------------------------------------------------------
def commands_callback(msg):

	global command_list

	# -----------------------------------------------------------
	# Critical Section 
	# -----------------------------------------------------------
	with lock:
		if msg.replace_previous_commands:
			command_list = msg.commands
		else:
			command_list.extend(msg.commands)
	# -----------------------------------------------------------

# ---------------------------------------------------------------------------------------
# Overwrite default parameters with the config.yaml configuration
# ---------------------------------------------------------------------------------------
def loadParameters():

	global conf_controlling_rate
	global conf_publishing_rate
	global conf_com_node
	global conf_sub_topic_command_list
	global conf_pub_topic_command_result
	global conf_pub_topic_joints
	global conf_pub_topic_wrench
	global conf_pub_topic_iiwa_frame
	global conf_pub_topic_iiwa_flange
	global conf_robot_base_name
	global conf_joint_names
	global conf_max_speed
	global conf_cartesian_delta
	global conf_joints_delta
	
	conf_controlling_rate =  rospy.get_param('~controlling_rate', conf_controlling_rate)
	conf_publishing_rate = rospy.get_param('~publishing_rate', conf_publishing_rate)
	
	conf_com_node = rospy.get_param('~commander_service_name', conf_com_node)
	conf_sub_topic_command_list	= rospy.get_param('~command_list', conf_sub_topic_command_list)
	conf_pub_topic_command_result = rospy.get_param('~command_result', conf_pub_topic_command_result)

	conf_pub_topic_joints = rospy.get_param('~topic_joints', conf_pub_topic_joints)
	conf_pub_topic_wrench = rospy.get_param('~topic_wrench', conf_pub_topic_wrench)
	conf_pub_topic_iiwa_frame = rospy.get_param('~topic_iiwa_frame', conf_pub_topic_iiwa_frame)
	conf_pub_topic_iiwa_flange = rospy.get_param('~topic_iiwa_flange', conf_pub_topic_iiwa_flange)
	
	conf_robot_base_name = rospy.get_param('~robot_base_name', conf_robot_base_name)
	conf_joint_names = rospy.get_param('~joint_names', conf_joint_names)
	conf_max_speed = rospy.get_param('~max_speed', conf_max_speed)
	
	conf_cartesian_delta = rospy.get_param('~cartesian_delta', conf_cartesian_delta)
	conf_joints_delta = rospy.get_param('~joints_delta', conf_joints_delta)

# ---------------------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------------------
if __name__ == '__main__':
	try:

		rospy.init_node('iiwa_robot_movement')
		
		loadParameters()
		
		rospy.wait_for_service(conf_com_node)

		thread.start_new_thread(publishers,(conf_publishing_rate,))
		thread.start_new_thread(move_manager,(conf_controlling_rate,))

		rospy.Subscriber(conf_sub_topic_command_list, CommandList, commands_callback)
		rospy.spin()

	except rospy.ROSInterruptException:
		pass
