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
# This node manages the joint_path_command interface defined in 
# http://wiki.ros.org/Industrial/Industrial_Robot_Driver_Spec
# -----------------------------------------------------------------------------
# TODO: Use config.yaml instead of python defined variables

import math
import rospy
import thread

from control_msgs.msg           import FollowJointTrajectoryFeedback
from geometry_msgs.msg			import WrenchStamped
from sensor_msgs.msg            import JointState
from trajectory_msgs.msg        import *
from iiwa_driver.srv            import *

# Important, conf_delta_blending + conf_direct_delta <= 0.087

# User editable configuration
conf_rate 				= 200 # Hz
conf_com_node     		= 'iiwa_telnet'
conf_joints_topic 		= 'joint_states'
conf_wrench_topic 		= 'tcp_wrench'
conf_base_name    		= 'base_link'
conf_node_name 			= 'iiwa_driver'
conf_trajectory_path_name 	= 'joint_path_command'
conf_delta_blending       	= 0.02  # 0.02 rad blending distance (distance at which next point is sent)

# Robot specs
conf_joint_names  = ['joint_0','joint_1','joint_2','joint_3','joint_4','joint_5','joint_6']
conf_max_speed    = [1.71042267, 1.71042267, 1.74532925, 2.26892803, 2.44346095, 3.14159265, 3.14159265] # rad / s joint_0..joint_1
conf_direct_delta = 0.05 # Joint angular distance per axis must not exceed this value

# ---------------------------------------------------------------------------------------
# Shared variables
# ---------------------------------------------------------------------------------------
lock = thread.allocate_lock()
lockCom = thread.allocate_lock()
current_joints    	= []
current_effort    	= []
current_velocity  	= []
current_tcp_force   = []
current_tcp_torque  = []
trajectory_points 	= [] # Stores the current trajectory
trajectory_start  	= None # Stores start time, intialized after init node
trajectory_active 	= None # Stores the current active position
# ---------------------------------------------------------------------------------------
# Publishes sensor_msgs/JointState.msg into joint_states topic
# It also stores these values as local thread-safe variables
# ---------------------------------------------------------------------------------------
def joint_states_publisher(rate):

	global conf_base_name

	global current_joints
	global current_effort
	global current_tcp_force
	global current_tcp_torque

	com = rospy.ServiceProxy(conf_com_node, StringCommand, persistent=True)	# Communication with the commander service
	# communication must be persistent in order to keep the rate stable
	
	publisher_joints = rospy.Publisher(conf_joints_topic, JointState, queue_size = conf_rate) # Joint State publisher
	publisher_wrench = rospy.Publisher(conf_wrench_topic, WrenchStamped, queue_size = conf_rate) # Force and Torque publisher

	sleeper = rospy.Rate(rate)

	while not rospy.is_shutdown():

		now = rospy.Time.now()

		joint_state_msg = JointState()
		joint_state_msg.header.stamp = now
		joint_state_msg.header.frame_id = conf_base_name

		wrench_stamp_msg = WrenchStamped()
		wrench_stamp_msg.header.stamp = now
		wrench_stamp_msg.header.frame_id = conf_base_name

		with lockCom:
			strJoints   = com('get joint position','').response
			strEffort   = com('get joint torque','').response
			strForceTor = com('get cartesian force','').response

		joint_state_msg.name = conf_joint_names

		# -----------------------------------------------------------
		# Critical Section - Always for accesing shared variables (both write an read)
		# -----------------------------------------------------------
		with lock:
			# Update variables
			current_joints = [float(i) for i in strJoints.split()]
			current_effort = [float(i) for i in strEffort.split()]
			current_tcp_force  = [float(i) for i in strForceTor.split()][:3]
			current_tcp_torque = [float(i) for i in strForceTor.split()][3:]
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

		publisher_joints.publish(joint_state_msg)
		publisher_wrench.publish(wrench_stamp_msg)

		sleeper.sleep()

# ---------------------------------------------------------------------------------------
# Publishes control_msgs/FollowJointTrajectoryFeedback with previously read in joint_states_publisher
# ---------------------------------------------------------------------------------------
def feedback_states_publisher(rate):

	global current_joints
	global current_velocity
	
	publisher = rospy.Publisher('feedback_states', FollowJointTrajectoryFeedback, queue_size = conf_rate)
	sleeper = rospy.Rate(rate)

	while not rospy.is_shutdown():

		msg = FollowJointTrajectoryFeedback()
		msg.header.stamp = rospy.Time.now()
		msg.header.frame_id = conf_base_name
		msg.joint_names = conf_joint_names
		msg.actual = JointTrajectoryPoint()
		# -----------------------------------------------------------
		# Critical Section
		# -----------------------------------------------------------
		with lock:
			msg.actual.positions = current_joints
			msg.actual.velocities = current_velocity
			msg.actual.effort = current_effort
			msg.actual.time_from_start = rospy.get_rostime() - trajectory_start
		# -----------------------------------------------------------

		publisher.publish(msg)
		sleeper.sleep()

# ---------------------------------------------------------------------------------------
# Move manager, this thread is the responsible for performing an smooth move up from the last received trajectory
# ---------------------------------------------------------------------------------------
def move_manager(rate):

	global current_joints

	global trajectory_points
	global trajectory_start
	global trajectory_active

	# Keep alive is a trick to keep the direct mode working. Kuka direct mode has as feature than after approx. 7 seconds activate the
	# brakes and throws exception
	keep_alive_last_iteration = 0
	
	com = rospy.ServiceProxy(conf_com_node, StringCommand, persistent=True) 
	# rospy seems to create a singleton when defining a ServiceProxy. This means messages can be mixed if
	# communication is not locked -> concurrence problem
	
	sleeper = rospy.Rate(rate)

	while not rospy.is_shutdown():

		# -----------------------------------------------------------
		# Critical Section
		# -----------------------------------------------------------
		with lock:

			# Move is finished when delta distance in rad is smaller than conf_delta_blending	
			if trajectory_active and getMaxJointDelta2(current_joints, trajectory_active.positions) < conf_delta_blending:
				trajectory_active = None

			if not trajectory_active and trajectory_points:
				trajectory_active = trajectory_points.pop(0)

				time_target = (trajectory_start + trajectory_active.time_from_start).to_sec() # time + duration = time
				time_current = rospy.get_rostime().to_sec()
				time_left = time_target - time_current # time left to arrive to next point

				# THIS MUST BE MODIFIED FOR IIWA 14
				# Speed is transformed from rad/s to percentage by using the robot specs
				if time_left > 0:
					joint_velocities = [ angle_difference(x,y) / time_left for x, y in zip(trajectory_active.positions, current_joints)]
					joint_velocities_relative = joint_velocities_to_percent(joint_velocities)
				else:
					joint_velocities_relative = [1] * 7 # Maximal speed, robot is late

				with lockCom:
					com('direct joint move', ' '.join(str(i) for i in trajectory_active.positions) + ' ' + ' '.join(str(i) for i in joint_velocities_relative))
			
				keep_alive_last_iteration = 0

			else:
				# After a second of inactivity, a keep alive command to the current position is sent
				# If not, robot controller will block the program (direct mode)
				if keep_alive_last_iteration > conf_rate:
					if not trajectory_active:
						with lockCom: 
							com('direct joint move', ' '.join(str(i) for i in current_joints) + ' ' + '0.1 0.1 0.1 0.1 0.1 0.1 0.1')	
					keep_alive_last_iteration = 0
			keep_alive_last_iteration += 1

		# -----------------------------------------------------------
		sleeper.sleep()

# ---------------------------------------------------------------------------------------
# Rearranges the point path following the name convention joint_0, ... joint_6
# Warning: This function has side effects
# ---------------------------------------------------------------------------------------
def rearrange(joint_trajectory):

	mapping = [joint_trajectory.joint_names.index(j) for j in conf_joint_names]

	for point in joint_trajectory.points:

		temp_positions = []
		temp_velocities = []
		temp_accelerations = []
		temp_effort = []

		for i in range(len(point.positions)):
			temp_positions.append(point.positions[mapping[i]])
		for i in range(len(point.velocities)):
			temp_velocities.append(point.velocities[mapping[i]])
		for i in range(len(point.accelerations)):
			temp_accelerations.append(point.accelerations[mapping[i]])
		for i in range(len(point.effort)):
			temp_effort.append(point.effort[mapping[i]])

		point.positions = temp_positions
		point.velocities = temp_velocities
		point.accelerations = temp_accelerations
		point.effort = temp_effort

	joint_trajectory.joint_names = conf_joint_names

# Returns the max delta angular distance between a and b
def getMaxJointDelta(a, b):
	return getMaxJointDelta2(a.positions, b.positions)

def getMaxJointDelta2(a,b):
	return max([ math.fabs(y - x) for x, y in zip(a, b)])

# ---------------------------------------------------------------------------------------
# This function returns intermediate configuration between two
# ---------------------------------------------------------------------------------------
def getMiddle(a, b):
	middle = JointTrajectoryPoint()

	middle.positions = [ x + (y - x) / 2.0 for x, y in zip(a.positions, b.positions)]
	middle.velocities = [ x + (y - x) / 2.0 for x, y in zip(a.velocities, b.velocities)]
	middle.accelerations = [ x + (y - x) / 2.0 for x, y in zip(a.accelerations, b.accelerations)]
	middle.effort = [ x + (y - x) / 2.0 for x, y in zip(a.effort, b.effort)]
	middle.time_from_start = rospy.Duration.from_sec(a.time_from_start.to_sec() + (b.time_from_start.to_sec() - a.time_from_start.to_sec()) / 2.0)

	return middle

# ---------------------------------------------------------------------------------------
# This function adds intermediate moves if joint distance is larger than conf_direct_delta distance
# ---------------------------------------------------------------------------------------
def splitIfNecessary(joint_trajectory):

	index = 0

	while index < (len(joint_trajectory.points) - 1):

		a = joint_trajectory.points[index]
		b = joint_trajectory.points[index + 1]

		if getMaxJointDelta(a,b) > conf_direct_delta:
			joint_trajectory.points.insert(index + 1, getMiddle(a,b))
		else:
			index += 1


# ---------------------------------------------------------------------------------------
# Subscription to joint_path_command (trajectory_msgs/JointTrajectory)
# Updates the current trajectory. Warning, side effect in msg
# ---------------------------------------------------------------------------------------
def joint_path_command_callback(msg):

	global trajectory_points
	global trajectory_start
	global trajectory_active
	global current_joints
	global conf_direct_delta

	rearrange(msg)

	# Check distance from current position to first target point, must be near
	if (msg.points):
		with lock:
			if getMaxJointDelta2(current_joints, msg.points[0].positions) > conf_direct_delta:
				return

	splitIfNecessary(msg)

	# -----------------------------------------------------------
	# Critical Section 
	# -----------------------------------------------------------
	with lock:
		trajectory_points = msg.points
		trajectory_start  = msg.header.stamp
		trajectory_active = None
	# -----------------------------------------------------------

# ---------------------------------------------------------------------------------------
# Returns true if the n-dimensional euclidian distance in radians is less than delta
# ---------------------------------------------------------------------------------------
def is_near(a, b, delta):
	return math.sqrt(sum([ (x-y)**2  for (x,y) in zip(a,b) ])) <= delta

# ---------------------------------------------------------------------------------------
# Converts rad / s in % (relative speed) for the iiwa robot
# ---------------------------------------------------------------------------------------
def joint_velocities_to_percent(joint_velocities):
	return [percent_fabs(x / y) for x, y in zip(joint_velocities, conf_max_speed)]

def percent_fabs(x):
	return math.fabs(x) if math.fabs(x) < 1 else 1

def angle_difference(a,b):
	d = math.fabs(a - b) % (2*math.pi)
	if d > math.pi:
		return 2 * math.pi - d
	else:
		return d

# ---------------------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------------------
if __name__ == '__main__':
	try:

		rospy.init_node(conf_node_name)
		rospy.wait_for_service(conf_com_node)

		trajectory_start = rospy.get_rostime()

		thread.start_new_thread(joint_states_publisher,(conf_rate,))
		thread.start_new_thread(feedback_states_publisher,(conf_rate,))
		thread.start_new_thread(move_manager,(conf_rate,))

		rospy.Subscriber(conf_trajectory_path_name, JointTrajectory, joint_path_command_callback)
		rospy.spin()

	except rospy.ROSInterruptException:
		pass
