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
# This nodes wraps the Robot Movement Interface into a follow_joint_trajectory action 
# according to http://wiki.ros.org/joint_trajectory_action
# -----------------------------------------------------------------------------
import rospy
import time
import thread
import actionlib

from control_msgs.msg    import FollowJointTrajectoryGoal
from control_msgs.msg    import FollowJointTrajectoryResult
from control_msgs.msg    import FollowJointTrajectoryFeedback
from control_msgs.msg    import FollowJointTrajectoryAction
from trajectory_msgs.msg import JointTrajectory
from robot_movement_interface.msg import *
from sensor_msgs.msg	 import JointState

conf_joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
lock = thread.allocate_lock()
joint_states = JointState()

conf_blending = 0.02 # M

# ------------------------------------------------------------------------
# Callback function executed after the publication of the current robot position
# ------------------------------------------------------------------------
def joint_states_callback(msg):
	with lock:
		global joint_states
		joint_states = msg

class MoveItAction(object):
	
	_feedback = FollowJointTrajectoryFeedback()
	_result = FollowJointTrajectoryResult()

	# ---------------------------------------------------------------------------------------
	# Rearranges the point path following the name convention joint_0, ... joint_6
	# Warning: This function has side effects
	# ---------------------------------------------------------------------------------------
	def rearrange(self, joint_trajectory):

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

	# Action initialisation
	def __init__(self, name):
		self.publisher = rospy.Publisher('command_list',CommandList)
		self._action_name = name
		self._as = actionlib.SimpleActionServer(self._action_name, FollowJointTrajectoryAction, execute_cb=self.execute_cb, auto_start = False)
		self._as.start()

	# Action callback
	def execute_cb(self, goal):

		# It is required to rearrange the arrays because MoveIt doesn't guarantee orden preservation
		self.rearrange(goal.trajectory)

		# A trajectory needs at least 2 points		
		if len(goal.trajectory.points) < 2:
			return

		time_start = rospy.Time.from_sec(time.time())

		# ------------- Send command list

		trajectory = CommandList()
		trajectory.replace_previous_commands = True

		last_point = goal.trajectory.points[0]
		for point in goal.trajectory.points[1:]:

			command = Command()
			command.command_id = goal.trajectory.points.index(point) - 1
			command.command_type = 'LIN_TIMED'
			command.pose_type = 'JOINTS'
			command.pose = point.positions
			command.blending_type = 'M'
			if goal.trajectory.points.index(point) == len(goal.trajectory.points) - 1:
				command.blending = [0.0]
			else:
				command.blending = [conf_blending]
			command.additional_values = [point.time_from_start.to_sec() - last_point.time_from_start.to_sec()]

			trajectory.commands.append(command)	
		
			last_point = point

		self.publisher.publish(trajectory)

		# ------------- Wait until the termination while providing feedback

		last_point = goal.trajectory.points[0]
		for point in goal.trajectory.points[1:]:
			# Wait	
			rospy.sleep(point.time_from_start - last_point.time_from_start)
			# Trajectory abort!
			# To abort the current movement, it is possible to send an empty trajectory
			if self._as.is_preempt_requested():
				trajectory_2 = CommandList()
				trajectory_2.replace_previous_commands = True
				self.publisher.publish(trajectory_2)
				self._as.set_preempted()
				return
			# ---------------------------------------
			# Feedback
			self._feedback.joint_names = goal.trajectory.joint_names
			self._feedback.desired = point
			with lock:
				self._feedback.actual.positions = joint_states.position
				self._feedback.actual.velocities = joint_states.velocity
				self._feedback.actual.time_from_start = rospy.Time.from_sec(time.time()) - time_start
			self._as.publish_feedback(self._feedback)
			# ---------------------------------------
			last_point = point

		# ---------------------------------------
		# Result
		self._result.error_code = 0
		self._as.set_succeeded(self._result)
		# ---------------------------------------

if __name__ == '__main__':
	rospy.init_node('ur_joint_trajectory_action')
	rospy.Subscriber('joint_states', JointState, joint_states_callback)
	MoveItAction(rospy.get_name())
	rospy.spin()
