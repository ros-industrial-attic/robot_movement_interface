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
# This node encapsulates the joint_path_command in a ROS action according to the ROS MoveIt standards
# -----------------------------------------------------------------------------

import rospy
import actionlib

from control_msgs.msg    import FollowJointTrajectoryGoal
from control_msgs.msg    import FollowJointTrajectoryResult
from control_msgs.msg    import FollowJointTrajectoryFeedback
from control_msgs.msg    import FollowJointTrajectoryAction
from trajectory_msgs.msg import JointTrajectory

class MoveItAction(object):
	
	_feedback = FollowJointTrajectoryFeedback()
	_result = FollowJointTrajectoryResult()

	def __init__(self, name):
		self.publisher = rospy.Publisher("joint_path_command", JointTrajectory)
		self._action_name = name
		self._as = actionlib.SimpleActionServer(self._action_name, FollowJointTrajectoryAction, execute_cb=self.execute_cb, auto_start = False)
		self._as.start()

	def execute_cb(self, goal):
		self.publisher.publish(goal.trajectory)

		try:
			rospy.sleep(goal.trajectory.points[-1].time_from_start)
			self._result.error_code = 0
		except:
			self._result.error_code = self._result.OLD_HEADER_TIMESTAMP

		self._as.set_succeeded(self._result)

if __name__ == '__main__':
	rospy.init_node('iiwa_driver_moveit')
	MoveItAction(rospy.get_name())
	rospy.spin()
