#!/usr/bin/env python

import rospy
import time
import thread
import actionlib

from robot_movement_interface.msg import *

# ------------------------------------------------------------------------
# Callback function executed after the publication of new result
# ------------------------------------------------------------------------
def result_callback(msg):
	print "---"
	print "Result Callback:"
	print msg
	actionizerInstance.last_result = msg.command_id

class Actionizer(object):
	
	_feedback = CommandsFeedback()
	_result = CommandsResult()

	# Action initialisation
	def __init__(self, name):
		self.publisher = rospy.Publisher('/iiwa_command_list', CommandList, queue_size=10)
		self._action_name = name
		self._as = actionlib.SimpleActionServer(self._action_name, CommandsAction, execute_cb=self.execute_cb, auto_start = False)
		self._as.start()

	# Action callback
	def execute_cb(self, goal): 
		print "---"
		print "Execute Callback with {} commands".format(len(goal.commands.commands))

		# setting a global variable for result callback method
		global last_result
		last_result = -1

		try:
			last_id = goal.commands.commands[-1].command_id
		except:
			print "Error trying to access to last command ID"
			self._as.set_succeeded(self._result)
			return

		self.publisher.publish(goal.commands)

		# set the timeout; +8 second timeout for each command
		timeout = time.time() + 8 * len(goal.commands.commands)
		last_goal_reached = False

		while True:

			if last_goal_reached:
				self._as.set_succeeded(self._result)
				return

			# check if timeout was reached for this loop
			if time.time() > timeout:
				print "Timeout reached (preempted)."
				self._as.set_preempted()
				return

			if self._as.is_preempt_requested():
				print "Command list aborted (preempted)."
				self._as.set_preempted()
				return

			rospy.sleep(0.05)

			try:
				if self.last_result == last_id:
					last_goal_reached = True
					print "---"
					print "Last trajectory command reached."
					self._result.command_id = last_id

			except:
				if len(goal.commands.commands) == 1:
					self._result.command_id = last_id
					print "Exception"

			

if __name__ == '__main__':
	rospy.init_node('commands_action_server')
	rospy.Subscriber('/iiwa_command_result', Result, result_callback)
	actionizerInstance = Actionizer(rospy.get_name())
	print "Action server started with name '/commands_action_server'"
	rospy.spin()
