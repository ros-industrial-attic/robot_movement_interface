#!/usr/bin/env python

import rospy
import time
import thread
import actionlib

from robot_movement_interface.msg import *

last_result = -1

# ------------------------------------------------------------------------
# Callback function executed after the publication of new result
# ------------------------------------------------------------------------
def result_callback(msg):
	global last_result
	last_result = msg

class Actionizer(object):
	
	_feedback = CommandsFeedback()
	_result = CommandsResult()

	# Action initialisation
	def __init__(self, name):
		self.publisher = rospy.Publisher('command_list', CommandList)
		self._action_name = name
		self._as = actionlib.SimpleActionServer(self._action_name, CommandsAction, execute_cb=self.execute_cb, auto_start = False)
		self._as.start()

	# Action callback
	def execute_cb(self, goal): 
		try:
			last_id = goal.commands.commands[-1].command_id
		except:
			print "Error trying to access to last command ID"
			self._as.set_succeeded(self._result)
			return

		self.publisher.publish(goal.commands)

		while True:
			if self._as.is_preempt_requested():
				print "Command list aborted (preempted)"
				self._as.set_preempted()
				return
			rospy.sleep(0.05)
		
			if last_result.command_id == last_id:
				self._result.last_result = last_result
				self._as.set_succeeded(self._result)
				return

if __name__ == '__main__':
	rospy.init_node('commands_action_server')
	rospy.Subscriber('/command_result', Result, result_callback)
	Actionizer(rospy.get_name())
	rospy.spin()
