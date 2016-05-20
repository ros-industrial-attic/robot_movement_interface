#!/usr/bin/env python

import rospy
import time
import thread
import actionlib

from robot_movement_interface.msg import *
from dnb_tf.srv import *

# ------------------------------------------------------------------------
# Variables
# ------------------------------------------------------------------------
transformServiceName = '/transform_pose'

# ------------------------------------------------------------------------
# Callback function executed after the publication of new result
# ------------------------------------------------------------------------
def result_callback(msg):
	print "---"
	print "Result Callback:"
	print msg
	actionizerInstance.last_result = msg

class Actionizer(object):
	
	_feedback = CommandsFeedback()
	_result = CommandsResult()

	# Action initialisation
	def __init__(self, name):
		self.publisher = rospy.Publisher('/command_list', CommandList, queue_size=10)
		self._action_name = name
		self._as = actionlib.SimpleActionServer(self._action_name, CommandsAction, execute_cb=self.execute_cb, auto_start = False)
		
		print "Starting actionizer..."

		# wait some seconds for the transform_pose service before
		# raising exception
		while not rospy.is_shutdown():
			try:				
				rospy.wait_for_service(transformServiceName, timeout=3)
				break
			except:
				rospy.logerr("Could not start actionizer. Check if service " + transformServiceName + " is running.")				

		# create the service client when waiting for it was successful
		self._transformService = rospy.ServiceProxy(transformServiceName, Transform)
		
		self._as.start()

	# Action callback
	def execute_cb(self, goal): 

		print "---"
		print "Execute Callback with {} commands".format(len(goal.commands.commands))

		# setting a global variable for result callback method
		global last_result
		#last_result = -1

		try:
			last_id = goal.commands.commands[-1].command_id
		except:
			print "Error trying to access to last command ID"
			self._as.set_succeeded(self._result)
			return

		# transform command before publishing the commands		
		transformedCommandList = self._transformService(goal.commands, goal.commands.frame).output_pose

		# when the response is not empty, in case of a successful transformation
		# set this new transformation as a goal. or just publish on topic
		if transformedCommandList.commands:
			goal.commands = transformedCommandList		
		
		# publish the goal to topic
		self.publisher.publish(goal.commands)

		# loop until last goal was reached or 
		# cancelation of the commands was requested
		while True:

			if self._as.is_preempt_requested():
				print "Command list aborted (preempted)."
				self._as.set_preempted()
				return

			rospy.sleep(0.05)

			try:
				# compare if the last_result which was delived by the robot
				# equals the last_id in the goal array
				if self.last_result.command_id == last_id:				
					print "---"
					print "Last trajectory command reached."

					# pass the result of type CommandsResult() to the member of 
					# the actionizer class and set state as succeeded				
					self._result.result = self.last_result
					self._as.set_succeeded(self._result)					
					return
			except:
				# message to user when last_result.command_id could not be read
				rospy.logdebug("Waiting for last_result message.")

			
if __name__ == '__main__':
	rospy.init_node('commands_action_server')
	rospy.Subscriber('/command_result', Result, result_callback)
	actionizerInstance = Actionizer(rospy.get_name())	

	print "Action server started with name '/commands_action_server'"
	rospy.spin()
