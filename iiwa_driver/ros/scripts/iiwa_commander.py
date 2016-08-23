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
# This node manages the connection between ROS and IIWA controller through a TCP/IP connection
# The service provided accepts string commands with parameters and returns the produced answer from the robot.
# -----------------------------------------------------------------------------
import sys
import time
import rospy
import random
import socket
import thread
from iiwa_driver.srv import *
# -----------------------------------------------------------------------------
# Default parameters, they will be overwritten by the config.yaml file
# -----------------------------------------------------------------------------
TCP_IP = "172.31.1.147"
TCP_PORT = 30000
SERVICE_NAME = 'iiwa_telnet'
# -----------------------------------------------------------------------------
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
lock = thread.allocate_lock() 	# Lock for tcp communication. Only one command can be sent at the same time
# -----------------------------------------------------------------------------
# Connect with the robot. It will try to reconnect in case of error
# -----------------------------------------------------------------------------
def connect():
	while not rospy.is_shutdown():
		try:
			rospy.logwarn('Connecting to ' + TCP_IP + ":" + str(TCP_PORT))
			sock.connect((TCP_IP, TCP_PORT))
			rospy.logwarn('Connected to ' + TCP_IP + ":" + str(TCP_PORT))
			break;
		except:	
			time.sleep(1)
# -----------------------------------------------------------------------------
# Flush line through the socket. In case of error it will try to reconnect.
# -----------------------------------------------------------------------------
def flush(line):
	while not rospy.is_shutdown():
		try:
			sock.send(line)
			return sock.recv(1024)
		except:
			connect()
# -----------------------------------------------------------------------------
# Ros service: transforms the received command into string
# -----------------------------------------------------------------------------
def send_command(req):
	with lock:	# rospy seems not to deal good with concurrent service calls therefore the need to do it manually
		response = StringCommandResponse()	# ROS StringCommandResponse service response
		response.error_code = 0				# No error by default
		
		rospy.logdebug("Sent: " + req.command + " : " + req.parameters + "\n")	# Debug	
		response.response = flush(req.command + " : " + req.parameters + "\n")	
		rospy.logdebug("Received: " + response.response)	# Debug
		
		if response.response == 'error':
			response.error_code = 1
		return response
# -----------------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------------
if __name__ == "__main__":

	# Start node
	rospy.init_node('iiwa_com')
	
	# Load variables from config.yaml
	TCP_IP = rospy.get_param('~robot_ip', TCP_IP)
	TCP_PORT = rospy.get_param('~robot_port', TCP_PORT)
	SERVICE_NAME = rospy.get_param('~commander_service_name', SERVICE_NAME)
	
	connect()

	s = rospy.Service(SERVICE_NAME, StringCommand, send_command) # Start ROS service after a succesful connection
	
	rospy.spin()
	
