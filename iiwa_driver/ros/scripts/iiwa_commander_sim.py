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
# This node simulates the connection between ROS and IIWA.
# The service provided accepts string commands with parameters and returns the produced answer from the robot.
# Motion is simulated by storing the given joints or frames and returning them back. No real simulation.
# Kinematics are not consistent (joints != frame).
# Forces are randomly generated.
# -----------------------------------------------------------------------------
import rospy
import random
import yaml
from iiwa_driver.srv import *
# -----------------------------------------------------------------------------
SERVICE_NAME = 'iiwa_telnet'
position = '0 0 0 0 0 0 0'
frame = '0 0 0 0 0 0'
# -----------------------------------------------------------------------------
# Generates a random value
# -----------------------------------------------------------------------------
def r():
	return str(random.uniform(-1.0, 1.0))
# -----------------------------------------------------------------------------
# ROS service - Simulates a robot
# -----------------------------------------------------------------------------
def send_command(req):

	response = StringCommandResponse()
	response.error_code = 0
	response.response = ''

	global position
	global frame

	if (req.command == 'get tool frame'): 
		response.response = frame
	if (req.command == 'get flange frame'): 
		response.response = frame
	if (req.command == 'get joint position'): 
		response.response = position
	if (req.command == 'get joint torque'):
		response.response = r() + ' ' + r() + ' ' + r() + ' ' + r() + ' ' + r() + ' ' + r() + ' ' + r()
	if (req.command == 'get cartesian force'):
		response.response = r() + ' ' + r() + ' ' + r() + ' ' + r() + ' ' + r() + ' ' + r()

	if (req.command == 'direct joint move'):
		position = ' '.join(req.parameters.split()[:7])
	if (req.command == 'smart joint move'):
		position = ' '.join(req.parameters.split()[:7])
	if (req.command == 'joint move'):
		position = ' '.join(req.parameters.split()[:7])
	if (req.command == 'lin move'):
		frame = ' '.join(req.parameters.split()[:6])
	if (req.command == 'ptp move'):
		frame = ' '.join(req.parameters.split()[:6])
	if (req.command == 'direct cartesian move'):
		frame = ' '.join(req.parameters.split()[:6])
	if (req.command == 'smart cartesian move'):
		frame = ' '.join(req.parameters.split()[:6])
	
	rospy.logwarn('Received ' + req.command + ':' + req.parameters + " -> " + response.response)
	
	return response
# -----------------------------------------------------------------------------
# Main starts service. Service name is loaded from config.yaml
# -----------------------------------------------------------------------------
if __name__ == "__main__":

	rospy.init_node('iiwa_com_sim', log_level=rospy.INFO)
	SERVICE_NAME = rospy.get_param('~commander_service_name', SERVICE_NAME)
	s = rospy.Service(SERVICE_NAME, StringCommand, send_command)
	rospy.loginfo("IIWA connection simulation is ready")
	rospy.spin()
