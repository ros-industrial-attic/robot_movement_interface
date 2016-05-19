#!/usr/bin/env python  
import roslib

import rospy
import math
import tf
import geometry_msgs.msg

from copy import deepcopy
# import our own services and messages
from dnb_tf.srv import *
from robot_movement_interface.msg import *

# the targetFrame is at the moment the 'base_link' frame because 
# that frame is used for calculations by the ROS-Drivers
targetFrame = "base"

# this will be filled on each transformation
inputFrame = ""

# --------------------------------------------------------
# This will publish a transformation send in combination with the
# command list and its used to transform the incoming commands. 
# --------------------------------------------------------
def _publishTransformation(eulerPose, transformer, timestamp):

    # the transformation
    transformation = geometry_msgs.msg.TransformStamped()
    transformation.header.frame_id = targetFrame
    transformation.header.stamp = timestamp
    transformation.child_frame_id = inputFrame
    transformation.transform.translation.x = eulerPose.x
    transformation.transform.translation.y = eulerPose.y
    transformation.transform.translation.z = eulerPose.z

    # Calculate the orientation from RPY    
    inputFrameQuaternions = tf.transformations.quaternion_from_euler(
        eulerPose.alpha,
        eulerPose.beta, 
        eulerPose.gamma, 
        axes='szyx')    

    transformation.transform.rotation.x = inputFrameQuaternions[0]
    transformation.transform.rotation.y = inputFrameQuaternions[1]
    transformation.transform.rotation.z = inputFrameQuaternions[2]
    transformation.transform.rotation.w = inputFrameQuaternions[3]

    # here the transformation frame is published 
    # the input pose is relativ to it 
    transformer.setTransform(transformation)

# --------------------------------------------------------
# Transform a complete incoming pose and send it back in the
# proper form.
# --------------------------------------------------------
def _transform(inPose, inTrafo, timestamp):

    # the transformer
    tr = tf.TransformerROS()    

    # INPUT POSE 
    inputPose = geometry_msgs.msg.PoseStamped()
    inputPose.header.frame_id = inputFrame
    inputPose.header.stamp = timestamp
    inputPose.pose.position.x = inPose[0]
    inputPose.pose.position.y = inPose[1]
    inputPose.pose.position.z = inPose[2]

    # Calculate the orientation from RPY
    inputQuaternions = tf.transformations.quaternion_from_euler(
        inPose[3], 
        inPose[4], 
        inPose[5], 
        axes='szyx')

    inputPose.pose.orientation.x = inputQuaternions[0]
    inputPose.pose.orientation.y = inputQuaternions[1]
    inputPose.pose.orientation.z = inputQuaternions[2]
    inputPose.pose.orientation.w = inputQuaternions[3]

    # create the transformation frame in tr
    # and publish it 
    _publishTransformation(inTrafo, tr, timestamp)

    # transform pose to target frame
    transformedPose = tr.transformPose(targetFrame, inputPose)

    # prepare the response
    outPose = [0] * 6
    outPose[0] = transformedPose.pose.position.x
    outPose[1] = transformedPose.pose.position.y
    outPose[2] = transformedPose.pose.position.z

    # transform quaternions back to euler for sending it back to d&b
    outputEuler = tf.transformations.euler_from_quaternion([
        transformedPose.pose.orientation.x,
        transformedPose.pose.orientation.y,
        transformedPose.pose.orientation.z,
        transformedPose.pose.orientation.w],
        axes='szyx')

    outPose[3] = outputEuler[0]
    outPose[4] = outputEuler[1]
    outPose[5] = outputEuler[2]

    # return the pose
    return outPose

# --------------------------------------------------------
# Handle incoming transformation requests.
# --------------------------------------------------------
def handle_transform(req):

    # set the input frame global for using it in the transform
    global inputFrame
    inputFrame = req.input_pose.header.frame_id
            
    # when frame_id was not filled return empty
    if not req.input_pose.header.frame_id:
        print 'No transformation. Reason: Empty input frame.'
        return TransformResponse()      

    # check if not already in base frame
    if req.input_pose.header.frame_id == 'base' or req.input_pose.header.frame_id == '/base':
        print 'No transformation. Reason: Pose already in target frame: ' + targetFrame
        return TransformResponse()
        

    # the timestamp equal on each called service request
    transformTime = rospy.Time.now()    

    # make a copy for the result
    res = TransformResponse()
    res.output_pose = deepcopy(req.input_pose)

    print 'Transforming: From frame /' + inputFrame + ' to frame /' + targetFrame + '.'

    # this is where the magic happens:
    # iterate over the commands and transform the pose    
    for index in range(0, len(req.input_pose.commands)):
        try:
            res.output_pose.commands[index].pose = _transform(req.input_pose.commands[index].pose, req.frame_pose, transformTime)
        except:
            "No transformation possible."
            return TransformResponse()

    # set the global target id for the result
    res.output_pose.header.frame_id = targetFrame

    return res

# service is setup
def transform_pose_server():
    rospy.init_node('transform_pose_server')
    s = rospy.Service('transform_pose', Transform, handle_transform)
    print "Ready to transform poses from Drag&Bot."
    rospy.spin()

# starts the server
if __name__ == '__main__':
    transform_pose_server()


