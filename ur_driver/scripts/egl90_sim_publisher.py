#!/usr/bin/env python
# license removed for brevity
import rospy
import math
import actionlib

from ur_driver.msg import *
from sensor_msgs.msg import JointState

# publishing rate of the gripper state
publishFrequency = 30

# the positions
closeGripperLimit = 0.011
openGripperLimit = 0.05

# gripper status
gripperCommand = 'Open' # till proved otherwise

# the client
client = actionlib.SimpleActionClient('digital_io_array', DigIOArrayAction)

def talker():

    # initialize gripper state publisher
    pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    rate = rospy.Rate(publishFrequency)

    # this is the service where we are listening too for the inputs
    #client = actionlib.SimpleActionClient('digital_io_array', DigIOArrayAction)
    print "Waiting for 'DigitalIO' server."

    # wait for the action until found
    client.wait_for_server();
    print "'DigitalIO' server found."    
    
    counter = publishFrequency
    actualGripperPosition = 0.03

    # cyclic gripper data update
    while not rospy.is_shutdown():

        # only every once in a while check IOs
        if counter == int(publishFrequency):            
            counter = 0
            checkIOs()

        # Open the gripper        
        if gripperCommand == 'Open':
            actualGripperPosition = openGripper(actualGripperPosition)

        # Close the gripper
        elif gripperCommand == 'Close':
            actualGripperPosition = closeGripper(actualGripperPosition)
        
                # publish gripper position        
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.position = [actualGripperPosition]
        msg.name = ["egl_position"]
        pub.publish(msg)

        print "published pos: " + str(actualGripperPosition)
        # count up and sleep
        counter = counter + 1
        rate.sleep()

def openGripper(actualPos):
    if actualPos <= openGripperLimit:
        newPos = actualPos + 0.04 / publishFrequency
        print "newPos :" + str(newPos)
        return newPos        
    else:
        return actualPos

def closeGripper(actualPos):
    if actualPos >= closeGripperLimit:
        newPos = actualPos - 0.04 / publishFrequency
        print "newPos :" + str(newPos)
        return newPos
    else:
        return actualPos

### Check the IOs in a loop. Conditions for a closed grippe are the following:
# IONr. 23: False && IONr. 24: True  -- Close the gripper
# IONr. 23: True  && IONr. 24: False -- Open the gripper
def checkIOs():
    ioNums = []

    # Queue IO 23 and IO 24
    # because we want to read only, it does not matter which state we are sending
    ioNums.append(23)
    ioNums.append(24)

    # read all appended IOs
    goal = ur_driver.msg.DigIOArrayGoal(ioNr=ioNums, newState=[True] * len(ioNums), readOnly=[True] * len(ioNums))

    # send to action server and wait for result
    client.send_goal(goal)
    client.wait_for_result()

    # save the results
    states = client.get_result().state    

    # set gripper command to one of those
    if states[0] == False and states[1] == True:
        gripperCommand = 'Close'
    elif states[0] == True and states[1] == False:
        gripperCommand = 'Open'
    else:
        gripperCommand = 'None'   

    global gripperCommand

if __name__ == '__main__':
    try:
        rospy.init_node('egl90_sim_publisher', anonymous=True)
        talker()
    except rospy.ROSInterruptException:
        print "Gripper simulation state publisher interrupted."