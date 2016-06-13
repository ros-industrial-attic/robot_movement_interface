#!/usr/bin/env python
# license removed for brevity
import rospy
import math
import actionlib

from ur_driver.msg import *
from sensor_msgs.msg import JointState

# publishing rate of the gripper state
publishFrequency = 50

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
    
    counter = 0
    gripperOpen = False

    # cyclic gripper data update
    while not rospy.is_shutdown():

        # publish gripper position
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = ["egl_position"]
        msg.position = [0.011] # 0.05 open | 0.011 closed
        pub.publish(msg)

        if counter == int(publishFrequency):            
            counter = 0            
            checkIOs()
            print client.get_result().state

        counter = counter + 1
        rate.sleep()

### Check the IOs in a loop. Conditions for a closed grippe are the following:
# IONr. 23: False && IONr. 24: True  -- Close the gripper
# IONr. 23: True  && IONr. 24: False -- Open the gripper
def checkIOs():
    ioNums = []

    # Queue IO 23 and IO 24
    # because we want to read only, it does not matter which state we are sending
    ioNums.append(23)
    ioNums.append(24)    

    goal = ur_driver.msg.DigIOArrayGoal(ioNr=ioNums, newState=[True] * len(ioNums), readOnly=[True] * len(ioNums))

    client.send_goal(goal)
    client.wait_for_result()

if __name__ == '__main__':
    try:
        rospy.init_node('egl90_sim_publisher', anonymous=True)
        talker()
    except rospy.ROSInterruptException:
        print "Gripper simulation state publisher interrupted."