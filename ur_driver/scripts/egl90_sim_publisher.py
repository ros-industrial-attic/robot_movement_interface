#!/usr/bin/env python
# license removed for brevity
import rospy
import math
from sensor_msgs.msg import JointState


def talker():
    pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    rospy.init_node('egl90_sim_publisher', anonymous=True)
    rate = rospy.Rate(50)

    checkIOs()
    while not rospy.is_shutdown():

        # publish gripper position
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = ["egl_position"]
        msg.position = [0.00] # 0.05 completly open # 0.0 closed
        pub.publish(msg)
        rate.sleep()

### Check the IOs in a loop. Conditions for a closed grippe are the following:
# IONr. 23: False && IONr. 24: True  -- Close the gripper
# IONr. 23: True  && IONr. 24: False -- Open the gripper
def checkIOs():
    ioNums = []
    states = []

    # Queue IO 23 and IO 24
    # because we want to read only, it does not matter which state we are sending
    ioNums.append(23)
    states.append(True)

    ioNums.append(24)
    states.append(True)

    actionMessage = {
        'ioNr': ioNums,
        'newState': states, 
        'readOnly': [True] * len(ioNums)
    }

    print actionMessage

    data = {
        'actionID': '/digital_io_array',
        'resultType': 'ur_driver/DigIOArrayActionResult',
        'actionMessage': actionMessage
    }

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass