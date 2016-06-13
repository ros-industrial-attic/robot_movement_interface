#!/usr/bin/env python  
import roslib
import rospy
import tf

if __name__ == '__main__':
    rospy.init_node('my_tf_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        br.sendTransform((0.392622, -0.22219, 0.16657),
                         (tf.transformations.quaternion_from_euler(
        -1.50832259655, # input RX
        -0.0002,        # input RY
        0.6858,         # input RZ
        axes='sxyz')),
         rospy.Time.now(),
         "my_frame",
         "base")
        rate.sleep()