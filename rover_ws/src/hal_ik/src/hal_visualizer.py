#!/usr/bin/env python
from HalKinematics import HalKinematics
from rover_msgs.msg import JointAngles as j_a_msg
import rospy
import numpy as np

'''
This program simply instantiates a HalKinematics object and uses it to publish
the DH frames of Hal to tf so they can be viewed in Rviz. The joint angles are
taken from the SetJointGoal topic.
'''

def ik_callback(msg):
    q = np.array(msg.q)
    print q*np.pi/180
    hal.publish_to_tf(q*np.pi/180)
    print "published to tf"

if __name__=='__main__':
    rospy.init_node('Rviz_IK_Control')
    hal = HalKinematics()
    rospy.Subscriber('SetJointGoal',j_a_msg,ik_callback,tcp_nodelay=False)
    rospy.spin()
