#!/usr/bin/env python
from HalKinematics import HalKinematics
from geometry_msgs.msg import Pose
from visualization_msgs.msg import InteractiveMarkerFeedback
from rover_msgs.msg import JointAngles as j_a_msg
import rospy
import numpy as np
import tf

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
