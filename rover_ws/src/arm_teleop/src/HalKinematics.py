#!/usr/bin/env python
import numpy as np
import hal_arm_kinematics as hal_arm_kinematics
from rover_msgs.msg import Pololu


class HalKinematics(hal_arm_kinematics):
    def __init__(self):
        self.q0 = 0
        self.q1 = 0
        self.q2 = 0
        self.q3 = 0
        self.q4 = 0
        self.q5 = 0
        self.jangles = [self.q0,self.q1,self.q2,self.q3,self.q4,self.q5]
        
        rospy.init_node('Hal_FK')

        rospy.Subscriber('pololu_feedback', Pololu, self.joint_angles_callback)

        rospy.Subscriber("/joint_angles",joint_angles_msg, self.joint_angles_callback, tcp_nodelay=True)

    def joint_angles_callback(self,msg):
        data = msg
        self.q0 = msg.q1
        self.q1 = msg.q2
        self.q2 = msg.q3
        self.q3 = msg.q4
        self.q4 = msg.q5
        self.q5 = msg.q6
        self.jangles = [self.q0,self.q1,self.q2,self.q3,self.q4,self.q5]

    def get_joint_angles(self):
        jt_angles = [self.q0,self.q1,self.q2,self.q3,self.q4,self.q5]
        return jt_angles
            

            
