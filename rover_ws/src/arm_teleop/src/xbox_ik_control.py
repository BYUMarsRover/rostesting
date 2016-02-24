#!/usr/bin/env python
import HalKinematics as hkin
import numpy as np
import rospy
from rover_msgs.msg import Pololu, Drive
from sensor_msgs.msg import Joy
from std_msgs.msg import String
import time

class XBOX():
    def __init__(self):
    # Variables
        self.joy = Joy()
        self.q_fb = Pololu()
        self.q_cmd = Pololu()
        self.speed = Drive()
        self.prev_y = 0
        self.case = 'Drive'
        self.kin = hkin()
        self.delta_x = [0,0,0,0,0,0]

    # Publishers and Subscribers
        self.sub1 = rospy.Subscriber('pololu_feedback', Pololu, self.polCallback)
        self.sub2 = rospy.Subscriber('joy', Joy, self.joyCallback)
        self.pub1 = rospy.Publisher('/pololu_command', Pololu, queue_size = 10)
        self.pub2 = rospy.Publisher('/drive_command', Drive, queue_size = 10)
        self.pub3 = rospy.Publisher('/mode', String, queue_size = 10)

    # Callbacks
    def polCallback(self,msg):
        self.q_fb.q1=msg.q1*some_conversion_from_0_4092_to_rad
        self.q_fb.q2=msg.q2*some_conversion_from_0_4092_to_rad
        self.q_fb.q3=msg.q3*some_conversion_from_0_4092_to_rad
        self.q_fb.q4=msg.q4*some_conversion_from_0_4092_to_rad
        self.q_fb.q5=msg.q5*some_conversion_from_0_4092_to_rad
        self.q_fb.q6=msg.q6*some_conversion_from_0_4092_to_rad

    def joyCallback(self,msg):
        self.joy=msg

    # Functions
    def check_method(self):
        # Check to see whether driving or using arm and return case
        y = self.joy.buttons[3]
        if y == 1 and self.case == 'Drive':
            self.case = 'Arm'
            time.sleep(.25)
        elif y == 1 and self.case == 'Arm':
            self.case = 'Drive'
            time.sleep(.25)

    def polCommand(self):
        self.delta_x = [0,0,0,0,0,0]
        # Establish which direction the user wants the EE to move in cartesian space
        # Y Direction (Forward and Back)
        if self.joy.axes[0] > .5:
            self.delta_x = [0,1,0,0,0,0]
        elif self.joy.axes[0] < -.5:
            self.delta_x = [0,-1,0,0,0,0]

        # Z Direction (Up and Down)
        if self.joy.axes[1] > .5:
            self.delta_x = [0,0,1,0,0,0]
        elif self.joy.axes[1]<-.5:
            self.delta_x = [0,0,-1,0,0,0]

        # X Direction (left and right)
        if self.joy.axes[7] > .5:
            self.delta_x = [-1,0,0,0,0,0]
        elif self.joy.axes[7] < -.5:
            self.delta_x = [1,0,0,0,0,0]

# This code will be useful when we have 6 DoF
#        # Establish how to change the orientation with roll pitch yaw
#        # Roll
#        if self.joy.axes[0] > .5:#Change this
#            self.delta_x = [0,0,0,1,0,0]
#        elif self.joy.axes[0] < -.5:#Change this
#            self.delta_x = [0,0,0,-1,0,0]
#
#        # Pitch
#        if self.joy.axes[1] > .5:#Change this
#            self.delta_x = [0,0,0,0,1,0]
#        elif self.joy.axes[1]<-.5:#Change this
#            self.delta_x = [0,0,0,0,-1,0]
#
#        # Yaw
#        if self.joy.axes[7] > .5: #Change this
#            self.delta_x = [0,0,0,0,0,1]
#        elif self.joy.axes[7] < -.5: #Change this
#            self.delta_x = [0,0,0,0,0,-1]

        # Calculate the new joint angles
        q_cmd = local_ik()

        # Convert the new joint angles and prepare to publish
        # Joint 1
        self.q_cmd.q1 = q_cmd[0]*some_conversion_from_rad_to_0_4092
        if self.q_cmd.q1 > 4092:
            self.q_cmd.q1 = 4092
        if self.q_cmd.q1 < 0:
            self.q_cmd.q1 = 0

        #Joint 2
        self.q_cmd.q2 = q_cmd[1]*some_conversion_from_rad_to_0_4092
        if self.q_cmd.q2 > 4092:
            self.q_cmd.q2 = 4092
        if self.q_cmd.q2 < 0:
            self.q_cmd.q2 = 0

        # Joint 3
        self.q_cmd.q3 = q_cmd[2]*some_conversion_from_rad_to_0_4092
        if self.q_cmd.q3 > 4092:
            self.q_cmd.q3 = 4092
        if self.q_cmd.q3 < 0:
            self.q_cmd.q3 = 0

        # Joint 4
        self.q_cmd.q4 = q_cmd[3]*some_conversion_from_rad_to_0_4092
        if self.q_cmd.q4 > 4092:
            self.q_cmd.q4 = 4092
        if self.q_cmd.q4 < 0:
            self.q_cmd.q4 = 0

        # Publish arm commands
        self.pub1.publish(self.q_cmd)

    def driveCommand(self):
        # Calculate drive speeds
        self.speed.lw = self.joy.axes[1]*-500 + 1500
        self.speed.rw = self.joy.axes[4]*-500 + 1500
        # Publish drive commands
        self.pub2.publish(self.speed)

    def local_IK(self):
        #Find the change in joint angles that moves the EE in delta x direction
        J = self.kin.jacobian04(self.kin.jangles)#Change this to jacobian06 with 6 DoF
        J_t = np.pinv(J)
        delta_q = np.dot(J_t,self.delta_x)*.001
        q_new = self.kin.jangles + delta_q
        return q_new

if __name__ == '__main__':
    rospy.init_node('xbox_control', anonymous = True)
    hz = 100
    rate = rospy.Rate(hz)
    xbox=XBOX()

    while not rospy.is_shutdown():
        if len(xbox.joy.buttons) > 0:
            xbox.check_method()
            if xbox.case == 'Drive':
                xbox.driveCommand()
            else:
                xbox.polCommand()
                
        xbox.pub3.publish(xbox.case)

        rate.sleep()
