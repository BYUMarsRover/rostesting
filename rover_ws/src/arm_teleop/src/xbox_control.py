#!/usr/bin/env python

import rospy
from rover_msgs.msg import Pololu, Drive, All
from sensor_msgs.msg import Joy
from std_msgs.msg import String
import time

class XBOX():
    def __init__(self):
    # Variables
        self.joy = Joy()
        # self.q_fb = Pololu()
        # self.q_cmd = Pololu()
        # self.speed = Drive()
        self.fb = All()
        self.cmd = All()
        self.prev_y = 0
        self.case = 'Drive'

        self.cmd.q1 = 2046
        self.cmd.q2 = 2046
        self.cmd.q3 = 2046
        self.cmd.q4 = 2046
        self.cmd.q5 = 0
        self.cmd.q6 = 0
        self.cmd.grip = 1000
        
        # self.q_cmd.q1 = 2046
        # self.q_cmd.q2 = 2046
        # self.q_cmd.q3 = 2046
        # self.q_cmd.q4 = 2046
        # self.q_cmd.q5 = 0
        # self.q_cmd.q6 = 0
        # self.q_cmd.grip = 1000

    # Publishers and Subscribers
        self.sub1 = rospy.Subscriber('pololu_feedback', Pololu, self.polCallback)
        self.sub2 = rospy.Subscriber('joy', Joy, self.joyCallback)
        # self.pub1 = rospy.Publisher('/pololu_command', Pololu, queue_size = 10)
        # self.pub2 = rospy.Publisher('/drive_command', Drive, queue_size = 10)
        self.pub1 = rospy.Publisher('/rover_command', All, queue_size = 10)
        self.pub3 = rospy.Publisher('/mode', String, queue_size = 10)
    # Callbacks
    def polCallback(self,msg):
        self.q_fb.q1=msg.q1
        self.q_fb.q2=msg.q2
        self.q_fb.q3=msg.q3
        self.q_fb.q4=msg.q4
        self.q_fb.q5=msg.q5
        self.q_fb.q6=msg.q6
        self.q_fb.grip=msg.grip

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
            self.case = 'Chutes'
            time.sleep(.25)
        elif y == 1 and self.case == 'Chutes':
            self.case = 'Drive'
            time.sleep(.25)

    # def polCommand(self):
    #     # Calculate how to command arm (position control)
    #     # Joint 1
    #     if self.joy.axes[0] > .5:
    #         self.q_cmd.q1 = self.q_fb.q1+50.0
    #         if self.q_cmd.q1 > 4092:
    #             self.q_cmd.q1 = 4092
    #     elif self.joy.axes[0] < -.5:
    #         self.q_cmd.q1 = self.q_fb.q1-50.0
    #         if self.q_cmd.q1 < 0:
    #             self.q_cmd.q1 = 0

    #     # Joint 2
    #     if self.joy.axes[1] > .5:
    #         self.q_cmd.q2 = self.q_fb.q2+50.0
    #         if self.q_cmd.q2 > 4092:
    #             self.q_cmd.q2 = 4092
    #     elif self.joy.axes[1]<-.5:
    #         self.q_cmd.q2 = self.q_fb.q2-50.0
    #         if self.q_cmd.q2 < 0:
    #             self.q_cmd.q2 = 0

    #     # Joint 3
    #     if self.joy.axes[7] > .5:
    #         self.q_cmd.q3 = self.q_fb.q3+50.0
    #         if self.q_cmd.q3 > 4092:
    #             self.q_cmd.q3 = 4092
    #     elif self.joy.axes[7] < -.5:
    #         self.q_cmd.q3 = self.q_fb.q3-50.0
    #         if self.q_cmd.q3 < 0:
    #             self.q_cmd.q3 = 0

    #     # Joint 4
    #     if self.joy.axes[6] > .5:
    #         self.q_cmd.q4 = self.q_fb.q4+50.0
    #         if self.q_cmd.q4 > 4092:
    #             self.q_cmd.q4 = 4092
    #     elif self.joy.axes[6]<-.5:
    #         self.q_cmd.q4 = self.q_fb.q4-50.0
    #         if self.q_cmd.q4 < 0:
    #             self.q_cmd.q4 = 0

    #     # Joint 5
    #     if self.joy.axes[4] > .5:
    #         self.q_cmd.q5 = self.q_fb.q5+50.0
    #         if self.q_cmd.q5 > 4092:
    #             self.q_cmd.q5 = 4092
    #     elif self.joy.axes[4]<-.5:
    #         self.q_cmd.q5 = self.q_fb.q5-50.0
    #         if self.q_cmd.q5 < 0:
    #             self.q_cmd.q5 = 0

    #     # Joint 6
    #     if self.joy.axes[3] > .5:
    #         self.q_cmd.q6 = self.q_fb.q6+50.0
    #         if self.q_cmd.q6 > 4092:
    #             self.q_cmd.q6 = 4092
    #     elif self.joy.axes[3]<-.5:
    #         self.q_cmd.q6 = self.q_fb.q6-50.0
    #         if self.q_cmd.q6 < 0:
    #             self.q_cmd.q6 = 0

    #     # Gripper
    #     if self.joy.buttons[5] > .5:
    #         self.q_cmd.grip = self.q_fb.grip+50.0
    #         if self.q_cmd.grip > 2000:
    #             self.q_cmd.grip = 2000
    #     elif self.joy.buttons[4] > .5:
    #         self.q_cmd.grip = self.q_fb.grip-50.0
    #         if self.q_cmd.grip < 1000:
    #             self.q_cmd.grip = 1000

    #     # Publish arm commands
    #     self.pub1.publish(self.q_cmd)

    def driveCommand(self):
        # Calculate drive speeds
        self.cmd.lw = self.joy.axes[1]*-500 + 1500
        self.cmd.rw = self.joy.axes[4]*-500 + 1500
        # Publish drive commands
        self.pub1.publish(self.cmd)

    def nofeedback(self):
        # Calculate how to command arm (position control)
        # Joint 1
        if self.joy.axes[0] > .5:
            self.cmd.q1 = self.cmd.q1+50.0
            if self.cmd.q1 > 4092:
                self.cmd.q1 = 4092
        elif self.joy.axes[0] < -.5:
            self.cmd.q1 = self.cmd.q1-50.0
            if self.cmd.q1 < 0:
                self.cmd.q1 = 0

        # Joint 2
        if self.joy.axes[1] > .5:
            self.cmd.q2 = self.cmd.q2+50.0
            if self.cmd.q2 > 4092:
                self.cmd.q2 = 4092
        elif self.joy.axes[1]<-.5:
            self.cmd.q2 = self.cmd.q2-50.0
            if self.cmd.q2 < 0:
                self.cmd.q2 = 0

        # Joint 3
        if self.joy.axes[7] > .5:
            self.cmd.q3 = self.cmd.q3+50.0
            if self.cmd.q3 > 4092:
                self.cmd.q3 = 4092
        elif self.joy.axes[7] < -.5:
            self.cmd.q3 = self.cmd.q3-50.0
            if self.cmd.q3 < 0:
                self.cmd.q3 = 0

        # Joint 4
        if self.joy.axes[6] > .5:
            self.cmd.q4 = self.cmd.q4+50.0
            if self.cmd.q4 > 4092:
                self.cmd.q4 = 4092
        elif self.joy.axes[6]<-.5:
            self.cmd.q4 = self.cmd.q4-50.0
            if self.cmd.q4 < 0:
                self.cmd.q4 = 0

        # Joint 5
        if self.joy.axes[4] > .5:
            self.cmd.q5 = self.cmd.q5+50.0
            if self.cmd.q5 > 4092:
                self.cmd.q5 = 4092
        elif self.joy.axes[4]<-.5:
            self.cmd.q5 = self.cmd.q5-50.0
            if self.cmd.q5 < 0:
                self.cmd.q5 = 0

        # Joint 6
        if self.joy.axes[3] > .5:
            self.cmd.q6 = self.cmd.q6+50.0
            if self.cmd.q6 > 4092:
                self.cmd.q6 = 4092
        elif self.joy.axes[3]<-.5:
            self.cmd.q6 = self.cmd.q6-50.0
            if self.cmd.q6 < 0:
                self.cmd.q6 = 0

        # Gripper
        if self.joy.buttons[5] > .5:
            self.cmd.grip = self.cmd.grip+50.0
            if self.cmd.grip > 2000:
                self.cmd.grip = 2000
        elif self.joy.buttons[4] > .5:
            self.cmd.grip = self.cmd.grip-50.0
            if self.cmd.grip < 1000:
                self.cmd.grip = 1000

        # Publish arm commands
        self.pub1.publish(self.cmd)
        
    def chutes(self):
        temp = [[],[],[],[],[],[],[],[]]
        temp[0] = self.joy.buttons[1] 
        temp[1] = self.joy.buttons[0]
        temp[2] = self.joy.buttons[2]
        temp[3] = self.joy.buttons[7]
        temp[4] = self.joy.buttons[5]
        temp[5] = self.joy.buttons[4]
        temp[6] = 0
        temp[7] = 0

        temp2 = str()
        for i in xrange(len(temp)):
            temp2 = temp2+str(temp[i])
        temp2 = int(temp2,2)
        self.cmd.chutes = temp2
        self.pub1.publish(self.cmd)

if __name__ == '__main__':
    rospy.init_node('xbox_control', anonymous = True)
    hz = 5
    rate = rospy.Rate(hz)
    xbox=XBOX()

    while not rospy.is_shutdown():

        if len(xbox.joy.buttons) > 0:
            xbox.check_method()
            if xbox.case == 'Drive':
                xbox.driveCommand()
            elif xbox.case == 'Arm':
                xbox.nofeedback()
            else:
                xbox.chutes()

        xbox.pub3.publish(xbox.case)

        rate.sleep()
