#!/usr/bin/env python

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
    # Publishers and Subscribers
        self.sub1 = rospy.Subscriber('pololu_feedback', Pololu, self.polCallback)
        self.sub2 = rospy.Subscriber('joy', Joy, self.joyCallback)
        self.pub1 = rospy.Publisher('/pololu_command', Pololu, queue_size = 1)
        self.pub2 = rospy.Publisher('/drive_command', Drive, queue_size = 1)
        self.pub3 = rospy.Publisher('/mode', String, queue_size = 1)
    # Callbacks
    def polCallback(self,msg):
        self.q_fb.q1=msg.q1
        self.q_fb.q2=msg.q2

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
        # Calculate how to command arm (position control)
        # Joint 1
        if self.joy.axes[1]>.5:
            self.q_cmd.q1 = self.q_fb.q1+1.0
            if self.q_cmd.q1 > 2000:
                self.q_cmd.q1 = 2000
        elif self.joy.axes[1]<-.5:
            self.q_cmd.q1 = self.q_fb.q1-1.0
            if self.q_cmd.q1 < 1000:
                self.q_cmd.q1 = 1000
        # Joint 2
        if self.joy.axes[7]>.5:
            self.q_cmd.q2 = self.q_fb.q2+1.0
            if self.q_cmd.q2 > 4092:
                self.q_cmd.q2 = 4092
        elif self.joy.axes[7]<-.5:
            self.q_cmd.q2 = self.q_fb.q2-1.0
            if self.q_cmd.q2 < 0:
                self.q_cmd.q2 = 0
        # Publish commands
        self.pub1.publish(self.q_cmd)

    def driveCommand(self):
        # Calculate drive speeds
        self.speed.lw = self.joy.axes[1]*-500 + 1500
        self.speed.rw = self.joy.axes[4]*-500 + 1500
        # Publish drive commands
        self.pub2.publish(self.speed)

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
