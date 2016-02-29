#!/usr/bin/env python
import HalKinematics as hkin
import numpy as np
import rospy
from rover_msgs.msg import Pololu
from std_msgs.msg import String
import time

class CompetitionFunctions():
    def __init__(self):
    # Variables
        self.q_fb = Pololu()
        self.q_cmd = Pololu()
        self.kin = hkin()
        
    # Publishers and Subscribers
        self.sub1 = rospy.Subscriber('pololu_feedback', Pololu, self.polCallback)
        self.pub1 = rospy.Publisher('/pololu_command', Pololu, queue_size = 10)
        self.pub2 = rospy.Publisher('/drive_command', Drive, queue_size = 10)
        self.pub3 = rospy.Publisher('/mode', String, queue_size = 10)

    # Callbacks
    def polCallback(self,msg):
        self.q_fb.q1=msg.q1
        self.q_fb.q2=msg.q2
        self.q_fb.q3=msg.q3
        self.q_fb.q4=msg.q4
        self.q_fb.q5=msg.q5
        self.q_fb.q6=msg.q6

    def valve_cw(self):
        # Close the gripper

        # Twist the wrist

        # Repeat?
        
    def valve_ccw(self):
        # Close the gripper

        # Twist the wrist

        # Repeat?

    def bins(self,bin_num):
        if bin_num == 1:
            jt_angles == [0,0,0,0,0,0] # Joint angles for bin 1
        elif bin_num == 2:
            jt_angles == [0,0,0,0,0,0] # Joint angles for bin 2
        elif bin_num == 3:
            jt_angles == [0,0,0,0,0,0] # Joint angles for bin 3
        elif bin_num == 4:
            jt_angles == [0,0,0,0,0,0] # Joint angles for bin 4
        elif bin_num == 5:
            jt_angles == [0,0,0,0,0,0] # Joint angles for bin 5

        # Command Joints 0 and 1 for 1 second to not hit the rover

        # Command The rest of the joints to the positions

        # When all of the joints have arriven, open the gripper

        # Go back to the original joint angles

    def gas_can(self):
        # This could be tricky. Below is an idea.

        # Slowly move the hand up and left in an arc while 
        # simultaneously  rotating the forearm ccw

        # Slowly move the hand up and right in an arc while 
        # simultaneously  rotating the forearm cw

if __name__=='__main__':
    while not rospy.is_shutdown():
        user_command = raw_input('Enter command')
        
        # Opening and closing Valves
        if user_command == 'open valve':
            CompetitionFunctions.valve_cw()

        elif user_command == 'close valve':
            CompetitionFunctions.valve_ccw()

        # Moving the arm above the chutes and letting go then returning
        elif user_command == '1':
            CompetitionFunctions.bins(1)

        elif user_command == '2':
            CompetitionFunctions.bins(2)

        elif user_command == '3':
            CompetitionFunctions.bins(3)

        elif user_command == '4':
            CompetitionFunctions.bins(4)

        elif user_command == '5':
            CompetitionFunctions.bins(5)

        # Pour the gas can once the nozzle is touching the rim of the container
        elif user_command == 'gas can':
            CompetitionFunctions.gas_can()

        else:
            print "Invalid command. Enter one of the following commands:"
            print "open valve"
            print "close valve"
            print "gas can"
            print '1'
            print '2'
            print '3'
            print '4'
            print '5'
            
            
