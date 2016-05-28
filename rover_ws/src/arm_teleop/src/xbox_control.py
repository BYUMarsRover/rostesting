#!/usr/bin/env python

import rospy, math
from ctypes import c_ushort
from rover_msgs.msg import Pololu, Drive, All, JointAngles
from sensor_msgs.msg import Joy
from std_msgs.msg import String,Float32MultiArray,UInt16MultiArray
import time
import lib_robotis as lr
from dynamixel_publisher import DynPub
import numpy as np

class XBOX():
    def __init__(self):
    # Variables
        self.joy = Joy()
        self.cmd = All()
        self.dyn = Float32MultiArray()
        self.dyn_cmd = Float32MultiArray()
        self.invkin = UInt16MultiArray()
        self.prev_y = 0
        self.case = 'Drive'
        self.slow_case = 'Fast'
        self.cam1_sel = 0
        self.cam2_sel = 0

        self.invkin.data.append(0)#2048)#2046)
        self.invkin.data.append(90)#2048)#400)
        self.invkin.data.append(-90)#2048)#3050)#1695)
        self.invkin.data.append(0)#2048)#1920)
        self.wristangle1 = 0.0
        self.wristangle2 = 0.0

        self.cmd.lw = 1500
        self.cmd.rw = 1500
        self.cmd.pan = 1500
        self.cmd.tilt = 1500
        self.cmd.camnum = 0
        self.cmd.q1 = 1859#2048#2046
        self.cmd.q2 = 968#2048#500
        self.cmd.q3 = 2523#2048#3050#1695
        self.cmd.q4 = 1968#2048#2046
        self.cmd.q5 = 0.0
        self.cmd.q6 = 0.0
        self.cmd.grip = 1000
        self.cmd.chutes = 0
        self.cmd.shovel = 1500
        self.check=True
        
        self.dyn.data.append(.17645)#0.0)
        self.dyn.data.append(.8761169)#0.0)
        self.dyn_cmd.data.append(.17645)#0.0)
        self.dyn_cmd.data.append(.8761169)#0.0)

    # Publishers and Subscribers
        self.sub2 = rospy.Subscriber('joy', Joy, self.joyCallback)
        self.sub3 = rospy.Subscriber('dynamixel_feedback', Float32MultiArray,self.dynCallback)
        self.sub4 = rospy.Subscriber('SetJointGoal', JointAngles, self.inversekin)
        self.pub1 = rospy.Publisher('/rover_command', All, queue_size = 10)
        self.pub3 = rospy.Publisher('/mode', String, queue_size = 10)
        self.pub4 = rospy.Publisher('/dynamixel_command',Float32MultiArray,queue_size = 1)
        self.pub5 = rospy.Publisher('/debug_invkin',UInt16MultiArray, queue_size = 1)

    # Callbacks
    def inversekin(self,msg):
        print "1: "
        print self.invkin.data
        if msg.solved == 1 and self.check == True:
            self.invkin.data[0] = msg.q[0]
            self.invkin.data[1] = msg.q[1]
            self.invkin.data[2] = msg.q[2]
            self.invkin.data[3] = msg.q[3]
            self.wristangle1 = msg.q[4]
            self.wristangle2 = msg.q[5]
            print "2: "
            print self.invkin.data
        print "3: "
        print self.invkin.data


    def joyCallback(self,msg):
        self.joy=msg
        if self.joy.buttons[9] == 1:
            if self.check==False:            
                self.check=True
            else:
                self.check=False
        print "5: "
        print self.invkin.data

    def dynCallback(self,msg):
        self.dyn.data[0] = msg.data[0]
        self.dyn.data[1] = msg.data[1]
        print "6: "
        print self.invkin.data

    # Functions
    def check_method(self):
        # Check to see whether driving or using arm and return case
        # [A, B, X, Y] = buttons[0, 1, 2, 3]
        y = self.joy.buttons[3] # toggle between modes

        if y == 1:
            if self.case == 'Drive':
                self.case = 'Arm-xbox'
            elif self.case == 'Arm-xbox':
                self.case = 'Arm-IK'#'Chutes'
            elif self.case == 'Arm-IK':
                self.case = 'Chutes'
            else:
                self.case = 'Drive'
            time.sleep(.25)

    def slow_check(self):
        rb = self.joy.buttons[5]
        if rb == 1 and self.case == 'Drive':
            if self.slow_case == 'Fast':
                self.slow_case = 'Slow'
                time.sleep(.25)
            elif self.slow_case == 'Slow':
                self.slow_case = 'Fast'
                time.sleep(.25)

    def camera_select(self):
        # a selects between cameras 0-2, b selects between cameras 3-5
        # cam1_sel is lower nybble, cam2_sel is upper nybble
        a = self.joy.buttons[0]
        b = self.joy.buttons[1]

        if a == 1:
            if self.cam1_sel == 2:
                self.cam1_sel = 0
            else:
                self.cam1_sel = self.cam1_sel + 1
            time.sleep(.25)
        if b == 1:
            if self.cam2_sel == 2:
                self.cam2_sel = 0
            else:
                self.cam2_sel = self.cam2_sel + 1
            time.sleep(.25)
        # Update command
        self.cmd.camnum = (self.cam1_sel & 0x0f) | ((self.cam2_sel & 0x0f) << 4)

    def cam_pan_tilt(self):
        x = self.joy.buttons[2]
        back = self.joy.buttons[6]
        start = self.joy.buttons[7]
        push_right = self.joy.buttons[10]
        push_left = self.joy.buttons[9]

        if x == 1:
            self.cmd.pan = 1500
            self.cmd.tilt = 1500
            time.sleep(.05)
        if start == 1:
            self.cmd.tilt = self.cmd.tilt + 10.0
            time.sleep(.05)
        if back == 1:
            self.cmd.tilt = self.cmd.tilt - 10.0
            time.sleep(.05)
        if push_right == 1:
            self.cmd.pan = self.cmd.pan + 10.0
            time.sleep(.05)
        if push_left == 1:
            self.cmd.pan = self.cmd.pan - 10.0
            time.sleep(.05)
        # bounds check
        if self.cmd.tilt > 2000:
            self.cmd.tilt = 2000
        if self.cmd.tilt < 1000:
            self.cmd.tilt = 1000
        if self.cmd.pan > 2000:
            self.cmd.pan = 2000
        if self.cmd.pan < 1000:
            self.cmd.pan = 1000

    def driveCommand(self):
        # Check for slow/fast mode
        self.slow_check()
        
        # Select between camera feeds with A & B on the xbox controller
        self.camera_select()

        # Calculate drive speeds
        if self.slow_case == 'Fast':
            self.cmd.lw = self.joy.axes[1]*500 + 1500
            self.cmd.rw = self.joy.axes[4]*-500 + 1500
        elif self.slow_case == 'Slow':
            self.cmd.lw = self.joy.axes[1]*250 + 1500
            self.cmd.rw = self.joy.axes[4]*-250 + 1500

        # Pan and Tilt
        self.cam_pan_tilt()
        
        # Publish drive commands
        self.pub1.publish(self.cmd)

    def arm_IK(self):

        print "7: "
        print self.invkin.data
        self.cmd.q1=int(round(-187+(-self.invkin.data[0]*np.pi/180.0+3.0*np.pi/4.0)*(4092/(3*np.pi/2))))#0#505
        self.cmd.q2=int(round(3696+(-self.invkin.data[1]*np.pi/180)*(4092/(3*np.pi/4))))#3128
        self.cmd.q3=int(round(1500+(self.invkin.data[2]*np.pi/180+3*np.pi/4)*(4092/(np.pi))))#2027#-3420+(-msg.q[2]*np.pi/180+3*np.pi/4)*(4092/(np.pi))))
        self.cmd.q4=int(round(945+(self.invkin.data[3]*np.pi/180+15*np.pi/4)*(4092/(15*np.pi))))#882
        print "8: "
        print self.invkin.data

        #shoulder: 401; elbow: -2069

        # make sure they are valid joint angles between [0, 4095]
        # turret
        if self.cmd.q1 < 0:
            self.cmd.q1 = 0
        elif self.cmd.q1 > 4095:
            self.cmd.q1 = 4095
        # shoulder
        if self.cmd.q2 < 0:
            self.cmd.q2 = 0
        elif self.cmd.q2 > 4095:
            self.cmd.q2 = 4095
        # elbow
        if self.cmd.q3 < 0:
            self.cmd.q3 = 0
        elif self.cmd.q3 > 4095:
            self.cmd.q3 = 4095
        # forearm
        if self.cmd.q4 < 0:
            self.cmd.q4 = 0
        elif self.cmd.q4 > 4095:
            self.cmd.q4 = 4095
        # wristangle1=msg.q[4]
        # wristangle2=msg.q[5]
        if self.wristangle1>30:
            self.wristangle1=30
        elif self.wristangle1<-30:
            self.wristangle1=-30
        if self.wristangle2>30:
            self.wristangle2=30
        elif self.wristangle2<-30:
            self.wristangle2=-30
        self.dyn.data[0]=(self.wristangle1+30)/60
        self.dyn.data[1]=(self.wristangle2+30)/60

        print "9: "
        print self.invkin.data
        print self.cmd

        # now publish the positions
        #self.pub5.publish(self.invkin)
        self.pub1.publish(self.cmd)


        # Select between camera feeds with A & B on the xbox controller
        self.camera_select()
        
        # Pan and Tilt
        cam_pan_tilt(self)

        # Calculate how to command arm (position control)

        #self.cmd.q1=self.invkin.data[0]
        #self.cmd.q2=self.invkin.data[1]
        #self.cmd.q3=self.invkin.data[2]
        #self.cmd.q4=self.invkin.data[3]

        '''
        # Joint 5
        if self.joy.axes[4] > .5:
            self.cmd.q5 = self.dyn.data[0]+5.0*math.pi/180.0
            self.dyn_cmd.data[0] = self.dyn.data[0]+5.0*math.pi/180.0
            if self.cmd.q5 > 360.0*math.pi/180.0:
                self.cmd.q5 = 360.0*math.pi/180.0
                self.dyn_cmd.data[0] = 360.0*math.pi/180.0
        elif self.joy.axes[4]<-.5:
            self.cmd.q5 = self.dyn.data[0]-5.0*math.pi/180.0
            self.dyn_cmd.data[0] = self.dyn.data[0]-5.0*math.pi/180.0
            if self.cmd.q5 < 0:
                self.cmd.q5 = 0.0
                self.dyn_cmd.data[0] = 0.0

        # Joint 6
        if self.joy.axes[3] > .5:
            self.cmd.q6 = self.dyn.data[1]+5.0*math.pi/180.0
            self.dyn_cmd.data[1] = self.dyn.data[1]+5.0*math.pi/180.0
            if self.cmd.q6 > 360.0*math.pi/180.0:
                self.cmd.q6 = 360.0*math.pi/180.0
                self.dyn_cmd.data[1] = 360.0*math.pi/180.0
        elif self.joy.axes[3]<-.5:
            self.cmd.q6 = self.dyn.data[1]-5.0*math.pi/180.0
            self.dyn_cmd.data[1] = self.dyn.data[1]-5.0*math.pi/180.0
            if self.cmd.q6 < 0:
                self.cmd.q6 = 0.0
                self.dyn_cmd.data[1] = 0.0
        '''
        self.pub4.publish(self.dyn_cmd)

        # Gripper
        rb = self.joy.buttons[5]
        lb = self.joy.buttons[4]
        if rb == 1:
            self.cmd.grip = 1
        elif lb == 1:
            self.cmd.grip = 2
        else:
            self.cmd.grip = 0
            

        # Shovel
        if self.joy.axes[2] < 0:
            self.cmd.shovel = self.cmd.shovel-10.0
            if self.cmd.shovel < 1000:
                self.cmd.shovel = 1000
        elif self.joy.axes[5] > 0:
            self.cmd.shovel = self.cmd.shovel+10.0
            if self.cmd.shovel > 2000:
                self.cmd.shovel = 2000

        # Publish arm commands
        self.pub1.publish(self.cmd)
        
    def nofeedback(self):
        # Select between camera feeds with A & B on the xbox controller
        self.camera_select()

        # Pan and Tilt
        self.cam_pan_tilt()

        # Calculate how to command arm (position control)
        # Joint 1
        
        if self.joy.axes[0] < -.5:
            self.cmd.q1 = self.cmd.q1-3
            if self.cmd.q1 < 0:
                self.cmd.q1 = int(round(0))
        elif self.joy.axes[0] > .5:
            self.cmd.q1 = self.cmd.q1+3
            if self.cmd.q1 > 4092:
                self.cmd.q1 = int(round(4092))

        # Joint 2
        if self.joy.axes[1] > .5:
            self.cmd.q2 = self.cmd.q2+5.0
            if self.cmd.q2 > 4092:
                self.cmd.q2 = 4092
        elif self.joy.axes[1] < -.5:
            self.cmd.q2 = self.cmd.q2-5.0
            if self.cmd.q2 < 0:
                self.cmd.q2 = 0

        # Joint 3
        if self.joy.axes[7] < -.9:
            self.cmd.q3 = self.cmd.q3+5.0
            if self.cmd.q3 > 4092:
                self.cmd.q3 = 4092
        elif self.joy.axes[7] > .9:
            self.cmd.q3 = self.cmd.q3-5.0
            if self.cmd.q3 < 0:
                self.cmd.q3 = 0

        # Joint 4
        if self.joy.axes[6] < -.9:
            self.cmd.q4 = self.cmd.q4+5.0
            if self.cmd.q4 > 4092:
                self.cmd.q4 = 4092
        elif self.joy.axes[6] > .9:
            self.cmd.q4 = self.cmd.q4-5.0
            if self.cmd.q4 < 0:
                self.cmd.q4 = 0

        # Send moved angles to IK
        #self.invkin.data[0] = -180/np.pi*((self.cmd.q1-3905)*3*np.pi/2/4092-3*np.pi/4)
        #self.invkin.data[1] = -180/np.pi*((self.cmd.q2-3696)*3*np.pi/4/4092)
        #self.invkin.data[2] = 180/np.pi*((self.cmd.q3-1500)*np.pi/4092-3*np.pi/4)
        #self.invkin.data[3] = 180/np.pi((self.cmd.q4-945)*15*np.pi/4092-15*np.pi/4)

        # Joint 5
        if self.joy.axes[4] > .5:
            # self.cmd.q5 = self.dyn.data[0]+5.0*math.pi/180.0
            self.dyn_cmd.data[0] = self.dyn.data[0]+5.0*math.pi/180.0
            # if self.cmd.q5 > 90.0*math.pi/180.0:
            #     self.cmd.q5 = 90.0*math.pi/180.0
            if self.dyn_cmd.data[0] > math.radians(90.0):
                self.dyn_cmd.data[0] = 90.0*math.pi/180.0
        elif self.joy.axes[4]<-.5:
            # self.cmd.q5 = self.dyn.data[0]-5.0*math.pi/180.0
            self.dyn_cmd.data[0] = self.dyn.data[0]-5.0*math.pi/180.0
            # if self.cmd.q5 < math.radians(-90.0):
            #     self.cmd.q5 = math.radians(-90.0)
            if self.dyn_cmd.data[0] < math.radians(-90.0):
                self.dyn_cmd.data[0] = math.radians(-90.0)

        # Joint 6
        if self.joy.axes[3] > .5:
            # self.cmd.q6 = self.dyn.data[1]+5.0*math.pi/180.0
            self.dyn_cmd.data[1] = self.dyn.data[1]+5.0*math.pi/180.0
            # if self.cmd.q6 > 720.0*math.pi/180.0:
            #     self.cmd.q6 = 720.0*math.pi/180.0
            if self.dyn_cmd.data[1] > math.radians(720.0):
                self.dyn_cmd.data[1] = 720.0*math.pi/180.0
        elif self.joy.axes[3]<-.5:
            # self.cmd.q6 = self.dyn.data[1]-5.0*math.pi/180.0
            self.dyn_cmd.data[1] = self.dyn.data[1]-5.0*math.pi/180.0
            # if self.cmd.q6 < math.radians(-720.0):
            #     self.cmd.q6 = math.radians(-720.0)
            if self.dyn_cmd.data[1] < math.radians(-720.0):
                self.dyn_cmd.data[1] = math.radians(-720.0)

        self.pub4.publish(self.dyn_cmd)

        # Gripper
        rb = self.joy.buttons[5]
        lb = self.joy.buttons[4]
        if rb == 1:
            self.cmd.grip = 2
        elif lb == 1:
            self.cmd.grip = 1
        else:
            self.cmd.grip = 0


        # Shovel
        if self.joy.axes[2] < 0:
            self.cmd.shovel = self.cmd.shovel-10.0
            if self.cmd.shovel < 1000:
                self.cmd.shovel = 1000
        elif self.joy.axes[5] > 0:
            self.cmd.shovel = self.cmd.shovel+10.0
            if self.cmd.shovel > 2000:
                self.cmd.shovel = 2000

        # Publish arm commands
        self.pub1.publish(self.cmd)
    
    def chutes(self):
    # temp = [[],[],[],[],[],[],[],[]]
    #     temp[0] = self.joy.buttons[1] 
    #     temp[1] = self.joy.buttons[4]
    #     temp[2] = self.joy.buttons[2]
    #     temp[3] = self.joy.buttons[7]
    #     temp[4] = self.joy.buttons[5]
    #     temp[5] = self.joy.buttons[6]
    #     temp[6] = 1 # chute enable
    #     temp[7] = 0 # box lid

        # self.cmd.chutes = self.joy.buttons[1] | (self.joy.buttons[4] << 1) | (self.joy.buttons[2] << 2) | (self.joy.buttons[7] << 3) | (self.joy.buttons[5] << 4) | (self.joy.buttons[6] << 5) | (1 << 6) 

        self.cmd.chutes = self.joy.buttons[1] | (self.joy.buttons[2] << 1) | (self.joy.buttons[7] << 2) | (self.joy.buttons[6] << 3) | (self.joy.buttons[5] << 4) | (self.joy.buttons[4] << 5) | (1 << 6) 

        # temp2 = str()
        # for i in xrange(len(temp)):
        #     temp2 = temp2+str(temp[i])
        # temp2 = int(temp2,2)
        # self.cmd.chutes = temp2

        # if self.joy.buttons[5] > .5:
        #     self.cmd.shovel = self.cmd.shovel+50.0
        #     if self.cmd.shovel > 2000:
        #         self.cmd.shovel = 2000
        # elif self.joy.buttons[4] > .5:
        #     self.cmd.shovel = self.cmd.shovel-50.0
        #     if self.cmd.shovel < 1000:
        #         self.cmd.shovel = 1000

        self.pub1.publish(self.cmd)

if __name__ == '__main__':
    rospy.init_node('xbox_control', anonymous = True)
    hz = 60.0
    rate = rospy.Rate(hz)
    xbox=XBOX()

    while not rospy.is_shutdown():

        if len(xbox.joy.buttons) > 0:
            xbox.check_method()
            if xbox.case == 'Drive':
                xbox.driveCommand()
            elif xbox.case == 'Arm-xbox':
                xbox.nofeedback()
            elif xbox.case == 'Arm-IK':
                xbox.arm_IK()
            else:
                xbox.chutes()

        xbox.pub3.publish(xbox.case)

        rate.sleep()

