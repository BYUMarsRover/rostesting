#!/usr/bin/env python 

import rospy
from std_msgs.msg import Float32MultiArray
import lib_robotis as lr
import math

class DynPub():
    def __init__(self):
        self.r_angles = Float32MultiArray()
        self.c_angles = Float32MultiArray()
        self.c_angles.data.append(0.0)
        self.c_angles.data.append(0.0)
        self.pub = rospy.Publisher('/dynamixel_feedback',Float32MultiArray,queue_size = 1)

        self.sub = rospy.Subscriber('dynamixel_command',Float32MultiArray,self.dynCallback)

    def dynCallback(self,msg):
        self.c_angles.data[0] = msg.data[0]
        self.c_angles.data[1] = msg.data[1]

if __name__ == "__main__":
    rospy.init_node('dynamixel_feedback_node',anonymous = True)
    hz = 15
    rate = rospy.Rate(hz)
    dynpub = DynPub()

    dynpub.r_angles.data.append(0.0)
    dynpub.r_angles.data.append(0.0)

    dyn = lr.USB2Dynamixel_Device('/dev/ttyUSB0',57600)
    servo2 = lr.Robotis_Servo(dyn, 1, series = 'MX')
    servo1 = lr.Robotis_Servo(dyn, 2, series = 'MX')

    while not rospy.is_shutdown():
        dynpub.r_angles.data[0] = servo1.read_angle()#+15.0*math.pi/180.0
        dynpub.r_angles.data[1] = servo2.read_angle()
        dynpub.pub.publish(dynpub.r_angles)
        servo1.move_angle(dynpub.c_angles.data[0])
        servo2.move_angle(dynpub.c_angles.data[1])

        rate.sleep()
