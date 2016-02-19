#!/usr/bin/env python

import rospy
import numpy
import serial
from rover_msgs.msg import Pololu

def read_feedback(ser):
    values = []
    count = 0
    while len(values) < 1:
        if int(ser.read(1).encode('hex'),16) == 227:
            while count < 6:
                values.append(int(ser.read(2).encode('hex'),16))
                count +=1
                
    return values

if __name__ == "__main__":
    
    ser = serial.Serial(port = '/dev/ttyUSB2',baudrate = 9600)

    rospy.init_node('pol_py_feedback',anonymous = True)
    hz = 5
    rate = rospy.Rate(hz)

    fb = Pololu()

    pub = rospy.Publisher('/pololu_feedback',Pololu,queue_size = 1)

    while not rospy.is_shutdown():
        values = read_feedback(ser)
        fb.q1 = values[0]
        fb.q2 = values[1]
        fb.q3 = values[2]
        fb.q4 = values[3]
        fb.q5 = values[4]
        fb.q6 = values[5]

        pub.publish(fb)

        rate.sleep()
        
