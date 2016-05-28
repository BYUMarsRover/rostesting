
#!/usr/bin/env python

"""
Copyright (c) 2011, Willow Garage, Inc.
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Willow Garage, Inc. nor the names of its
      contributors may be used to endorse or promote products derived from
      this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES LOSS OF USE, DATA, OR PROFITS OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
"""
#Edits by Fred Fagergren 5/3/2016
import rospy
import copy
import numpy as np
from rover_msgs.msg import All
from sensor_msgs.msg import Joy #Edit by FF
import time #edited by FF

from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from tf.broadcaster import TransformBroadcaster
from tf.transformations import quaternion_from_euler, euler_from_quaternion, euler_from_matrix
from std_msgs.msg import String

from random import random
from math import sin, exp, pi, cos

update=False
server = None
menu_handler = MenuHandler()
br = None
sub4 =None# rospy.Subscriber('rover_command', All, findCurrent)
counter = 0
X=0.0
Y=0.0
Z=0.0
RotateX=0
RotateY=0
RotateZ=0
px=0
py=0
pz=0
rx=0
ry=0
rz=0
Scale = 0
marker = Marker()
marker.type = Marker.CUBE
marker.color.r = 0.5
marker.color.g = 0.5
marker.color.b = 0.5
marker.color.a = 1.0
int_marker = InteractiveMarker()
int_marker.header.frame_id = "world"
int_marker.scale = .25
int_marker.name = "hal_ee"

def frameCallback( msg ):
    global counter, br, X, Y, Z, px, py,pz, RotateX, RotateY, RotateZ, rx,ry,rz, marker, int_marker, update, Scale, sub4
    time2 = rospy.Time.now()
    br.sendTransform( (0, 0, sin(counter/140.0)*2.0), (0, 0, 0, 1.0), time2, "base_link", "moving_frame" )
    sub2 = rospy.Subscriber('joy', Joy, joyCallback)
    sub3 = rospy.Subscriber('mode', String, ModeCallback)
    if update==False:
        sub4 = rospy.Subscriber('rover_command', All, findCurrent, queue_size=1)
        position = Point( px, py, pz)
        int_marker.pose.position = position
        quaternion= quaternion_from_euler(rx,ry,rz)#tf.transformations.quaternion_from_euler(rx,ry,rz)
        #print quaternion
        #print pz
        int_marker.pose.orientation.x = quaternion[0]
        int_marker.pose.orientation.y = quaternion[1]
        int_marker.pose.orientation.z = quaternion[2]
        int_marker.pose.orientation.w = quaternion[3]
        #int_marker.name = "hal_ee" 
        makeBoxControl(int_marker)

        server.insert(int_marker, processFeedback)
        menu_handler.apply( server, int_marker.name )
    #sub4 = rospy.Subscriber('rover_command', All, findCurrent)
    if (abs(X) > 0.5 or abs(Y) > 0.5 or abs(Z) > 0.5 or abs(RotateX) > 0.5 or abs(RotateY) > 0.5 or abs(RotateZ) > 0.5) and update==True:

        if RotateZ>.9:
            RotateZ=1.0
        elif RotateZ<-.9:
            RotateZ=-1.0
        else:
            RotateZ=0.0

        if RotateX>.9:
            RotateX=1.0
        elif RotateX<-.9:
            RotateX=-1.0
        else:
            RotateX=0.0

        if RotateY>.9:
            RotateY=1.0
        elif RotateY<-.9:
            RotateY=-1.0
        else:
            RotateY=0.0
        if X < .1 and X > -.1:
            X = 0
        if Y < .1 and Y > -.1:
            Y = 0
        if Z < .1 and Z > -.1:
            Z = 0       
        if Scale==1:
            Z=Z*.25
            Y=Y*.25
            X=X*.25
            RotateZ=RotateZ*.5
        px=px+X*.01
        py=py+Y*.01
        pz=pz+Z*.01
        rx=rx+RotateX*3.1415/180*2
        ry=ry+RotateY*3.1415/180*2
        rz=RotateZ*3.1415/180*2+rz
        position = Point( px, py, pz)
        int_marker.pose.position = position
        quaternion= quaternion_from_euler(rx,ry,rz)#tf.transformations.quaternion_from_euler(rx,ry,rz)
        #print quaternion
        #print pz
        int_marker.pose.orientation.x = quaternion[0]
        int_marker.pose.orientation.y = quaternion[1]
        int_marker.pose.orientation.z = quaternion[2]
        int_marker.pose.orientation.w = quaternion[3]
        #int_marker.name = "hal_ee" 
        makeBoxControl(int_marker)

        server.insert(int_marker, processFeedback)
        menu_handler.apply( server, int_marker.name )
        time.sleep(.05)
    
    #print "frameCallback"
def ModeCallback( msg ):
    global update, sub4
    if msg.data =='Arm-IK':
        update=True
        #sub4.unregister()
    if msg.data =='Arm-xbox':
        update=False
        #sub4 = rospy.Subscriber('rover_command', All, findCurrent)

def processFeedback( feedback ):
    server.applyChanges()

def findCurrent( msg ):
        #Calculate current position
        #Rotational Matrix 1
    global update, px, py, pz, rx, ry, rz, sub4
    sub4.unregister()
    if update==False:
        T1 = pi/2-((msg.q1+187)*3.0*np.pi/2/4092-3*np.pi/4)#*180/np.pi
        T2 = -((msg.q2-3696)*3.0*np.pi/4/4092)#*180/np.pi
        T3 = pi/2+((msg.q3-1500)*np.pi/4092-3*np.pi/4)#*180/np.pi
        T4 = ((msg.q4-945)*15*np.pi/4092-15*np.pi/4)#*180/np.pi
        #print "T1"
        #print T1, T2, T3, T4
        #print pz
        #T1=msg.q1
        #T2=msg.q2
        #T3=msg.q3
        #T4=msg.q4
        T5=0
        T6=0
        #print cos(T1)
        #print update
        #RM1=np.matrix([[cos(T1),0,sin(T1),0],#0*4.25*.0254*cos(T1)],
        #    [sin(T1),0,-cos(T1),0],#*4.25*.0254*sin(T1)],
        #    [0,1,0,3.5*.0254],
        #    [0,0,0,1]])
        #RM2=np.matrix([[cos(T2),-sin(T2),0,15*.0254*cos(T2)],
        #    [sin(T2),cos(T2),0,15*.0254*sin(T2)],
        #    [0,0,1,0],
        #    [0,0,0,1]])
        #RM3=np.matrix([[cos(T3),0,sin(T3),2.75*.0254*cos(T3)],
        #    [sin(T3),0,-cos(T3),2.75*.0254*sin(T3)],
        #    [0,1,0,0*.0254],
        #    [0,0,0,1]])
        RM4=np.matrix([[cos(T4),0,-sin(T4),0],
            [sin(T4),0,cos(T4),0],
            [0,-1,0,14*.0254],
            [0,0,0,1]])
        RM5=np.matrix([[cos(T5),0,sin(T5),0],
            [sin(T5),0,-cos(T5),0],
            [0,1,0,0*.0254],
            [0,0,0,1]])
        RM6=np.matrix([[cos(T6),-sin(T6),0,0],
            [sin(T6),cos(T6),0,0],
            [0,0,1,9.5*.0254],
            [0,0,0,1]])
        RM0=np.matrix([[1,0,0,0],
            [0,1,0,0],
            [0,0,1,0],
            [0,0,0,1]])
        Test1=np.matrix([[cos(T1),-sin(T1),0,4.25*.0254*cos(T1)],
            [sin(T1),cos(T1),0,4.25*.0254*sin(T1)],
            [0,0,1,3.5*.0254],
            [0,0,0,1]])
        #Test2=np.matrix([[1,0,0,15*.0254*cos(T2)],
        #                [0,cos(T2),-sin(T2),15*.0254*sin(T2)],
        #                [0,sin(T2),cos(T2),0],
        #                [0,0,0,1]])
        Test2=np.matrix([[cos(T2),0,sin(T2),15*.0254*sin(T2)],#15*.0254*cos(T2)],
                        [0,1,0,0],
                        [-sin(T2),0,cos(T2),15*.0254*cos(T2)],#15*.0254*sin(T2)],
                        [0,0,0,1]])
        Test3=np.matrix([[cos(T3),0,sin(T3),2.75*.0254*sin(T3)],#15*.0254*cos(T2)],
                        [0,1,0,0],
                        [-sin(T3),0,cos(T3),2.75*.0254*cos(T3)],#15*.0254*sin(T2)],
                        [0,0,0,1]])
        #Test1=np.matrix([1,2])*np.matrix([[1],[1]])
        Test3b=np.matrix([[1],[1],[1],[1]])
#        Final=RM0*RM1*RM2*RM3*RM4*RM5*RM6
        #Final=RM6*RM5*RM4*RM3*RM2*RM1*RM0
        Final=RM0*Test1*Test2*Test3#*Test2b
        px=Final[0,3]
        py=Final[1,3]
        pz=Final[2,3]
        #rx,ry,rz= euler_from_matrix(Final,'rxyz')
        #time.sleep(.5)
        #Final2=euler_from_quaternion(Final)
        #print Final
        
    """// Build a simulated arm using DH parameters
KDL::Chain build_arm()
{
    KDL::Chain chain;

    // DH params: a, alpha, d, theta
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(4.25*.0254, M_PI/2.0, 3.5*.0254, M_PI/2.0)));  // Turret to Shoulder
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(15.0*.0254, 0.0, 0.0, 0.0)));  // Shoulder to Elbow
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(2.75*.0254, M_PI/2.0, 0.0, M_PI/2.0)));  // Elbow to Forearm
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(0.0, -M_PI/2.0, 14*.0254, 0.0)));  // Forearm to Wrist Flop
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(0.0, M_PI/2.0, 0.0, 0.0)));  // Wrist Flop to Wrist Twist
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(0.0, 0.0, 9.5*.0254, 0.0)));  // Wrist Twist to EE

    return chain;
}

// Establish initial joint angles from which to start finding suitable joint angles
KDL::JntArray init_jnt_angles(int nJoints)
{
    KDL::JntArray q_init(nJoints);
    q_init(0) = 0;
    q_init(1) = M_PI/2.0;
    q_init(2) = -M_PI/2.0;  
    q_init(3) = 0.0;  
    q_init(4) = -M_PI/2.0;  
    q_init(5) = 0;  """

def makeBox( msg ):
    global rx,ry,rz, marker
    marker.scale.x = msg.scale * .125
    marker.scale.y = msg.scale * .25
    marker.scale.z = msg.scale * 0.375


    return marker


def makeBoxControl( msg ):
    global marker
    control =  InteractiveMarkerControl()
    control.always_visible = True
    #print len(control.markers)
    if len(control.markers)==0:    
        control.markers.append( makeBox(msg) )
    else:
        control.markers[0]=makeBox(msg)
    #print control.markers
    if len(msg.controls)==0:
        msg.controls.append( control )
    else:
        msg.controls[0]=control
    #print len(msg.controls)
    #print msg.controls[0]
    return control


#####################################################################
# Marker Creation

def joyCallback(msg):
    global X, Y, Z, RotateZ, RotateX, RotateY, Scale
    X=msg.axes[0]
    Y=-1.0*msg.axes[1]
    Z=msg.axes[7]
    Scale=msg.buttons[1]
    #print Scale
    RotateX=msg.axes[3]
    RotateY=msg.axes[4]
    RotateZ=msg.axes[6]


def makeMarker( fixed, interaction_mode, position, show_6dof = False):
    global X,Y    
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "world"
    int_marker.pose.position = position
    int_marker.scale = .25

    int_marker.name = "hal_ee" 
    
    # insert a box
    #makeBoxControl(int_marker)

    if show_6dof: 

        control = InteractiveMarkerControl()
        #print control
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        if len(int_marker.controls)==0:        
            int_marker.controls.append(control)
        else:
            int_marker.controls[0]=control
        print int_marker
    server.insert(int_marker, processFeedback)
    menu_handler.apply( server, int_marker.name )
    
if __name__=="__main__":
    rospy.init_node("hal_teleop",anonymous=True)
    br = TransformBroadcaster()

    # create a timer to update the published transforms
    rospy.Timer(rospy.Duration(0.01), frameCallback)

    server = InteractiveMarkerServer("hal_teleop")

    #menu_handler.insert( "Send Pose", callback=processFeedback )
    #menu_handler.insert( "Open Gripper", callback=processFeedback )
    #menu_handler.insert( "Close Gripper", callback=processFeedback )

    position = Point( 0, 1.09, 0)
    makeMarker( True, InteractiveMarkerControl.MOVE_3D, position, True )
    print "Made the marker"

    server.applyChanges()

    rospy.spin()

