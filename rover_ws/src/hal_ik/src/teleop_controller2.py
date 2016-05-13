
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
from sensor_msgs.msg import Joy #Edit by FF
import time #edited by FF

from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from tf.broadcaster import TransformBroadcaster
from tf.transformations import quaternion_from_euler
from std_msgs.msg import String

from random import random
from math import sin, exp

update=False
server = None
menu_handler = MenuHandler()
br = None
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
    global counter, br, X, Y, Z, px, py,pz, RotateX, RotateY, RotateZ, rx,ry,rz, marker, int_marker, update, Scale
    time2 = rospy.Time.now()
    br.sendTransform( (0, 0, sin(counter/140.0)*2.0), (0, 0, 0, 1.0), time2, "base_link", "moving_frame" )
    #counter += 1
    sub2 = rospy.Subscriber('joy', Joy, joyCallback)
    sub3 = rospy.Subscriber('mode', String, ModeCallback)
    #print Z
    #print Scale
    if update==True and (abs(X) > 0.5 or abs(Y) > 0.5 or abs(Z) > 0.5 or abs(RotateX) > 0.5 or abs(RotateY) > 0.5 or abs(RotateZ) > 0.5):
        #interaction_mode=InteractiveMarkerControl.MOVE_3D
        #int_marker = InteractiveMarker()
        #int_marker.header.frame_id = "world"
        '''
        if X>.9:
            X=1.0
        elif X<-.9:
            X=-1.0
        else:
            X=0.0
    
        if Y>.9:
            Y=1.0
        elif Y<-.9:
            Y=-1.0
        else:
            Y=0.0
    
        if Z>.9:
            Z=1.0
        elif Z<-.9:
            Z=-1.0
        else:
            Z=0.0
        '''
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
        #print X,Y,Z
        px=px+X*.01
        py=py+Y*.01
        pz=pz+Z*.01
        rx=rx+RotateX*3.1415/180*2
        ry=ry+RotateY*3.1415/180*2
        rz=RotateZ*3.1415/180*2+rz
        #print ry
        position = Point( px, py, pz)
        int_marker.pose.position = position
        #int_marker.scale = .25
        quaternion= quaternion_from_euler(rx,ry,rz)#tf.transformations.quaternion_from_euler(rx,ry,rz)
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
    global update
    if msg.data =='Arm-IK':
        update=True
    if msg.data =='Arm-xbox':
        update=False


def processFeedback( feedback ):
    '''
    s = "Feedback from marker '" + feedback.marker_name
    s += "' / control '" + feedback.control_name + "'"

    mp = ""
    if feedback.mouse_point_valid:
        mp = " at " + str(feedback.mouse_point.x)
        mp += ", " + str(feedback.mouse_point.y)
        mp += ", " + str(feedback.mouse_point.z)
        mp += " in frame " + feedback.header.frame_id

    if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
        rospy.loginfo( s + ": button click" + mp + "." )
    elif feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
        rospy.loginfo( s + ": menu item " + str(feedback.menu_entry_id) + " clicked" + mp + "." )
    elif feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
        rospy.loginfo( s + ": pose changed")
    elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
        rospy.loginfo( s + ": mouse down" + mp + "." )
    elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
        rospy.loginfo( s + ": mouse up" + mp + "." )
    '''
    server.applyChanges()
    


def makeBox( msg ):
    global rx,ry,rz, marker
    #marker = Marker()
    #marker.type = Marker.CUBE
    marker.scale.x = msg.scale * .125
    marker.scale.y = msg.scale * .25
    marker.scale.z = msg.scale * 0.375
    #marker.color.r = 0.5
    #marker.color.g = 0.5
    #marker.color.b = 0.5
    #marker.color.a = 1.0
    #marker.pose.orientation.x = rx
    #marker.pose.orientation.y = ry
    #marker.pose.orientation.z = rz

    return marker
    


def makeBoxControl( msg ):
    global marker
    control =  InteractiveMarkerControl()
    control.always_visible = True
    #print len(control.markers)
    control.markers.append( makeBox(msg) )
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
    #print msg
    #print "in call back"
    '''
    if X>.9:
        X=1.0
    elif X<-.9:
        X=-1.0
    else:
        X=0.0

    if Y>.9:
        Y=1.0
    elif Y<-.9:
        Y=-1.0
    else:
        Y=0.0

    if Z>.9:
        Z=1.0
    elif Z<-.9:
        Z=-1.0
    else:
        Z=0.0

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
    if scale<-.9:
        Z=Z*.05
        Y=Y*.05
        X=X*.05
        RotateZ=RotateZ*.5
    

    if X < .1 and X > -.1:
        X = 0
    if Y < .1 and Y > -.1:
        Y = 0
    if Z < .1 and Z > -.1:
        Z = 0
    '''


def makeMarker( fixed, interaction_mode, position, show_6dof = False):
    global X,Y    
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "world"
    int_marker.pose.position = position
    int_marker.scale = .25

    int_marker.name = "hal_ee" 
    
    # insert a box
    #makeBoxControl(int_marker)
    '''
    int_marker.controls[0].interaction_mode = interaction_mode
    
    if interaction_mode != InteractiveMarkerControl.NONE:
        control_modes_dict = { 
                          InteractiveMarkerControl.MOVE_3D : "MOVE_3D",
                          InteractiveMarkerControl.ROTATE_3D : "ROTATE_3D",
                          InteractiveMarkerControl.MOVE_ROTATE_3D : "MOVE_ROTATE_3D" }
    '''
    print "does this repeat?"
    if show_6dof: 
        '''
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "rotate_x"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "move_x"

        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "rotate_z"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "move_z"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "rotate_y"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)
        '''
        control = InteractiveMarkerControl()
        print control
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)
        print int_marker
    server.insert(int_marker, processFeedback)
    menu_handler.apply( server, int_marker.name )
    
if __name__=="__main__":
    rospy.init_node("hal_teleop",anonymous=True)
    br = TransformBroadcaster()
    
    # create a timer to update the published transforms
    rospy.Timer(rospy.Duration(0.01), frameCallback)

    server = InteractiveMarkerServer("hal_teleop")

    menu_handler.insert( "Send Pose", callback=processFeedback )
    menu_handler.insert( "Open Gripper", callback=processFeedback )
    menu_handler.insert( "Close Gripper", callback=processFeedback )
        
    position = Point( 0, 1.09, 0)
    makeMarker( True, InteractiveMarkerControl.MOVE_3D, position, True )
    print "Made the marker"

    server.applyChanges()

    rospy.spin()
