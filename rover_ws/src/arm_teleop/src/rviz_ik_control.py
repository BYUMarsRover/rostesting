#!/usr/bin/env python
from HalKinematics2 import HalKinematics
from geometry_msgs.msg import Pose
from visualization_msgs.msg import InteractiveMarkerFeedback
import rospy
import numpy as np
import tf

nom_rot = np.matrix([[0,1,0,0],
                     [0,0,1,0],
                     [1,0,0,0],
                     [0,0,0,1]])


def rviz_callback(msg):
    if msg.menu_entry_id == 1:
        quat = [msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w]
        pos = np.matrix([msg.pose.position.x,msg.pose.position.y,msg.pose.position.z])
        goal = np.matrix(tf.transformations.quaternion_matrix(quat))*nom_rot
        goal[0:3,3:4] = pos.T
        print "rviz goal pose", goal
        hal.solve_local_ik(goal,1)

    elif msg.menu_entry_id == 2:
        #Open the gripper
        pass

    elif msg.menu_entry_id == 3:
        #Close the gripper
        pass
        

if __name__=='__main__':
    rospy.init_node('Rviz_IK_Control')
    hal = HalKinematics()
    rospy.Subscriber('hal_teleop/feedback',InteractiveMarkerFeedback,rviz_callback,tcp_nodelay=False)
    np.set_printoptions(precision=2)
    rospy.spin()
