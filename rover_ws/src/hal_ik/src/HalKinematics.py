#!/usr/bin/env python
from hal_arm_kinematics import hal_arm_kinematics
import tf
import rospy
'''
This class inherits from and extends hal_arm_kinematics. It allows a HalKinematics 
object to contain joint angles, return joint angles and also to publish DH frames 
to tf so they can be viewed in Rviz
'''
class HalKinematics(hal_arm_kinematics):
    def __init__(self):
        self.q0 = 0
        self.q1 = 0
        self.q2 = 0
        self.q3 = 0
        self.q4 = 0
        self.q5 = 0
        self.jangles = [self.q0,self.q1,self.q2,self.q3,self.q4,self.q5]

    def get_joint_angles(self):
        jt_angles = [self.q0,self.q1,self.q2,self.q3,self.q4,self.q5]
        return jt_angles

    def publish_to_tf(self,q):
        br0 = tf.TransformBroadcaster()
        br1 = tf.TransformBroadcaster()
        br2 = tf.TransformBroadcaster()
        br3 = tf.TransformBroadcaster()
        br4 = tf.TransformBroadcaster()
        br5 = tf.TransformBroadcaster()

        Tb0 = self.FK[0](q)
        Tb1 = self.FK[1](q)
        Tb2 = self.FK[2](q)
        Tb3 = self.FK[3](q)
        Tb4 = self.FK[4](q)
        Tbee = self.FK[5](q)
        br0.sendTransform((Tb0[0:3,3]),
                          tf.transformations.quaternion_from_matrix(Tb0),
                          rospy.Time.now(),
                          'Tb0',
                          'world')

        br1.sendTransform((Tb1[0:3,3]),
                          tf.transformations.quaternion_from_matrix(Tb1),
                          rospy.Time.now(),
                          'Tb1',
                          'world')

        br2.sendTransform((Tb2[0:3,3]),
                          tf.transformations.quaternion_from_matrix(Tb2),
                          rospy.Time.now(),
                          'Tb2',
                          'world')
        
        br3.sendTransform((Tb3[0:3,3]),
                          tf.transformations.quaternion_from_matrix(Tb3),
                          rospy.Time.now(),
                          'Tb3',
                          'world')

        br4.sendTransform((Tb4[0:3,3]),
                          tf.transformations.quaternion_from_matrix(Tb4),
                          rospy.Time.now(),
                          'Tb4',
                          'world')

        br5.sendTransform((Tbee[0:3,3]),
                          tf.transformations.quaternion_from_matrix(Tbee),
                          rospy.Time.now(),
                          'Tbee',
                          'world')
