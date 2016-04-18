#!/usr/bin/env python
import numpy as np
from hal_arm_kinematics import hal_arm_kinematics
import tf
import time
import sys
import rospy
#from rover_msgs.msg import Pololu


class HalKinematics(hal_arm_kinematics):
    def __init__(self):
        self.q0 = 0
        self.q1 = 0
        self.q2 = 0
        self.q3 = 0
        self.q4 = 0
        self.q5 = 0
        self.jangles = [self.q0,self.q1,self.q2,self.q3,self.q4,self.q5]
        
        #rospy.init_node('Hal_FK')

        #rospy.Subscriber('pololu_feedback', Pololu, self.joint_angles_callback)

        #rospy.Subscriber("/joint_angles",joint_angles_msg, self.joint_angles_callback, tcp_nodelay=True)

    def joint_angles_callback(self,msg):
        data = msg
        self.q0 = msg.q1
        self.q1 = msg.q2
        self.q2 = msg.q3
        self.q3 = msg.q4
        self.q4 = msg.q5
        self.q5 = msg.q6
        self.jangles = [self.q0,self.q1,self.q2,self.q3,self.q4,self.q5]

    def get_joint_angles(self):
        jt_angles = [self.q0,self.q1,self.q2,self.q3,self.q4,self.q5]
        return jt_angles

    def skew(self,d):
        return np.matrix([[0., -d[2], d[1]], [d[2], 0., -d[0]], [-d[1], d[0], 0.]])

    def greatest_diff(self,lst_a, lst_b):
        big_diff = 0
        for i in range(lst_a.__len__()):
            diff = lst_a[i] - lst_b[i]
            if np.sqrt(np.square(diff)) > np.sqrt(np.square(big_diff)):
                big_diff = np.sqrt(np.square(diff))
        return big_diff

    def analytical_jacobian(self,jangles):
        pose = self.FK[5](jangles)
        Jac = self.J[5](jangles)
        epsilon = []
        pose = tf.transformations.quaternion_from_matrix(pose)
        #print pose
        eita = pose[3]
        epsilon.append(pose[0])
        epsilon.append(pose[1])
        epsilon.append(pose[2])
        p = np.array(.5*((eita*np.eye(3,3))-self.skew(epsilon)))
        a = np.hstack((np.eye(3,3), np.zeros((3,3))))
        b = np.hstack((np.zeros((3,3)), p))
        J_t = np.vstack((a,b)).dot(Jac)
        #print Jac
        return J_t

    def quat_divide(self,pose_a, pose_b):
        #Divides pose_a by pose_b
        pose_a = tf.transformations.quaternion_from_matrix(pose_a)
        pose_b = tf.transformations.quaternion_from_matrix(pose_b)
        divided = tf.transformations.quaternion_multiply(pose_a, tf.transformations.quaternion_inverse(pose_b))
        return divided

    def delta_x_and_e(self,pose_d, pose_e,method):
        quat_divided = self.quat_divide(pose_d, pose_e)

        delta_epsilon = []
        delta_epsilon.append(pose_d[0,3] - pose_e[0,3])
        delta_epsilon.append(pose_d[1,3] - pose_e[1,3])
        delta_epsilon.append(pose_d[2,3] - pose_e[2,3])

        if method == 2:
            delta_epsilon.append(quat_divided[0])
            delta_epsilon.append(quat_divided[1])
            delta_epsilon.append(quat_divided[2])

            return delta_epsilon

        if method == 1:
        
            pose_e = tf.transformations.quaternion_from_matrix(pose_e)
            pose_d = tf.transformations.quaternion_from_matrix(pose_d)
        
            e0 = pose_e[3]*np.matrix(pose_d[0:3]).T - pose_d[3]*np.matrix(pose_e[0:3]).T - self.skew(pose_d[0:3])*np.matrix(pose_e[0:3]).T
            delta_epsilon.append(e0[0,0])
            delta_epsilon.append(e0[1,0])
            delta_epsilon.append(e0[2,0])
        
            return delta_epsilon
            
    def solve_local_ik(self, goal_pose,method):
        if method !=1:
            method=2

        goal = goal_pose
        deltas = self.delta_x_and_e(goal, self.FK[5](self.jangles),method)
        count = 0
        ik_start = time.time()
        q_next = np.matrix([])
        q_curr = [0,np.pi/2,-np.pi/4,0,.1,0]
        q_init = self.jangles
        q_curr = self.jangles
        diff = 10
        a1 = .5
        b1 = .5
        while diff > .00001:
            Jac = self.J[5](self.jangles)
            a = np.hstack((a1*(np.eye(3,3)), np.zeros((3,3))))
            b = np.hstack((np.zeros((3,3)), b1*(np.eye(3,3))))
            constants = np.vstack((a,b))
            deltas = np.array(self.delta_x_and_e(goal, self.FK[5](q_curr),method))
            deltas = deltas.dot(constants)
            J_t = self.analytical_jacobian(q_curr)
            rank = np.linalg.matrix_rank(np.linalg.pinv(Jac))
            if method == 1:
                delta_q = (.01)*np.dot(np.linalg.pinv(Jac), deltas)
            elif method == 2:
                delta_q = (.01)*np.dot(J_t.T, deltas)

            if rank < 6:
                print "INSUFFICIENT RANK! THIS IS A SINGULARITY"

            q_next = q_curr + delta_q
            
            #Establish Joint Limits
            q0_max = np.pi
            q0_min = -np.pi
            q1_max = np.pi
            q1_min = -np.pi
            q2_max = 2*np.pi
            q2_min = -2*np.pi
            q3_max = 2*np.pi
            q3_min = -2*np.pi
            q4_max = np.pi
            q4_min = -np.pi
            q5_max = 2*np.pi
            q5_min = -2*np.pi

            if q_next[0]<q0_min:
                q_next[0] = q0_min
                #q_next = [0,0,0,0,0,0]
            if q_next[0] > q0_max:
                q_next[0] = q0_max
                #q_next = [0,0,0,0,0,0]
                
            if q_next[1] < q1_min:
                q_next[1] = q1_min
                #q_next = [0,0,0,0,0,0]
            elif q_next[1] > q1_max:
                q_next[1] = q1_max
                #q_next = [0,0,0,0,0,0]
            
            if q_next[2] < q2_min:
                q_next[2] = q2_min
                #q_next = [0,0,0,0,0,0]
            elif q_next[2] > q2_max:
                q_next[2] = q2_max
                #q_next = [0,0,0,0,0,0]
            
            if q_next[3] < q3_min:
                q_next[3] = q3_min
                #q_next = [0,0,0,0,0,0]
            elif q_next[3] > q3_max:
                q_next[3] = q3_max
                #q_next = [0,0,0,0,0,0]

            if q_next[4] < q4_min:
                q_next[4] = q4_min
                #q_next = [0,0,0,0,0,0]
            elif q_next[4] > q4_max:
                q_next[4] = q4_max
                #q_next = [0,0,0,0,0,0]
            
            if q_next[5] < q5_min:
                q_next[5] = q5_min
                #q_next = [0,0,0,0,0,0]
            elif q_next[5] > q5_max:
                q_next[5] = q5_max
                #q_next = [0,0,0,0,0,0]

            diff = self.greatest_diff(q_next, q_curr)
            self.publish_to_tf(q_next)
            q_curr = q_next
            count = count + 1
            if count>10000:
                break
        ik_end = time.time()
        #print "Total solve time: ", ik_end - ik_start
        print "Iterations: ", count
        print "Error Measure: \n", np.linalg.norm((self.FK[5](q_next)-goal))
        error = np.linalg.norm((self.FK[5](q_next)-goal))
        if method == 1 and error > .5:
            q_next = q_init
            method = 2
            print "No Solution Found using method 1"
            q_next = self.solve_local_ik(goal_pose,method)
        elif method == 2 and error > .5:
            q_next = q_init
            print "No Solution Found using method 2"
        else:
            print "Solution Found"

        self.jangles = q_next
        self.publish_to_tf(q_next)
        return q_next

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

if __name__=='__main__':
    rospy.init_node('hfds')
    np.set_printoptions(precision=4)

    hal = HalKinematics()
    hal.jangles = [0,np.pi/2,-np.pi/4,0,.1,np.pi]
    #hal.jangles = [0,0,0,0,0*np.pi/2,0*np.pi]

    goal = np.matrix([[0,-1,0,0],
                      [0,0,1,1.09],
                      [-1,0,0,.16],
                      [0.0,0.0,0.0,1.0]])

    '''goal = np.matrix([[1,0,0,.25],
                      [0,1,0,.5],
                      [0,0,1,0.25],
                      [0.0,0.0,0.0,1.0]])'''

    print "Goal Pose: \n", goal


    hal.solve_local_ik(goal)

    '''
    while not rospy.is_shutdown():

        goal = np.matrix([[0.0,1.0,0.0,0],
                          [0.0,0.0,1.0,1],
                          [1.0,0.0,0.0,0],
                          [0.0,0.0,0.0,1.0]])

        x = eval(raw_input('x'))
        y = eval(raw_input('y'))
        z = eval(raw_input('z'))

        goal[0,3] = x
        goal[1,3] = y
        goal[2,3] = z
    
        hal.solve_local_ik(goal)
'''
