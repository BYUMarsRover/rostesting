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
        eita = pose[3]
        epsilon.append(pose[0])
        epsilon.append(pose[1])
        epsilon.append(pose[2])
        p = np.array(.5*((eita*np.eye(3,3))-self.skew(epsilon)))
        a = np.hstack((np.eye(3,3), np.zeros((3,3))))
        b = np.hstack((np.zeros((3,3)), p))
        J_t = np.vstack((a,b)).dot(Jac)
        return J_t

    def delta_x_and_e(self,pose_b, pose_a):
        pose_a_quat = tf.transformations.quaternion_from_matrix(pose_a)
        pose_b_quat = tf.transformations.quaternion_from_matrix(pose_b)

        quat_divided = tf.transformations.quaternion_multiply(pose_a_quat, tf.transformations.quaternion_inverse(pose_b_quat))

        delta_epsilon = []
        delta_epsilon.append(pose_b[0,3] - pose_a[0,3])
        delta_epsilon.append(pose_b[1,3] - pose_a[1,3])
        delta_epsilon.append(pose_b[2,3] - pose_a[2,3])
        delta_epsilon.append(quat_divided[0])
        delta_epsilon.append(quat_divided[1])
        delta_epsilon.append(quat_divided[2])

        #print delta_epsilon
        
        return delta_epsilon
            
    def solve_local_ik(self, goal_pose):

        goal = goal_pose
        deltas = self.delta_x_and_e(goal, self.FK[5](self.jangles))
        count = 0
        ik_start = time.time()

        q_next = np.matrix([])
        q_curr = self.jangles
        #print q_curr
        diff = 10

        while diff > .00001:
            a = np.hstack((.75*(np.eye(3,3)), np.zeros((3,3))))
            b = np.hstack((np.zeros((3,3)), .75*(np.eye(3,3))))
            constants = np.vstack((a,b))
            deltas = np.array(self.delta_x_and_e(goal, self.FK[5](q_curr)))
            deltas = deltas.dot(constants)
            J_t = self.analytical_jacobian(q_curr)
            delta_q = (.1)*np.dot(J_t.T, deltas)
            q_next = q_curr + delta_q
            rank = np.linalg.matrix_rank(np.linalg.pinv(J_t))
            if rank < 6:
                print "INSUFFICIENT RANK! THIS IS A SINGULARITY"
            
        
            #Establish Joint Limits
            q0_max = np.pi
            q0_min = -np.pi
            q1_max = np.pi
            q1_min = -np.pi
            q2_max = 2*np.pi
            q2_min = -2*np.pi
            q3_max = np.pi
            q3_min = -np.pi
            q4_max = np.pi
            q4_min = -np.pi
            q5_max = 2*np.pi
            q5_min = -2*np.pi

            if q_next[0]<q0_min:
                q_next[0] = q0_min
            if q_next[0] > q0_max:
                q_next[0] = q0_max
                
            if q_next[1] < q1_min:
                q_next[1] = q1_min
            elif q_next[1] > q1_max:
                q_next[1] = q1_max
            
            if q_next[2] < q2_min:
                q_next[2] = q2_min
            elif q_next[2] > q2_max:
                q_next[2] = q2_max
            
            if q_next[3] < q3_min:
                q_next[3] = q3_min
            elif q_next[3] > q3_max:
                q_next[3] = q3_max
            
            if q_next[4] < q4_min:
                q_next[4] = q4_min
            elif q_next[4] > q4_max:
                q_next[4] = q4_max
            
            if q_next[5] < q5_min:
                q_next[5] = q5_min
            elif q_next[5] > q5_max:
                q_next[5] = q5_max
            
            #print q_next
            #time.sleep(.001)

            deltas = self.delta_x_and_e(goal,self.FK[5](q_next))
            diff = self.greatest_diff(q_next, q_curr)
            #print deltas
            #print self.FK[5](q_next)[:,3]
            #print q_next
            self.publish_to_tf(q_next)
            q_curr = q_next
            count = count + 1
            if count>20000:
                break
        ik_end = time.time()
        print "Total solve time: ", ik_end - ik_start
        print "Iterations: ", count
        print "Solution Pose: \n", self.FK[5](q_next)
        #print "Error Measure: \n", np.sum(self.FK[5](q_next)-goal)
        #print '\n\n\n\ndifference in matrices\n',self.FK[5](q_next)-goal
        #print "Joint angle solution: \n", q_next
        
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
    np.set_printoptions(precision=2)

    hal = HalKinematics()
    hal.jangles = [0,np.pi/2,-np.pi/4,0,.1,np.pi]
    #hal.jangles = [0,0,0,0,0,.1*np.pi]

    #goal = np.matrix([[0,1,0,0],
    #                  [0,0,1,1.09],
    #                  [1,0,0,.16],
    #                  [0.0,0.0,0.0,1.0]])

    goal = np.matrix([[0,-1,0,.5],
                      [0,0,1,.5],
                      [-1,0,0,.5],
                      [0.0,0.0,0.0,1.0]])

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
