#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/frames_io.hpp>
#include <stdio.h>
#include <iostream>
#include <Eigen/Core>
#include <trac_ik/trac_ik.hpp>
#include <fstream>
#include <math.h>
#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "visualization_msgs/InteractiveMarkerFeedback.h"
#include "rover_msgs/JointAngles.h"

KDL::Chain chain;
int nJoints;
KDL::JntArray q_init, q_min, q_max;
ros::Publisher pub_jnts, pub_gripper;

// Build a simulated arm using DH parameters
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
    q_init(5) = 0;  
    
    return q_init;
}

// Establish minimum joint angles
KDL::JntArray min_jnt_angles(int nJoints)
{
    KDL::JntArray q_min(nJoints); 
    q_min(0) = -135*M_PI/180.0;
    q_min(1) = -M_PI/4;
    q_min(2) = -135*M_PI/180.0;
    q_min(3) = -1035*M_PI/180.0;
    q_min(4) = -M_PI/2;
    q_min(5) = -M_PI/2;
    
    return q_min;
}

// Establish maximum joint angles
KDL::JntArray max_jnt_angles(int nJoints)
{
    KDL::JntArray q_max(nJoints); 
    q_max(0) = 135*M_PI/180.0;
    q_max(1) = 135*M_PI/180.0;
    q_max(2) = 45*M_PI/180.0;
    q_max(3) = 1035*M_PI/180.0;
    q_max(4) = M_PI/2;
    q_max(5) = M_PI/2;
    
    return q_max;
}

// Build a JointAngles message that can be published over ROS
rover_msgs::JointAngles build_ja_msg(KDL::JntArray q_ik,int result)
{
    rover_msgs::JointAngles msg;
    float q0 = q_ik(0)*180/M_PI;
    float q1 = q_ik(1)*180/M_PI;
    float q2 = q_ik(2)*180/M_PI;
    float q3 = q_ik(3)*180/M_PI;
    float q4 = q_ik(4)*180/M_PI;
    float q5 = q_ik(5)*180/M_PI;
    q_init = q_ik;
    msg.q.push_back(round(q0*100)/100);
    msg.q.push_back(round(q1*100)/100);
    msg.q.push_back(round(q2*100)/100);
    msg.q.push_back(round(q3*100)/100);
    msg.q.push_back(round(q4*100)/100);
    msg.q.push_back(round(q5*100)/100);
    msg.solved = result;

    return msg;
}

// Do IK and return a flag representing whether or not a solution was reached
int inverse_kinematics(geometry_msgs::Point coord, geometry_msgs::Quaternion orient, KDL::Chain chain, int nJoints, KDL::JntArray q_init, KDL::JntArray q_min, KDL::JntArray q_max)
{
    KDL::Frame desFrame;
    desFrame.p = KDL::Vector(coord.x, coord.y, coord.z);   // desired position
    desFrame.M = KDL::Rotation::Quaternion(orient.x, orient.y, orient.z, orient.w);  // desired orientation
  
    int result = -1;
    float acc_p = 0.0;
    float acc_o = 0.0;
    KDL::JntArray q_ik(nJoints);
    float q_vel[5] = {};
    
    KDL::Vector vel(acc_p, acc_p, acc_p);
    KDL::Vector rot(acc_o, acc_o, acc_o);
    KDL::Twist bound(vel, rot);
    TRAC_IK::TRAC_IK ik(chain, q_min, q_max);
    result = ik.CartToJnt(q_init, desFrame, q_ik, bound);
    rover_msgs::JointAngles msg = build_ja_msg(q_ik,result);
    pub_jnts.publish(msg);
}

// Update desired pose based on Interactive Marker
void callback(const visualization_msgs::InteractiveMarkerFeedback& msg)
{
    geometry_msgs::Point coord = msg.pose.position;
    geometry_msgs::Quaternion orient = msg.pose.orientation;

    if (msg.menu_entry_id == 1 || msg.menu_entry_id == 0){inverse_kinematics(coord, orient, chain, nJoints, q_init, q_min, q_max);}
    //else if (msg.menu_entry_id == 2){open_gripper(1);}
    //else if (msg.menu_entry_id == 3){open_gripper(0);}
}


int main(int argc, char **argv)
{
    chain = build_arm();
    nJoints = chain.getNrOfJoints();
    q_init = init_jnt_angles(nJoints);
    q_min = min_jnt_angles(nJoints);
    q_max = max_jnt_angles(nJoints);
  
    ros::init(argc, argv, "hal_teleop");
    ros::NodeHandle n;
    pub_jnts = n.advertise<rover_msgs::JointAngles>("SetJointGoal",1000);
    //pub_gripper = n.advertise<low_level_control::gripper_status>("SetGrippers",1000);
    ros::Subscriber sub_pose = n.subscribe("hal_teleop/feedback", 1000, callback);
    ros::spin();
}


