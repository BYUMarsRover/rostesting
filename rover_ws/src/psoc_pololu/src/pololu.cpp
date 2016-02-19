
#include <ros/ros.h>
#include "pololu.h"
#include "conn_interface.h"
#include <std_msgs/String.h>
#include <rover_msgs/Pololu.h>
#include <inttypes.h>
// #include <iostream>

// using namespace std;
using namespace pololu;
using namespace conn;

Pololu::Pololu() :
    nh_(ros::NodeHandle()),
    nh_private_(ros::NodeHandle("~"))
{

    nh_private_.param<std::string>("serial_port", serialName_, "/dev/ttyUSB2");
    nh_private_.param<int>("baudrate", baudrate_, 9600);

    std::string fcu_url(serialName_.c_str());

    try {
        link = ConnInterface::open_url(fcu_url, baudrate_);
        // may be overridden by URL
    }
    catch (conn::DeviceError &ex) {
        //ROS_FATAL("FCU: %s", ex.what());
        //ros::shutdown();
        return;
    }

    link->message_received.connect(boost::bind(&Pololu::receive, this, _1, _2));
    link->port_closed.connect(boost::bind(&Pololu::terminate_cb, this));

    // subscriptions
    command_subscriber_ = nh_.subscribe("pololu_command", 1, &Pololu::commandCallback_2, this);

    // publications
    data_publisher_ = nh_.advertise<rover_msgs::Pololu>("pololu_feedback", 1);
}

void Pololu::receive(const uint8_t *bytes, ssize_t nbytes)
{

    received = true;
    char* output = new char[nbytes];
    memcpy(output, bytes, nbytes);

    // rover_msgs::Pololu result;
    // result.q1 = (output[1] << 8 | output[2]);
    // result.q2 = (output[3] << 8 | output[4]);
    // result.q3 = (output[5] << 8 | output[6]);
    // result.q4 = (output[7] << 8 | output[8]);
    // result.q5 = (output[9] << 8 | output[10]);
    // result.q6 = (output[11] << 8 | output[12]);
    
    // data_publisher_.publish(result);

    // rover_msgs::Pololu result;
    // result.q1 = out;
    // data_publisher_.publish(result);
}

void Pololu::send(uint16_t q1, uint16_t q2, uint16_t q3, uint16_t q4, uint16_t q5, uint16_t q6)
{
  uint8_t array[14];
  array[0] = 0xEA;
  array[1] = 0xE3;
  array[2] = q1&0xff;
  array[3] = (q1>>8)&0xff;
  array[4] = q2&0xff;
  array[5] = (q2>>8)&0xff;
  array[6] = q3&0xff;
  array[7] = (q3>>8)&0xff;
  array[8] = q4&0xff;
  array[9] = (q4>>8)&0xff;
  array[10] = q5&0xff;
  array[11] = (q5>>8)&0xff;
  array[12] = q6&0xff;
  array[13] = (q6>>8)&0xff;

  link->send_bytes(array,14);
}

void Pololu::terminate_cb() 
{
}

void Pololu::commandCallback_2(const rover_msgs::Pololu &command_msg)
{
  this->send(command_msg.q1,command_msg.q2,command_msg.q3,command_msg.q4,command_msg.q5,command_msg.q6);
}
