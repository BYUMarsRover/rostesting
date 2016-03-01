
#include <ros/ros.h>
#include "psoc.h"
#include "conn_interface.h"

using namespace psoc;
using namespace conn;

Psoc::Psoc() :
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

    link->message_received.connect(boost::bind(&Psoc::receive, this, _1, _2));
    link->port_closed.connect(boost::bind(&Psoc::terminate_cb, this));

    // subscriptions
    command_subscriber_ = nh_.subscribe("rover_command", 1, &Psoc::commandCallback_2, this);

    // publications
    data_publisher_ = nh_.advertise<rover_msgs::SciFeedback>("science_data", 1);
}

void Psoc::receive(const uint8_t *bytes, ssize_t nbytes)
{

    received = true;
    char* output = new char[nbytes];
    memcpy(output, bytes, nbytes);

    if(nbytes == 5 && true)//output[0] == something) //todo: figure out the start byte
    {
        rover_msgs::SciFeedback msg;
        msg.temp = (output[1] << 8 | output[2]);
        msg.humidity = (output[3] << 8 | output[4]);
        data_publisher_.publish(msg);
    }
}


void Psoc::send(uint16_t lw, uint16_t rw, uint16_t pan, uint16_t tilt, uint8_t camnum, uint16_t q1, uint16_t q2, uint16_t q3, uint16_t q4, uint16_t q5, uint16_t q6, uint16_t grip, uint16_t chutes)
{
  uint8_t array[25];
  array[0] = 0xEA;
  array[1] = lw&0xff;
  array[2] = (lw>>8)&0xff;
  array[3] = rw&0xff;
  array[4] = (rw>>8)&0xff;
  array[5] = pan&0xff;
  array[6] = (pan>>8)&0xff;
  array[7] = tilt&0xff;
  array[8] = (tilt>>8)&0xff;
  array[9] = camnum;
  array[10] = q1&0xff;
  array[11] = (q1>>8)&0xff;
  array[12] = q2&0xff;
  array[13] = (q2>>8)&0xff;
  array[14] = q3&0xff;
  array[15] = (q3>>8)&0xff;
  array[16] = q4&0xff;
  array[17] = (q4>>8)&0xff;
  array[18] = q5&0xff;
  array[19] = (q5>>8)&0xff;
  array[20] = q6&0xff;
  array[21] = (q6>>8)&0xff;
  array[22] = grip&0xff;
  array[23] = (grip>>8)&0xff;
  array[24] = chutes;

  link->send_bytes(array,25);
}

void Psoc::terminate_cb() 
{
}

void Psoc::commandCallback_2(const rover_msgs::All &command_msg)
{
  this->send(command_msg.lw,command_msg.rw,command_msg.pan,command_msg.tilt,command_msg.camnum,command_msg.q1,command_msg.q2,command_msg.q3,command_msg.q4,command_msg.q5,command_msg.q6,command_msg.grip,command_msg.chutes);
}
