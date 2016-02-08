#pragma once

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <conn_interface.h>
#include <string>
#include <rover_msgs/Pololu.h>
#include <stdint.h>

namespace pololu {

/**
 * @brief Pololu node class
 *
 * This class implement pololu_node
 */
class Pololu
{
public:
    explicit
    Pololu();

    ~Pololu() {}

    bool received;
    std::stringstream out;
    /* void send(std::string command); */
    void send(uint16_t, uint16_t, uint16_t, uint16_t);

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Subscriber command_subscriber_;
    ros::Publisher data_publisher_;

    void commandCallback(const std_msgs::String& command_msg);
    void commandCallback_2(const rover_msgs::Pololu& command_msg);

    std::string serialName_;
    int baudrate_;

    conn::ConnInterface::Ptr link;
    void receive(const uint8_t *bytes, ssize_t nbytes);


    void terminate_cb();

};

}; // namespace pololu
