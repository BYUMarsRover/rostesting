#pragma once

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <conn_interface.h>
#include <string>

namespace psoc {

/**
 * @brief PSOC node class
 *
 * This class implement psoc_node
 */
class Psoc
{
public:
    explicit
    Psoc();

    ~Psoc() {}

    bool received;
    std::stringstream out;
    void send(std::string command);

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Subscriber command_subscriber_;
    ros::Publisher data_publisher_;

    void commandCallback(const std_msgs::String& command_msg);

    std::string serialName_;
    int baudrate_;

    conn::ConnInterface::Ptr link;
    void receive(const uint8_t *bytes, ssize_t nbytes);


    void terminate_cb();

};

}; // namespace psoc