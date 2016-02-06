/*!
 * \file psoc_node.cpp
 * \author Gary Ellingson
 *
 * \brief This file provides the entry point for the psoc node
 */

#include <ros/ros.h>
#include "pololu.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pololu_node");
  ros::NodeHandle nh;

  pololu::Pololu pololu;

  ros::spin();

  return 0;
}
