#include <ros/ros.h>
#include "ros/console.h"
#include <std_msgs/Int32.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_srvs/Empty.h"

#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_ssh_commands");

  ros::NodeHandle nh;
  ROS_INFO("Command message received!");
  
  return 0;
}
