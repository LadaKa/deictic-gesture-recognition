#include "ros/ros.h"
#include "std_msgs/String.h"


// SUBSCRIBERS
ros::Subscriber object_detection_done;

// PUBLISHERS
// ros::Publisher 

void objectDetectionDone_Callback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("Callback [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "main_node");

 
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("pcl_object_detection/detection_done", 1000, objectDetectionDone_Callback);

 
  ros::spin();

  return 0;
}
