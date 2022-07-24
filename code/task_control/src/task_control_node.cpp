#include <ros/ros.h>
#include "ros/console.h"
#include <std_msgs/Empty.h>

class task_control_node
{

private:
    ros::NodeHandle _nh;
    ros::Publisher pub_stop_object_detection_stream;
    ros::Subscriber sub_object_detection_done;

    void object_detection_done_cb(const std_msgs::Empty::ConstPtr& msg)
    {
        ROS_INFO(
            "PCL Objection Detection done. Terminating ROS Astra Stream.");
            
        std_msgs::Empty empty_msg;
        pub_stop_object_detection_stream.publish(empty_msg);
    }

public:
    task_control_node(
        ros::NodeHandle nh) : _nh(nh)
    {
        sub_object_detection_done = nh.subscribe(
            "pcl_object_detection/object_detection_done",
             1, 
             &task_control_node::object_detection_done_cb, this);

        pub_stop_object_detection_stream = _nh.advertise<std_msgs::Empty>(
            "stop_object_detection_stream",
             1);
    }
};

int main(int argc, char **argv)
{
    ROS_INFO("task_control_node: Initializing ROS... ");
    ros::init(argc, argv, "task_control_node");
    ros::NodeHandle nh;

    task_control_node node(nh);
    ros::spin();
    return 0;
}