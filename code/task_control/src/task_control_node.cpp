#include <limits>
#include <ros/ros.h>
#include "ros/console.h"
#include <std_msgs/Empty.h>
#include <geometry_msgs/Point32.h>

// TODO: fix 'existing target error' when added as dependence
#include "/home/lada/cat_git/src/pcl_object_detecton/include/DetectedObjects.h"

class task_control_node
{

private:
    ros::NodeHandle _nh;
    ros::Publisher pub_stop_object_detection_stream;
    ros::Subscriber sub_object_detection_done;
    ros::Subscriber sub_detected_objects;

    bool pointingGestureDetected = false;

    void object_detection_done_cb(const std_msgs::Empty::ConstPtr& msg)
    {
        ROS_INFO(
            "PCL Objection Detection done. Terminating ROS Astra Stream.");
        
        std_msgs::Empty empty_msg;
        pub_stop_object_detection_stream.publish(empty_msg);
    }

    void detected_objects_cb(const pcl_object_detection::DetectedObjects::ConstPtr& msg)
    {
        if (pointingGestureDetected)
        {

        }
    }

    // returns index of object that is nearest to pointed floor intersection)
    int GetPointedObjectIndex(
        geometry_msgs::Point32 pointedFloorIntersection,
        geometry_msgs::Point32 objectsCenters[])
    { 
        int minDistanceIndex = 0;
        float minDistance = std::numeric_limits<float>::max();
        int size = sizeof(objectsCenters)/sizeof(geometry_msgs::Point32);
        ROS_INFO("%d", size);
        for (int i = 0; i < size; i++)
        {
            geometry_msgs::Point32 center = objectsCenters[i];
            float x = center.x - pointedFloorIntersection.x;
            float y = center.y - pointedFloorIntersection.y;
            float distance = sqrt(pow(x, 2) + pow(y, 2));

            if (distance < minDistance)
            {
                minDistance = distance;
                minDistanceIndex = i;
            }
        }

        ROS_INFO(
            "Pointed object:\n\tIndex: %d.\n\tDistance: %f\n",
            minDistanceIndex,
            minDistance);

        return minDistanceIndex;
    }

public:
    task_control_node(
        ros::NodeHandle nh) : _nh(nh)
    {
        sub_object_detection_done = nh.subscribe(
            "pcl_object_detection/object_detection_done",
             1, 
             &task_control_node::object_detection_done_cb, this);

        sub_detected_objects = nh.subscribe(
            "pcl_object_detection/detected_objects",
            1,
            &task_control_node::detected_objects_cb, this);

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