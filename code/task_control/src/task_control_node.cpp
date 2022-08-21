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
    std::string _name;
    ros::Publisher pub_stop_object_detection_stream;
    ros::Subscriber sub_object_detection_done;
    ros::Subscriber sub_detected_objects;
    ros::Subscriber sub_pointed_intersection;

    bool objectsDetected = false;
    bool pointingGestureDetected = false;

    static const int detectedObjectsCount = 3; // !!

    geometry_msgs::Point32 detectedObjectsCenters[detectedObjectsCount];

    void object_detection_done_cb(const std_msgs::Empty::ConstPtr &msg)
    {
        // TODO: set proper published msg and subscriber
        std_msgs::Empty empty_msg;
        pub_stop_object_detection_stream.publish(empty_msg);
    }

    void detected_objects_cb(const pcl_object_detection::DetectedObjects &msg)
    {
        if (objectsDetected)
            return;

        for (int i = 0; i < detectedObjectsCount; i++)
        {
            detectedObjectsCenters[i] = msg.objectsCenters[i];
        };
        objectsDetected = true;
    }

    void pointed_intersection_cb(const geometry_msgs::Point32::ConstPtr &msg)
    {
        if (pointingGestureDetected)
            return;

        if (objectsDetected)
        {
            pointingGestureDetected = true;
            int pointedObjectIndex = GetPointedObjectIndex(msg);
        }
    }

    // returns index of object that is nearest to pointed floor intersection)
    int GetPointedObjectIndex(const geometry_msgs::Point32::ConstPtr &pointedFloorIntersection)
    {
        int minDistanceIndex = 0;
        float minDistance = std::numeric_limits<float>::max();

        for (int i = 0; i < detectedObjectsCount; i++)
        {
            geometry_msgs::Point32 center = detectedObjectsCenters[i];
            float x = center.x - pointedFloorIntersection->x;
            float y = center.y - pointedFloorIntersection->y;
            float distance = sqrt(pow(x, 2) + pow(y, 2));

            if (distance < minDistance)
            {
                minDistance = distance;
                minDistanceIndex = i;
            }
        }

        ROS_INFO(
            "TASK_CONTROL: Pointed object:\n\t\tIndex: %d.\n\t\tDistance: %f\n",
            minDistanceIndex,
            minDistance);

        return minDistanceIndex;
    }

public:
    task_control_node(std::string name) : _name(name)
    {
        sub_object_detection_done = _nh.subscribe(
            "pcl_object_detection/object_detection_done",
            1,
            &task_control_node::object_detection_done_cb,
            this);

        sub_detected_objects = _nh.subscribe(
            "pcl_object_detection/detected_objects",
            1,
            &task_control_node::detected_objects_cb,
            this);

        sub_pointed_intersection = _nh.subscribe(
            "body_tracker/intersection",
            1,
            &task_control_node::pointed_intersection_cb,
            this);

        pub_stop_object_detection_stream = _nh.advertise<std_msgs::Empty>(
            "task_control/stop_object_detection_stream",
            1);
    }

    void runLoop()
    {
        do
        {
            ros::spinOnce();

        } while (true); // TODO
    }
};

int main(int argc, char **argv)
{
    ROS_INFO("task_control_node: Initializing ROS... ");
    ros::init(argc, argv, "task_control_node");

    task_control_node node(ros::this_node::getName());
    node.runLoop();
    return 0;
}