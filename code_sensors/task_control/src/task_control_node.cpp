#include <limits>
#include <ros/ros.h>
#include "ros/console.h"
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Point32.h>

#include <iostream>
#include <fstream>

// TODO: fix 'existing target error' when added as dependence
// #include "/home/lada/cat_git/src/pcl_object_detection/include/DetectedObjects.h"
#include "//home/robot/catkin_ws/src/pcl_object_detection/include/DetectedObjects.h"


class task_control_node
{

private:
    ros::NodeHandle _nh;
    std::string _name;

    ros::Publisher pub_pointed_object_index;

    ros::Subscriber sub_object_detection_done;
    ros::Subscriber sub_detected_objects;
    ros::Subscriber sub_pointed_intersection;

    bool objectsDetected = false;
    bool pointingGestureDetected = false;
    bool pointedObjectSelected = false;
    bool targetLocationSelected = false;

    std_msgs::Int32 pointedObjectIndexMsg;

    static const int detectedObjectsCount = 3; // !!

    geometry_msgs::Point32 detectedObjectsCenters[detectedObjectsCount];

    /*void object_detection_done_cb(const std_msgs::Empty::ConstPtr &msg)
    {
        // TODO: set proper published msg and subscriber
        std_msgs::Empty empty_msg;
        pub_stop_object_detection_stream.publish(empty_msg);
    }*/

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
        // objects and both pointing gestures already detected
        if (targetLocationSelected)
            return;

        if (objectsDetected)
        {
            // first pointing gesture
            if (!pointedObjectSelected)
            {
                pointingGestureDetected = true;
                pointedObjectIndexMsg.data = GetPointedObjectIndex(msg);
                pointedObjectSelected = true;
            }
            else
            {
                targetLocationSelected = true;
                SendResultFile(pointedObjectIndexMsg.data, msg);
            }

            
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

        ROS_INFO("RESULT: \n");

        ROS_INFO(
            "Pointed object:\t\tIndex: %d.\t\tDistance: %f\n",
            minDistanceIndex,
            minDistance);

        ROS_INFO("Detected objects centers:\n");
        for (int i = 0; i < detectedObjectsCount; i++)
        {
            ROS_INFO("[%d]:\t%f \t %f \t %f \n",
                i,
                detectedObjectsCenters[i].x,
                detectedObjectsCenters[i].y,
                detectedObjectsCenters[i].z);

        };

        return minDistanceIndex;
    }

    
    void WriteResultToTempFile(
        int selectedObjectIndex,
        float target_x,
        float target_y)
    {
        std::ofstream result_file;
        // TODO: temp filename + folder as arg (config)
        result_file.open("/home/robot/Desktop/result.txt"); 

        result_file << detectedObjectsCount << "\n";
        if (!result_file.is_open())
        {
            ROS_INFO("Can't open result file.");
            return;
        }

        for (int i = 0; i < detectedObjectsCount; i++)
        {
            result_file 
                << detectedObjectsCenters[i].x
                << " "
                << detectedObjectsCenters[i].y
                << " "
                << detectedObjectsCenters[i].z
                << "\n";

        };

        result_file << selectedObjectIndex << "\n";

        result_file << target_x
                    << " "
                    << target_y
                    << "\n";

        result_file.close();

        ROS_INFO("Result was written to result file.");
    }
   

    void SendResultFile(
        int pointedObjectIndex,
        const geometry_msgs::Point32::ConstPtr &pointedFloorIntersection)
    {
        WriteResultToTempFile(
            pointedObjectIndex,
            pointedFloorIntersection->x;
            pointedFloorIntersection->y);

        ROS_INFO("Launching bash script to send results via ssh.");
        // TODO: refactor
        system("/bin/bash -c /home/robot/hello.sh");
    }

public:
    task_control_node(std::string name) : _name(name)
    {
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

        pub_pointed_object_index = _nh.advertise<std_msgs::Int32>(
            "task_control/pointed_object_index",
            1);
    }

    void runLoop()
    {
        do
        {
            if (pointedObjectSelected)
            {
                pub_pointed_object_index.publish(pointedObjectIndexMsg);

            }
            ros::spinOnce();

        } while (true); // TODO
    }
};

int main(int argc, char **argv)
{
    ROS_INFO("TASK_CONTROL: Initializing ROS... ");
    ros::init(argc, argv, "task_control_node");

    task_control_node node(ros::this_node::getName());
    node.runLoop();
    return 0;
}
