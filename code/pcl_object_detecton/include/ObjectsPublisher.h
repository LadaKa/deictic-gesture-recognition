#ifndef OBJECTSPUBLISHER_H
#define OBJECTSPUBLISHER_H

#include <ros/ros.h>
#include <geometry_msgs/Point32.h>

class ObjectsPublisher
{
private:
    ros::Publisher publisher;
    geometry_msgs::Point32 objectsCenters[3];

public:
    ObjectsPublisher();

    void SetPublisher(ros::Publisher pub);

    void SetObjectsCenters(
        geometry_msgs::Point32 center_0,
        geometry_msgs::Point32 center_1,
        geometry_msgs::Point32 center_2);

    void Publish();
};

#endif
