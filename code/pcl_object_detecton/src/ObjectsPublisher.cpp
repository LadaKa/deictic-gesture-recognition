#include "ObjectsPublisher.h"

ObjectsPublisher::ObjectsPublisher(){};

void ObjectsPublisher::SetPublisher(ros::Publisher pub)
{
    publisher = pub;
}

void ObjectsPublisher::SetObjectsCenters(
    geometry_msgs::Point32 center_0,
    geometry_msgs::Point32 center_1,
    geometry_msgs::Point32 center_2)
{
    objectsCenters[0] = center_0;
    objectsCenters[1] = center_1;
    objectsCenters[2] = center_2;
}

void ObjectsPublisher::Publish()
{

};