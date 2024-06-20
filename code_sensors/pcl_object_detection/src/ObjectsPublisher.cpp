#include "ObjectsPublisher.h"

ObjectsPublisher::ObjectsPublisher(){};

void ObjectsPublisher::SetPublisher(ros::Publisher pub)
{
    publisher = pub;
}

void ObjectsPublisher::CreateObjectsMsg(
    pcl::PointXYZ center_0,
    pcl::PointXYZ center_1,
    pcl::PointXYZ center_2)
{
    objectsMsg.objectsCenters[0] = ConvertPclPointToGeometryMsg(center_0);
    objectsMsg.objectsCenters[1] = ConvertPclPointToGeometryMsg(center_1);
    objectsMsg.objectsCenters[2] = ConvertPclPointToGeometryMsg(center_2);
}

geometry_msgs::Point32 ObjectsPublisher::ConvertPclPointToGeometryMsg(
    pcl::PointXYZ pclPoint)
{
    geometry_msgs::Point32 geomMsgPoint;
    geomMsgPoint.x = pclPoint.x;
    geomMsgPoint.y = pclPoint.y;
    geomMsgPoint.z = pclPoint.z;

    return geomMsgPoint;
}

void ObjectsPublisher::Publish()
{
    publisher.publish(objectsMsg);
};