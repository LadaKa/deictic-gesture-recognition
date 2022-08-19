#ifndef OBJECTSPUBLISHER_H
#define OBJECTSPUBLISHER_H

#include <ros/ros.h>
#include <geometry_msgs/Point32.h>
#include <pcl/point_types.h>
#include "DetectedObjects.h"

class ObjectsPublisher
{
private:
    ros::Publisher publisher;
    pcl_object_detection::DetectedObjects objectsMsg;

    geometry_msgs::Point32 ConvertPclPointToGeometryMsg(pcl::PointXYZ pclPoint);

public:
    ObjectsPublisher();

    void SetPublisher(ros::Publisher pub);

    void CreateObjectsMsg(
        pcl::PointXYZ center_0,
        pcl::PointXYZ center_1,
        pcl::PointXYZ center_2);

    void Publish();
};

#endif
