#ifndef POINTCLOUDPUBLISHERS_H
#define POINTCLOUDPUBLISHERS_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

class PointCloudPublishers
{
private:
    static const int objectsCount = 3;
    static const int planesCount = 4;
public:
    // PUBLISHERS

    ros::Publisher pub_clusters[objectsCount];
    ros::Publisher pub_planes[planesCount];

    ros::Publisher pub_voxel;
    ros::Publisher pub_nearest_object;

    ros::Publisher pub_remaining;
    ros::Publisher pub_objects;

    ros::Publisher pub_marker;

     PointCloudPublishers();

    void SetClustersPublishers(
        ros::Publisher pub_cluster0,
        ros::Publisher pub_cluster1,
        ros::Publisher pub_cluster2);

    // planes and other publishers - no use so far
    // camera calibration needed
    void SetPlanesPublishers(
        ros::Publisher pub_plane0,
        ros::Publisher pub_plane1,
        ros::Publisher pub_plane2,
        ros::Publisher pub_plane3);

    void SetOtherPublishers(
        ros::Publisher publisher_voxel,
        ros::Publisher publisher_nearest_object,
        ros::Publisher publisher_remaining,
        ros::Publisher publisher_objects,
        ros::Publisher publisher_marker
    );

    void PublishClusterMessage(
        int index, 
        sensor_msgs::PointCloud2Ptr message);

    void PublishPlaneMessage(
        int index, 
        sensor_msgs::PointCloud2 message);

    
};

#endif
