#include "PointCloudPublishers.h"

PointCloudPublishers::PointCloudPublishers(
    ros::Publisher publisher_voxel,
    ros::Publisher publisher_nearest_object,
    ros::Publisher publisher_remaining,
    ros::Publisher publisher_objects,
    ros::Publisher publisher_marker)
{
    PointCloudPublishers::pub_voxel = publisher_voxel;
    PointCloudPublishers::pub_nearest_object = publisher_nearest_object;
    PointCloudPublishers::pub_remaining = publisher_remaining;
    PointCloudPublishers::pub_objects = publisher_objects;
    PointCloudPublishers::pub_marker = publisher_marker;
};

void PointCloudPublishers::SetClustersPublishers(
    ros::Publisher pub_cluster0,
    ros::Publisher pub_cluster1,
    ros::Publisher pub_cluster2,
    ros::Publisher pub_cluster3,
    ros::Publisher pub_cluster4)
{
    PointCloudPublishers::pub_clusters[0] = pub_cluster0;
    PointCloudPublishers::pub_clusters[1] = pub_cluster1;
    PointCloudPublishers::pub_clusters[2] = pub_cluster2;
    PointCloudPublishers::pub_clusters[3] = pub_cluster3;
    PointCloudPublishers::pub_clusters[4] = pub_cluster4;
}

void PointCloudPublishers::SetPlanesPublishers(
    ros::Publisher pub_plane0,
    ros::Publisher pub_plane1,
    ros::Publisher pub_plane2,
    ros::Publisher pub_plane3)
{
    PointCloudPublishers::pub_planes[0] = pub_plane0;
    PointCloudPublishers::pub_planes[1] = pub_plane1;
    PointCloudPublishers::pub_planes[2] = pub_plane2;
    PointCloudPublishers::pub_planes[3] = pub_plane3;
}

void PointCloudPublishers::PublishClusterMessage(
    int index,
    sensor_msgs::PointCloud2 message)
{
    PointCloudPublishers::pub_clusters[index].publish(message);
}

void PointCloudPublishers::PublishPlaneMessage(
    int index,
    sensor_msgs::PointCloud2 message)
{
    PointCloudPublishers::pub_planes[index].publish(message);
}
