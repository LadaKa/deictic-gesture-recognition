#include "geometry_msgs/PoseStamped.h"
#include <visualization_msgs/Marker.h>
#include "Skeleton.h"

#include "RVizPointMarker.h"

void RVizPointMarker::SetMarkerProperties(
    geometry_msgs::Point32_<pointing_gesture::Skeleton> position,
    float r, float g, float b,
    uint32_t shape)
{
    marker.type = shape;

    marker.action = visualization_msgs::Marker::ADD;

    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1.0;

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.07; // size of marker in meters
    marker.scale.y = 0.07;
    marker.scale.z = 0.07;

    marker.pose.position.x = position.x;
    marker.pose.position.y = position.y;
    marker.pose.position.z = position.z;
}

RVizPointMarker::RVizPointMarker(
    int id,
    geometry_msgs::Point32_<pointing_gesture::Skeleton> position,
    float r, float g, float b,
    uint32_t shape)
{
    marker.ns = "astra_body_tracker";
    marker.id = id;

    marker.header.frame_id = "camera_link"; // "base_link";
    marker.header.stamp = ros::Time::now();
    marker.lifetime = ros::Duration(1.0); // seconds

    RVizPointMarker::SetMarkerProperties(position, r, g, b, shape);
};

visualization_msgs::Marker RVizPointMarker::GetMarker()
{
    return marker;
};
