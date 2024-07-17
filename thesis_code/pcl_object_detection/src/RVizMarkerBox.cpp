// Modified pcl_object_detection package:
// Added properties check. Modified object detection: fixed total object count. Refactored.
// https://github.com/shinselrobots/pcl_object_detection

#include "RVizMarkerBox.h"

visualization_msgs::Marker RVizMarkerBox::GetMarker()
{
    return RVizMarkerBox::marker;
}

// creates rVizMarkerBox with modified color of marker
RVizMarkerBox::RVizMarkerBox(
    visualization_msgs::Marker msgMarker,
    float color_r, float color_g, float color_b)
{
    marker = msgMarker;
    marker.color.r = color_r;
    marker.color.g = color_g;
    marker.color.b = color_b;
}

RVizMarkerBox::RVizMarkerBox(
    std::string frame_id,
    int id,
    float x, float y, float z,
    float size_x, float size_y, float size_z,
    float color_r, float color_g, float color_b)
{
    marker.header.frame_id = frame_id; // input_cloud_frame_; //"camera_depth_frame"; //"base_link";
    marker.header.stamp = ros::Time::now();
    marker.lifetime = ros::Duration(1); // seconds
    marker.ns = "pcl_object_detection";
    marker.id = id; 

    uint32_t shape = visualization_msgs::Marker::CUBE;
    marker.type = shape;

    marker.action = visualization_msgs::Marker::ADD;
    marker.color.r = color_r;
    marker.color.g = color_g;
    marker.color.b = color_b;
    marker.color.a = 0.75;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = size_x; // size of marker in meters
    marker.scale.y = size_y;
    marker.scale.z = size_z;

    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
}