// Modified 'astra_body_tracker_node.cpp'
// https://github.com/shinselrobots/astra_body_tracker

#include "RVizLineMarker.h"

visualization_msgs::Marker line_list;

void RVizLineMarker::SetLineListProperties(
    int id,
    float r, float g, float b)
{
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.id = id;

    line_list.header.frame_id = "camera_link";
    line_list.header.stamp = ros::Time::now();
    line_list.ns = "astra_body_tracker";
    line_list.lifetime = ros::Duration(1.0); // seconds
    line_list.action = visualization_msgs::Marker::ADD;

    line_list.scale.x = 0.05;

    line_list.color.r = r;
    line_list.color.g = g;
    line_list.color.b = b;
    line_list.color.a = 1;
}

void RVizLineMarker::SetLineListPoints(
    geometry_msgs::Point32_<pointing_gesture::Skeleton> positions[],
    int positions_count)
{
    for (int i = 0; i < positions_count - 1; i++)
    {
        geometry_msgs::Point32_<pointing_gesture::Skeleton> position0 = positions[i];
        geometry_msgs::Point32_<pointing_gesture::Skeleton> position1 = positions[i + 1];

        geometry_msgs::Point p0;
        p0.x = position0.x;
        p0.y = position0.y;
        p0.z = position0.z;

        geometry_msgs::Point p1;
        p1.x = position1.x;
        p1.y = position1.y;
        p1.z = position1.z;

        line_list.points.push_back(p0);
        line_list.points.push_back(p1);
    }
}

RVizLineMarker::RVizLineMarker(
    int id,
    geometry_msgs::Point32_<pointing_gesture::Skeleton> positions[],
    int positions_count,
    float r, float g, float b)
{
    RVizLineMarker::SetLineListProperties(id, r, g, b);
    RVizLineMarker::SetLineListPoints(positions, positions_count);
};

visualization_msgs::Marker RVizLineMarker::GetLineList()
{
    return line_list;
}
