#include "PointingGesture.h"

PointingGesture::PointingGesture(
    geometry_msgs::Point32 right_elbow_pos,
    geometry_msgs::Point32 right_wrist_pos,
    astra_plane_t floor_plane)
{
    right_elbow_position = right_elbow_pos;
    right_wrist_position = right_wrist_pos;
    floor = floor_plane;
};

geometry_msgs::Point32 PointingGesture::GetPointsDifference(
    geometry_msgs::Point32 point_0,
    geometry_msgs::Point32 point_1)
{
    geometry_msgs::Point32 difference;

    difference.x = point_0.x - point_1.x;
    difference.y = point_0.y - point_1.y;
    difference.z = point_0.z - point_1.z;

    return difference;
};

geometry_msgs::Point32 PointingGesture::GetFloorIntersection()
{
    geometry_msgs::Point32 difference = PointingGesture::GetPointsDifference(
        right_elbow_position,
        right_wrist_position);

    geometry_msgs::Point32 intersection;

    intersection.x =
        right_elbow_position.x + (right_elbow_position.z * difference.x) / difference.z;

    intersection.y =
        right_elbow_position.y + (right_elbow_position.z * difference.y) / difference.z;

    intersection.z = 
        floor.d;

    return intersection;
};