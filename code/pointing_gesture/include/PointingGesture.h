#ifndef POINTINGGESTURE_H
#define POINTINGGESTURE_H

#include <geometry_msgs/Point32.h>
#include <astra/capi/astra.h>

class PointingGesture
{
private:
    geometry_msgs::Point32 right_elbow_position;
    geometry_msgs::Point32 right_wrist_position;
    astra_plane_t floor;

    geometry_msgs::Point32 GetPointsDifference(
        geometry_msgs::Point32 point_0,
        geometry_msgs::Point32 point_1);

public:
    PointingGesture(
        geometry_msgs::Point32 right_elbow_pos,
        geometry_msgs::Point32 right_wrist_pos,
        astra_plane_t floor_plane);

    geometry_msgs::Point32 GetFloorIntersection();
};

#endif