#ifndef POINTINGGESTURE_H
#define POINTINGGESTURE_H

#include <geometry_msgs/Point32.h>
#include <astra/capi/astra.h>
#include "Skeleton.h"

class PointingGesture
{
private:
    
    astra_plane_t floor;

    geometry_msgs::Point32_<pointing_gesture::Skeleton> GetPointsDifference(
        geometry_msgs::Point32_<pointing_gesture::Skeleton> point_0,
        geometry_msgs::Point32_<pointing_gesture::Skeleton> point_1);

public:

    geometry_msgs::Point32_<pointing_gesture::Skeleton> right_elbow_position;
    geometry_msgs::Point32_<pointing_gesture::Skeleton> right_hand_position;
    
    PointingGesture();

    PointingGesture(
        geometry_msgs::Point32_<pointing_gesture::Skeleton> right_elbow_pos,
        geometry_msgs::Point32_<pointing_gesture::Skeleton> right_hand_pos,
        astra_plane_t floor_plane);

    geometry_msgs::Point32_<pointing_gesture::Skeleton> GetFloorIntersection();
};

#endif