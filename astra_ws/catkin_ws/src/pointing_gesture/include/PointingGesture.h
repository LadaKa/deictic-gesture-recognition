#ifndef POINTINGGESTURE_H
#define POINTINGGESTURE_H

#include <geometry_msgs/Point32.h>
#include <astra/capi/astra.h>
#include "Skeleton.h"

class PointingGesture
{
private:

    geometry_msgs::Point32_<pointing_gesture::Skeleton> GetPointsDifference(
        geometry_msgs::Point32_<pointing_gesture::Skeleton> point_0,
        geometry_msgs::Point32_<pointing_gesture::Skeleton> point_1);

    geometry_msgs::Point32_<pointing_gesture::Skeleton> ComputeFloorIntersection();

public:

    geometry_msgs::Point32_<pointing_gesture::Skeleton> upper_joint_position;
    geometry_msgs::Point32_<pointing_gesture::Skeleton> right_hand_position;
    geometry_msgs::Point32_<pointing_gesture::Skeleton> right_foot_position;

    geometry_msgs::Point32_<pointing_gesture::Skeleton> intersection;

    PointingGesture();

    PointingGesture(
        geometry_msgs::Point32_<pointing_gesture::Skeleton> upper_joint_pos,
        geometry_msgs::Point32_<pointing_gesture::Skeleton> right_hand_pos,
        geometry_msgs::Point32_<pointing_gesture::Skeleton> right_foot_pos);

    geometry_msgs::Point32_<pointing_gesture::Skeleton> GetFloorIntersection();

    geometry_msgs::Point32 GetIntersectionMessage();

    // DEBUG:
    void OutputPosition(
        std::string header,
        geometry_msgs::Point32_<pointing_gesture::Skeleton> position);

    void OutputGestureIntersection(
        geometry_msgs::Point32_<pointing_gesture::Skeleton> intersection);

    // stashed code:
    bool HasIntersectionInsideFrame(
        int frame_min_x,
        int frame_max_x,
        int frame_min_y,
        int frame_max_y);
};

#endif