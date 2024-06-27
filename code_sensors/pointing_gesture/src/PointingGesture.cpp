#include "PointingGesture.h"

PointingGesture::PointingGesture(
    geometry_msgs::Point32_<pointing_gesture::Skeleton> upper_joint_pos,
    geometry_msgs::Point32_<pointing_gesture::Skeleton> right_hand_pos,
    geometry_msgs::Point32_<pointing_gesture::Skeleton> right_foot_pos)
{
    upper_joint_position = upper_joint_pos;
    right_hand_position = right_hand_pos;
    right_foot_position = right_foot_pos;

    intersection = ComputeFloorIntersection();
};


geometry_msgs::Point32_<pointing_gesture::Skeleton> PointingGesture::GetPointsDifference(
    geometry_msgs::Point32_<pointing_gesture::Skeleton> point_0,
    geometry_msgs::Point32_<pointing_gesture::Skeleton> point_1)
{
    geometry_msgs::Point32_<pointing_gesture::Skeleton> difference;

    difference.x = point_0.x - point_1.x;
    difference.y = point_0.y - point_1.y;
    difference.z = point_0.z - point_1.z;

    return difference;
};

geometry_msgs::Point32_<pointing_gesture::Skeleton> PointingGesture::ComputeFloorIntersection()
{
    geometry_msgs::Point32_<pointing_gesture::Skeleton> difference = PointingGesture::GetPointsDifference(
        right_hand_position,
        upper_joint_position
        );

    intersection.z =
        right_foot_position.z;

    intersection.x =
        upper_joint_position.x + ((intersection.z - upper_joint_position.z) * difference.x) / difference.z;

    intersection.y =
        upper_joint_position.y + ((intersection.z - upper_joint_position.z) * difference.y) / difference.z;


    // OutputPosition("difference", difference);
    OutputGestureIntersection(intersection);

    return intersection;
};

geometry_msgs::Point32_<pointing_gesture::Skeleton> PointingGesture::GetFloorIntersection()
{
    return intersection;
}

void PointingGesture::OutputPosition(
    std::string header,
    geometry_msgs::Point32_<pointing_gesture::Skeleton> position)
{
    printf("\t\t%s:\t%f %f %f \n",
           header.c_str(),
           position.x, position.y, position.z);
}

void PointingGesture::OutputGestureIntersection(
    geometry_msgs::Point32_<pointing_gesture::Skeleton> intersection)
{
    printf("\t\tPointing gesture: \n");
    OutputPosition("right upper joint", upper_joint_position);
    OutputPosition("right hand", right_hand_position);
    OutputPosition("intersection", intersection);
}

geometry_msgs::Point32 PointingGesture::GetIntersectionMessage()
{
    geometry_msgs::Point32 intersectionMsg;
    intersectionMsg.x = intersection.x;
    intersectionMsg.y = intersection.y;
    intersectionMsg.z = intersection.z;

    return intersectionMsg;
}
