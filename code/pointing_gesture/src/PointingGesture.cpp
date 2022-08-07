#include "PointingGesture.h"

PointingGesture::PointingGesture(
    geometry_msgs::Point32_<pointing_gesture::Skeleton> right_elbow_pos,
    geometry_msgs::Point32_<pointing_gesture::Skeleton> right_hand_pos,
    astra_plane_t floor_plane)
{
    right_elbow_position = right_elbow_pos;
    right_hand_position = right_hand_pos;
    floor = floor_plane;
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

geometry_msgs::Point32_<pointing_gesture::Skeleton> PointingGesture::GetFloorIntersection()
{
    geometry_msgs::Point32_<pointing_gesture::Skeleton> difference = PointingGesture::GetPointsDifference(
        right_elbow_position,
        right_hand_position);

    geometry_msgs::Point32_<pointing_gesture::Skeleton> intersection;

    intersection.x =
        right_elbow_position.x + (right_elbow_position.z * difference.x) / difference.z;

    intersection.y =
        right_elbow_position.y + (right_elbow_position.z * difference.y) / difference.z;

    intersection.z =
        floor.d;

    // DEBUG:
    OutputPosition("difference", difference);
    OutputGestureIntersection(intersection);

    return intersection;
};

// DEBUG:
/*
Pointing gesture: 
right elbow:
 2.450810 -0.040403 0.440874 
right hand:
 2.186281 -0.064913 0.492674 
intersection:
 0.199423 -0.249009 0.000000 
difference:
 0.264529 0.024510 -0.051801 
*/
void PointingGesture::OutputPosition(
    std::string header,
    geometry_msgs::Point32_<pointing_gesture::Skeleton> position)
{
    printf("%s:\n %f %f %f \n",
           header.c_str(),
           position.x, position.y, position.z);
}

void PointingGesture::OutputGestureIntersection(
    geometry_msgs::Point32_<pointing_gesture::Skeleton> intersection)
{
    printf("Pointing gesture: \n");
    OutputPosition("right elbow", right_elbow_position);
    OutputPosition("right hand", right_hand_position);
    OutputPosition("intersection", intersection);
}
