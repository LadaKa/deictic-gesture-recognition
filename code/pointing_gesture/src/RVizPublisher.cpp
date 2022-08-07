#include "RVizPublisher.h"
#include "RVizLineMarker.h"
#include "RVizPointMarker.h"

// For Markers info, see http://wiki.ros.org/rviz/Tutorials/Markers%3A%20Basic%20Shapess

void RVizPublisher::PublishPointMarker(
    int id,
    geometry_msgs::Point32_<pointing_gesture::Skeleton> position,
    float color_r, float color_g, float color_b,
    uint32_t shape)
{
    RVizPointMarker rVizPointMarker(id, position, color_r, color_g, color_b, shape);
    visualization_msgs::Marker marker = rVizPointMarker.GetMarker();
    marker_pub_.publish(marker);
    // PrintJointPositionDebugInfo("PublishPointMarker", position);
}

void RVizPublisher::PublishLinesMarkers(
    int id,
    geometry_msgs::Point32_<pointing_gesture::Skeleton> positions[],
    int positions_count,
    float color_r, float color_g, float color_b)
{
    RVizLineMarker rVizLineMarker(
        id,
        positions,
        positions_count,
        color_r, color_g, color_b);
    visualization_msgs::Marker line_list = rVizLineMarker.GetLineList();

    marker_pub_.publish(line_list);
}

// sphere marker for skeleton joints
void RVizPublisher::PublishSphereMarker(
      int id, geometry_msgs::Point32_<pointing_gesture::Skeleton> position,
      float color_r, float color_g, float color_b)
  {
    RVizPublisher::PublishPointMarker(id, position, color_r, color_g, color_b, visualization_msgs::Marker::SPHERE);
  }

void RVizPublisher::PublishJoints(pointing_gesture::Skeleton_<pointing_gesture::Skeleton> skeleton_data)
{
    // head:
    RVizPublisher::PublishSphereMarker(3, skeleton_data.joint_position_head, 0.3, 0.3, 0.6);

    // spine:
    PublishSphereMarker(4, skeleton_data.joint_position_spine_top, 0.3, 0.3, 0.4);
    PublishSphereMarker(5, skeleton_data.joint_position_spine_mid, 0.3, 0.3, 0.4);
    PublishSphereMarker(6, skeleton_data.joint_position_spine_bottom, 0.3, 0.3, 0.4);

    // left arm:
    PublishSphereMarker(7, skeleton_data.joint_position_left_shoulder, 0.5, 0.1, 0.1);
    PublishSphereMarker(8, skeleton_data.joint_position_left_elbow, 0.5, 0.1, 0.1);
    PublishSphereMarker(9, skeleton_data.joint_position_left_hand, 0.5, 0.1, 0.1);

    // rigth arm:
    PublishSphereMarker(10, skeleton_data.joint_position_right_shoulder, 0.1, 0.1, 0.5);
    PublishSphereMarker(11, skeleton_data.joint_position_right_elbow, 0.1, 0.1, 0.5);
    PublishSphereMarker(12, skeleton_data.joint_position_right_hand, 0.1, 0.1, 0.5);
}

void RVizPublisher::PublishBones(pointing_gesture::Skeleton_<pointing_gesture::Skeleton> skeleton_data)
{
    // spine:
    geometry_msgs::Point32_<pointing_gesture::Skeleton> spinePositions[]{
        skeleton_data.joint_position_head,
        skeleton_data.joint_position_spine_top,
        skeleton_data.joint_position_spine_mid,
        skeleton_data.joint_position_spine_bottom};
    PublishLinesMarkers(13, spinePositions, 4, 0, 0, 0);

    // left arm:
    geometry_msgs::Point32_<pointing_gesture::Skeleton> leftArmPositions[]{
        skeleton_data.joint_position_spine_top,
        skeleton_data.joint_position_left_shoulder,
        skeleton_data.joint_position_left_elbow,
        skeleton_data.joint_position_left_hand};
    PublishLinesMarkers(14, leftArmPositions, 4, 0, 0, 0);
    // right arm:
    geometry_msgs::Point32_<pointing_gesture::Skeleton> rightArmPositions[]{
        skeleton_data.joint_position_spine_top,
        skeleton_data.joint_position_right_shoulder,
        skeleton_data.joint_position_right_elbow,
        skeleton_data.joint_position_right_hand};
    PublishLinesMarkers(15, rightArmPositions, 4, 0, 0, 0);
}

RVizPublisher::RVizPublisher(ros::Publisher marker_pub)
{
    RVizPublisher::marker_pub_ = marker_pub;
}

void RVizPublisher::PublishSkeleton(
    pointing_gesture::Skeleton_<pointing_gesture::Skeleton> skeleton)
{
    RVizPublisher::PublishJoints(skeleton);
    RVizPublisher::PublishBones(skeleton);
}

void RVizPublisher::PublishPointingGesture(
    PointingGesture *gesture)
{
    // intersection with floor
    geometry_msgs::Point32_<pointing_gesture::Skeleton> floorIntersection =
        gesture->GetFloorIntersection();

    // pointed ray
    geometry_msgs::Point32_<pointing_gesture::Skeleton> pointedRay[]{
        gesture->right_hand_position,
        floorIntersection};
    PublishLinesMarkers(101, pointedRay, 2, 0.2, 1, 0.2);
}
