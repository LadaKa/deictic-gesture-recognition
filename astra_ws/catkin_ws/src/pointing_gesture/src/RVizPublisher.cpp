// Modified 'astra_body_tracker_node.cpp'
// https://github.com/shinselrobots/astra_body_tracker

#include "RVizPublisher.h"
#include "RVizLineMarker.h"
#include "RVizPointMarker.h"

// For Markers info, see http://wiki.ros.org/rviz/Tutorials/Markers%3A%20Basic%20Shapess

void RVizPublisher::PublishPointMarker(
    geometry_msgs::Point32_<pointing_gesture::Skeleton> position,
    float color_r, float color_g, float color_b,
    uint32_t shape)
{
    RVizPointMarker rVizPointMarker(published_ids++, position, color_r, color_g, color_b, shape);
    visualization_msgs::Marker marker = rVizPointMarker.GetMarker();
    marker_pub_.publish(marker);
}

void RVizPublisher::PublishLinesMarkers(
    geometry_msgs::Point32_<pointing_gesture::Skeleton> positions[],
    int positions_count,
    float color_r, float color_g, float color_b)
{
    RVizLineMarker rVizLineMarker(
        published_ids++,
        positions,
        positions_count,
        color_r, color_g, color_b);
    visualization_msgs::Marker line_list = rVizLineMarker.GetLineList();

    marker_pub_.publish(line_list);
}

// sphere marker for skeleton joints
void RVizPublisher::PublishSphereMarker(
      geometry_msgs::Point32_<pointing_gesture::Skeleton> position,
      float color_r, float color_g, float color_b)
{
    RVizPublisher::PublishPointMarker(position, color_r, color_g, color_b, visualization_msgs::Marker::SPHERE);
}

void RVizPublisher::PublishJoints(pointing_gesture::Skeleton_<pointing_gesture::Skeleton> skeleton_data)
{
    // head:
    PublishSphereMarker(skeleton_data.joint_position_head, 0.3, 0.3, 0.6);

    // spine:
    PublishSphereMarker(skeleton_data.joint_position_spine_top, 0.3, 0.3, 0.4);
    PublishSphereMarker(skeleton_data.joint_position_spine_mid, 0.3, 0.3, 0.4);
    PublishSphereMarker(skeleton_data.joint_position_spine_bottom, 0.3, 0.3, 0.4);

    // left arm:
    PublishSphereMarker(skeleton_data.joint_position_left_shoulder, 0.5, 0.1, 0.1);
    PublishSphereMarker(skeleton_data.joint_position_left_elbow, 0.5, 0.1, 0.1);
    PublishSphereMarker(skeleton_data.joint_position_left_hand, 0.5, 0.1, 0.1);

    // left leg:
    PublishSphereMarker(skeleton_data.joint_position_left_hip, 0.5, 0.1, 0.1);
    PublishSphereMarker(skeleton_data.joint_position_left_knee, 0.5, 0.1, 0.1);
    PublishSphereMarker(skeleton_data.joint_position_left_foot, 0.5, 0.1, 0.1);    

    // right arm:
    PublishSphereMarker(skeleton_data.joint_position_right_shoulder, 0.1, 0.1, 0.5);
    PublishSphereMarker(skeleton_data.joint_position_right_elbow, 0.1, 0.1, 0.5);
    PublishSphereMarker(skeleton_data.joint_position_right_hand, 0.1, 0.1, 0.5);

    // rigth leg:
    PublishSphereMarker(skeleton_data.joint_position_right_hip, 0.1, 0.1, 0.5);
    PublishSphereMarker(skeleton_data.joint_position_right_knee, 0.1, 0.1, 0.5);
    PublishSphereMarker(skeleton_data.joint_position_right_foot, 0.1, 0.1, 0.5);   
}

void RVizPublisher::PublishBones(pointing_gesture::Skeleton_<pointing_gesture::Skeleton> skeleton_data)
{
    // spine:
    geometry_msgs::Point32_<pointing_gesture::Skeleton> spinePositions[]{
        skeleton_data.joint_position_head,
        skeleton_data.joint_position_spine_top,
        skeleton_data.joint_position_spine_mid,
        skeleton_data.joint_position_spine_bottom};
    PublishLinesMarkers(spinePositions, 4, 0, 0, 0);

    // left arm:
    geometry_msgs::Point32_<pointing_gesture::Skeleton> leftArmPositions[]{
        skeleton_data.joint_position_spine_top,
        skeleton_data.joint_position_left_shoulder,
        skeleton_data.joint_position_left_elbow,
        skeleton_data.joint_position_left_hand};
    PublishLinesMarkers(leftArmPositions, 4, 0, 0, 0);

    // left leg:
    geometry_msgs::Point32_<pointing_gesture::Skeleton> leftLegPositions[]{
        skeleton_data.joint_position_spine_bottom,
        skeleton_data.joint_position_left_hip,
        skeleton_data.joint_position_left_knee,
        skeleton_data.joint_position_left_foot};
    PublishLinesMarkers(leftLegPositions, 4, 0, 0, 0);

    // right arm:
    geometry_msgs::Point32_<pointing_gesture::Skeleton> rightArmPositions[]{
        skeleton_data.joint_position_spine_top,
        skeleton_data.joint_position_right_shoulder,
        skeleton_data.joint_position_right_elbow,
        skeleton_data.joint_position_right_hand};
    PublishLinesMarkers(rightArmPositions, 4, 0, 0, 0);

    // right leg:
    geometry_msgs::Point32_<pointing_gesture::Skeleton> rightLegPositions[]{
        skeleton_data.joint_position_spine_bottom,
        skeleton_data.joint_position_right_hip,
        skeleton_data.joint_position_right_knee,
        skeleton_data.joint_position_right_foot};
    PublishLinesMarkers(rightLegPositions, 4, 0, 0, 0);
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
    PublishLinesMarkers(pointedRay, 2, 1, 1, 0.4);
}

// stashed code
void RVizPublisher::PublishTarget(
    geometry_msgs::Point32_<pointing_gesture::Skeleton> target)
{   
    RVizPointMarker rVizPointMarker(
        published_ids++, target, 
        1.0, 0.0, 0.1, 
        visualization_msgs::Marker::SPHERE);
    visualization_msgs::Marker marker = rVizPointMarker.GetMarker();
    rVizPointMarker.SetMarkerLifeTime(0.0);
    marker_pub_.publish(marker);
}

void RVizPublisher::PublishSafetyFrame(
    int safety_frame_min_x,
    int safety_frame_max_x,
    int safety_frame_min_y,
    int safety_frame_max_y)
{   
    float z = -0.75;
    geometry_msgs::Point p0;
    p0.x = safety_frame_min_x;
    p0.y = safety_frame_min_y;
    p0.z = z;

    geometry_msgs::Point p1;
    p1.x = safety_frame_min_x;
    p1.y = safety_frame_max_y;
    p1.z = z;
        
    geometry_msgs::Point p2;
    p2.x = safety_frame_max_x;
    p2.y = safety_frame_max_y;
    p2.z = z;

    geometry_msgs::Point p3;
    p3.x = safety_frame_max_x;
    p3.y = safety_frame_min_y;
    p3.z = z;

    RVizLineMarker rVizLineMarker(published_ids++, p0, p1, p2, p3);
    visualization_msgs::Marker line_list = rVizLineMarker.GetLineList();

    marker_pub_.publish(line_list);
}
