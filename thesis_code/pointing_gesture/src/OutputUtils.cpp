// Modified 'astra_body_tracker_node.cpp'
// https://github.com/shinselrobots/astra_body_tracker

#include "OutputUtils.h"

#include "ros/console.h"

void OutputUtils::PrintBasicTrackingInfo(
    std::string nodeName,
    int bodyId,
    astra_body_tracking_feature_flags_t features,
    astra_vector3f_t *centerOfMass)
{
    ROS_INFO("%s: detected person ID %d", nodeName.c_str(), bodyId);

    const bool jointTrackingEnabled =
        (features & ASTRA_BODY_TRACKING_JOINTS) == ASTRA_BODY_TRACKING_JOINTS;
    const bool handPoseRecognitionEnabled =
        (features & ASTRA_BODY_TRACKING_HAND_POSES) == ASTRA_BODY_TRACKING_HAND_POSES;

    ROS_INFO("Body %d CenterOfMass (%f, %f, %f)\n",
             bodyId, centerOfMass->x, centerOfMass->y, centerOfMass->z);
    ROS_INFO("    Joint Tracking Enabled: %s     Hand Pose Recognition Enabled: %s\n",
             jointTrackingEnabled ? "True" : "False",
             handPoseRecognitionEnabled ? "True" : "False");
}

void OutputUtils::PrintBodyStatus(
    int bodyId,
    int bodyStatus)
{
    // Tracking status
    // NOT_TRACKING = 0
    // TRACKING_LOST = 1
    // TRACKING_STARTED = 2
    // TRACKING = 3

    if (bodyStatus == ASTRA_BODY_STATUS_TRACKING_STARTED)
    {
        printf("Body Id: %d Status: Tracking started\n", bodyId);
    }
    else if (bodyStatus == ASTRA_BODY_STATUS_TRACKING)
    {
        printf("Body Id: %d Status: Tracking\n", bodyId);
    }
    else if (bodyStatus == ASTRA_BODY_STATUS_LOST)
    {
        printf("Body %u Status: Tracking lost.\n", bodyId);
    }
    else // bodyStatus == ASTRA_BODY_STATUS_NOT_TRACKING
    {
        printf("Body Id: %d Status: Not Tracking\n", bodyId);
    }
}

void OutputUtils::PrintJointPosition(
    std::string header,
    geometry_msgs::Point32_<pointing_gesture::Skeleton> joint_position)
{
    std::string info =
        header + ":" + std::to_string(joint_position.x) + "; " + std::to_string(joint_position.y) + "; " + std::to_string(joint_position.z);
    ROS_INFO_STREAM(info);
}

void OutputUtils::PrintJointData(std::string joint_name, const int32_t bodyId, const astra_joint_t *joint)
{

    printf("%14s:", joint_name.c_str());

    // jointType is one of ASTRA_JOINT_* which exists for each joint type
    const astra_joint_type_t jointType = joint->type;

    // jointStatus is one of:

    // ASTRA_JOINT_STATUS_NOT_TRACKED = 0,
    // ASTRA_JOINT_STATUS_LOW_CONFIDENCE = 1,
    // ASTRA_JOINT_STATUS_TRACKED = 2,
    const astra_joint_status_t jointStatus = joint->status;

    const astra_vector3f_t *worldPos = &joint->worldPosition;

    // depthPosition is in pixels from 0 to width and 0 to height
    // where width and height are member of astra_bodyframe_info_t
    // which is obtained from astra_bodyframe_info().
    const astra_vector2f_t *depthPos = &joint->depthPosition;

    printf("Body %u Joint %d status %d @ world (%.1f, %.1f, %.1f) depth (%.1f, %.1f)\n",
           bodyId,
           jointType,
           jointStatus,
           worldPos->x,
           worldPos->y,
           worldPos->z,
           depthPos->x,
           depthPos->y);
}
