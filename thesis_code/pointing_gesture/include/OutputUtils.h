#ifndef OUTPUTUTILS_H
#define OUTPUTUTILS_H

#include <astra/capi/astra.h>
#include "BodyTracker.h"
#include "Skeleton.h"
#include <geometry_msgs/Point32.h>

class OutputUtils
{
private:
public:
    OutputUtils();

    void PrintBasicTrackingInfo(
        std::string nodeName,
        int bodyId,
        astra_body_tracking_feature_flags_t features,
        astra_vector3f_t *centerOfMass);

    void PrintBodyStatus(
        int bodyId,
        int bodyStatus);

    void PrintJointPosition(
        std::string header,
        geometry_msgs::Point32_<pointing_gesture::Skeleton> joint_position);

    void PrintJointData(std::string joint_name, const int32_t bodyId, const astra_joint_t *joint);
};

#endif