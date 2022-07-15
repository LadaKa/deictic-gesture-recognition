#ifndef TRACKEDSKELETON_H
#define TRACKEDSKELETON_H

#include <astra/capi/astra.h>
#include "BodyTracker.h"
#include "Skeleton.h"

class TrackedSkeleton
{
private:
        pointing_gesture::Skeleton_<pointing_gesture::Skeleton> skeleton_data;

    void SetJointPositionByWorldPosition(
        astra_body_t *body,
        _astra_joint_type joint_type,
        geometry_msgs::Point32_<pointing_gesture::Skeleton> &joint_position);

public:
    TrackedSkeleton(astra_body_t *body);
};

#endif