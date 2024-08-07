#ifndef TRACKEDPERSON_H
#define TRACKEDPERSON_H

#include <astra/capi/astra.h>
#include "BodyTracker.h"
#include "Skeleton.h"
#include "TrackedSkeleton.h"
#include "PointingGesture.h"

class TrackedPerson
{
private:
    pointing_gesture::BodyTracker_<pointing_gesture::BodyTracker> position_data;
    PointingGesture *pointing_gesture;

    void Set2DPositionDataByKeyJoint(
        int bodyId,
        int bodyStatus,
        astra_joint_t *keyJoint);

public:

    TrackedSkeleton *pointingTrackedSkeleton;

    TrackedPerson(
        astra_body_t *body);

    pointing_gesture::BodyTracker_<pointing_gesture::BodyTracker> GetPositionData();
    
    void SetPointingTrackedSkeleton(TrackedSkeleton trackedSkeleton);

    void SetPointingGesture(PointingGesture gesture);

    PointingGesture* GetPointingGesture();
};

#endif