#ifndef TRACKEDPERSON_H
#define TRACKEDPERSON_H

#include <astra/capi/astra.h>
#include "BodyTracker.h"
#include "Skeleton.h"
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
    TrackedPerson(
        astra_body_t *body);

    pointing_gesture::BodyTracker_<pointing_gesture::BodyTracker> GetPositionData();
    
    void SetPointingGesture(
        pointing_gesture::Skeleton_<pointing_gesture::Skeleton> skeleton_data,
        astra_plane_t floorPlane);
};

#endif