// Modified 'astra_body_tracker_node.cpp'
// https://github.com/shinselrobots/astra_body_tracker

#include "TrackedPerson.h"
#include <astra/capi/astra.h>

#define KEY_JOINT_TO_TRACK ASTRA_JOINT_SHOULDER_SPINE

void TrackedPerson::Set2DPositionDataByKeyJoint(
    int bodyId,
    int bodyStatus,
    astra_joint_t *keyJoint)
{

    position_data.body_id = bodyId;
    position_data.tracking_status = bodyStatus;
    position_data.gesture = -1;

    // 2D position for camera servo tracking
    const float ASTRA_MINI_FOV_X = 1.047200;  // (60 degrees horizontal)
    const float ASTRA_MINI_FOV_Y = -0.863938; // (49.5 degrees vertical)

    // Convert projection to radians
    // Astra proj is 0.0 (right) --> 0.628 (left)
    //           and 0.0 (top)   --> 0.628 (botom)

    float projection_x = ((astra_vector2f_t *)&keyJoint->depthPosition)->x / 1000.0;
    float projection_y = ((astra_vector2f_t *)&keyJoint->depthPosition)->y / 1000.0;

    position_data.position2d.x = (projection_x - 0.314) * ASTRA_MINI_FOV_X;
    position_data.position2d.y = (projection_y - 0.314) * ASTRA_MINI_FOV_Y;
    position_data.position2d.z = 0.0;

   
}

TrackedPerson::TrackedPerson(astra_body_t *body)
{
    int bodyId = (int)body->id;
    int bodyStatus = body->status;

    // THIS IS THE MOST RELIABLE TRACKING POINT, so we use it for person position in 3D!
    astra_joint_t *keyJoint = &body->joints[KEY_JOINT_TO_TRACK];

    TrackedPerson::Set2DPositionDataByKeyJoint(bodyId, bodyStatus, keyJoint);

    position_data.position3d.x = ((astra_vector3f_t *)&keyJoint->worldPosition)->z / 1000.0;
    position_data.position3d.y = ((astra_vector3f_t *)&keyJoint->worldPosition)->x / 1000.0;
    position_data.position3d.z = ((astra_vector3f_t *)&keyJoint->worldPosition)->y / 1000.0;
}

pointing_gesture::BodyTracker_<pointing_gesture::BodyTracker> TrackedPerson::GetPositionData()
{
    return TrackedPerson::position_data;
}

void TrackedPerson::SetPointingGesture(
    PointingGesture gesture)
{
    pointing_gesture = &gesture;
}

PointingGesture *TrackedPerson::GetPointingGesture()
{
    return pointing_gesture;
}

void TrackedPerson::SetPointingTrackedSkeleton(TrackedSkeleton trackedSkeleton)
{
    pointingTrackedSkeleton = &trackedSkeleton;
}