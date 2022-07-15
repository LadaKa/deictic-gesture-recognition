#include "TrackedSkeleton.h"


void TrackedSkeleton::SetJointPositionByWorldPosition(
      astra_body_t *body,
      _astra_joint_type joint_type,
      geometry_msgs::Point32_<pointing_gesture::Skeleton> &joint_position)
  {
    astra_joint_t *joint = &body->joints[joint_type];
    joint_position.x = ((astra_vector3f_t *)&joint->worldPosition)->z / 1000.0; // why so weird?
    joint_position.y = ((astra_vector3f_t *)&joint->worldPosition)->x / 1000.0;
    joint_position.z = ((astra_vector3f_t *)&joint->worldPosition)->y / 1000.0;
  }

TrackedSkeleton::TrackedSkeleton(astra_body_t *body)
{
    TrackedSkeleton::skeleton_data.body_id = (int)body->id;;
    TrackedSkeleton::skeleton_data.tracking_status = body->status;
}
