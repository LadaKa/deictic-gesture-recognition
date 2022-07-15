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


void TrackedSkeleton::SetJointPositions(
    astra_body_t *body)
{
  SetJointPositionByWorldPosition(body, ASTRA_JOINT_HEAD, skeleton_data.joint_position_head);
  SetJointPositionByWorldPosition(body, ASTRA_JOINT_NECK, skeleton_data.joint_position_neck);

  SetJointPositionByWorldPosition(body, ASTRA_JOINT_SHOULDER_SPINE, skeleton_data.joint_position_spine_top);
  SetJointPositionByWorldPosition(body, ASTRA_JOINT_MID_SPINE, skeleton_data.joint_position_spine_mid);
  SetJointPositionByWorldPosition(body, ASTRA_JOINT_BASE_SPINE, skeleton_data.joint_position_spine_bottom);

  SetJointPositionByWorldPosition(body, ASTRA_JOINT_LEFT_SHOULDER, skeleton_data.joint_position_left_shoulder);
  SetJointPositionByWorldPosition(body, ASTRA_JOINT_LEFT_ELBOW, skeleton_data.joint_position_left_elbow);
  SetJointPositionByWorldPosition(body, ASTRA_JOINT_LEFT_HAND, skeleton_data.joint_position_left_hand);

  SetJointPositionByWorldPosition(body, ASTRA_JOINT_RIGHT_SHOULDER, skeleton_data.joint_position_right_shoulder);
  SetJointPositionByWorldPosition(body, ASTRA_JOINT_RIGHT_ELBOW, skeleton_data.joint_position_right_elbow);
  SetJointPositionByWorldPosition(body, ASTRA_JOINT_RIGHT_HAND, skeleton_data.joint_position_right_hand);
}


TrackedSkeleton::TrackedSkeleton(astra_body_t *body)
{
  TrackedSkeleton::skeleton_data.body_id = (int)body->id;
  TrackedSkeleton::skeleton_data.tracking_status = body->status;
  TrackedSkeleton::SetJointPositions(body);
}
