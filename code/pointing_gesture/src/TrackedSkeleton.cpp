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

  SetJointPositionByWorldPosition(body, ASTRA_JOINT_LEFT_HIP, skeleton_data.joint_position_left_hip);
  SetJointPositionByWorldPosition(body, ASTRA_JOINT_LEFT_KNEE, skeleton_data.joint_position_left_knee);
  SetJointPositionByWorldPosition(body, ASTRA_JOINT_LEFT_FOOT, skeleton_data.joint_position_left_foot);

  SetJointPositionByWorldPosition(body, ASTRA_JOINT_RIGHT_HIP, skeleton_data.joint_position_right_hip);
  SetJointPositionByWorldPosition(body, ASTRA_JOINT_RIGHT_KNEE, skeleton_data.joint_position_right_knee);
  SetJointPositionByWorldPosition(body, ASTRA_JOINT_RIGHT_FOOT, skeleton_data.joint_position_right_foot);
}

TrackedSkeleton::TrackedSkeleton(astra_body_t *body)
{
  TrackedSkeleton::skeleton_data.body_id = (int)body->id;
  TrackedSkeleton::skeleton_data.tracking_status = body->status;
  TrackedSkeleton::SetJointPositions(body);
}

pointing_gesture::Skeleton_<pointing_gesture::Skeleton> TrackedSkeleton::GetSkeleton()
{
  return TrackedSkeleton::skeleton_data;
}

void TrackedSkeleton::GetGestureJointPosition(
    geometry_msgs::Point32_<pointing_gesture::Skeleton> &right_elbow,
    geometry_msgs::Point32_<pointing_gesture::Skeleton> &right_hand)
{
  right_elbow = skeleton_data.joint_position_right_elbow;
  right_hand = skeleton_data.joint_position_right_hand;

}

void TrackedSkeleton::PrintAllJointsPositions()
{
  PrintJointPosition("HEAD", skeleton_data.joint_position_head);
  PrintJointPosition("NECK", skeleton_data.joint_position_neck);

  PrintJointPosition("SHOULDER_SPINE", skeleton_data.joint_position_spine_top);
  PrintJointPosition("MID_SPINE", skeleton_data.joint_position_spine_mid);
  PrintJointPosition("BASE_SPINE", skeleton_data.joint_position_spine_bottom);

  PrintJointPosition("LEFT_SHOULDER", skeleton_data.joint_position_left_shoulder);
  PrintJointPosition("LEFT_ELBOW", skeleton_data.joint_position_left_elbow);
  PrintJointPosition("LEFT_HAND", skeleton_data.joint_position_left_hand);

  PrintJointPosition("RIGHT_SHOULDER", skeleton_data.joint_position_right_shoulder);
  PrintJointPosition("RIGHT_ELBOW", skeleton_data.joint_position_right_elbow);
  PrintJointPosition("RIGHT_HAND", skeleton_data.joint_position_right_hand);

  PrintJointPosition("LEFT_HIP", skeleton_data.joint_position_left_hip);
  PrintJointPosition("LEFT_KNEE", skeleton_data.joint_position_left_knee);
  PrintJointPosition("LEFT_FOOT", skeleton_data.joint_position_left_foot);

  PrintJointPosition("RIGHT_HIP", skeleton_data.joint_position_right_hip);
  PrintJointPosition("RIGHT_KNEE", skeleton_data.joint_position_right_knee);
  PrintJointPosition("RIGHT_FOOT", skeleton_data.joint_position_right_foot);
}

// TODO: align better
void TrackedSkeleton::PrintJointPosition(
    std::string joint_name,
    geometry_msgs::Point32_<pointing_gesture::Skeleton> joint_position)
{
    printf("\t%-20s:\t\t\t%f %f %f \n",
           joint_name.c_str(),
           joint_position.x, joint_position.y, joint_position.z);
}