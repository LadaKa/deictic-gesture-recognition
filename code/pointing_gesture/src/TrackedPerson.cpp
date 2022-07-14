#include "TrackedPerson.h"
#include <astra/capi/astra.h>

#define KEY_JOINT_TO_TRACK ASTRA_JOINT_SHOULDER_SPINE

TrackedPerson::TrackedPerson(astra_body_t* body)
{
      

      // THIS IS THE MOST RELIABLE TRACKING POINT, so we use it for person position in 3D!
      astra_joint_t *keyJoint = &body->joints[KEY_JOINT_TO_TRACK];

      pointing_gesture::BodyTracker_<pointing_gesture::BodyTracker> position_data;
      //Set2DPositionDataByKeyJoint(bodyId, bodyStatus, keyJoint, position_data);

      ///////////////////////////////////////////////////////////////
      // 3D position of person
      // TODO:
      position_data.position3d.x = ((astra_vector3f_t *)&keyJoint->worldPosition)->z / 1000.0;
      position_data.position3d.y = ((astra_vector3f_t *)&keyJoint->worldPosition)->x / 1000.0;
      position_data.position3d.z = ((astra_vector3f_t *)&keyJoint->worldPosition)->y / 1000.0;

}