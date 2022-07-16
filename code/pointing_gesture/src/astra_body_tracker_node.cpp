// Modified 'astra_body_tracker_node.cpp'
// https://github.com/shinselrobots/astra_body_tracker

/* Body Tracker Node
   Publish data as 3 messages:

   Publish data messages:

   1. body_tracking_position_pub_ custom message:  <body_tracker_msgs::BodyTracker>
   Includes:
   2D position of person relative to head camera; allows for fast, smooth tracking
     when camera is mounted on pan/tilt servos.
     (x, y from 0.0 to 1.0, z is real)
     Astra Mini FOV: 60 horz, 49.5 vert (degrees)

   3D position of the neck joint in relation to robot (using TF)
     Joint.real: position in real world coordinates
     Useful for tracking person in 3D

   2. body_tracking_skeleton_pub_ custom message: <body_tracker_msgs::Skeleton>
   Includes:
   Everyting in BodyTracker message above, plus 3D position of upper body.
     joints in relation to robot (using TF)
     Joint.real: position in real world coordinates

   3. marker_pub_  message: <visualization_msgs::Marker>
   Publishes 3d markers for selected joints.  Visible as markers in RVIZ

*/

#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include <sstream>
#include <string>

// ros

#include "ros/console.h"

// Orbbec Astra SDK
#include <astra/capi/astra.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <key_handler.h>

// custom message
#include "BodyTracker.h"

#include "RVizLineMarker.h"
#include "RVizPointMarker.h"
#include "RVizPublisher.h"

#include "TrackedPerson.h"
#include "TrackedSkeleton.h"



class astra_body_tracker_node
{
public:
  astra_body_tracker_node(std::string name) : _name(name)
  {
    ROS_INFO("Hallo Spaceboy!");
    ROS_INFO("%s: Initializing node", _name.c_str());
    bool initialized = false;
    last_id_ = -1;
    ros::NodeHandle nodeHandle("~");
    nodeHandle.param<std::string>("myparm1", myparm1_, "mydefault");

    // PUBLISHERS
    // Publish tracked person in 2D and 3D
    // 2D: x,y in camera frame.   3D: x,y,z in world coordinates
    body_tracking_position_pub_ = nh_.advertise<pointing_gesture::BodyTracker>("body_tracker/position", 1);

    // Tracked person upper body skeleton
    body_tracking_skeleton_pub_ = nh_.advertise<pointing_gesture::Skeleton>("body_tracker/skeleton", 1);

    // Publish markers to show where robot thinks person is in RViz
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("body_tracker/marker", 1);

    ROS_INFO("astra_body_tracker: Advertised Publisher: body_tracker/pose, skeleton, marker");
  }

  ~astra_body_tracker_node()
  {
    ROS_INFO("astra_body_tracker_node shutting down");
  }

  //////////////////////////////////////////////////////////
  /*
    Modified Orbec Astra sample code and Shinsel Robotics code

    Removed:      void output_floor(astra_bodyframe_t bodyFrame)
                  void output_body_mask(astra_bodyframe_t bodyFrame)
                  void output_bodyframe_info(astra_bodyframe_t bodyFrame)

  */

  void output_joint(std::string joint_name, const int32_t bodyId, const astra_joint_t *joint)
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

  void output_bodies(astra_bodyframe_t bodyFrame)
  {
    int i;
    astra_body_list_t bodyList;
    const astra_status_t rc = astra_bodyframe_body_list(bodyFrame, &bodyList);
    if (rc != ASTRA_STATUS_SUCCESS)
    {
      printf("Error %d in astra_bodyframe_body_list()\n", rc);
      return;
    }

    for (i = 0; i < bodyList.count; ++i)
    {
      astra_body_t *body = &bodyList.bodies[i];
      int bodyId = (int)body->id;
      int bodyStatus = body->status;

      // person position
      TrackedPerson person(body);
      body_tracking_position_pub_.publish(person.GetPositionData()); 

      // full skeleton data of person
      TrackedSkeleton trackedSkeleton(body);
      pointing_gesture::Skeleton_<pointing_gesture::Skeleton> skeleton_data = trackedSkeleton.GetSkeleton();
      body_tracking_skeleton_pub_.publish(skeleton_data); // full skeleton data

      RVizPublisher rVizPublisher(marker_pub_);
      rVizPublisher.PublishSkeleton(skeleton_data);
    }
  }

  void output_bodyframe(astra_bodyframe_t bodyFrame)
  {
    // TBA:   methods to get some reference points
    output_bodies(bodyFrame);
  }

  void PrintBodyStatus(
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

  void PrintBasicTrackingInfo(
      int bodyId,
      astra_body_tracking_feature_flags_t features,
      astra_vector3f_t *centerOfMass)
  {
    if (bodyId != last_id_)
    {
      ROS_INFO("%s: detected person ID %d", _name.c_str(), bodyId);
      last_id_ = bodyId;
    }

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

  void PrintJointPositionDebugInfo(
      std::string header,
      geometry_msgs::Point32_<pointing_gesture::Skeleton> joint_position)
  {
    std::string info =
        header + ":" + std::to_string(joint_position.x) + "; " + std::to_string(joint_position.y) + "; " + std::to_string(joint_position.z);
    ROS_INFO_STREAM(info);
  }

  
  void runLoop()
  {
    set_key_handler();

    //  2022-04-23 18:46:05,946 ERROR [orbbec.ni.device_streamset]
    //  failed to open device: 	Could not open "2bc5/0401@1/13": Failed to set USB interface!

    //  initialization cannot be skipped -> rc = 7
    astra_initialize();

    const char *licenseString = "<INSERT LICENSE KEY HERE>";
    orbbec_body_tracking_set_license(licenseString);

    astra_streamsetconnection_t sensor;
    astra_streamset_open("device/default", &sensor);

    astra_reader_t reader;
    astra_reader_create(sensor, &reader);

    astra_bodystream_t bodyStream;
    astra_reader_get_bodystream(reader, &bodyStream);

    astra_stream_start(bodyStream);
    do
    {
      //  read skeleton data
      astra_update();
      astra_reader_frame_t frame;
      astra_status_t rc = astra_reader_open_frame(reader, 0, &frame);

      if (rc == ASTRA_STATUS_SUCCESS)
      {
        astra_bodyframe_t bodyFrame;
        astra_frame_get_bodyframe(frame, &bodyFrame);

        astra_frame_index_t frameIndex;
        astra_bodyframe_get_frameindex(bodyFrame, &frameIndex);

        output_bodyframe(bodyFrame);
        astra_reader_close_frame(&frame);
      }

      //  read depth data

      ros::spinOnce();

    } while (shouldContinue);

    astra_reader_destroy(&reader);
    astra_streamset_close(&sensor);

    astra_terminate();
  }

private:
  /////////////// DATA MEMBERS /////////////////////

  std::string _name;
  ros::NodeHandle nh_;
  std::string myparm1_;
  int last_id_;

  ros::Publisher body_tracking_position_pub_;
  ros::Publisher body_tracking_skeleton_pub_;
  ros::Publisher marker_pub_;
};

// The main entry point for this node.
int main(int argc, char *argv[])
{
  ros::init(argc, argv, "astra_body_tracker");
  astra_body_tracker_node node(ros::this_node::getName());
  node.runLoop();
  // ros::spin();
  return 0;
}
