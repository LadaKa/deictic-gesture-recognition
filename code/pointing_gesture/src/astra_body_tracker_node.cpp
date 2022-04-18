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
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include <sstream>
#include "ros/console.h"
#include <string>
#include "geometry_msgs/PoseStamped.h"

// For Orbbec Astra SDK
#include <astra/capi/astra.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <key_handler.h>

#include "BodyTracker.h" // Publish custom message
#include "Skeleton.h"    // Publish custom message

#include <visualization_msgs/Marker.h>

#define KEY_JOINT_TO_TRACK ASTRA_JOINT_SHOULDER_SPINE
//#define M_PI                  3.1415926535897931

class astra_body_tracker_node
{
public:
  astra_body_tracker_node(std::string name) : _name(name)
  {
    ROS_INFO("Hallo Spaceboy!");
    ROS_INFO("%s: Initializing", _name.c_str());
    bool initialized = false;
    last_id_ = -1;

    ros::NodeHandle nodeHandle("~");
    nodeHandle.param<std::string>("myparm1", myparm1_, "mydefault");

    // PUBLISHERS
    // Publish tracked person in 2D and 3D
    // 2D: x,y in camera frame.   3D: x,y,z in world coordinates
    body_tracking_position_pub_ = nh_.advertise<pointing_gesture::BodyTracker>("body_tracker/position", 1);

    // Publish tracked person upper body skeleton for advanced uses
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
      //PrintBodyStatus(bodyId, bodyStatus);
      //PrintBasicTrackingInfo(bodyId, body->features, &body->centerOfMass);

      // THIS IS THE MOST RELIABLE TRACKING POINT, so we use it for person position in 3D!
      astra_joint_t *keyJoint = &body->joints[KEY_JOINT_TO_TRACK];

      pointing_gesture::BodyTracker_<pointing_gesture::BodyTracker> position_data;
      Set2DPositionDataByKeyJoint(bodyId, bodyStatus, keyJoint, position_data);

      ///////////////////////////////////////////////////////////////
      // 3D position of person
      // TODO:
      position_data.position3d.x = ((astra_vector3f_t *)&keyJoint->worldPosition)->z / 1000.0;
      position_data.position3d.y = ((astra_vector3f_t *)&keyJoint->worldPosition)->x / 1000.0;
      position_data.position3d.z = ((astra_vector3f_t *)&keyJoint->worldPosition)->y / 1000.0;

      ///////////////////////////////////////////////////////////////
      // Skeleton data - published in skeleton message

      /// skeleton_data.frame_id = "astra_camera_link"; // "base_link";  // not sure about this
      pointing_gesture::Skeleton_<pointing_gesture::Skeleton> skeleton_data;
      skeleton_data.body_id = bodyId;
      skeleton_data.tracking_status = bodyStatus;

      //  Skeleton joints
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

      // head:
      PublishSphereMarker(3, skeleton_data.joint_position_head, 0.3, 0.3, 0.6);

      // spine:
      PublishSphereMarker(4, skeleton_data.joint_position_spine_top, 0.3, 0.3, 0.4);
      PublishSphereMarker(5, skeleton_data.joint_position_spine_mid, 0.3, 0.3, 0.4);
      PublishSphereMarker(6, skeleton_data.joint_position_spine_bottom, 0.3, 0.3, 0.4);

      // left arm:
      PublishSphereMarker(7, skeleton_data.joint_position_left_shoulder, 0.5, 0.1, 0.1);
      PublishSphereMarker(8, skeleton_data.joint_position_left_elbow, 0.5, 0.1, 0.1);
      PublishSphereMarker(9, skeleton_data.joint_position_left_hand, 0.5, 0.1, 0.1);

      // rigth arm:
      PublishSphereMarker(10, skeleton_data.joint_position_right_shoulder, 0.1, 0.1, 0.5);
      PublishSphereMarker(11, skeleton_data.joint_position_right_elbow, 0.1, 0.1, 0.5);
      PublishSphereMarker(12, skeleton_data.joint_position_right_hand, 0.1, 0.1, 0.5);

      // Skeleton

      // spine:
      geometry_msgs::Point32_<pointing_gesture::Skeleton> spinePositions[]{
          skeleton_data.joint_position_head,
          skeleton_data.joint_position_spine_top,
          skeleton_data.joint_position_spine_mid,
          skeleton_data.joint_position_spine_bottom};
      PublishLinesMarkers(13, spinePositions, 4, 0.4, 0.4, 0.4);

      // left arm:
      geometry_msgs::Point32_<pointing_gesture::Skeleton> leftArmPositions[]{
          skeleton_data.joint_position_left_shoulder,
          skeleton_data.joint_position_left_elbow,
          skeleton_data.joint_position_left_hand};
      PublishLinesMarkers(14, leftArmPositions, 3, 0.4, 0.4, 0.4);

      // right arm:
      geometry_msgs::Point32_<pointing_gesture::Skeleton> rightArmPositions[]{
          skeleton_data.joint_position_right_shoulder,
          skeleton_data.joint_position_right_elbow,
          skeleton_data.joint_position_right_hand};
      PublishLinesMarkers(15, rightArmPositions, 3, 0.4, 0.4, 0.4);

      ////////////////////////////////////////////////////
      // Publish messages
      body_tracking_position_pub_.publish(position_data); // position data
      body_tracking_skeleton_pub_.publish(skeleton_data); // full skeleton data
    }
  }

  // cube marker for objects
  void PublishCubeMarker(
      int id, geometry_msgs::Point32_<pointing_gesture::Skeleton> position,
      float color_r, float color_g, float color_b)
  {
    PublishPointMarker(id, position, color_r, color_g, color_b, visualization_msgs::Marker::CUBE);
  }

  // sphere marker for skeleton joints
  void PublishSphereMarker(
      int id, geometry_msgs::Point32_<pointing_gesture::Skeleton> position,
      float color_r, float color_g, float color_b)
  {
    PublishPointMarker(id, position, color_r, color_g, color_b, visualization_msgs::Marker::SPHERE);
  }

  // TODO: refactor -> rViz markers as class; markers setting for all types as separate method
  void PublishPointMarker(
      int id,
      geometry_msgs::Point32_<pointing_gesture::Skeleton> position,
      float color_r, float color_g, float color_b,
      uint32_t shape)
  {
    // Display marker for RVIZ to show where robot thinks person is
    // For Markers info, see http://wiki.ros.org/rviz/Tutorials/Markers%3A%20Basic%20Shapes

    visualization_msgs::Marker marker;
    marker.header.frame_id = "astra_camera_link"; // "base_link";
    marker.header.stamp = ros::Time::now();
    marker.lifetime = ros::Duration(1.0); // seconds

    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "astra_body_tracker";
    marker.id = id; // This must be id unique for each marker

    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;
    marker.color.r = color_r;
    marker.color.g = color_g;
    marker.color.b = color_b;
    marker.color.a = 1.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.07; // size of marker in meters
    marker.scale.y = 0.07;
    marker.scale.z = 0.07;

    marker.pose.position.x = position.x;
    marker.pose.position.y = position.y;
    marker.pose.position.z = position.z;

    marker_pub_.publish(marker);
    PrintJointPositionDebugInfo("PublishPointMarker", position);
  }

  //  TODO: code properly
  void PublishLinesMarkers(
      int id,
      geometry_msgs::Point32_<pointing_gesture::Skeleton> positions[],
      int positions_count,
      float color_r, float color_g, float color_b)
  {
    visualization_msgs::Marker line_list;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.id = id; // This must be id unique for each marker
    line_list.header.frame_id = "astra_camera_link"; 
    line_list.header.stamp = ros::Time::now();
    line_list.ns = "astra_body_tracker";
    line_list.lifetime = ros::Duration(1.0); // seconds
    line_list.action = visualization_msgs::Marker::ADD;

    line_list.scale.x = 0.05;
    line_list.color.r = color_r;
    line_list.color.g = color_g;
    line_list.color.b = color_b;
    line_list.color.a = 0.5;

    /*
    line_list.pose.orientation.x = 0.0;
    line_list.pose.orientation.y = 0.0;
    line_list.pose.orientation.z = 0.0;
    line_list.pose.orientation.w = 1.0;

    line_list.pose.position.x = positions[0].x;
    line_list.pose.position.y = positions[0].y;
    line_list.pose.position.z = positions[0].z;
    */
   
    for (int i = 0; i < positions_count - 1; i++)
    {
      geometry_msgs::Point32_<pointing_gesture::Skeleton> position0 = positions[i];
      geometry_msgs::Point32_<pointing_gesture::Skeleton> position1 = positions[i + 1];

      geometry_msgs::Point p0;
      p0.x = position0.x;
      p0.y = position0.y;
      p0.z = position0.z;

      geometry_msgs::Point p1;
      p1.x = position1.x;
      p1.y = position1.y;
      p1.z = position1.z;

      line_list.points.push_back(p0);
      line_list.points.push_back(p1);

      ROS_INFO_STREAM(std::to_string(i));
      PrintJointPositionDebugInfo("x", position0);
      PrintJointPositionDebugInfo("x", position0);
    }

    marker_pub_.publish(line_list);
  }

  void SetJointPositionByWorldPosition(
      astra_body_t *body,
      _astra_joint_type joint_type,
      geometry_msgs::Point32_<pointing_gesture::Skeleton> &joint_position)
  {
    astra_joint_t *joint = &body->joints[joint_type];
    joint_position.x = ((astra_vector3f_t *)&joint->worldPosition)->z / 1000.0; // why so weird?
    joint_position.y = ((astra_vector3f_t *)&joint->worldPosition)->x / 1000.0;
    joint_position.z = ((astra_vector3f_t *)&joint->worldPosition)->y / 1000.0;
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

  void Set2DPositionDataByKeyJoint(
      int bodyId,
      int bodyStatus,
      astra_joint_t *keyJoint,
      pointing_gesture::BodyTracker_<pointing_gesture::BodyTracker> &position_data)
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
    // TODO: TUNE THESE VALUSE AS NEEDED FOR YOUR CAMERA AND APPLICATION!

    float projection_x = ((astra_vector2f_t *)&keyJoint->depthPosition)->x / 1000.0;
    float projection_y = ((astra_vector2f_t *)&keyJoint->depthPosition)->y / 1000.0;

    position_data.position2d.x = (projection_x - 0.314) * ASTRA_MINI_FOV_X;
    position_data.position2d.y = (projection_y - 0.314) * ASTRA_MINI_FOV_Y;
    position_data.position2d.z = 0.0;

    std::cout << std::setprecision(4) << std::setw(7)
              << "Astra: "
              << "2D Tracking for ID "
              << bodyId << " :  "
              << " px: " << projection_x
              << " py: " << projection_y
              << std::endl;

    std::cout << std::setprecision(4) << std::setw(7)
              << " x: " << position_data.position2d.x
              << " y: " << position_data.position2d.y
              << " z: " << position_data.position2d.z
              << std::endl;
  }

  void runLoop()
  {
    set_key_handler();
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
      astra_update();

      astra_reader_frame_t frame;
      astra_status_t rc = astra_reader_open_frame(reader, 0, &frame);

      if (rc == ASTRA_STATUS_SUCCESS)
      {
        astra_bodyframe_t bodyFrame;
        astra_frame_get_bodyframe(frame, &bodyFrame);

        astra_frame_index_t frameIndex;
        astra_bodyframe_get_frameindex(bodyFrame, &frameIndex);
        // printf("Frame index: %d\n", frameIndex);

        output_bodyframe(bodyFrame);

        // printf("----------------------------\n");

        astra_reader_close_frame(&frame);
      }

      ros::spinOnce(); // ROS

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
