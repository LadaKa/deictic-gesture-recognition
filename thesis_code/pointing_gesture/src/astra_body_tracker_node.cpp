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
#include <unistd.h>

#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>

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

// added to handle build errors on ACER

#include <astra/capi/streams/body_capi.h>
#include <astra/capi/streams/body_types.h>

// custom message
#include "BodyTracker.h"

// rViz utils
#include "RVizLineMarker.h"
#include "RVizPointMarker.h"
#include "RVizPublisher.h"

// console output
#include "OutputUtils.h"

// basic tracked objects
#include "TrackedPerson.h"
#include "TrackedSkeleton.h"

// gesture
#include "PointingGesture.h"

class astra_body_tracker_node
{
public:

  astra_body_tracker_node(std::string name) : _name(name)
  {
    PrintRosInfo("Initializing.");
    bool initialized = false;

    ros::NodeHandle nodeHandle("~");
    nodeHandle.param<std::string>("myparm1", myparm1_, "mydefault");

    // SUBSCRIBERS

    // hotfix for ros_astra_camera
    sub_object_detection_done = nh_.subscribe(
        "camera/object_detection_done",
        1,
        &astra_body_tracker_node::object_detection_done_cb, this);

    // ui:
    // rostopic pub /gui/pointing_upper_joint std_msgs/Int32 "data: 2" 
    sub_pointing_upper_joint = nh_.subscribe(
        "gui/pointing_upper_joint",
        1,
        &astra_body_tracker_node::pointing_upper_joint_cb, this);

    // rostopic pub /gui/visible_pointing_ray std_msgs/Bool "data: False" 
    sub_visible_pointing_ray = nh_.subscribe(
        "gui/visible_pointing_ray",
        1,
        &astra_body_tracker_node::visible_pointing_ray_cb, this);

    // PUBLISHERS
    // Publish tracked person in 2D and 3D
    // 2D: x,y in camera frame.   3D: x,y,z in world coordinates
    body_tracking_position_pub_ = nh_.advertise<pointing_gesture::BodyTracker>("body_tracker/position", 1);

    // Tracked person upper body skeleton
    body_tracking_skeleton_pub_ = nh_.advertise<pointing_gesture::Skeleton>("body_tracker/skeleton", 1);

    // Publish markers to show where robot thinks person is in RViz
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("body_tracker/marker", 1);

    // default pointing upper joint is the head joint
    current_upper_joint = head;
  }

  ~astra_body_tracker_node()
  {
    PrintRosInfo("Shutting down.");
  }

  void output_bodies_with_gestures(astra_bodyframe_t bodyFrame)
  {
	  astra_body_list_t bodyList;
    const astra_status_t rc = astra_bodyframe_body_list(bodyFrame, &bodyList);
    if (rc != ASTRA_STATUS_SUCCESS)
    {
      ROS_INFO("Error %d in astra_bodyframe_body_list()\n", rc);
      return;
    }

    for (int i = 0; i < bodyList.count; ++i)
    {
      astra_body_t *body = &bodyList.bodies[i];

      // person position
      TrackedPerson person(body);
      body_tracking_position_pub_.publish(person.GetPositionData());

      // full skeleton data of person
      TrackedSkeleton trackedSkeleton(body);
      pointing_gesture::Skeleton_<pointing_gesture::Skeleton> skeleton_data = trackedSkeleton.GetSkeleton();
      body_tracking_skeleton_pub_.publish(skeleton_data); // full skeleton data

      RVizPublisher rVizPublisher(marker_pub_);
      rVizPublisher.PublishSkeleton(skeleton_data);
     
      geometry_msgs::Point32_<pointing_gesture::Skeleton> current_upper_joint_position = getPointingUpperJointPosition(skeleton_data);

      // confirmed pointing gesture?
      if (
        // left hand raised
        (skeleton_data.joint_position_left_hand.z > skeleton_data.joint_position_head.z) 
        && 
        // rigth arm pointing downward
        (current_upper_joint_position.z > skeleton_data.joint_position_right_hand.z))
      {

       // trackedSkeleton.PrintAllJointsPositions();

       // PointingGesture::current_upper_joint = PointingGesture::r_elbow;
        PointingGesture pointingGesture(
          current_upper_joint_position,
          skeleton_data.joint_position_right_hand,
          skeleton_data.joint_position_right_foot);

        PrintRosInfo("Gesture confirmed");
        pointingPerson = &person;

        pointingPerson->SetPointingGesture(pointingGesture);
        pointingPerson->SetPointingTrackedSkeleton(trackedSkeleton);
        if (!firstPointingGestureDetected)
        {
          firstPointingGestureDetected = true;
        }
        else 
        {
          secondPointingGestureDetected = true;
        }

        geometry_msgs::Point32 intersectionMsg = pointingGesture.GetIntersectionMessage();
        ros::Publisher intersectionPub = nh_.advertise<geometry_msgs::Point32>("body_tracker/intersection", 1);

        // publish current gesture for 5 seconds        
        int repeatCounter = 5;
        do
        {
          rVizPublisher.PublishSkeleton(skeleton_data);
          rVizPublisher.PublishPointingGesture(
              &pointingGesture);
          
          intersectionPub.publish(intersectionMsg);
          ros::spinOnce();
          repeatCounter = repeatCounter - 1;
          sleep(1);
        } while (shouldContinue && ((repeatCounter > 0))); 
        if (secondPointingGestureDetected)
        {
            shouldContinue = false;
        }
        
      }
      else if (
        show_pointing_ray
        &&
        // rigth arm pointing downward
        (current_upper_joint_position.z > skeleton_data.joint_position_right_hand.z))
      {
        PointingGesture pointingGesture(
          current_upper_joint_position,
          skeleton_data.joint_position_right_hand,
          skeleton_data.joint_position_right_foot);

        pointingPerson = &person;

        pointingPerson->SetPointingGesture(pointingGesture);
        pointingPerson->SetPointingTrackedSkeleton(trackedSkeleton);
        rVizPublisher.PublishSkeleton(skeleton_data);
        rVizPublisher.PublishPointingGesture(
              &pointingGesture);
      }
      
    }
  }

  // grip detection doesn't work as expected 
  bool detect_both_hands_grip(const astra_body_t *body)
  {
    const astra_handpose_info_t *handPoses = &body->handPoses;

    // astra_handpose_t is one of:
    // ASTRA_HANDPOSE_UNKNOWN = 0
    // ASTRA_HANDPOSE_GRIP = 1
    const astra_handpose_t leftHandPose = handPoses->leftHand;
    const astra_handpose_t rightHandPose = handPoses->rightHand;

    return ((leftHandPose == 1) || (rightHandPose == 1));
  }

  void output_frame(astra_bodyframe_t bodyFrame)
  {
    if (!floorDetected && try_output_floor(bodyFrame, &floorPlane))
    {
      floorDetected = true;
    };
    output_bodies_with_gestures(bodyFrame);
  }

  void runLoop()
  {
    set_key_handler();
    do
    {
      ros::spinOnce();

    } while (shouldContinue && !objectsDetected);

    sleep(5); //sleeps for 5 second

    if (shouldContinue)
    {
      runAstraStreamLoop();
    }
  }

private:

  enum pointing_upper_joint
  { 
      r_elbow, 
      r_shoulder, 
      head
  } current_upper_joint;
  
  bool show_pointing_ray = true;

  geometry_msgs::Point32_<pointing_gesture::Skeleton> getPointingUpperJointPosition(
    pointing_gesture::Skeleton_<pointing_gesture::Skeleton> skeleton)
  {
    switch (current_upper_joint)
    {
        case r_elbow:
            return skeleton.joint_position_right_elbow;
        case r_shoulder:
            return skeleton.joint_position_right_shoulder;
        case head:
            return skeleton.joint_position_head;
        default:
            return skeleton.joint_position_right_elbow;
    }
  }

  void setCurrentUpperJoint(int index)
  {
    switch (index)
    {
        case 0:
            current_upper_joint = r_elbow;
            return;
        case 1:
            current_upper_joint = r_shoulder;
            return;
        case 2:
            current_upper_joint = head;
            return;
        default:
        ROS_INFO("Invalid upper joint value selected. Valid values: 0, 1, 2.");
            return;
    }
  }


  // only for debugging
  // height over floor issue
  // https://3dclub.orbbec3d.com/t/calculating-height-over-floor/1225/6
  bool try_output_floor(astra_bodyframe_t bodyFrame, astra_plane_t *floorPlane)
  {
    astra_floor_info_t floorInfo;

    astra_status_t rc = astra_bodyframe_floor_info(bodyFrame, &floorInfo);
    if (rc != ASTRA_STATUS_SUCCESS)
    {
      return false;
    }

    const astra_bool_t floorDetected = floorInfo.floorDetected;
    floorPlane = &floorInfo.floorPlane;
    const astra_floormask_t *floorMask = &floorInfo.floorMask;

    if (floorDetected != ASTRA_FALSE)
    {
      printf("\t\tFloor plane: [%f, %f, %f, %f]\n",
             floorPlane->a,
             floorPlane->b,
             floorPlane->c,
             floorPlane->d);

      const int32_t bottomCenterIndex = floorMask->width / 2 + floorMask->width * (floorMask->height - 1);
      /*printf("\t\tFloor mask: width: %d height: %d bottom center value: %d\n\n",
             floorMask->width,
             floorMask->height,
             floorMask->data[bottomCenterIndex]);*/
      return true;
    }
    return false;
  }

  void object_detection_done_cb(const std_msgs::Empty::ConstPtr &msg)
  {
    PrintRosInfo(
        "Received object_detection_done MSG");
    objectsDetected = true;
  }

  void pointing_upper_joint_cb(const std_msgs::Int32 &msg)
  {
    PrintRosInfo(
        "Received pointing_upper_joint MSG");
    setCurrentUpperJoint(msg.data);
  }

  void visible_pointing_ray_cb(const std_msgs::Bool::ConstPtr& msg)
  {
    PrintRosInfo(
        "Received visible_pointing_ray MSG");
    show_pointing_ray = msg->data;
  }

  // starts after receiving 'object_detection_done' msg
  void runAstraStreamLoop()
  {
    astra_streamsetconnection_t sensor;
    astra_reader_t reader;
    astra_bodystream_t bodyStream;



    // handle delayed ROS Astra Device stream termination
    bool astra_stream_started = false;
    do
    {
      astra_stream_started = tryStartAstraStream(
          sensor, reader, bodyStream);
    } while (
        shouldContinue &&
        !astra_stream_started);

    // read data from stream
    do
    {
      //  read skeleton data
      
      astra_update();
      astra_reader_frame_t frame;
      astra_status_t rc = astra_reader_open_frame(reader, 500, &frame);

      if (rc == ASTRA_STATUS_SUCCESS)
      {
        astra_bodyframe_t bodyFrame;
        astra_frame_get_bodyframe(frame, &bodyFrame);

        astra_frame_index_t frameIndex;
        astra_bodyframe_get_frameindex(bodyFrame, &frameIndex);

        output_frame(bodyFrame);
        astra_reader_close_frame(&frame);
      }
      else
      {
    	  PrintRosInfo("FAIL");
    	  printf("ASTRA_STATUS: %d \n", rc);

      }
      //  read depth data if needed
      //  ...

      ros::spinOnce();

    } while (
        shouldContinue);

    astra_reader_destroy(&reader);
    astra_streamset_close(&sensor);

    astra_terminate();
  }

  bool tryStartAstraStream(
      astra_streamsetconnection_t &sensor,
      astra_reader_t &reader,
      astra_bodystream_t &bodyStream)
  {
    try
    {
      //  initialization cannot be skipped -> rc = 7
      astra_initialize();
      astra_streamset_open("device/default", &sensor);
      astra_reader_create(sensor, &reader);
      astra_reader_get_bodystream(reader, &bodyStream);
      astra_stream_start(bodyStream);
      PrintRosInfo(
          "Starting Astra Stream");

      return true;
    }
    catch (...)
    {
      // delayed ROS Astra Device stream termination
      PrintRosInfo(
          "Astra Stream unavailable.");
      return false;
    }
  }

  void PrintRosInfo(std::string info)
  {
    ROS_INFO("ASTRA BODY TRACKER: %s.", info.c_str());
  }

  /////////////// DATA MEMBERS /////////////////////
  bool objectsDetected = false;
  bool floorDetected = false;

  bool firstPointingGestureDetected = false;
  bool secondPointingGestureDetected = false;

  TrackedPerson *pointingPerson;

  astra_plane_t floorPlane;

  std::string _name;
  ros::NodeHandle nh_;
  std::string myparm1_;

  ros::Publisher body_tracking_position_pub_;
  ros::Publisher body_tracking_skeleton_pub_;
  ros::Publisher marker_pub_;

  ros::Subscriber sub_object_detection_done;
  ros::Subscriber sub_pointing_upper_joint;
  ros::Subscriber sub_visible_pointing_ray;
};

// The main entry point for this node.
int main(int argc, char *argv[])
{
  ros::init(argc, argv, "astra_body_tracker");
  astra_body_tracker_node node(ros::this_node::getName());
  node.runLoop();
  return 0;
}