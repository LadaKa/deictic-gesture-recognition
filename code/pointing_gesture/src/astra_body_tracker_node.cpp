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
#include <std_msgs/Empty.h>

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

// rViz utils
#include "RVizLineMarker.h"
#include "RVizPointMarker.h"
#include "RVizPublisher.h"

// console output
#include "OutputUtils.h"

// basic tracked objects
#include "TrackedPerson.h"
#include "TrackedSkeleton.h"

class astra_body_tracker_node
{
public:
  astra_body_tracker_node(std::string name) : _name(name)
  {
    ROS_INFO("ASTRA_BODY_TRACKER: %s: Initializing node", _name.c_str());
    bool initialized = false;

    ros::NodeHandle nodeHandle("~");
    nodeHandle.param<std::string>("myparm1", myparm1_, "mydefault");

    // SUBSCRIBERS
    
    // hotfix for ros_astra_camera
    sub_object_detection_done = nh_.subscribe(
        "camera/object_detection_done",
        1,
        &astra_body_tracker_node::object_detection_done_cb, this);

    // PUBLISHERS
    // Publish tracked person in 2D and 3D
    // 2D: x,y in camera frame.   3D: x,y,z in world coordinates
    body_tracking_position_pub_ = nh_.advertise<pointing_gesture::BodyTracker>("body_tracker/position", 1);

    // Tracked person upper body skeleton
    body_tracking_skeleton_pub_ = nh_.advertise<pointing_gesture::Skeleton>("body_tracker/skeleton", 1);

    // Publish markers to show where robot thinks person is in RViz
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("body_tracker/marker", 1);

    ROS_INFO("ASTRA_BODY_TRACKER: Advertised Publisher: body_tracker/pose, skeleton, marker");
  }

  ~astra_body_tracker_node()
  {
    ROS_INFO("ASTRA_BODY_TRACKER: node shutting down");
  }

  void output_bodies(astra_bodyframe_t bodyFrame)
  {
    astra_body_list_t bodyList;
    const astra_status_t rc = astra_bodyframe_body_list(bodyFrame, &bodyList);
    if (rc != ASTRA_STATUS_SUCCESS)
    {
      printf("Error %d in astra_bodyframe_body_list()\n", rc);
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
    }
  }

  void output_frame(astra_bodyframe_t bodyFrame)
  {
   // astra_plane_t floorPlane;
   // if (try_output_floor(bodyFrame, &floorPlane))
   // {
      //  TODO: 
      //  use floor coords for pointing gesture processing
   // }
    output_bodies(bodyFrame);
  }

  void runLoop()
  {
    set_key_handler();

    ROS_INFO(
        "ASTRA_BODY_TRACKER: Starting Astra Loop");
    do
    {
      ros::spinOnce();

    } while (shouldContinue && !objectsDetected);

    if (shouldContinue)
    {
      runAstraStreamLoop();
    }
  }

private:

  bool try_output_floor(astra_bodyframe_t bodyFrame, astra_plane_t  *floorPlane)
  {
    astra_floor_info_t floorInfo;

    astra_status_t rc = astra_bodyframe_floor_info(bodyFrame, &floorInfo);
    if (rc != ASTRA_STATUS_SUCCESS)
    {
      printf("Error %d in astra_bodyframe_floor_info()\n", rc);
      return false;
    }

    const astra_bool_t floorDetected = floorInfo.floorDetected;
    floorPlane = &floorInfo.floorPlane;
    const astra_floormask_t* floorMask = &floorInfo.floorMask;

    if (floorDetected != ASTRA_FALSE)
    {
      printf("Floor plane: [%f, %f, %f, %f]\n",
             floorPlane->a,
             floorPlane->b,
             floorPlane->c,
             floorPlane->d);

      const int32_t bottomCenterIndex = floorMask->width / 2 + floorMask->width * (floorMask->height - 1);
      printf("Floor mask: width: %d height: %d bottom center value: %d\n",
            floorMask->width,
            floorMask->height,
            floorMask->data[bottomCenterIndex]);

      return true;
    }
    return false;
  }

  void object_detection_done_cb(const std_msgs::Empty::ConstPtr &msg)
  {
    ROS_INFO(
        "ASTRA_BODY_TRACKER: received object_detection_done MSG.");
    objectsDetected = true;
  }

  // starts after receiving 'object_detection_done' msg
  void runAstraStreamLoop()
  {
    ROS_INFO(
        "ASTRA_BODY_TRACKER: Starting Astra Loop");

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
      shouldContinue
      &&
      !astra_stream_started);
    
    // read data from stream
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

        output_frame(bodyFrame);
        astra_reader_close_frame(&frame);
      }

      //  read depth data if needed
      //  ...

      ros::spinOnce();

    } while (shouldContinue);

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
      ROS_INFO(
        "ASTRA_BODY_TRACKER: Astra Stream initialized");
      astra_streamset_open("device/default", &sensor);
      astra_reader_create(sensor, &reader);
      astra_reader_get_bodystream(reader, &bodyStream);
      astra_stream_start(bodyStream);
      ROS_INFO(
        "ASTRA_BODY_TRACKER: Starting Astra Stream");

      return true;
    }
    catch (...)
    {
      // delayed ROS Astra Device stream termination
      ROS_INFO(
        "ASTRA_BODY_TRACKER: Astra Stream Unavailable");
      return false;
    }

  }

/////////////// DATA MEMBERS /////////////////////
  bool objectsDetected = false;

  std::string _name;
  ros::NodeHandle nh_;
  std::string myparm1_;

  ros::Publisher body_tracking_position_pub_;
  ros::Publisher body_tracking_skeleton_pub_;
  ros::Publisher marker_pub_;

  ros::Subscriber sub_object_detection_done;
};

// The main entry point for this node.
int main(int argc, char *argv[])
{
  ros::init(argc, argv, "astra_body_tracker");
  astra_body_tracker_node node(ros::this_node::getName());
  node.runLoop();
  return 0;
}
