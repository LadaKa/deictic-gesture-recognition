// Modified 'astra_body_tracker_node.cpp'
// https://github.com/shinselrobots/astra_body_tracker

/*
  Pointing Gesture Node

  So far publishes only one message: 

  1.  PointingArmJoints.msg
      - 3D positions of selected joints (shoulder and wrist) of right arm 

*/


#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include <sstream>
#include "ros/console.h"
#include "geometry_msgs/PoseStamped.h"

//For Orbbec Astra SDK
#include <astra/capi/astra.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <key_handler.h>

//#include "PointingArmJoints.h"     // Publish custom message


#define KEY_JOINT_TO_TRACK    ASTRA_JOINT_SHOULDER_SPINE 

class pointing_gesture_node 
{
public:
  pointing_gesture_node(std::string name) :
    _name(name)
  {
    ROS_INFO("%s: Initializing", _name.c_str());
    bool initialized = false;
    last_id_ = -1;

    ros::NodeHandle nodeHandle("~");
    nodeHandle.param<std::string>("myparm1",myparm1_,"mydefault");

    // Subscribers
    //robot_behavior_state_ = nh_.subscribe("/behavior/cmd", 1, &behavior_logic_node::behaviorStateCB, this);

    
    // Publishers 
    //body_tracking_position_pub_ = nh_.advertise<body_tracker_msgs::BodyTracker>
    //  ("body_tracker/position", 1); 

    ROS_INFO("pointing_gesture_node: Advertised Publisher: ..TODO..");

  }

  ~pointing_gesture_node()
  {
    ROS_INFO("pointing_gesture_node shutting down");
  }



  //////////////////////////////////////////////////////////
  // Modified Orbec Astra sample code
  //////////////////////////////////////////////////////////
  // Modified Orbec Astra sample code

  void output_floor(astra_bodyframe_t bodyFrame)
  {
    astra_floor_info_t floorInfo;

    astra_status_t rc = astra_bodyframe_floor_info(bodyFrame, &floorInfo);
    if (rc != ASTRA_STATUS_SUCCESS)
    {
      printf("Error %d in astra_bodyframe_floor_info()\n", rc);
      return;
    }

    const astra_bool_t floorDetected = floorInfo.floorDetected;
    const astra_plane_t* floorPlane = &floorInfo.floorPlane;
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
    }
  }



  void output_body(astra_bodyframe_t bodyFrame)
  {
    int i;
    astra_body_list_t bodyList;
    const astra_status_t rc = astra_bodyframe_body_list(bodyFrame, &bodyList);
    if (rc != ASTRA_STATUS_SUCCESS)
    {
        printf("Error %d in astra_bodyframe_body_list().\n", rc);
        return;
    }

    if (bodyList.count > 1)
    {
        printf("Invalid input: multiple bodies are visible. \n");
        return;
    }

    // TODO:
    
      astra_body_t* body = &bodyList.bodies[i];
      int bodyId = (int)body->id; // astra_body_id_t 

      // Tracking status
      // NOT_TRACKING = 0
      // TRACKING_LOST = 1
      // TRACKING_STARTED = 2
      // TRACKING = 3

      astra_body_status_t bodyStatus = body->status;
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

      const astra_vector3f_t* centerOfMass = &body->centerOfMass;
      const astra_body_tracking_feature_flags_t features = body->features;
      astra_joint_t* joint;  // AstraSDK/include/astra/capi/streams/body_types.h

      const bool jointTrackingEnabled = 
        (features & ASTRA_BODY_TRACKING_JOINTS) == ASTRA_BODY_TRACKING_JOINTS;

      printf("Body %d CenterOfMass (%f, %f, %f)\n",
          bodyId, centerOfMass->x, centerOfMass->y, centerOfMass->z);
      printf("    Joint Tracking Enabled: %s    \n",
          jointTrackingEnabled       ? "True" : "False");

      ///////////////////////////////////////////////////////////////
      // Publish body tracking information, and display joint info for debug

      // Create structures for ROS Publisher data     
      // body_tracker_msgs::Skeleton_ <body_tracker_msgs::Skeleton> skeleton_data;

      // Skeleton Data for publishing more detail

      // position_data.body_id = bodyId;
      // position_data.tracking_status = bodyStatus;
      // position_data.gesture = -1; // No gesture yet


      ///////////////////////////////////////////////////////////////
      // 2D position for camera servo tracking
      const float ASTRA_MINI_FOV_X = 1.047200; // (60 degrees horizontal)
      const float ASTRA_MINI_FOV_Y = -0.863938; // (49.5 degrees vertical)


// TODO:
      // Convert projection to radians
      // Astra proj is 0.0 (right) --> 0.628 (left)
      //           and 0.0 (top)   --> 0.628 (botom)
      // NOTE: TUNE THESE VALUSE AS NEEDED FOR YOUR CAMERA AND APPLICATION!

      joint = &body->joints[KEY_JOINT_TO_TRACK];
      float projection_x = ((astra_vector2f_t*)&joint->depthPosition)->x / 1000.0;
      float projection_y = ((astra_vector2f_t*)&joint->depthPosition)->y / 1000.0;
  /*  position_data.position2d.x = (projection_x - 0.314) * ASTRA_MINI_FOV_X;
      position_data.position2d.y = (projection_y - 0.314) * ASTRA_MINI_FOV_Y;
      position_data.position2d.z = 0.0;

      std::cout << std::setprecision(4) << std::setw(7) 
        << "Astra: " << "2D Tracking for ID "
        << bodyId << " :  "  
        << " px: " << projection_x 
        << " py: " << projection_y
        << std::endl;

      std::cout << std::setprecision(4) << std::setw(7) 
        << " x: " << position_data.position2d.x 
        << " y: " << position_data.position2d.y
        << " z: " << position_data.position2d.z
        << std::endl;*/


      ///////////////////////////////////////////////////////////////
      // 3D position of person

      // *** POSITION 3D ***
      // THIS IS THE MOST RELIABLE TRACKING POINT, so we use it for person position in 3D!
      joint = &body->joints[KEY_JOINT_TO_TRACK];
      position_data.position3d.x = ((astra_vector3f_t*)&joint->worldPosition)->z / 1000.0;
      position_data.position3d.y = ((astra_vector3f_t*)&joint->worldPosition)->x / 1000.0;
      position_data.position3d.z = ((astra_vector3f_t*)&joint->worldPosition)->y / 1000.0;


      ///////////////////////////////////////////////////////////////
      // TODO: modify to pointing arm data

      // Skeleton data - published in skeleton message

      /// skeleton_data.frame_id = "astra_camera_link"; // "base_link";
      skeleton_data.body_id = bodyId;
      skeleton_data.tracking_status = bodyStatus;

      joint = &body->joints[ASTRA_JOINT_RIGHT_SHOULDER];
      // TODO: output_joint("Right Shoulder", bodyId, joint );
      skeleton_data.joint_position_right_shoulder.x = ((astra_vector3f_t*)&joint->worldPosition)->z / 1000.0;
      skeleton_data.joint_position_right_shoulder.y = ((astra_vector3f_t*)&joint->worldPosition)->x / 1000.0;
      skeleton_data.joint_position_right_shoulder.z = ((astra_vector3f_t*)&joint->worldPosition)->y / 1000.0;

      joint = &body->joints[ASTRA_JOINT_RIGHT_WRIST];
      // TODO: output_joint("Right Wrist", bodyId, joint );
      skeleton_data.joint_position_right_hand.x = ((astra_vector3f_t*)&joint->worldPosition)->z / 1000.0;
      skeleton_data.joint_position_right_hand.y = ((astra_vector3f_t*)&joint->worldPosition)->x / 1000.0;
      skeleton_data.joint_position_right_hand.z = ((astra_vector3f_t*)&joint->worldPosition)->y / 1000.0;

      ////////////////////////////////////////////////////
      // Publish everything
      // body_tracking_position_pub_.publish(position_data); // position data
      

  }

  void output_bodyframe(astra_bodyframe_t bodyFrame)
  {
    output_floor(bodyFrame);
    output_body(bodyFrame);
  }

  void runLoop()
  {
    set_key_handler();
    astra_initialize();
    const char* licenseString = "<INSERT LICENSE KEY HERE>";
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

        output_bodyframe(bodyFrame);

        //printf("----------------------------\n");

        astra_reader_close_frame(&frame);
      }

      ros::spinOnce();  // ROS

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

  // ros::Publisher body_tracking_position_pub_;
 

};


// The main entry point for this node.
int main( int argc, char *argv[] )
{
  ros::init( argc, argv, "astra_body_tracker" );
  pointing_gesture_node node(ros::this_node::getName());
  node.runLoop();

  return 0;
}


