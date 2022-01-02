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

// Orbbec Astra SDK
#include <astra/capi/astra.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <key_handler.h>

// TODO: create and include msg package (instead of including generated headers)

// Publish custom message
#include "PointingArmJoints.h"

#define KEY_JOINT_TO_TRACK ASTRA_JOINT_SHOULDER_SPINE

namespace pointing_gesture
{

    class pointing_gesture_node
    {

    public:

        pointing_gesture_node(std::string name) : _name(name)
        {
            ROS_INFO("%s: Initializing", _name.c_str());
            bool initialized = false;
            last_id_ = -1;

            // Node handle
            ros::NodeHandle nodeHandle("~");

            // Subscribers  - robot control

            // Publishers   - arm joints positions msg
            pointing_arm_joints_pub_ = nh_.advertise<PointingArmJoints>
              ("PointingArmJoints", 1); 
            ROS_INFO("pointing_gesture_node: Advertised Publisher: PointingArmJoints");
        }


        ~pointing_gesture_node()
        {
            ROS_INFO("pointing_gesture_node shutting down");
        };


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

            // TODO:
            // so far the camera and also all objects are in fixed location,
            // I should detect their position before the loop starts

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

                    astra_reader_close_frame(&frame);
                }

                ros::spinOnce(); // ROS

            } while (shouldContinue);

            astra_reader_destroy(&reader);
            astra_streamset_close(&sensor);

            astra_terminate();
        }

    private:
        void output_bodyframe(astra_bodyframe_t bodyFrame)
        {
            output_floor(bodyFrame);
            output_body(bodyFrame);
        }

        //////////////////////////////////////////////////////////
        // Modified Orbec Astra sample code / Shinsel Robots code

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
            const astra_plane_t *floorPlane = &floorInfo.floorPlane;
            const astra_floormask_t *floorMask = &floorInfo.floorMask;

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

        // TODO: simplify & refactor
        void output_body(astra_bodyframe_t bodyFrame)
        {

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

            astra_body_t *body = &bodyList.bodies[0];
            int bodyId = (int)body->id;

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

            const astra_vector3f_t *centerOfMass = &body->centerOfMass;
            const astra_body_tracking_feature_flags_t features = body->features;
            astra_joint_t *joint; // AstraSDK/include/astra/capi/streams/body_types.h

            const bool jointTrackingEnabled =
                (features & ASTRA_BODY_TRACKING_JOINTS) == ASTRA_BODY_TRACKING_JOINTS;
            const bool handPoseRecognitionEnabled =
                (features & ASTRA_BODY_TRACKING_HAND_POSES) == ASTRA_BODY_TRACKING_HAND_POSES;

            printf("Body %d CenterOfMass (%f, %f, %f)\n",
                   bodyId, centerOfMass->x, centerOfMass->y, centerOfMass->z);
            printf("    Joint Tracking Enabled: %s     Hand Pose Recognition Enabled: %s\n",
                   jointTrackingEnabled ? "True" : "False",
                   handPoseRecognitionEnabled ? "True" : "False");

            ///////////////////////////////////////////////////////////////
            // Publish body tracking information, and display joint info for debug

            // Create structure for ROS Publisher data


            // TODO:    Is the arm really pointing? Handle pointing detection using FSA (open hand / grasping)?
            PointingArmJoints_<PointingArmJoints> pointing_arm_joints;
            astra_joint_t *joint;

            joint = &body->joints[ASTRA_JOINT_RIGHT_ELBOW];
            pointing_arm_joints.joint_position_right_elbow.x = ((astra_vector3f_t *)&joint->worldPosition)->z / 1000.0;
            pointing_arm_joints.joint_position_right_elbow.y = ((astra_vector3f_t *)&joint->worldPosition)->x / 1000.0;
            pointing_arm_joints.joint_position_right_elbow.z = ((astra_vector3f_t *)&joint->worldPosition)->y / 1000.0;

            joint = &body->joints[ASTRA_JOINT_RIGHT_WRIST];
            pointing_arm_joints.joint_position_right_wrist.x = ((astra_vector3f_t *)&joint->worldPosition)->z / 1000.0;
            pointing_arm_joints.joint_position_right_wrist.y = ((astra_vector3f_t *)&joint->worldPosition)->x / 1000.0;
            pointing_arm_joints.joint_position_right_wrist.z = ((astra_vector3f_t *)&joint->worldPosition)->y / 1000.0;

            pointing_arm_joints_pub_.publish(pointing_arm_joints);
        }

        /////////////// DATA MEMBERS /////////////////////

        std::string _name;
        ros::NodeHandle nh_;
        ros::Publisher pointing_arm_joints_pub_;
        int last_id_; // there will be only one id
    };

    // The main entry point for this node.
    int main(int argc, char *argv[])
    {
        ros::init(argc, argv, "pointing_gesture");
        pointing_gesture_node node(ros::this_node::getName());
        node.runLoop();
        return 0;
    }

}