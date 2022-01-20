// Modified 'astra_body_tracker_node.cpp'
// https://github.com/shinselrobots/astra_body_tracker

/*
  Pointing Gesture Node - TIMEOUT TEST 

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
// TODO:  Set paths correctly.
#include </home/ladak/Desktop/AstraSDK/samples/common/key_handler.h>
#include </home/ladak/Desktop/AstraSDK/include/astra/capi/astra_ctypes.h>
#include </home/ladak/Desktop/AstraSDK/include/astra/capi/streams/body_capi.h>
#include </home/ladak/Desktop/AstraSDK/include/astra_core/capi/astra_types.h>
#include </home/ladak/Desktop/AstraSDK/include/astra_core/capi/astra_core.h>

// TODO: create and include msg package (instead of including generated headers)

// Publish custom message
#include "PointingArmJoints.h"

#define KEY_JOINT_TO_TRACK ASTRA_JOINT_SHOULDER_SPINE

class TestTimeoutNode
{

public:
    /////////////// DATA MEMBERS /////////////////////

    std::string _name;


    TestTimeoutNode(std::string name) : _name(name)
    {
        ROS_INFO("Initializing node.");

        bool initialized = false;
    }


    //////////////////////////////////////////////////////////
    // Modified Orbec Astra sample code / Shinsel Robots code

    void runLoop()
    {
        set_key_handler();
        astra_initialize();
        const char *licenseString = "<INSERT LICENSE KEY HERE>";
        orbbec_body_tracking_set_license(licenseString);

		std::string state;

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
			state = get_state_as_string(rc);

            if (rc == ASTRA_STATUS_SUCCESS)
            {
				ROS_INFO_STREAM(state);
				// shouldContinue = false;
            }
            else
            {
                ROS_INFO_STREAM(state);
            }

            ros::spinOnce();

        } while (shouldContinue);

        astra_reader_destroy(&reader);
        astra_streamset_close(&sensor);

        astra_terminate();
    }


private:

std::string get_state_as_string(astra_status_t status)
{
    switch (status)
    {
    case ASTRA_STATUS_SUCCESS:
        return "ASTRA_STATUS_SUCCESS";
        
    case ASTRA_STATUS_INVALID_PARAMETER :
        return "ASTRA_STATUS_INVALID_PARAMETER";

    case ASTRA_STATUS_DEVICE_ERROR:
        return "ASTRA_STATUS_DEVICE_ERROR ";

    case ASTRA_STATUS_TIMEOUT:
        return "ASTRA_STATUS_TIMEOUT";

    case ASTRA_STATUS_INVALID_PARAMETER_TOKEN:
        return "ASTRA_STATUS_INVALID_PARAMETER_TOKEN";

    case ASTRA_STATUS_INVALID_OPERATION:
        return "ASTRA_STATUS_INVALID_OPERATION";

    case  ASTRA_STATUS_INTERNAL_ERROR:
        return "ASTRA_STATUS_INTERNAL_ERROR";
    
    default:
       return "ASTRA_STATUS_UNINITIALIZED";
    }
}
 
};

// The main entry point for this node.
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "pointing_gesture");
    TestTimeoutNode node(ros::this_node::getName());
    node.runLoop();
    return 0;
};
