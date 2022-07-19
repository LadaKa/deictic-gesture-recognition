#ifndef OBJECTDETECTION_H
#define OBJECTDETECTION_H

//  TODO: clean includes
#include <ros/ros.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include <sensor_msgs/PointCloud2.h>

#include "PointCloudPublishers.h"

class ObjectDetection
{
private:
    PointCloudPublishers publishers;

public:
    void SetPublishers(PointCloudPublishers pcPublishers);

    void PublishMarkerBox(
        std::string frame_id,
        int id,
        float x, float y, float z,
        float size_x, float size_y, float size_z,
        float color_r, float color_g, float color_b);

    void ProcessPointCloud(
        const sensor_msgs::PointCloud2ConstPtr &input_cloud_msg,
        tf2_ros::Buffer tf2_,
        std::string target_frame,
        double tf_tolerance);
};

#endif
