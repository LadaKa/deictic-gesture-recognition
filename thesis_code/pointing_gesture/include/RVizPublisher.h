#ifndef RVIZPUBLISHER_H
#define RVIZPUBLISHER_H

#include <geometry_msgs/Point32.h>

#include "Skeleton.h"
#include "PointingGesture.h"

#include "ros/ros.h"

class RVizPublisher
{

private:
    ros::Publisher marker_pub_;

    int published_ids;

    void PublishPointMarker(
        geometry_msgs::Point32_<pointing_gesture::Skeleton> position,
        float color_r, float color_g, float color_b,
        uint32_t shape);

    void PublishLinesMarkers(
        geometry_msgs::Point32_<pointing_gesture::Skeleton> positions[],
        int positions_count,
        float color_r, float color_g, float color_b);

    void PublishSphereMarker(
        geometry_msgs::Point32_<pointing_gesture::Skeleton> position,
        float color_r, float color_g, float color_b);

    void PublishJoints(pointing_gesture::Skeleton_<pointing_gesture::Skeleton> skeleton);

    void PublishBones(pointing_gesture::Skeleton_<pointing_gesture::Skeleton> skeleton);

public:
    RVizPublisher(ros::Publisher marker_pub_);

    void PublishSkeleton(pointing_gesture::Skeleton_<pointing_gesture::Skeleton> skeleton);

    void PublishPointingGesture(PointingGesture *gesture);
};

#endif