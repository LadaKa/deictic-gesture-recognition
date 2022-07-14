#ifndef RVIZLINEMARKER_H
#define RVIZLINEMARKER_H

#include "geometry_msgs/PoseStamped.h"
#include <visualization_msgs/Marker.h>
#include "Skeleton.h"

class RVizLineMarker
{
private:
    visualization_msgs::Marker line_list;

    void SetLineListProperties(
        int id,
        float r, float g, float b);

    void SetLineListPoints(
        geometry_msgs::Point32_<pointing_gesture::Skeleton> positions[],
        int positions_count);

public:
    RVizLineMarker(
        int id,
        geometry_msgs::Point32_<pointing_gesture::Skeleton> positions[],
        int positions_count,
        float r, float g, float b);

    visualization_msgs::Marker GetLineList();
};

#endif
