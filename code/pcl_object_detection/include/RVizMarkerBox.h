#ifndef RVIZMARKERBOX_H
#define RVIZMARKERBOX_H

#include <visualization_msgs/Marker.h>

class RVizMarkerBox
{
private:
    visualization_msgs::Marker marker;

public:
    visualization_msgs::Marker GetMarker();

    RVizMarkerBox(
        std::string frame_id,
        int id, 
        float x, float y, float z,
        float size_x, float size_y, float size_z,
        float color_r, float color_g, float color_b);

    // creates rVizMarkerBox with modified color of marker
    RVizMarkerBox(
        visualization_msgs::Marker marker,
        float color_r, float color_g, float color_b);
};

#endif