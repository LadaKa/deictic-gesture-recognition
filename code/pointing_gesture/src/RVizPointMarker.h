#ifndef RVIZPOINTMARKER_H
#define RVIZPOINTMARKER_H

#include <visualization_msgs/Marker.h>

class RVizPointMarker
{
private:
  visualization_msgs::Marker marker;

  void SetMarkerProperties(
      geometry_msgs::Point32_<pointing_gesture::Skeleton> position,
      float r, float g, float b,
      uint32_t shape);


public:
  RVizPointMarker(
      int id,
      geometry_msgs::Point32_<pointing_gesture::Skeleton> position,
      float r, float g, float b,
      uint32_t shape);

  visualization_msgs::Marker GetMarker();
};

#endif