// PCL Object Detection Node
// Detects objects resting on a Plane
// Currently, tested with objects on the floor

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <visualization_msgs/Marker.h>

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
//#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
#include <pcl/common/common.h>

#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/filters/voxel_grid.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <boost/lexical_cast.hpp>

#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>

#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
// NOTE: you must install TF2 Sensor Messages: sudo apt-get install ros-kinetic-tf2-sensor-msgs

#include "ObjectDetection.h"
#include "PointCloudPublishers.h"

////////////////////////////////////////////////////////////////////////////////////////////////////////
class PclObjectDetection
{
public:
  PclObjectDetection(ros::NodeHandle n);

private:
  // FUNCTIONS
  void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &input_cloud_msg);
  void PublishMarkerBox(
      std::string frame_id, int id, float x, float y, float z,
      float size_x, float size_y, float size_z,
      float color_r, float color_g, float color_b);

  // CONSTANTS

  const char *DEFAULT_TARGET_FRAME = "base_link"; // TF frame for sensors
  const double DEFAULT_TF_TOLERANCE = 0.05;       // TF latency tolerance
  const char *DEFAULT_DEPTH_TOPIC = "/camera/depth";
  // const char*   DEFAULT_COLOR_TOPIC = "/camera/color";

  // VARIABLES
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  tf2_ros::Buffer tf2_;
  tf2_ros::TransformListener tfListener_;
  std::string input_cloud_frame_;
  std::string target_frame_;
  double tf_tolerance_;
  std::string depth_topic_;

  // SUBSCRIBERS
  ros::Subscriber depth_cloud_sub_;

  // PUBLISHERS
  ros::Publisher pub_cluster0;
  ros::Publisher pub_cluster1;
  ros::Publisher pub_cluster2;
  ros::Publisher pub_cluster3;
  ros::Publisher pub_cluster4;
  ros::Publisher pub_nearest_object;

  ros::Publisher pub0;
  ros::Publisher pub1;
  ros::Publisher pub2;
  ros::Publisher pub3;
  ros::Publisher pub4;
  ros::Publisher pub5;

  ros::Publisher pub_remaining;
  ros::Publisher pub_objects;
  ros::Publisher marker_pub_;
};
////////////////////////////////////////////////////////////////////////////////////////////////////////

PclObjectDetection::PclObjectDetection(ros::NodeHandle n) : nh_(n),
                                                            private_nh_("~"),
                                                            tfListener_(tf2_),
                                                            input_cloud_frame_("")
{

  ROS_INFO("PclObjectDetection: Initializing...");

  // PARAMETERS
  // TODO USE THESE IN CODE
  private_nh_.param<std::string>("target_frame", target_frame_, DEFAULT_TARGET_FRAME);
  private_nh_.param<double>("transform_tolerance", tf_tolerance_, DEFAULT_TF_TOLERANCE);
  private_nh_.param<std::string>("depth_topic", depth_topic_, DEFAULT_DEPTH_TOPIC);
  // private_nh_.param<std::string>("color_topic", color_topic_, DEFAULT_COLOR_TOPIC);

  // PUBLISHERS
  // Create a ROS publisher for the output point cloud
  pub_cluster0 = nh_.advertise<sensor_msgs::PointCloud2>("pcl_object_detection/cluster0", 1);
  pub_cluster1 = nh_.advertise<sensor_msgs::PointCloud2>("pcl_object_detection/cluster1", 1);
  pub_cluster2 = nh_.advertise<sensor_msgs::PointCloud2>("pcl_object_detection/cluster2", 1);
  pub_cluster3 = nh_.advertise<sensor_msgs::PointCloud2>("pcl_object_detection/cluster3", 1);
  pub_cluster4 = nh_.advertise<sensor_msgs::PointCloud2>("pcl_object_detection/cluster4", 1);
  pub_nearest_object = nh_.advertise<sensor_msgs::PointCloud2>("pcl_object_detection/nearest_object", 1);

  pub0 = nh_.advertise<sensor_msgs::PointCloud2>("pcl_object_detection/plane0", 1);
  pub1 = nh_.advertise<sensor_msgs::PointCloud2>("pcl_object_detection/plane1", 1);
  pub2 = nh_.advertise<sensor_msgs::PointCloud2>("pcl_object_detection/plane2", 1);
  pub3 = nh_.advertise<sensor_msgs::PointCloud2>("pcl_object_detection/plane3", 1);
  pub4 = nh_.advertise<sensor_msgs::PointCloud2>("pcl_object_detection/plane4", 1);
  pub5 = nh_.advertise<sensor_msgs::PointCloud2>("pcl_object_detection/voxel_output", 1);

  pub_remaining = nh_.advertise<sensor_msgs::PointCloud2>("pcl_object_detection/remaining", 1);
  pub_objects = nh_.advertise<sensor_msgs::PointCloud2>("pcl_object_detection/objects", 1);

  // Create a ROS publisher for the output model coefficients
  // pub = nh_.advertise<pcl_msgs::ModelCoefficients> ("pcl_object_detection/segment_plane", 1);

  // Publish markers to show where robot thinks object is in RViz
  marker_pub_ = nh_.advertise<visualization_msgs::Marker>("pcl_object_detection/marker", 1);

  // SUBSCRIBERS
  // Create a ROS subscriber for the input point cloud
  depth_cloud_sub_ = nh_.subscribe(depth_topic_, 1, &PclObjectDetection::cloud_cb, this);

  ROS_INFO("PclObjectDetection: Initializing completed.");
}

void PclObjectDetection::cloud_cb(const sensor_msgs::PointCloud2ConstPtr &input_cloud_msg)
{
  ROS_INFO("PclObjectDetection: cloud_cb...");
  input_cloud_frame_ = input_cloud_msg->header.frame_id; // TF Frame of the point cloud
  ObjectDetection objectDetection();
  //  TODO
}

int main(int argc, char **argv)
{

  ROS_INFO("PclObjectDetection: Initializing ROS... ");
  ros::init(argc, argv, "pcl_object_detection");

  ros::NodeHandle n;
  // ros::NodeHandle n("~");
  PclObjectDetection pcl_object_detection_node(n);
  ros::spin();
  return 0;
}
