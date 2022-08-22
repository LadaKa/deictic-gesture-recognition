
// PCL Object Detection Node
// Detects objects resting on a Plane
// Currently, tested with objects on the floor

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <visualization_msgs/Marker.h>

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

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

#include <geometry_msgs/Point32.h>
#include <std_msgs/Int32.h>

#include "DetectedObjects.h"
#include "ObjectsPublisher.h"
#include "ObjectDetection.h"

// NOTE: you must install TF2 Sensor Messages: sudo apt-get install ros-kinetic-tf2-sensor-msgs


class PclObjectDetection
{
public:
  PclObjectDetection(ros::NodeHandle n);
  void runLoop();


private:
  // FUNCTIONS
  void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &input_cloud_msg);
  void pointed_object_index_cb(const std_msgs::Int32 &index_msg);
  void PrintRosInfo(std::string info);

  // CONSTANTS

  const char *DEFAULT_TARGET_FRAME = "base_link"; // TF frame for sensors
  const double DEFAULT_TF_TOLERANCE = 0.05;       // TF latency tolerance
  const char *DEFAULT_DEPTH_TOPIC = "/depth";

  // VARIABLES
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  tf2_ros::Buffer tf2_;
  tf2_ros::TransformListener tfListener_;
  std::string input_cloud_frame_;
  std::string target_frame_;
  double tf_tolerance_;
  std::string depth_topic_;
  bool objectsDetected = false;
  bool pointedObjectSelected = false;
  ObjectDetection objectDetection;
  
  // SUBSCRIBERS
  ros::Subscriber depth_cloud_sub_;
  ros::Subscriber pointed_object_index_sub;

  // PUBLISHERS
  ros::Publisher pub_cluster0;
  ros::Publisher pub_cluster1;
  ros::Publisher pub_cluster2;
  ros::Publisher pub_nearest_object;

  ros::Publisher pub0;
  ros::Publisher pub1;
  ros::Publisher pub2;
  ros::Publisher pub3;
  ros::Publisher pub4;
  ros::Publisher pub_voxel;

  ros::Publisher pub_remaining;
  ros::Publisher pub_objects;
  ros::Publisher marker_pub_;

  ros::Publisher pub_detected_objects;

  PointCloudPublishers pcl_publishers;
  ObjectsPublisher objects_publisher;

  ros::Publisher pub_object_detection_done;
};

PclObjectDetection::PclObjectDetection(ros::NodeHandle n) : nh_(n),
                                                            private_nh_("~"),
                                                            tfListener_(tf2_),
                                                            input_cloud_frame_("")
{

  PrintRosInfo("Initializing...");

  // PARAMETERS
  // TODO USE THESE IN CODE
  private_nh_.param<std::string>("target_frame", target_frame_, DEFAULT_TARGET_FRAME);
  private_nh_.param<double>("transform_tolerance", tf_tolerance_, DEFAULT_TF_TOLERANCE);
  private_nh_.param<std::string>("depth_topic", depth_topic_, DEFAULT_DEPTH_TOPIC);

  // PUBLISHERS
  // Create a ROS publisher for the output point cloud
  pub_cluster0 = nh_.advertise<sensor_msgs::PointCloud2>("pcl_object_detection/cluster0", 1);
  pub_cluster1 = nh_.advertise<sensor_msgs::PointCloud2>("pcl_object_detection/cluster1", 1);
  pub_cluster2 = nh_.advertise<sensor_msgs::PointCloud2>("pcl_object_detection/cluster2", 1);
  pub_nearest_object = nh_.advertise<sensor_msgs::PointCloud2>("pcl_object_detection/nearest_object", 1);

  pub0 = nh_.advertise<sensor_msgs::PointCloud2>("pcl_object_detection/plane0", 1);
  pub1 = nh_.advertise<sensor_msgs::PointCloud2>("pcl_object_detection/plane1", 1);
  pub2 = nh_.advertise<sensor_msgs::PointCloud2>("pcl_object_detection/plane2", 1);
  pub3 = nh_.advertise<sensor_msgs::PointCloud2>("pcl_object_detection/plane3", 1);
  pub4 = nh_.advertise<sensor_msgs::PointCloud2>("pcl_object_detection/plane4", 1);
  pub_voxel = nh_.advertise<sensor_msgs::PointCloud2>("pcl_object_detection/voxel_output", 1);

  pub_remaining = nh_.advertise<sensor_msgs::PointCloud2>("pcl_object_detection/remaining", 1);
  pub_objects = nh_.advertise<sensor_msgs::PointCloud2>("pcl_object_detection/objects", 1);

  // Publish markers to show where robot thinks object is in RViz
  marker_pub_ = nh_.advertise<visualization_msgs::Marker>("pcl_object_detection/marker", 1);

  // Publish positions of detected objects
  pub_detected_objects = nh_.advertise<pcl_object_detection::DetectedObjects>("pcl_object_detection/detected_objects", 1);

  pcl_publishers.SetClustersPublishers(
      pub_cluster0, pub_cluster1, pub_cluster2);
  pcl_publishers.SetPlanesPublishers(
      pub0, pub1, pub2, pub3);
  pcl_publishers.SetOtherPublishers(
      pub_voxel, pub_nearest_object, pub_remaining, pub_objects, marker_pub_);

  objects_publisher.SetPublisher(
      pub_detected_objects);

 // ugly hotfix for ros_astra_camera termination
  pub_object_detection_done = nh_.advertise<std_msgs::Empty>(
        "camera/object_detection_done", 1);

  // SUBSCRIBERS

  // Create a ROS subscriber for the input point cloud
  depth_cloud_sub_ = nh_.subscribe(
    depth_topic_, 1, &PclObjectDetection::cloud_cb, this);

  pointed_object_index_sub = nh_.subscribe(
    "task_control/pointed_object_index", 1, &PclObjectDetection::pointed_object_index_cb, this);
}

void PclObjectDetection::runLoop()
{
  do
    {
      ros::spinOnce();
      if (objectsDetected)
      {
        objectDetection.PublishObjectsMessages();
      }
  } while (true); // stopped by key handler [Ctrl+C]
  
}

void PclObjectDetection::cloud_cb(const sensor_msgs::PointCloud2ConstPtr &input_cloud_msg)
{
  if (objectsDetected)
  {
    return;
  }
    
  PrintRosInfo("Received point cloud MSG.");
  input_cloud_frame_ = input_cloud_msg->header.frame_id; 
  
  objectDetection.SetPublishers(pcl_publishers, objects_publisher);

  objectsDetected = objectDetection.Detect(
          input_cloud_msg,
          target_frame_,
          tf_tolerance_);

  if (objectsDetected)
  {
    PrintRosInfo("Object Detection done.");
    std_msgs::Empty empty_msg;
    pub_object_detection_done.publish(empty_msg);
  }
}

void PclObjectDetection::pointed_object_index_cb(const std_msgs::Int32 &index_msg)
{
  if (pointedObjectSelected)
  {
    return;
  }

  pointedObjectSelected = true;
  objectDetection.ChangeMarkerColor(
    index_msg.data,
    0, 1, 0.2);
}

void PclObjectDetection::PrintRosInfo(std::string info)
{
  ROS_INFO("PCL OBJECT DETECTION: %s.", info.c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pcl_object_detection");

  ros::NodeHandle n;
  PclObjectDetection pcl_object_detection_node(n);
  pcl_object_detection_node.runLoop();
  return 0;
}