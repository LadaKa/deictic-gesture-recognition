#include "ObjectDetection.h"

#include <pcl_conversions/pcl_conversions.h>

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

#include "RVizMarkerBox.h"

ObjectDetection::ObjectDetection(): 
    tfListener_(ObjectDetection::tf2_)
{};

void ObjectDetection::SetPublishers(PointCloudPublishers pcPublishers)
{
    ObjectDetection::publishers = pcPublishers;
}

void ObjectDetection::AddMarkerBox(
    std::string frame_id,
    int id,
    float x, float y, float z,
    float size_x, float size_y, float size_z,
    float color_r, float color_g, float color_b)
{
    RVizMarkerBox rVizMarkerBox(
        frame_id,
        id,
        x, y, z,
        size_x, size_y, size_z,
        color_r, color_g, color_b);
    visualization_msgs::Marker marker = rVizMarkerBox.GetMarker();
    ObjectDetection::objects_markers[id] = marker;

    
}

void ObjectDetection::PublishObjectsMarkers()
{
    for (int i = 0; i < 3; i++) // const
    {
        publishers.pub_marker.publish(objects_markers[i]);
    }
}

bool ObjectDetection::Detect(
    const sensor_msgs::PointCloud2ConstPtr &input_cloud_msg,
    std::string target_frame,
    double tf_tolerance)
{
    //  TF Frame of the point cloud
    std::string input_cloud_frame_ = input_cloud_msg->header.frame_id;

    // CLOUD DATA STRUCTURES
    pcl::PCLPointCloud2::Ptr
        cloud_blob(new pcl::PCLPointCloud2),
        cloud_filtered_blob(new pcl::PCLPointCloud2);

    pcl::PointCloud<pcl::PointXYZ>::Ptr
        downsampled_XYZ(new pcl::PointCloud<pcl::PointXYZ>),
        cloud_plane_p(new pcl::PointCloud<pcl::PointXYZ>),
        cloud_f(new pcl::PointCloud<pcl::PointXYZ>);

    // Container for original & filtered data
    pcl::PCLPointCloud2 *pcl2_input_cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr pcl2_input_cloud_p(pcl2_input_cloud);
    pcl::PCLPointCloud2 cloud_filtered2;

    // Convert from ROS to PCL2 cloud
    pcl_conversions::toPCL(*input_cloud_msg, *pcl2_input_cloud);

    // Perform the actual filtering
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(pcl2_input_cloud_p);
    sor.setLeafSize(0.01, 0.01, 0.01);

    sor.setDownsampleAllData(false);
    sor.filter(cloud_filtered2);

    sensor_msgs::PointCloud2 voxel_output;
    pcl_conversions::fromPCL(cloud_filtered2, voxel_output);
    publishers.pub_voxel.publish(voxel_output);

    // Convert from PCL2 to PCL XYZ cloud?
    pcl::fromPCLPointCloud2(cloud_filtered2, *downsampled_XYZ);

    // Segment the Plane (Floor)  ... (or Tabletop??? TODO)
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.025); // 0.02

    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZ> extract;

    int i = 0, nr_points = (int)downsampled_XYZ->points.size();
    // While 30% of the original cloud is still there
    while (downsampled_XYZ->points.size() > 0.3 * nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud(downsampled_XYZ);
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.size() == 0)
        {
            std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        /*
          // Publish the model coefficients for each plane
          pcl_msgs::ModelCoefficients ros_coefficients;
          pcl_conversions::fromPCL(*coefficients, ros_coefficients);
          pub.publish (ros_coefficients);
        */

        // Extract the inliers
        extract.setInputCloud(downsampled_XYZ);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*cloud_plane_p);
        if (i < 5)
        {
            // Publish Plane point clouds for debug
            sensor_msgs::PointCloud2 output;
            pcl::PCLPointCloud2 tmp_cloud;
            pcl::toPCLPointCloud2(*cloud_plane_p, tmp_cloud);
            pcl_conversions::fromPCL(tmp_cloud, output);

            publishers.PublishPlaneMessage(i, output);
        }

        // Create the filtering object
        extract.setNegative(true);
        extract.filter(*cloud_f);
        downsampled_XYZ.swap(cloud_f);
        i++;
    }

    // Convert and publish the cloud of remaining points (outside the plane)
    sensor_msgs::PointCloud2 output;
    pcl::PCLPointCloud2 tmp_cloud;
    pcl::toPCLPointCloud2(*downsampled_XYZ, tmp_cloud);
    pcl_conversions::fromPCL(tmp_cloud, output);
    publishers.pub_remaining.publish(output);

    // Look for object Clusters

    bool objectsDetected = false;

    if ((downsampled_XYZ->width > 0) && (downsampled_XYZ->height > 0))
    {

        // Creating the KdTree object for the search method of the extraction
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(downsampled_XYZ);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        // ClusterTolerance: too small and object can be seen as multiple clusters.
        // too high and multiple objects are seen as one cluster.
        ec.setClusterTolerance(0.04); // 4 cm
        ec.setMinClusterSize(100);
        ec.setMaxClusterSize(25000);
        ec.setSearchMethod(tree);
        ec.setInputCloud(downsampled_XYZ);
        ec.extract(cluster_indices);

        // Nearest object
        sensor_msgs::PointCloud2Ptr nearest_object_cloud_msg;
        int nearest_object_index = -1;
        float nearest_object_weighted_distance = 10000.0;
        pcl::PointXYZ nearest_obj_minPt, nearest_obj_maxPt, nearest_obj_bb_size, nearest_obj_center;

        // Clusters

        float marker_r = 0.0;
        float marker_g = 1.0;
        float marker_b = 1.0;


        int j = 0;
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
            for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
                cloud_cluster->points.push_back(downsampled_XYZ->points[*pit]);

            cloud_cluster->width = cloud_cluster->points.size();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;

            // Convert the pointcloud to be used in ROS
            sensor_msgs::PointCloud2::Ptr output(new sensor_msgs::PointCloud2);
            pcl::toROSMsg(*cloud_cluster, *output);
            output->header.frame_id = input_cloud_frame_;

            // Rotate the clusters found
            sensor_msgs::PointCloud2Ptr cloud_rotated_msg;
            pcl::PCLPointCloud2 pcl2_rotated_cluster;
            pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_rotated_cluster_XYZ(new pcl::PointCloud<pcl::PointXYZ>);
            // double  tf_tolerance_ = 0.05;
            pcl::PointXYZ minPt, maxPt, bb_size, obj_center;

            try
            {
                cloud_rotated_msg.reset(new sensor_msgs::PointCloud2);
                tf2_.transform(*output, *cloud_rotated_msg, target_frame, ros::Duration(tf_tolerance));

                // Convert from ROS to PCL2 cloud
                pcl_conversions::toPCL(*cloud_rotated_msg, pcl2_rotated_cluster);
                // Convert from PCL2 to PCL XYZ cloud? (surely there is a better way!?)
                pcl::fromPCLPointCloud2(pcl2_rotated_cluster, *pcl_rotated_cluster_XYZ);

                // Find bounding box of rotated cluster
                pcl::getMinMax3D(*pcl_rotated_cluster_XYZ, minPt, maxPt);
                /*
                std::cout << "CLUSTER [" << j << "] : "
                  << "Max x: " << maxPt.x
                  << ", y: " << maxPt.y
                  << ", z: " << maxPt.z
                  << "    Min x: " << minPt.x
                  << ", y: " << minPt.y
                  << ", z: " << minPt.z << std::endl;
                */

                bb_size.x = maxPt.x - minPt.x;
                bb_size.y = maxPt.y - minPt.y;
                bb_size.z = maxPt.z - minPt.z;

                obj_center.x = minPt.x + (bb_size.x / 2);
                obj_center.y = minPt.y + (bb_size.y / 2);
                obj_center.z = minPt.z + (bb_size.z / 2);

                /*
                std::cout << "OBJECT [" << j << "] : "
                  << "  x: " << obj_center.x
                  << ", y: " << obj_center.y
                  << ", z: " << obj_center.z
                  << ", l: " << bb_size.x
                  << ", w: " << bb_size.y
                  << ", h: " << bb_size.z
                  << ", top: " << maxPt.z
                  << ", bottom: " << minPt.z;
                */
                
                
                // std::cout << " PASS" << std::endl;
                const int PICKUP_ZONE_MAX_Y = 400; // mm from center of robot
                // find the nearest object to the robot
                float weighted_obj_center_y = obj_center.y;
                if (fabs(weighted_obj_center_y) > PICKUP_ZONE_MAX_Y)
                    weighted_obj_center_y *= 2.0; // penalize objects too far to the side (prefer objects ahead)

                float weighted_distance = sqrt(pow(obj_center.x, 2) + pow(weighted_obj_center_y, 2));
                if (weighted_distance < nearest_object_weighted_distance)
                {
                    // New Nearest Object
                    nearest_object_weighted_distance = weighted_distance;
                    nearest_object_index = j;
                    nearest_object_cloud_msg = cloud_rotated_msg;
                    nearest_obj_minPt = minPt;
                    nearest_obj_maxPt = maxPt;
                    nearest_obj_bb_size = bb_size;
                    nearest_obj_center = obj_center;
                }

                if (j < 3)
                {
                    std::cout << "Checking object " << j << std::endl;
                    
                    if (CheckObjectSize(maxPt, bb_size))
                    {
                        marker_r = marker_r + 0.2;
                        marker_g = marker_g - 0.2;

                        ObjectDetection::AddMarkerBox(            // Publish the bounding box as a marker
                            target_frame,                             // Transform Frame from camera to robot base
                            j,                                        // Marker ID
                            obj_center.x, obj_center.y, obj_center.z, // Object Center
                            bb_size.x, bb_size.y, bb_size.z,          // Object Size
                            marker_r, marker_g, marker_b); 


                        publishers.PublishClusterMessage(j, cloud_rotated_msg);
                                      
                    }
                    else
                        j--; //!!
                }
                else 
                {
                    objectsDetected = true;
                    break;
                }
            }
            catch (tf2::TransformException &ex)
            {
                ROS_WARN_STREAM("Transform failure: " << ex.what());
                break;
            }

            ++j;
        }

        // Publish the nearest object found in front of the robot
        if (-1 != nearest_object_index)
        {

            publishers.pub_nearest_object.publish(nearest_object_cloud_msg);

           /* ObjectDetection::AddMarkerBox(
                target_frame,
                10,
                nearest_obj_center.x, nearest_obj_center.y, nearest_obj_center.z,
                nearest_obj_bb_size.x, nearest_obj_bb_size.y, nearest_obj_bb_size.z,
                0.0, 1.0, 0.0); // Green
           */

          /*  std::cout << "NEAREST OBJECT: index: "
                      << nearest_object_index
                      << ",  Center x: " << nearest_obj_center.x
                      << ", y: " << nearest_obj_center.y
                      << ", z: " << nearest_obj_center.z
                      << std::endl
                      << "  Bounding Box l: " << nearest_obj_bb_size.x
                      << ", w: " << nearest_obj_bb_size.y
                      << ", h: " << nearest_obj_bb_size.z
                      << ",   minPt.x: " << nearest_obj_minPt.x
                      << ", maxPt.x: " << nearest_obj_maxPt.x
                      << ", minPt.z: " << nearest_obj_minPt.z
                      << ", maxPt.z: " << nearest_obj_maxPt.z
                      << std::endl;*/
        }
    }
    return objectsDetected;
} // cloud_cb

bool ObjectDetection::CheckObjectSize(
    pcl::PointXYZ maxPt, 
    pcl::PointXYZ bb_size)
{
    
    if (maxPt.x > 2.0)
    {      
        return false;
    }
    

    std::cout << " Max points: "
                << maxPt.x << " "
                << maxPt.y << " "
                << maxPt.z << " "
                << std::endl;
   
    return true;

}