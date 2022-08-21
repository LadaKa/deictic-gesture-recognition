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

void ObjectDetection::SetPublishers(
    PointCloudPublishers pcPublishers,
    ObjectsPublisher objPublisher)
{
    publishers = pcPublishers;
    objectsPublisher = objPublisher;
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

void ObjectDetection::PublishObjectsMessages()
{
    for (int i = 0; i < 3; i++) // const
    {
        publishers.pub_marker.publish(objects_markers[i]);
    }
    objectsPublisher.Publish();
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

        // Clusters

        float marker_r = 0.0;
        float marker_g = 1.0;
        float marker_b = 1.0;

        ROS_INFO("PCL OBJECT DETECTION: Processing point cloud.");

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

                bb_size.x = maxPt.x - minPt.x;
                bb_size.y = maxPt.y - minPt.y;
                bb_size.z = maxPt.z - minPt.z;

                obj_center.x = minPt.x + (bb_size.x / 2);
                obj_center.y = minPt.y + (bb_size.y / 2);
                obj_center.z = minPt.z + (bb_size.z / 2);

                if (j < 3)
                {
                    if (CheckObjectSize(maxPt, bb_size))
                    {
                        marker_r = marker_r + 0.2;
                        marker_g = marker_g - 0.2;

                        detected_objects_centers[j] = obj_center;

                        ObjectDetection::AddMarkerBox(            // Publish the bounding box as a marker
                            target_frame,                             // Transform Frame from camera to robot base
                            j,                                        // Marker ID
                            obj_center.x, obj_center.y, obj_center.z, // Object Center
                            bb_size.x, bb_size.y, bb_size.z,          // Object Size
                            marker_r, marker_g, marker_b); 

                        publishers.PublishClusterMessage(j, cloud_rotated_msg);
                        ROS_INFO(
                            "PCL OBJECT DETECTION: Detected object [%i].\nCenter: %f %f %f.\nMin: %f %f %f. ",
                            j, 
                            obj_center.x, obj_center.y, obj_center.z,
                            minPt.x, minPt.y, minPt.z); 
                    }
                    else
                        j--; //!!
                }
                else 
                {
                    objectsDetected = true;

                    objectsPublisher.CreateObjectsMsg(
                        detected_objects_centers[0],
                        detected_objects_centers[1],
                        detected_objects_centers[2]);
                        
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
   
    return true;

}