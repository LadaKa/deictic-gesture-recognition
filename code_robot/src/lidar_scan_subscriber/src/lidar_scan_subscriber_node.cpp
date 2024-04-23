#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <iostream>
#include <cmath>
#include <map>

/* Lidar Scan Subscriber Node
   Process sensor_msgs/LaserScan messages:

   float32[] ranges         # range data [m]
                            # Note: values < range_min or > range_max should be discarded

*/

class lidar_scan_subscriber_node

{

    /*   void scanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
        {
             ROS_INFO("LaserScan (val,angle)=(%f,%f", msg->range_min, msg->angle_min);
        }
    */

    float maxNearRange = 1.5;

    float maxDifference = 0.2;

    int rangesCount = 536;

public:
    std::map<int, float> selectNearObjects(std::vector<float> ranges)
    {
        //  rangeIndex and range of detected near objects
        std::map<int, float> nearObjects;
        int i = 0;

        while (i < rangesCount)
        {
            // check detection of near object
            if (ranges[i] > 0 && ranges[i] < maxNearRange)
            {
                // new near object
                int start = i;
                bool sameObject = true;
                while (i < rangesCount && sameObject)
                {
                    i++;
                    // check whether next range corresponds to some other object
                    if ((abs(ranges[i] - ranges[i - 1]) > maxDifference) || (i == rangesCount))
                    {
                        int center = start + ((i - start) / 2);
                        sameObject = false;
                        nearObjects[center] = ranges[center];
                        //  TODO: remove output
                        std::cout << start << " - " << center << " - " << i - 1 << ": [";
                        std::cout << ranges[center] << "]\n";
                    }
                }
            }
            else
            {
                // no near object detected
                i++;
            }
        }
        std::cout << std::endl;
        return nearObjects;
    }
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "lidar_scan_subscriber_node");
    ros::NodeHandle nh;
    ROS_INFO_STREAM("Created lidar_scan_subscriber_node.");
    // ros::Subscriber sub = nh.subscribe("/sensor_msgs/LaserScan", 1000, scanCallback);

    lidar_scan_subscriber_node *node = new lidar_scan_subscriber_node();

    sensor_msgs::LaserScan::ConstPtr lidar_scan_msg = ros::topic::waitForMessage<sensor_msgs::LaserScan>("/lidar_1/scan_filtered", nh);
    ROS_INFO_STREAM("Received laser scan.");

    int ranges_size = lidar_scan_msg->ranges.size();

    //  rangeIndex and range of detected near objects
    std::map<int, float> nearObjects = node->selectNearObjects(lidar_scan_msg->ranges);

    ros::spin();
    return 0;
}
