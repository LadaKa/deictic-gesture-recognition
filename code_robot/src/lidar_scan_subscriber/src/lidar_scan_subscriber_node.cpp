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

    float maxNearRange = 1.0;

    float maxDifference = 0.2;

    int rangesCount = 536;

    float angle_increment = 0.008808203972876072;

    int ur_base_shift_Y = 0.2;

    struct Point
    {
        float x, y;
        Point(float x_coord, float y_coord)
        {
            this->x = x_coord;
            this->y = y_coord;
        }
    };

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

    // lidar coordinates of detected object (origin in lidar)
    Point getObjectLidarCoordinates(
        int rangeIndex, float range, int zeroAngleIndex, float rad)
    {
        // angle in degrees
        float theta = 90 + ((zeroAngleIndex - rangeIndex) * angle_increment * rad);

        float x = range * cos(theta);
        float y = range * sin(theta);

        std::cout << "Lidar: " << x << "  " << y << "\n";

        return Point(x, y);
    }

    Point convertLidarToURCoordinates(Point lidarCoordinates)
    {
        // flip over X axis
        float ur_x = - lidarCoordinates.x;

        // shift along Y axis (forward)
        float ur_y = lidarCoordinates.y - ur_base_shift_Y;

        return Point(ur_x, ur_y);
    }

    // convert lidar coordinates of all detected objects
    // to UR arm coordinates

    // TODO: rename and return result as map; units?
    void printAllObjectsCoordinates(std::map<int, float> objectsRanges)
    {
        float rad = 180 / M_PI;
        int zeroAngleIndex = rangesCount / 2;

        for (const auto &pair : objectsRanges)
        {
            Point lidarCoord = getObjectLidarCoordinates(
                pair.first, pair.second, zeroAngleIndex, rad);
            
            Point urCoord = convertLidarToURCoordinates(lidarCoord);

            std::cout << "UR arm: " << urCoord.x << "  " << urCoord.y << "\n";
        }
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
    node->printAllObjectsCoordinates(nearObjects);

    ros::spin();
    return 0;
}
