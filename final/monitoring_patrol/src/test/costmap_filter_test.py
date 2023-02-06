
// This is a test script to fill the costmap area cptured by Realsense

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <nav_msgs/OccupancyGrid.h>

ros::Publisher costmap_publisher;

void pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    // Convert the point cloud message to PCL format
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*cloud_msg, cloud);

    // Create an occupancy grid with the same dimensions as the point cloud
    nav_msgs::OccupancyGrid costmap;
    costmap.header = cloud_msg->header;
    costmap.info.width = cloud.width;
    costmap.info.height = cloud.height;
    costmap.info.resolution = 0.05;  // set the resolution of the costmap

    // Fill in the costmap data with the point cloud information
    costmap.data.resize(costmap.info.width * costmap.info.height);
    for (int i = 0; i < cloud.points.size(); i++)
    {
        if (!std::isnan(cloud.points[i].z))
        {
            int x = (cloud.points[i].x / costmap.info.resolution) + (costmap.info.width / 2);
            int y = (cloud.points[i].y / costmap.info.resolution) + (costmap.info.height / 2);
            int index = x + y * costmap.info.width;
            costmap.data[index] = 100;  // set the occupied value to 100
        }
    }

    // Publish the occupancy grid as a costmap
    costmap_publisher.publish(costmap);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "costmap_node");
    ros::NodeHandle nh;

    // Subscribe to the Realsense point cloud topic
    ros::Subscriber pointcloud_subscriber = nh.subscribe("/camera/depth/points", 1, pointcloudCallback);

    // Advertise the costmap publisher
    costmap_publisher = nh.advertise<nav_msgs/OccupancyGrid>("/costmap", 1);

    ros::spin();

    return 0;
}

