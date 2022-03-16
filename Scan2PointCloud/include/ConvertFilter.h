#include <iostream>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

class ConvertFilter
{
public:
    ConvertFilter(const std::string &current_path);

    //订阅 LaserScan　数据，并发布 PointCloud2 点云
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan);
    //订阅 LaserScan 数据，先转换为 PointCloud2，再转换为 pcl::PointCloud
    void scanCallback_2(const sensor_msgs::LaserScan::ConstPtr &scan);
    //直接订阅 PointCloud2 然后自动转换为 pcl::PointCloud
    void pclCloudCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud);

private:
    ros::NodeHandle node_;
    laser_geometry::LaserProjection projector_;
    tf::TransformListener tfListener_;

    //发布　"PointCloud2"
    ros::Publisher point_cloud_publisher_;
    //订阅 "/scan"
    ros::Subscriber scan_sub_;
    //订阅 "/cloud2" -> "PointCloud2"
    ros::Subscriber pclCloud_sub_;

    std::string pcd_path;
};
