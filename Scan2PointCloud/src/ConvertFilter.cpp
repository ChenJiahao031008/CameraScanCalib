#include "ConvertFilter.h"

ConvertFilter::ConvertFilter(const std::string &current_path)
{
    pcd_path = current_path + "/pcd/";
    //订阅　"/scan"
    scan_sub_ = node_.subscribe<sensor_msgs::LaserScan>("/scan", 100, &ConvertFilter::scanCallback_2, this);

    //发布LaserScan转换为PointCloud2的后的数据
    point_cloud_publisher_ = node_.advertise<sensor_msgs::PointCloud2>("/cloud2", 100, false);

    //此处的tf是 laser_geometry 要用到的
    tfListener_.setExtrapolationLimit(ros::Duration(0.1));

    // //前面提到的通过订阅PointCloud2，直接转化为pcl::PointCloud的方式
    // pclCloud_sub_ = node_.subscribe<pcl::PointCloud<pcl::PointXYZ>>("/cloud2", 10, &ConvertFilter::pclCloudCallback, this);
}

void ConvertFilter::scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    sensor_msgs::PointCloud2 cloud;
    projector_.transformLaserScanToPointCloud("base_scan", *scan, cloud, tfListener_);
    point_cloud_publisher_.publish(cloud);
}

void ConvertFilter::scanCallback_2(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    sensor_msgs::PointCloud2 cloud;

    /*laser_geometry包中函数，将 sensor_msgs::LaserScan 转换为 sensor_msgs::PointCloud2 */
    //普通转换
    // projector_.projectLaser(*scan, cloud);
    //使用tf的转换
    projector_.transformLaserScanToPointCloud("base_scan", *scan, cloud, tfListener_);

    int row_step = cloud.row_step;
    int height = cloud.height;

    /*将 sensor_msgs::PointCloud2 转换为　pcl::PointCloud<T> */
    //注意要用fromROSMsg函数需要引入pcl_versions（见头文件定义）
    pcl::PointCloud<pcl::PointXYZ> rawCloud;
    pcl::fromROSMsg(cloud, rawCloud);

    for (size_t i = 0; i < rawCloud.points.size(); i++)
    {
        rawCloud.points[i].z = 0.20;
        std::cout << rawCloud.points[i].x << "\t" << rawCloud.points[i].y << "\t" << rawCloud.points[i].z << std::endl;
    }

    std::stringstream ss;
    std::string pcd_id;
    ss << ros::Time::now();
    ss >> pcd_id;
    std::string pcd_file = pcd_path + pcd_id + ".pcd";
    std::cout << "[INFO] PCD File: " << pcd_file << std::endl;
    pcl::io::savePCDFileBinary(pcd_file, rawCloud);

    point_cloud_publisher_.publish(cloud);
}

void ConvertFilter::pclCloudCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
{
    for (size_t i = 0; i < cloud->points.size(); i++)
    {
        std::cout << "direct_trans: " << cloud->points[i].x << "\t" << cloud->points[i].y << "\t" << cloud->points[i].z << std::endl;
    }
}
