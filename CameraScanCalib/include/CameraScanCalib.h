#include <iostream>
#include <math.h>
#include <stdio.h>
#include <sstream>
// ros
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
// opencv
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
// 多线程相关
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
// pcl
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
//yaml
#include "yaml-cpp/yaml.h"

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;


class CameraScanCalib
{
public:
    ros::NodeHandle nh_;                 // 节点名
    ros::Publisher rgbd_cloud_pub_;
    ros::Publisher scan_cloud_pub_;
    double g_yaw = 0;
    double g_pitch = 0;
    double g_roll = 0;
    double g_x = 0;
    double g_y = 0;
    double g_z = 0;

public:
    CameraScanCalib(const std::string &current_path);

    static void UpdateZ(int value, void *this_ptr);
    static void UpdateY(int value, void *this_ptr);
    static void UpdateX(int value, void *this_ptr);
    static void UpdateRoll(int value, void *this_ptr);
    static void UpdatePitch(int value, void *this_ptr);
    static void UpdateYaw(int value, void *this_ptr);

    void Run();
    void Project();

    ~CameraScanCalib(){};

private:
    Eigen::Affine3d extrinsic;
    std::string scan_cloud_file;
    std::string rgbd_cloud_file;
    pcl::PointCloud<pcl::PointXYZ>::Ptr scan_cloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgbd_cloud;

private:
    bool LoadExtrinsic(const std::string &file_path, Eigen::Affine3d &extrinsic);
};
