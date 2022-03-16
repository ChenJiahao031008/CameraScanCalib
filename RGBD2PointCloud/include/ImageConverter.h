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
//yaml
#include "yaml-cpp/yaml.h"

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;


class ImageConverter
{
public:
    ros::NodeHandle nh_;                 // 节点名
    image_transport::ImageTransport it_; // 图像转换

    std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> image_sub_color;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> image_sub_depth;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
    ros::Publisher cloud_pub_;

public:
    ImageConverter(const std::string &current_path);

    ~ImageConverter(){};

    void Callback(const sensor_msgs::ImageConstPtr &msgImg, const sensor_msgs::ImageConstPtr &msgDepth);

private:
    cv::Mat dist_coeffs;
    cv::Mat intrisic_mat;
    double camera_factor;
    std::string pcd_path = "";

private:
    bool LoadIntrinsic(const std::string &intrinsics_path,
                       cv::Mat &dist_coeffs,
                       cv::Mat &intrisic_mat);
};
