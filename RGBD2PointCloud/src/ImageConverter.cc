#include "ImageConverter.h"

ImageConverter::ImageConverter(const std::string &current_path)
    : it_(nh_), pcd_path(current_path)
{
    image_sub_color = std::make_shared<message_filters::Subscriber<sensor_msgs::Image>>(nh_, "/camera/color/image_raw", 1);
    image_sub_depth = std::make_shared<message_filters::Subscriber<sensor_msgs::Image>>(nh_, "/camera/aligned_depth_to_color/image_raw", 1);
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), *image_sub_color, *image_sub_depth);
    sync_->registerCallback(boost::bind(&ImageConverter::Callback, this, _1, _2));

    cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/rgbd_cloud", 10000);

    const std::string intrinsics_path = current_path + "/config/config.yaml";
    LoadIntrinsic(intrinsics_path, dist_coeffs, intrisic_mat);

    ROS_INFO("SUCCESS TO READ PARAM!");
}

void ImageConverter::Callback(const sensor_msgs::ImageConstPtr &msgImg, const sensor_msgs::ImageConstPtr &msgDepth)
{
    // ros->opencv 的常用套路
    cv_bridge::CvImagePtr cvImgPtr, cvDepthPtr;
    try
    {
        cvImgPtr = cv_bridge::toCvCopy(msgImg, sensor_msgs::image_encodings::BGR8);
        cvDepthPtr = cv_bridge::toCvCopy(msgDepth, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch (cv_bridge::Exception e)
    {
        ROS_ERROR_STREAM("Cv_bridge Exception:" << e.what());
        return;
    }

    // 数据类型转换，得到彩色图、灰度图和深度图
    cv::Mat cvColorImgMat = cvImgPtr->image;
    cv::Mat cvDepthMat = cvDepthPtr->image;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    sensor_msgs::PointCloud2 cloud_output;
    for (int m = 0; m < cvDepthMat.rows; m++)
    {
        for (int n = 0; n < cvDepthMat.cols; n++)
        {
            float d = cvDepthMat.ptr<float>(m)[n]; // 获取深度图中(m,n)处的值
            if (d == 0.0)
            {
                continue;
            }
            //注入点云
            pcl::PointXYZRGB point;
            point.z = double(d) * camera_factor;
            point.x = ((n - intrisic_mat.at<double>(0, 2)) * point.z / intrisic_mat.at<double>(0, 0));
            point.y = ((m - intrisic_mat.at<double>(1, 2)) * point.z / intrisic_mat.at<double>(1, 1));
            std::cout << point.x << "\t" << point.y << "\t" << point.z << std::endl;
            point.b = cvColorImgMat.ptr<uchar>(m)[n * 3];
            point.g = cvColorImgMat.ptr<uchar>(m)[n * 3 + 1];
            point.r = cvColorImgMat.ptr<uchar>(m)[n * 3 + 2];
            cloud->points.push_back(point);
        }
    }

    std::stringstream ss;
    std::string pcd_id;
    ss << ros::Time::now();
    ss >> pcd_id;
    std::string pcd_file = pcd_path + "/pcd/" + pcd_id + ".pcd";
    std::cout << "[INFO] PCD File: " << pcd_file << std::endl;
    pcl::io::savePCDFileBinary(pcd_file, *cloud);

    cloud->height = 1;
    cloud->width = cloud->points.size();
    cloud->is_dense = false;
    pcl::toROSMsg(*cloud, cloud_output);
    cloud_output.header.frame_id = "base_scan";
    cloud_output.header.stamp = ros::Time::now();
    cloud_pub_.publish(cloud_output);
}

bool ImageConverter::LoadIntrinsic(const std::string &intrinsics_path,
                                   cv::Mat &dist_coeffs,
                                   cv::Mat &intrisic_mat)
{
    if (!(boost::filesystem::exists(intrinsics_path)))
    {
        return false;
    }
    YAML::Node config = YAML::LoadFile(intrinsics_path);
    std::cout << "-----------------------------------------" << std::endl;

    if (config["K"] && config["D"])
    {
        std::vector<double> K = config["K"].as<std::vector<double>>();
        std::vector<double> D = config["D"].as<std::vector<double>>();
        intrisic_mat = cv::Mat(3, 3, cv::DataType<double>::type);
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                intrisic_mat.at<double>(i, j) = K[i * 3 + j];
            }
        }
        dist_coeffs = cv::Mat(5, 1, cv::DataType<double>::type);
        for (int i = 0; i < 5; i++)
        {
            dist_coeffs.at<double>(i) = D[i];
        }
    }
    if (config["Factor"]){
        camera_factor = config["Factor"].as<double>();
    }

    return true;
}
