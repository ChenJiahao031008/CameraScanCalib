#include "CameraScanCalib.h"

CameraScanCalib::CameraScanCalib(const std::string &current_path)
{

    rgbd_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/rgbd_calib_cloud", 10000);
    scan_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/scan_calib_cloud", 10000);

    const std::string extrinsics_path = current_path + "/config/config.yaml";
    bool success = LoadExtrinsic(extrinsics_path, extrinsic);
    if (!success)
    {
        std::cerr << "Load camera extrinsic failed." << std::endl;
        return;
    }

    rgbd_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
    if (pcl::io::loadPCDFile(rgbd_cloud_file, *rgbd_cloud) == -1)
    {
        std::cout << "couldn't read file" << rgbd_cloud_file << std::endl;
        return;
    }

    scan_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
    if (pcl::io::loadPCDFile(scan_cloud_file, *scan_cloud) == -1)
    {
        std::cout << "couldn't read file" << scan_cloud_file << std::endl;
        return;
    }


    ROS_INFO("SUCCESS TO READ PARAM!");
}

void CameraScanCalib::Run()
{
    int value_yaw = 18000;
    int value_pitch = 18000;
    int value_roll = 18000;
    int value_x = 50;
    int value_y = 50;
    int value_z = 50;
    cv::namedWindow("mainWin", cv::WINDOW_NORMAL);
    cv::moveWindow("mainWin", 200, 200);
    cv::createTrackbar("yaw", "mainWin", &value_yaw, 36000, UpdateYaw, this);
    cv::createTrackbar("pitch", "mainWin", &value_pitch, 36000, UpdatePitch, this);
    cv::createTrackbar("roll", "mainWin", &value_roll, 36000, UpdateRoll, this);
    cv::createTrackbar("x", "mainWin", &value_x, 100, UpdateX, this);
    cv::createTrackbar("y", "mainWin", &value_y, 100, UpdateY, this);
    cv::createTrackbar("z", "mainWin", &value_z, 100, UpdateZ, this);

    Project();

    while (1)
    {
        cv::waitKey(100);
    }
}

void CameraScanCalib::Project()
{
    Eigen::Affine3d Tdelta = Eigen::Affine3d::Identity();
    Tdelta.translation() = Eigen::Vector3d(g_x, g_y, g_z);
    Tdelta.linear() =
        Eigen::AngleAxisd(g_yaw, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(g_pitch, Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd(g_roll, Eigen::Vector3d::UnitY()).toRotationMatrix();

    Eigen::Affine3d extrinsic_final = extrinsic * Tdelta;

    std::cout << "extrinsic_final translation\n"
              << extrinsic_final.translation() << std::endl;
    std::cout << "extrinsic_final rotation\n"
              << extrinsic_final.linear() << std::endl;
    Eigen::Quaterniond q(extrinsic_final.linear());
    std::cout << "extrinsic_final quaternion qx qy qz qw\n"
              << q.x() << " " << q.y() << " " << q.z() << " " << q.w()
              << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_scan_tmp(
        new pcl::PointCloud<pcl::PointXYZ>());
    pcl::transformPointCloud(*scan_cloud, *cloud_scan_tmp, extrinsic_final.inverse());

    sensor_msgs::PointCloud2 output_scan_cloud;
    cloud_scan_tmp->height = 1;
    cloud_scan_tmp->width = cloud_scan_tmp->points.size();
    cloud_scan_tmp->is_dense = false;
    pcl::toROSMsg(*cloud_scan_tmp, output_scan_cloud);
    output_scan_cloud.header.frame_id = "base_scan";
    output_scan_cloud.header.stamp = ros::Time::now();
    scan_cloud_pub_.publish(output_scan_cloud);

    sensor_msgs::PointCloud2 output_rgbd_cloud;
    rgbd_cloud->height = 1;
    rgbd_cloud->width = rgbd_cloud->points.size();
    rgbd_cloud->is_dense = false;
    pcl::toROSMsg(*rgbd_cloud, output_rgbd_cloud);
    output_rgbd_cloud.header.frame_id = "base_scan";
    output_rgbd_cloud.header.stamp = ros::Time::now();
    rgbd_cloud_pub_.publish(output_rgbd_cloud);
    cv::waitKey(100);
}

bool CameraScanCalib::LoadExtrinsic(const std::string &file_path, Eigen::Affine3d &extrinsic)
{
    YAML::Node config = YAML::LoadFile(file_path);
    std::cout << "-----------------------------------------" << std::endl;
    if (config["transform"])
    {
        if (config["transform"]["translation"])
        {
            extrinsic.translation()(0) =
                config["transform"]["translation"]["x"].as<double>();
            extrinsic.translation()(1) =
                config["transform"]["translation"]["y"].as<double>();
            extrinsic.translation()(2) =
                config["transform"]["translation"]["z"].as<double>();
            if (config["transform"]["rotation"])
            {
                double qx = config["transform"]["rotation"]["x"].as<double>();
                double qy = config["transform"]["rotation"]["y"].as<double>();
                double qz = config["transform"]["rotation"]["z"].as<double>();
                double qw = config["transform"]["rotation"]["w"].as<double>();
                extrinsic.linear() =
                    Eigen::Quaterniond(qw, qx, qy, qz).toRotationMatrix();
            }else{
                return false;
            }
        }else{
            return false;
        }
    }

    if (config["transform"])
    {
        scan_cloud_file = config["scan_cloud"].as<std::string>();
        rgbd_cloud_file = config["rgbd_cloud"].as<std::string>();
    }else{
        return false;
    }

    return true;
}

void CameraScanCalib::UpdateYaw(int value, void *this_ptr)
{
    CameraScanCalib *calib_ptr = static_cast<CameraScanCalib *>(this_ptr);
    calib_ptr->g_yaw = (value - 18000) * 0.01 * M_PI / 360;
    calib_ptr->Project();
}

void CameraScanCalib::UpdatePitch(int value, void *this_ptr)
{
    CameraScanCalib *calib_ptr = static_cast<CameraScanCalib *>(this_ptr);
    calib_ptr->g_pitch = (value - 18000) * 0.01 * M_PI / 360;
    calib_ptr->Project();
}

void CameraScanCalib::UpdateRoll(int value, void *this_ptr)
{
    CameraScanCalib *calib_ptr = static_cast<CameraScanCalib *>(this_ptr);
    calib_ptr->g_roll = (value - 18000) * 0.01 * M_PI / 360;
    calib_ptr->Project();
}

void CameraScanCalib::UpdateX(int value, void *this_ptr)
{
    CameraScanCalib *calib_ptr = static_cast<CameraScanCalib *>(this_ptr);
    calib_ptr->g_x = (value - 50) * 0.01;
    calib_ptr->Project();
}

void CameraScanCalib::UpdateY(int value, void *this_ptr)
{
    CameraScanCalib *calib_ptr = static_cast<CameraScanCalib *>(this_ptr);
    calib_ptr->g_y = (value - 50) * 0.01;
    calib_ptr->Project();
}

void CameraScanCalib::UpdateZ(int value, void *this_ptr)
{
    CameraScanCalib *calib_ptr = static_cast<CameraScanCalib *>(this_ptr);
    calib_ptr->g_z = (value - 50) * 0.01;
    calib_ptr->Project();
}
