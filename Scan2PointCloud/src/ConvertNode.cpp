#include <ros/ros.h>
#include <ros/package.h>
#include "ConvertFilter.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "scan_to_pointcloud");
    std::string current_path = ros::package::getPath("scan2pointcloud");
    ConvertFilter filter(current_path);

    ros::spin();

    return 0;
}
