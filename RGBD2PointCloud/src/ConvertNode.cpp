#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>
#include "ImageConverter.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "convert_node");
    std::string current_path = ros::package::getPath("rgbd_to_cloud");
    ImageConverter ic(current_path);
    ros::spin();
    return 0;
}
