#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>
#include "CameraScanCalib.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "calib_node");
    std::string current_path = ros::package::getPath("camera_scan_calib");
    CameraScanCalib csc(current_path);
    csc.Run();
    return 0;
}
