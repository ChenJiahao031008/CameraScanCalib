cmake_minimum_required(VERSION 2.8.3)
project(scan2pointcloud)

find_package(catkin REQUIRED COMPONENTS
  laser_geometry
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  tf
)

find_package(PCL 1.7 REQUIRED)
add_definitions( ${PCL_DEFINITIONS} )
link_directories( ${PCL_LIBRARY_DIRS} )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/ConvertFilter.cpp src/ConvertNode.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
