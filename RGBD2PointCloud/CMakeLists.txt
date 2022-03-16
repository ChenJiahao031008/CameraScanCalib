cmake_minimum_required(VERSION 3.10.2)
project(rgbd2cloud)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE release)
ENDIF()

add_compile_options(-std=c++14)

# LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

## 找到依赖安装
find_package( catkin REQUIRED )
find_package( Eigen3 REQUIRED)
find_package( OpenCV 3 REQUIRED )
find_package( yaml-cpp REQUIRED )
find_package( PCL 1.7 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  cv_bridge
  roscpp
  rospy
  std_msgs
  image_transport
  geometry_msgs
)


include_directories(
  include ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/include
  ${OpenCV_INCLUDE_DIRS}
  ${YAML_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# 链接库文件
add_library(${PROJECT_NAME} SHARED
  src/ImageConverter.cc
  # src/RGBD2PointCloud.cc
)

# 可执行节点
add_executable(convert_node
  src/ConvertNode.cpp
)
add_dependencies(convert_node
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(convert_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
  yaml-cpp
)

