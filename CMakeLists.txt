cmake_minimum_required(VERSION 3.2)
project(ct_lio)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14 -O3")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread -std=c++0x -std=c++14 -fexceptions")

# 包含自己的cmake
include(cmake/packages.cmake)

# ros
find_package(catkin REQUIRED COMPONENTS
geometry_msgs
nav_msgs
sensor_msgs
roscpp
rospy
rosbag
std_msgs
tf
livox_ros_driver  ## for livox
eigen_conversions
message_generation
)

catkin_package( CATKIN_DEPENDS 
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  rosbag
  std_msgs
  tf
  livox_ros_driver  ## for livox
  eigen_conversions
  message_generation
)

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

# PROJECT_SOURCE_DIR = .../ctlio
include_directories(
    ${PROJECT_SOURCE_DIR}
)

# src中的头文件
include_directories(
    ${PROJECT_SOURCE_DIR}/src
)

# sophus的头文件
include_directories(
    ${PROJECT_SOURCE_DIR}/thirdparty/sophus
)

add_subdirectory(
    src
)
