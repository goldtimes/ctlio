#include <livox_ros_driver/CustomMsg.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <yaml-cpp/yaml.h>
#include <iostream>
#include "lidar_process/lidar_feature/lidar_feature_extractor.hh"
#include "lidar_process/segment/segment.hh"

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
ctlio::LidarFeatureExtractor feature_extractor;
// 全量点云
ros::Publisher pubFullCloud;
// 平面点云
ros::Publisher pubSurfCloud;
// 角点
ros::Publisher pubCornerCloud;
// 无规则点
ros::Publisher pubNonFeature;
PointCloud::Ptr lidarFull;
PointCloud::Ptr cornerCloud;
PointCloud::Ptr surfCloud;
PointCloud::Ptr nonFeatureCloud;
// 参与计算曲率的点数
int NumCurvSize;
// 距离阈值
float DistanceFaraway;
// 平面个数
int NumFlat;
// 分段数
int PartNum;
// 平面判断阈值
float FlatThresh;
float BreakCornerDist;
float LidarNearestDist;
float KdTreeCornerOutlierDis;
int N_SCAN_LINES = 6;

void lidar_callback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
}

void lidarCallBackHorizon(const livox_ros_driver::CustomMsg::ConstPtr& msg) {
    feature_extractor->FeatureExtract_with_segment(msg, lidarFull, cornerCloud, surfCloud, nonFeatureCloud, msg2,
                                                   N_SCAN_LINES);
}

void load_yaml(const std::string& path) {
    auto yaml = YAML::LoadFile(path);
    NumCurvSize = yaml["feature"]["NumCurvSize"].as<int>();
    DistanceFaraway = yaml["feature"]["DistanceFaraway"].as<float>();
    NumFlat = yaml["feature"]["NumFlat"].as<int>();
    PartNum = yaml["feature"]["PartNum"].as<int>();
    N_SCAN_LINES = yaml["feature"]["N_SCAN_LINES"].as<int>();
    FlatThresh = yaml["feature"]["FlatThresh"].as<float>();
    BreakCornerDist = yaml["feature"]["BreakCornerDist"].as<float>();
    LidarNearestDist = yaml["feature"]["LidarNearestDist"].as<float>();
    KdTreeCornerOutlierDis = yaml["feature"]["KdTreeCornerOutlierDis"].as<float>();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_feature_extractor");
    ros::NodeHandle nh;
    ros::Subscriber lidar_sub = nh.subscribe("lslidar_point_cloud", 10, lidarCallback);
    ros::Subscriber custom_sub = nh.subscribe("/livox/lidar", 100, lidarCallBackHorizon);
    pubFullCloud = nh.advertise<sensor_msgs::PointCloud2>("lidar_full_cloud", 10);
    pubSurfCloud = nh.advertise<sensor_msgs::PointCloud2>("lidar_surf_cloud", 10);
    pubCornerCloud = nh.advertise<sensor_msgs::PointCloud2>("lidar_corner_cloud", 10);
    pubNonFeature = nh.advertise<sensor_msgs::PointCloud2>("lidar_nonfeature_cloud", 10);
    std::string config_path;
    nh.param<std::string>("config_path", config_path, "");
    load_yaml(config_path);
    ros::spin();
    return 0;
}