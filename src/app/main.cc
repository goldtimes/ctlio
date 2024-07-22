#include <gflags/gflags.h>
#include <glog/logging.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <yaml-cpp/yaml.h>
#include <memory>
#include <thread>
#include "lidar_process/lidar_process.hh"
#include "sensors/imu.hh"
#include "sensors/point_types.hh"
#include "tools/lidar_utils.hh"
#include "tools/timer/timer.hh"

DEFINE_string(config_yaml, "./config/ctlio.yaml", "配置文件");
// ROOT_DIR在cmake中定义
#define DEBUG_FILE_DIR(name) (std::string(std::string(ROOT_DIR) + "log/" + name))

std::string lidar_topic;
std::string imu_topic;
std::shared_ptr<ctlio::LidarProcess> lidar_process;
ros::Publisher pub_scan;
ros::Publisher pub_lidar_odom;
ros::Publisher pub_lidar_path;
ros::Subscriber lidar_sub;
ros::Subscriber imu_sub;

void lidar_callback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    // ConstPtr to Ptr;
    sensor_msgs::PointCloud2::Ptr cloud_in(new sensor_msgs::PointCloud2(*msg));
    static int count = 0;
    std::vector<ctlio::point3D> cloud_out;
    ctlio::Timer::Evaluate([&]() { lidar_process->process(cloud_in, cloud_out); }, "lidar process");
    // 利用voxel降采样
    // lio push_cloud();
}

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg) {
    Eigen::Vector3d acc(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    Eigen::Vector3d gyro(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
    ctlio::IMU imu(msg->header.stamp.toSec(), acc, gyro);
    // lio push_imu();
}

// 这里的nh不能为const
void init_sub_pub(ros::NodeHandle& nh) {
    lidar_sub = nh.subscribe<sensor_msgs::PointCloud2>(lidar_topic, 10, lidar_callback);
    imu_sub = nh.subscribe<sensor_msgs::Imu>(imu_topic, 500, imu_callback);
}

int main(int argc, char** argv) {
    // glog
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);
    // ros
    ros::init(argc, argv, "main_node");
    ros::NodeHandle nh;
    // yaml
    std::string yaml_path = std::string(ROOT_DIR) + "config/ctlio.yaml";

    YAML::Node yaml = YAML::LoadFile(yaml_path);
    lidar_topic = yaml["common"]["lidar_topic"].as<std::string>();
    imu_topic = yaml["common"]["imu_topic"].as<std::string>();
    // lio
    // lidar_process
    lidar_process = std::make_shared<ctlio::LidarProcess>();
    lidar_process->LoadYaml(yaml_path);
    init_sub_pub(nh);

    // std::thread measurment_process();

    ros::spin();
    return 0;
}